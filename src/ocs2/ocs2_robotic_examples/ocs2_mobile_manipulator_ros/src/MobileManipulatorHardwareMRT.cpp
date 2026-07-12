/******************************************************************************
Copyright (c) 2026.
******************************************************************************/

#include <ocs2_mobile_manipulator/AccessHelperFunctions.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h>
#include <ocs2_mobile_manipulator_ros/JointStateHardwareBridgeHelpers.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <Eigen/Geometry>

#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>

#include <sensor_msgs/msg/joint_state.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

using namespace ocs2;
using namespace mobile_manipulator;

namespace {

constexpr const char* kRobotName = "mobile_manipulator";

class MobileManipulatorHardwareMrtNode {
 public:
  explicit MobileManipulatorHardwareMrtNode(const rclcpp::Node::SharedPtr& node)
      : node_(node),
        taskFile_(node_->declare_parameter<std::string>("taskFile", "")),
        libFolder_(node_->declare_parameter<std::string>("libFolder", "")),
        urdfFile_(node_->declare_parameter<std::string>("urdfFile", "")),
        observationTopic_(node_->declare_parameter<std::string>("observation_topic", "/gensong/joint_states")),
        commandTopic_(node_->declare_parameter<std::string>("command_topic", "/gensong/joint_command")),
        observationTimeoutSec_(node_->declare_parameter<double>("observation_timeout_sec", 0.1)),
        commandBlendAlpha_(node_->declare_parameter<double>("command_blend_alpha", 0.15)),
        maxJointDeltaPerCommand_(node_->declare_parameter<double>("max_joint_delta_per_command", 0.03)),
        interface_(taskFile_, libFolder_, urdfFile_),
        mrt_(kRobotName),
        pinocchioInterface_(interface_.getPinocchioInterface()),
        modelInfo_(interface_.getManipulatorModelInfo()),
        pinocchioMapping_(modelInfo_),
        latestInput_(vector_t::Zero(modelInfo_.inputDim)),
        mpcUpdateRatio_(std::max(static_cast<size_t>(interface_.mpcSettings().mrtDesiredFrequency_ /
                                                     std::max(interface_.mpcSettings().mpcDesiredFrequency_, scalar_t(1.0))),
                                 size_t(1))),
        mpcObservationTopic_(std::string(kRobotName) + "_mpc_observation"),
        mpcPolicyTopic_(std::string(kRobotName) + "_mpc_policy"),
        mpcResetService_(std::string(kRobotName) + "_mpc_reset") {
    if (taskFile_.empty() || libFolder_.empty() || urdfFile_.empty()) {
      throw std::runtime_error(
          "[MobileManipulatorHardwareMRT] Parameters 'taskFile', 'libFolder', and 'urdfFile' are required.");
    }

    outputJointNames_ = loadMovableJointNamesFromUrdf(urdfFile_);
    if (outputJointNames_.empty()) {
      RCLCPP_WARN(node_->get_logger(),
                  "[MobileManipulatorHardwareMRT] No movable joints found in URDF. Falling back to modelInfo_.dofNames.");
      outputJointNames_ = modelInfo_.dofNames;
    }

    mrt_.initRollout(&interface_.getRollout());
    mrt_.launchNodes(node_);

    commandPublisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(commandTopic_, rclcpp::QoS(10));
    observationSubscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        observationTopic_, rclcpp::SensorDataQoS(),
        std::bind(&MobileManipulatorHardwareMrtNode::jointStateCallback, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "Hardware MRT subscribing %s and publishing %s", observationTopic_.c_str(),
                commandTopic_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Hardware MRT publishes commands only when a fresh hardware observation arrives.");
    RCLCPP_INFO(node_->get_logger(), "Hardware MRT requests a fresh MPC policy every %zu hardware observations.", mpcUpdateRatio_);
    RCLCPP_INFO(node_->get_logger(), "Hardware MRT filters commands with blend alpha %.3f and max delta %.3f rad per publish.",
                commandBlendAlpha_, maxJointDeltaPerCommand_);
    RCLCPP_INFO(node_->get_logger(),
                "Hardware MRT initializes MPC with the configured initial end-effector pose and follows MPC joint commands.");
  }

  void run() {
    waitForUniqueMpcEndpoints();
    waitForInitialObservation();
    const auto initialObservation = getLatestObservationOrThrow();
    auto initialTargetObservation = initialObservation;
    initialTargetObservation.state = interface_.getInitialState();
    const auto initTargetTrajectories = buildCurrentPoseTargetTrajectories(initialTargetObservation);
    RCLCPP_INFO(node_->get_logger(),
                "Resetting MPC with the configured initial end-effector pose; joint redundancy remains under MPC control.");
    mrt_.resetMpcNode(initTargetTrajectories);

    rclcpp::Rate rate(interface_.mpcSettings().mrtDesiredFrequency_);
    while (!mrt_.initialPolicyReceived() && rclcpp::ok()) {
      mrt_.spinMRT();
      SystemObservation observation;
      if (getLatestObservation(observation, nullptr)) {
        mrt_.setCurrentObservation(observation);
      }
      rate.sleep();
    }

    while (rclcpp::ok()) {
      mrt_.spinMRT();

      SystemObservation observation;
      std::string errorMessage;
      if (!getLatestObservation(observation, &errorMessage)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "[MobileManipulatorHardwareMRT] Command gated: %s", errorMessage.c_str());
        rate.sleep();
        continue;
      }

      const bool policyUpdated = mrt_.updatePolicy();

      if (!mrt_.initialPolicyReceived()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "[MobileManipulatorHardwareMRT] Command gated: initial MPC policy not received yet.");
        rate.sleep();
        continue;
      }

      if (pendingPolicyRequest_) {
        ++pendingPolicyRequestAgeTicks_;
      }

      if (pendingPolicyRequest_ && policyUpdated) {
        const auto& updatedPolicy = mrt_.getPolicy();
        if (isPolicyFreshEnoughForObservationTime(updatedPolicy.timeTrajectory_, pendingPolicyObservationTime_,
                                                  interface_.mpcSettings().mpcDesiredFrequency_)) {
          pendingPolicyRequest_ = false;
          pendingPolicyRequestAgeTicks_ = 0;
        }
      }

      bool forcePolicyRefresh = false;
      if (pendingPolicyRequest_ && shouldRetryPendingPolicyRequest(pendingPolicyRequestAgeTicks_, mpcUpdateRatio_)) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "[MobileManipulatorHardwareMRT] MPC policy update is lagging behind hardware observations. Republishing the latest observation to refresh policy.");
        pendingPolicyRequest_ = false;
        pendingPolicyRequestAgeTicks_ = 0;
        forcePolicyRefresh = true;
      }

      if (!shouldPublishCommandForObservationTime(observation.time, lastPublishedObservationTime_)) {
        rate.sleep();
        continue;
      }

      const auto& policy = mrt_.getPolicy();
      const double policyCoverageToleranceSec = 0.5 / interface_.mpcSettings().mrtDesiredFrequency_;
      double policyQueryTime = observation.time;
      std::string queryTimeError;
      if (!computeBoundedPolicyQueryTime(policy.timeTrajectory_, observation.time, policyCoverageToleranceSec, &policyQueryTime,
                                         &queryTimeError)) {
        const double policyFront = policy.timeTrajectory_.empty() ? observation.time : policy.timeTrajectory_.front();
        const double policyBack = policy.timeTrajectory_.empty() ? observation.time : policy.timeTrajectory_.back();
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "[MobileManipulatorHardwareMRT] Command gated: %s. observation time %.6f is outside active policy [%.6f, %.6f].",
            queryTimeError.c_str(), observation.time, policyFront, policyBack);
        if (shouldRefreshExpiredPolicy(pendingPolicyRequest_)) {
          mrt_.setCurrentObservation(observation);
          pendingPolicyObservationTime_ = observation.time;
          pendingPolicyRequest_ = true;
          pendingPolicyRequestAgeTicks_ = 0;
        }
        rate.sleep();
        continue;
      }

      vector_t nextState;
      vector_t nextInput;
      size_t nextMode = observation.mode;
      const scalar_t dt = 1.0 / interface_.mpcSettings().mrtDesiredFrequency_;
      mrt_.rolloutPolicy(policyQueryTime, observation.state, dt, nextState, nextInput, nextMode);
      updateLatestInput(nextInput, nextMode);

      std_msgs::msg::Header commandHeader;
      commandHeader.stamp = node_->get_clock()->now();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        commandHeader.frame_id = latestHeader_.frame_id;
      }

      const auto& nextStateConst = static_cast<const vector_t&>(nextState);
      const vector_t desiredArmState = vector_t(getArmJointAngles(nextStateConst, modelInfo_));
      const auto filteredArmState =
          filterCommandPositions(desiredArmState, lastPublishedControlledState_, commandBlendAlpha_, maxJointDeltaPerCommand_);
      auto commandMsg = buildCommandJointState(commandHeader, filteredArmState, modelInfo_.dofNames, outputJointNames_);
      commandPublisher_->publish(std::move(commandMsg));
      lastPublishedObservationTime_ = observation.time;
      lastPublishedControlledState_ = filteredArmState;

      if (forcePolicyRefresh || shouldRequestPolicyUpdate(processedObservationCount_, mpcUpdateRatio_, pendingPolicyRequest_)) {
        mrt_.setCurrentObservation(observation);
        pendingPolicyObservationTime_ = observation.time;
        pendingPolicyRequest_ = true;
        pendingPolicyRequestAgeTicks_ = 0;
      }
      ++processedObservationCount_;
      rate.sleep();
    }
  }

 private:
  void waitForUniqueMpcEndpoints() {
    rclcpp::Rate rate(10.0);
    const auto deadline = node_->get_clock()->now() + rclcpp::Duration::from_seconds(5.0);
    while (rclcpp::ok()) {
      mrt_.spinMRT();

      const size_t observationPublishers = node_->count_publishers(mpcObservationTopic_);
      const size_t policyPublishers = node_->count_publishers(mpcPolicyTopic_);
      const size_t resetServices = node_->count_services(mpcResetService_);

      std::string duplicateError;
      if (hasDuplicateMpcRosGraphEndpoints(observationPublishers, policyPublishers, resetServices, &duplicateError)) {
        throw std::runtime_error("[MobileManipulatorHardwareMRT] " + duplicateError +
                                 ". Stop stale mobile_manipulator launch instances before starting a new one.");
      }

      if (observationPublishers == 1u && resetServices == 1u) {
        return;
      }

      if (node_->get_clock()->now() >= deadline) {
        throw std::runtime_error("[MobileManipulatorHardwareMRT] Timed out waiting for a unique MPC graph. observation publishers=" +
                                 std::to_string(observationPublishers) + ", policy publishers=" +
                                 std::to_string(policyPublishers) + ", reset services=" + std::to_string(resetServices) + ".");
      }

      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "[MobileManipulatorHardwareMRT] Waiting for a unique MPC graph before reset: observation publishers=%zu, policy publishers=%zu, reset services=%zu.",
          observationPublishers, policyPublishers, resetServices);
      rate.sleep();
    }

    throw std::runtime_error("[MobileManipulatorHardwareMRT] ROS shutdown while waiting for a unique MPC graph.");
  }

  void waitForInitialObservation() {
    rclcpp::Rate rate(interface_.mpcSettings().mrtDesiredFrequency_);
    while (rclcpp::ok()) {
      mrt_.spinMRT();
      std::string errorMessage;
      SystemObservation observation;
      if (getLatestObservation(observation, &errorMessage)) {
        return;
      }
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "[MobileManipulatorHardwareMRT] Waiting for first complete observation: %s",
                           errorMessage.c_str());
      rate.sleep();
    }
    throw std::runtime_error("[MobileManipulatorHardwareMRT] ROS shutdown before receiving a complete observation.");
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
    Eigen::VectorXd state;
    std::string errorMessage;
    if (!mapJointStateToControlledPositions(*msg, modelInfo_.dofNames, &state, &errorMessage)) {
      std::lock_guard<std::mutex> lock(mutex_);
      hasCompleteObservation_ = false;
      latestObservationError_ = std::move(errorMessage);
      latestHeader_ = msg->header;
      return;
    }

    const auto now = node_->get_clock()->now();
    const auto stamp = rclcpp::Time(msg->header.stamp);
    const bool hasValidStamp = (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
    const double observationTimeSec = resolveObservationTime(now.seconds(), stamp.seconds(), hasValidStamp, observationTimeoutSec_);

    std::lock_guard<std::mutex> lock(mutex_);
    latestObservation_.mode = latestMode_;
    latestObservation_.time = observationTimeSec;
    latestObservation_.state = state;
    latestObservation_.input = latestInput_;
    latestHeader_ = msg->header;
    latestObservationError_.clear();
    hasCompleteObservation_ = true;
  }

  bool getLatestObservation(SystemObservation& observation, std::string* errorMessage) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!hasCompleteObservation_) {
      if (errorMessage != nullptr) {
        *errorMessage = latestObservationError_.empty() ? "no complete observation received yet" : latestObservationError_;
      }
      return false;
    }

    const double nowSec = node_->get_clock()->now().seconds();
    if (isObservationTimedOut(nowSec, latestObservation_.time, observationTimeoutSec_)) {
      if (errorMessage != nullptr) {
        *errorMessage = "latest observation timed out";
      }
      return false;
    }

    observation = latestObservation_;
    observation.input = latestInput_;
    observation.mode = latestMode_;
    return true;
  }

  SystemObservation getLatestObservationOrThrow() const {
    SystemObservation observation;
    std::string errorMessage;
    if (!getLatestObservation(observation, &errorMessage)) {
      throw std::runtime_error("[MobileManipulatorHardwareMRT] " + errorMessage);
    }
    return observation;
  }

  void updateLatestInput(const vector_t& nextInput, size_t nextMode) {
    std::lock_guard<std::mutex> lock(mutex_);
    latestInput_ = nextInput;
    latestMode_ = nextMode;
    if (hasCompleteObservation_) {
      latestObservation_.input = latestInput_;
      latestObservation_.mode = latestMode_;
    }
  }

  TargetTrajectories buildCurrentPoseTargetTrajectories(const SystemObservation& observation) {
    const auto q = pinocchioMapping_.getPinocchioJointPosition(observation.state);
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    const auto& eeFrames = modelInfo_.eeFrames.empty() ? std::vector<std::string>{modelInfo_.eeFrame} : modelInfo_.eeFrames;
    vector_t target(7 * eeFrames.size());
    for (size_t i = 0; i < eeFrames.size(); ++i) {
      const auto frameId = model.getBodyId(eeFrames[i]);
      target.segment<3>(7 * i) = data.oMf[frameId].translation();
      Eigen::Quaternion<scalar_t> orientation(data.oMf[frameId].rotation());
      orientation.normalize();
      target.segment<4>(7 * i + 3) = orientation.coeffs();
    }

    return TargetTrajectories({observation.time}, {target}, {vector_t::Zero(modelInfo_.inputDim)});
  }

  rclcpp::Node::SharedPtr node_;
  const std::string taskFile_;
  const std::string libFolder_;
  const std::string urdfFile_;
  const std::string observationTopic_;
  const std::string commandTopic_;
  const double observationTimeoutSec_;
  const double commandBlendAlpha_;
  const double maxJointDeltaPerCommand_;
  const std::string mpcObservationTopic_;
  const std::string mpcPolicyTopic_;
  const std::string mpcResetService_;

  MobileManipulatorInterface interface_;
  MRT_ROS_Interface mrt_;
  PinocchioInterface pinocchioInterface_;
  const ManipulatorModelInfo modelInfo_;
  MobileManipulatorPinocchioMapping pinocchioMapping_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr commandPublisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr observationSubscription_;

  std::vector<std::string> outputJointNames_;

  mutable std::mutex mutex_;
  SystemObservation latestObservation_;
  vector_t latestInput_;
  std_msgs::msg::Header latestHeader_;
  std::string latestObservationError_;
  size_t latestMode_ = 0;
  bool hasCompleteObservation_ = false;
  const size_t mpcUpdateRatio_;
  double pendingPolicyObservationTime_ = 0.0;
  bool pendingPolicyRequest_ = false;
  size_t pendingPolicyRequestAgeTicks_ = 0;
  size_t processedObservationCount_ = 0;
  std::optional<double> lastPublishedObservationTime_;
  std::optional<vector_t> lastPublishedControlledState_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mobile_manipulator_hardware_mrt");
  MobileManipulatorHardwareMrtNode hardwareMrt(node);
  hardwareMrt.run();
  rclcpp::shutdown();
  return 0;
}
