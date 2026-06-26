/****************************************************************************
Copyright (c) 2026.
****************************************************************************/

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mobile_manipulator/FactoryFunctions.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_msgs/msg/mpc_observation.hpp>

#include "rclcpp/rclcpp.hpp"

namespace ocs2 {
namespace mobile_manipulator {

class DualArmGraspWaypointPlanner final {
 public:
  DualArmGraspWaypointPlanner(const rclcpp::Node::SharedPtr& node, const std::string& topicPrefix,
                              PinocchioInterface pinocchioInterface, ManipulatorModelInfo modelInfo)
      : node_(node),
        pinocchioInterface_(std::move(pinocchioInterface)),
        modelInfo_(std::move(modelInfo)),
        pinocchioMapping_(modelInfo_),
        topicPrefix_(topicPrefix) {
    if (modelInfo_.eeFrames.size() != 2) {
      throw std::runtime_error("[DualArmGraspWaypointPlanner] Exactly two ee frames are required.");
    }

    const auto& model = pinocchioInterface_.getModel();
    eeFrameIds_[0] = model.getBodyId(modelInfo_.eeFrames[0]);
    eeFrameIds_[1] = model.getBodyId(modelInfo_.eeFrames[1]);

    planningFrame_ = node_->declare_parameter<std::string>("planning_frame", modelInfo_.baseFrame);
    boxPoseTopic_ = node_->declare_parameter<std::string>("box_pose_topic", "box_pose");
    leftGraspPoseTopic_ = node_->declare_parameter<std::string>("left_grasp_pose_topic", "left_grasp_pose");
    rightGraspPoseTopic_ = node_->declare_parameter<std::string>("right_grasp_pose_topic", "right_grasp_pose");

    approachDistance_ = node_->declare_parameter<double>("approach_distance", 0.15);
    retreatDistance_ = node_->declare_parameter<double>("retreat_distance", 0.15);
    liftDistance_ = node_->declare_parameter<double>("lift_distance", 0.15);
    viaExtraHeight_ = node_->declare_parameter<double>("via_extra_height", 0.10);
    minBoxClearance_ = node_->declare_parameter<double>("min_box_clearance", 0.05);
    minTableClearance_ = node_->declare_parameter<double>("min_table_clearance", 0.05);
    minGraspSeparation_ = node_->declare_parameter<double>("min_grasp_separation", 0.05);

    dtCurrentToVia_ = node_->declare_parameter<double>("dt_current_to_via", 1.0);
    dtViaToPregrasp_ = node_->declare_parameter<double>("dt_via_to_pregrasp", 1.0);
    dtPregraspToGrasp_ = node_->declare_parameter<double>("dt_pregrasp_to_grasp", 0.8);
    dtGraspToRetreat_ = node_->declare_parameter<double>("dt_grasp_to_retreat", 0.8);
    dtRetreatToLift_ = node_->declare_parameter<double>("dt_retreat_to_lift", 1.0);

    if (planningFrame_ != modelInfo_.baseFrame) {
      RCLCPP_WARN(node_->get_logger(),
                  "Planning frame '%s' differs from task base frame '%s'; this node assumes they are already aligned.",
                  planningFrame_.c_str(), modelInfo_.baseFrame.c_str());
    }

    observationSubscriber_ = node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
        topicPrefix_ + "_mpc_observation", 1,
        [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
          hasObservation_ = true;
        });

    boxPoseSubscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        boxPoseTopic_, 1, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          boxPoseMsg_ = *msg;
          hasBoxPose_ = true;
        });

    leftGraspPoseSubscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        leftGraspPoseTopic_, 1, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          graspPoseMsgs_[0] = *msg;
          hasGraspPose_[0] = true;
        });

    rightGraspPoseSubscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        rightGraspPoseTopic_, 1, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          graspPoseMsgs_[1] = *msg;
          hasGraspPose_[1] = true;
        });

    targetTrajectoriesPublisherPtr_ = std::make_unique<TargetTrajectoriesRosPublisher>(node_, topicPrefix_);
    planService_ = node_->create_service<std_srvs::srv::Trigger>(
        "plan_and_send_grasp_trajectory",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) { handlePlanRequest(*response); });

    RCLCPP_INFO(node_->get_logger(),
                "Dual-arm grasp waypoint planner ready. Subscribing to %s, %s, %s and publishing to %s_mpc_target.",
                boxPoseTopic_.c_str(), leftGraspPoseTopic_.c_str(), rightGraspPoseTopic_.c_str(), topicPrefix_.c_str());
  }

  void spin() { rclcpp::spin(node_); }

 private:
  struct PoseData {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  };

  struct PlannerSnapshot {
    SystemObservation observation;
    PoseData boxPose;
    std::array<PoseData, 2> graspPoses;
  };

  static bool isFinite(const Eigen::Vector3d& vector) { return vector.array().isFinite().all(); }

  static bool isFinite(const Eigen::Quaterniond& quaternion) { return quaternion.coeffs().array().isFinite().all(); }

  static PoseData poseDataFromMsg(const geometry_msgs::msg::PoseStamped& msg) {
    PoseData pose;
    pose.position.x() = msg.pose.position.x;
    pose.position.y() = msg.pose.position.y;
    pose.position.z() = msg.pose.position.z;
    pose.orientation = Eigen::Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
    return pose;
  }

  static std::string formatVector(const Eigen::Vector3d& vector) {
    std::ostringstream stream;
    stream << "[" << vector.x() << ", " << vector.y() << ", " << vector.z() << "]";
    return stream.str();
  }

  bool normalizeQuaternion(Eigen::Quaterniond& quaternion, std::string* errorMessage) const {
    if (!isFinite(quaternion)) {
      if (errorMessage != nullptr) {
        *errorMessage = "Quaternion contains NaN/Inf.";
      }
      return false;
    }

    const double norm = quaternion.norm();
    if (norm < 1e-9) {
      if (errorMessage != nullptr) {
        *errorMessage = "Quaternion norm is too small to normalize.";
      }
      return false;
    }

    quaternion.normalize();
    return true;
  }

  bool readSnapshot(PlannerSnapshot& snapshot, std::string& errorMessage) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!hasObservation_) {
      errorMessage = "No mobile_manipulator_mpc_observation received yet.";
      return false;
    }
    if (!hasBoxPose_) {
      errorMessage = "No box PoseStamped received yet.";
      return false;
    }
    if (!hasGraspPose_[0]) {
      errorMessage = "No left grasp PoseStamped received yet.";
      return false;
    }
    if (!hasGraspPose_[1]) {
      errorMessage = "No right grasp PoseStamped received yet.";
      return false;
    }
    if (latestObservation_.input.size() <= 0) {
      errorMessage = "Observation input dimension is empty.";
      return false;
    }

    const auto allFramesMatch = [this](const geometry_msgs::msg::PoseStamped& msg) {
      return msg.header.frame_id == planningFrame_;
    };
    if (!allFramesMatch(boxPoseMsg_) || !allFramesMatch(graspPoseMsgs_[0]) || !allFramesMatch(graspPoseMsgs_[1])) {
      std::ostringstream stream;
      stream << "All PoseStamped inputs must use frame_id '" << planningFrame_ << "'.";
      errorMessage = stream.str();
      return false;
    }

    snapshot.observation = latestObservation_;
    snapshot.boxPose = poseDataFromMsg(boxPoseMsg_);
    snapshot.graspPoses[0] = poseDataFromMsg(graspPoseMsgs_[0]);
    snapshot.graspPoses[1] = poseDataFromMsg(graspPoseMsgs_[1]);
    return true;
  }

  std::array<PoseData, 2> computeCurrentEndEffectorPoses(const SystemObservation& observation) {
    std::array<PoseData, 2> currentPoses;

    vector_t q = pinocchioMapping_.getPinocchioJointPosition(observation.state);
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    for (size_t i = 0; i < 2; ++i) {
      currentPoses[i].position = data.oMf[eeFrameIds_[i]].translation();
      currentPoses[i].orientation = Eigen::Quaterniond(data.oMf[eeFrameIds_[i]].rotation());
      currentPoses[i].orientation.normalize();
    }

    return currentPoses;
  }

  Eigen::Vector3d getApproachDirection(size_t armIndex, const Eigen::Quaterniond& orientation) const {
    const Eigen::Vector3d localYAxis = orientation.toRotationMatrix().col(1);
    return (armIndex == 0) ? localYAxis : -localYAxis;
  }

  bool buildWaypoints(const PlannerSnapshot& snapshot, std::array<std::vector<PoseData>, 2>& armWaypoints, std::string& errorMessage) {
    auto currentPoses = computeCurrentEndEffectorPoses(snapshot.observation);

    const double clearanceBaseZ = std::max(minTableClearance_, snapshot.boxPose.position.z() + minBoxClearance_);

    for (size_t armIndex = 0; armIndex < 2; ++armIndex) {
      auto graspPose = snapshot.graspPoses[armIndex];
      if (!normalizeQuaternion(graspPose.orientation, &errorMessage)) {
        errorMessage = (armIndex == 0 ? "Left grasp pose invalid: " : "Right grasp pose invalid: ") + errorMessage;
        return false;
      }
      const Eigen::Vector3d approachDirection = getApproachDirection(armIndex, graspPose.orientation);
      if (!isFinite(approachDirection) || approachDirection.norm() < 1e-9) {
        errorMessage = armIndex == 0 ? "Left approach direction is invalid." : "Right approach direction is invalid.";
        return false;
      }

      PoseData current = currentPoses[armIndex];
      PoseData preGrasp;
      preGrasp.orientation = graspPose.orientation;
      preGrasp.position = graspPose.position - approachDistance_ * approachDirection;

      PoseData grasp = graspPose;

      PoseData retreat;
      retreat.orientation = graspPose.orientation;
      retreat.position = graspPose.position - retreatDistance_ * approachDirection;

      PoseData lift = retreat;
      lift.position.z() = std::max(retreat.position.z() + liftDistance_, clearanceBaseZ);

      PoseData via = preGrasp;
      via.position.z() = std::max({current.position.z(), preGrasp.position.z(), clearanceBaseZ}) + viaExtraHeight_;

      const double preGraspProjection = (grasp.position - preGrasp.position).dot(approachDirection);
      const double retreatProjection = (grasp.position - retreat.position).dot(approachDirection);
      if (preGraspProjection <= 0.0 || retreatProjection <= 0.0) {
        errorMessage = armIndex == 0 ? "Left pre-grasp/retreat direction does not match +Y approach."
                                     : "Right pre-grasp/retreat direction does not match -Y approach.";
        return false;
      }

      const std::array<PoseData, 6> poses{current, via, preGrasp, grasp, retreat, lift};
      for (const auto& pose : poses) {
        if (!isFinite(pose.position) || !isFinite(pose.orientation)) {
          errorMessage = "Generated waypoint contains NaN/Inf values.";
          return false;
        }
        if (pose.position.z() < minTableClearance_ - 1e-9) {
          errorMessage = "Generated waypoint violates min_table_clearance.";
          return false;
        }
      }
      if (via.position.z() < clearanceBaseZ - 1e-9 || lift.position.z() < clearanceBaseZ - 1e-9) {
        errorMessage = "Generated via/lift waypoint violates box or table clearance.";
        return false;
      }

      armWaypoints[armIndex] = std::vector<PoseData>(poses.begin(), poses.end());
    }

    for (size_t stage = 1; stage < armWaypoints[0].size(); ++stage) {
      const double separation = (armWaypoints[0][stage].position - armWaypoints[1][stage].position).norm();
      if (separation < minGraspSeparation_) {
        std::ostringstream stream;
        stream << "Left/right waypoint separation too small at stage " << stage << ": " << separation;
        errorMessage = stream.str();
        return false;
      }
    }

    return true;
  }

  TargetTrajectories buildTargetTrajectories(const PlannerSnapshot& snapshot, const std::array<std::vector<PoseData>, 2>& armWaypoints) const {
    const std::vector<double> timeOffsets{0.0,
                                          dtCurrentToVia_,
                                          dtCurrentToVia_ + dtViaToPregrasp_,
                                          dtCurrentToVia_ + dtViaToPregrasp_ + dtPregraspToGrasp_,
                                          dtCurrentToVia_ + dtViaToPregrasp_ + dtPregraspToGrasp_ + dtGraspToRetreat_,
                                          dtCurrentToVia_ + dtViaToPregrasp_ + dtPregraspToGrasp_ + dtGraspToRetreat_ + dtRetreatToLift_};

    scalar_array_t timeTrajectory;
    vector_array_t stateTrajectory;
    vector_array_t inputTrajectory;

    timeTrajectory.reserve(timeOffsets.size());
    stateTrajectory.reserve(timeOffsets.size());
    inputTrajectory.reserve(timeOffsets.size());

    const vector_t zeroInput = vector_t::Zero(snapshot.observation.input.size());

    for (size_t stage = 0; stage < timeOffsets.size(); ++stage) {
      vector_t state(14);

      state.segment<3>(0) = armWaypoints[0][stage].position;
      state.segment<4>(3) = armWaypoints[0][stage].orientation.coeffs();
      state.segment<3>(7) = armWaypoints[1][stage].position;
      state.segment<4>(10) = armWaypoints[1][stage].orientation.coeffs();

      timeTrajectory.push_back(snapshot.observation.time + timeOffsets[stage]);
      stateTrajectory.push_back(std::move(state));
      inputTrajectory.push_back(zeroInput);
    }

    return {timeTrajectory, stateTrajectory, inputTrajectory};
  }

  void handlePlanRequest(std_srvs::srv::Trigger::Response& response) {
    PlannerSnapshot snapshot;
    std::string errorMessage;

    if (!readSnapshot(snapshot, errorMessage)) {
      response.success = false;
      response.message = errorMessage;
      RCLCPP_WARN(node_->get_logger(), "%s", errorMessage.c_str());
      return;
    }

    std::array<std::vector<PoseData>, 2> armWaypoints;
    if (!buildWaypoints(snapshot, armWaypoints, errorMessage)) {
      response.success = false;
      response.message = errorMessage;
      RCLCPP_WARN(node_->get_logger(), "%s", errorMessage.c_str());
      return;
    }

    const auto targetTrajectories = buildTargetTrajectories(snapshot, armWaypoints);
    if (targetTrajectories.timeTrajectory.size() != targetTrajectories.stateTrajectory.size() ||
        targetTrajectories.stateTrajectory.size() != targetTrajectories.inputTrajectory.size()) {
      response.success = false;
      response.message = "Constructed trajectory arrays have mismatched lengths.";
      RCLCPP_ERROR(node_->get_logger(), "%s", response.message.c_str());
      return;
    }

    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);

    std::ostringstream stream;
    stream << "Published " << targetTrajectories.stateTrajectory.size() << " grasp waypoints. "
           << "Left grasp=" << formatVector(snapshot.graspPoses[0].position) << ", "
           << "Right grasp=" << formatVector(snapshot.graspPoses[1].position) << ".";
    response.success = true;
    response.message = stream.str();
    RCLCPP_INFO(node_->get_logger(), "%s", response.message.c_str());
  }

  rclcpp::Node::SharedPtr node_;
  PinocchioInterface pinocchioInterface_;
  ManipulatorModelInfo modelInfo_;
  MobileManipulatorPinocchioMapping pinocchioMapping_;
  std::string topicPrefix_;

  std::string planningFrame_;
  std::string boxPoseTopic_;
  std::string leftGraspPoseTopic_;
  std::string rightGraspPoseTopic_;

  double approachDistance_ = 0.15;
  double retreatDistance_ = 0.15;
  double liftDistance_ = 0.15;
  double viaExtraHeight_ = 0.10;
  double minBoxClearance_ = 0.05;
  double minTableClearance_ = 0.05;
  double minGraspSeparation_ = 0.05;

  double dtCurrentToVia_ = 1.0;
  double dtViaToPregrasp_ = 1.0;
  double dtPregraspToGrasp_ = 0.8;
  double dtGraspToRetreat_ = 0.8;
  double dtRetreatToLift_ = 1.0;

  std::array<pinocchio::FrameIndex, 2> eeFrameIds_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSubscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr boxPoseSubscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leftGraspPoseSubscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rightGraspPoseSubscriber_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr planService_;

  std::mutex mutex_;
  SystemObservation latestObservation_;
  bool hasObservation_ = false;
  geometry_msgs::msg::PoseStamped boxPoseMsg_;
  bool hasBoxPose_ = false;
  std::array<geometry_msgs::msg::PoseStamped, 2> graspPoseMsgs_;
  std::array<bool, 2> hasGraspPose_{{false, false}};
};

}  // namespace mobile_manipulator
}  // namespace ocs2

int main(int argc, char* argv[]) {
  const std::string robotName = "mobile_manipulator";
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(robotName + "_dual_arm_grasp_waypoint_planner");

  const std::string taskFile = node->declare_parameter<std::string>("taskFile", "");
  const std::string urdfFile = node->declare_parameter<std::string>("urdfFile", "");
  if (taskFile.empty() || urdfFile.empty()) {
    throw std::runtime_error("[DualArmGraspWaypointPlanner] Parameters 'taskFile' and 'urdfFile' are required.");
  }
  if (!std::ifstream(taskFile).good()) {
    throw std::runtime_error("[DualArmGraspWaypointPlanner] Task file not found: " + taskFile);
  }
  if (!std::ifstream(urdfFile).good()) {
    throw std::runtime_error("[DualArmGraspWaypointPlanner] URDF file not found: " + urdfFile);
  }

  ocs2::mobile_manipulator::ManipulatorModelType modelType =
      ocs2::mobile_manipulator::loadManipulatorType(taskFile, "model_information.manipulatorModelType");
  std::vector<std::string> removeJointNames;
  ocs2::loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, false);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string baseFrame;
  std::vector<std::string> eeFrames;
  ocs2::loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", false);
  ocs2::loadData::loadStdVector<std::string>(taskFile, "model_information.eeFrames", eeFrames, false);
  if (eeFrames.empty()) {
    std::string eeFrame;
    ocs2::loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", false);
    eeFrames.push_back(eeFrame);
  }

  if (eeFrames.size() != 2) {
    throw std::runtime_error("[DualArmGraspWaypointPlanner] 'model_information.eeFrames' must contain exactly 2 frames.");
  }

  auto pinocchioInterface = ocs2::mobile_manipulator::createPinocchioInterface(urdfFile, modelType, removeJointNames);
  auto modelInfo =
      ocs2::mobile_manipulator::createManipulatorModelInfo(pinocchioInterface, modelType, baseFrame, eeFrames);

  ocs2::mobile_manipulator::DualArmGraspWaypointPlanner planner(node, robotName, std::move(pinocchioInterface),
                                                                 std::move(modelInfo));
  planner.spin();
  return 0;
}
