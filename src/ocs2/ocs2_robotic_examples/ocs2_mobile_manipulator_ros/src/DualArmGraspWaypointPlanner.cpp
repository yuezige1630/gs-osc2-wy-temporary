/****************************************************************************
Copyright (c) 2026.
****************************************************************************/

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <Eigen/Geometry>

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
    placeBoxPoseTopic_ = node_->declare_parameter<std::string>("place_box_pose_topic", "place_box_pose");
    boxSizeX_ = node_->declare_parameter<double>("box_size_x", 0.1978);
    boxSizeY_ = node_->declare_parameter<double>("box_size_y", 0.2966);
    boxSizeZ_ = node_->declare_parameter<double>("box_size_z", 0.1464);
    graspEdgeInsetY_ = node_->declare_parameter<double>("grasp_edge_inset_y", 0.0);
    graspXOffset_ = node_->declare_parameter<double>("grasp_x_offset", 0.0);
    graspZOffset_ = node_->declare_parameter<double>("grasp_z_offset", 0.0);
    graspFrameOrientation_ = rpyToQuaternion(node_->declare_parameter<double>("grasp_frame_rpy.roll", 0.0),
                                             node_->declare_parameter<double>("grasp_frame_rpy.pitch", 0.0),
                                             node_->declare_parameter<double>("grasp_frame_rpy.yaw", 0.0));
    wristOrientationCompensations_[0] =
        rpyToQuaternion(node_->declare_parameter<double>("left_wrist_compensation_rpy.roll", 0.0),
                        node_->declare_parameter<double>("left_wrist_compensation_rpy.pitch", 0.0),
                        node_->declare_parameter<double>("left_wrist_compensation_rpy.yaw", 0.0));
    wristOrientationCompensations_[1] =
        rpyToQuaternion(node_->declare_parameter<double>("right_wrist_compensation_rpy.roll", 0.0),
                        node_->declare_parameter<double>("right_wrist_compensation_rpy.pitch", 0.0),
                        node_->declare_parameter<double>("right_wrist_compensation_rpy.yaw", 0.0));

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
    graspHoldSec_ = node_->declare_parameter<double>("grasp_hold_sec", 2.0);
    dtGraspToRetreat_ = node_->declare_parameter<double>("dt_grasp_to_retreat", 0.8);
    dtRetreatToLift_ = node_->declare_parameter<double>("dt_retreat_to_lift", 1.0);
    enableTransportStage_ = node_->declare_parameter<bool>("enable_transport_stage", false);
    enablePlaceStage_ = node_->declare_parameter<bool>("enable_place_stage", false);
    maintainRigidGraspAfterContact_ = node_->declare_parameter<bool>("maintain_rigid_grasp_after_contact", true);
    transportOffsetIsAbsolute_ = node_->declare_parameter<bool>("transport_offset_is_absolute", false);
    dtLiftToTransport_ = node_->declare_parameter<double>("dt_lift_to_transport", 1.0);
    dtTransportToPrePlace_ = node_->declare_parameter<double>("dt_transport_to_pre_place", 0.5);
    dtPrePlaceToPlace_ = node_->declare_parameter<double>("dt_pre_place_to_place", 1.0);
    dtPlaceToRelease_ = node_->declare_parameter<double>("dt_place_to_release", 0.5);
    dtReleaseToPostReleaseRetreat_ = node_->declare_parameter<double>("dt_release_to_post_release_retreat", 1.0);
    dtPostReleaseRetreatToHome_ = node_->declare_parameter<double>("dt_post_release_retreat_to_home", 1.0);
    trajectoryTimeScale_ = node_->declare_parameter<double>("trajectory_time_scale", 1.0);
    prePlaceHeight_ = node_->declare_parameter<double>("pre_place_height", 0.10);
    postReleaseRetreatDistance_ = node_->declare_parameter<double>("post_release_retreat_distance", 0.15);
    postReleaseRetreatHeight_ = node_->declare_parameter<double>("post_release_retreat_height", 0.05);
    transportOffset_.x() = node_->declare_parameter<double>("transport_offset_x", 0.0);
    transportOffset_.y() = node_->declare_parameter<double>("transport_offset_y", 0.0);
    transportOffset_.z() = node_->declare_parameter<double>("transport_offset_z", 0.0);

    if (planningFrame_ != modelInfo_.baseFrame) {
      RCLCPP_WARN(node_->get_logger(),
                  "Planning frame '%s' differs from task base frame '%s'; this node assumes they are already aligned.",
                  planningFrame_.c_str(), modelInfo_.baseFrame.c_str());
    }
    if (!(std::isfinite(boxSizeX_) && std::isfinite(boxSizeY_) && std::isfinite(boxSizeZ_)) || boxSizeX_ <= 0.0 ||
        boxSizeY_ <= 0.0 || boxSizeZ_ <= 0.0) {
      throw std::runtime_error("[DualArmGraspWaypointPlanner] Box dimensions must be finite and strictly positive.");
    }
    if (!std::isfinite(graspEdgeInsetY_) || !std::isfinite(graspXOffset_) || !std::isfinite(graspZOffset_)) {
      throw std::runtime_error("[DualArmGraspWaypointPlanner] Grasp offset parameters must be finite.");
    }
    if (!std::isfinite(graspHoldSec_) || graspHoldSec_ < 0.0) {
      throw std::runtime_error("[DualArmGraspWaypointPlanner] grasp_hold_sec must be finite and non-negative.");
    }
    if (!std::isfinite(trajectoryTimeScale_) || trajectoryTimeScale_ <= 0.0) {
      throw std::runtime_error("[DualArmGraspWaypointPlanner] trajectory_time_scale must be finite and positive.");
    }
    graspEdgeOffsetY_ = std::max(0.0, 0.5 * boxSizeY_ - graspEdgeInsetY_);

    if (enableTransportStage_ || enablePlaceStage_ || transportOffsetIsAbsolute_ || transportOffset_.norm() > 1e-9) {
      RCLCPP_WARN(node_->get_logger(),
                  "enable_transport_stage, enable_place_stage, and transport_offset_* are deprecated in the two-stage "
                  "workflow and are ignored for automatic place execution.");
    }
    if (!maintainRigidGraspAfterContact_) {
      RCLCPP_WARN(node_->get_logger(),
                  "maintain_rigid_grasp_after_contact=false is not supported in the two-stage workflow; rigid grasp will "
                  "be enforced after contact.");
    }

    observationSubscriber_ = node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
        topicPrefix_ + "_mpc_observation", 1,
        [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg) { handleObservationMsg(*msg); });

    boxPoseSubscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        boxPoseTopic_, 1, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) { handleBoxPoseMsg(*msg); });

    placeBoxPoseSubscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        placeBoxPoseTopic_, 1,
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) { handlePlaceBoxPoseMsg(*msg); });

    targetTrajectoriesPublisherPtr_ = std::make_unique<TargetTrajectoriesRosPublisher>(node_, topicPrefix_);
    planService_ = node_->create_service<std_srvs::srv::Trigger>(
        "plan_and_send_grasp_trajectory",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) { handlePlanRequest(*response); });

    RCLCPP_INFO(node_->get_logger(),
                "Dual-arm two-stage planner ready. box_pose_topic=%s, place_box_pose_topic=%s, publishing to "
                "%s_mpc_target.",
                boxPoseTopic_.c_str(), placeBoxPoseTopic_.c_str(), topicPrefix_.c_str());
  }

  void spin() { rclcpp::spin(node_); }

 private:
  enum class PlannerState { IDLE, EXECUTING_GRASP, HOLDING_OBJECT, EXECUTING_PLACE };

  struct PoseData {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  };

  struct GraspSnapshot {
    SystemObservation observation;
    PoseData boxPose;
    std::array<PoseData, 2> graspPoses;
  };

  struct PlaceSnapshot {
    SystemObservation observation;
    PoseData placeBoxPose;
  };

  struct ActiveCarrySession {
    bool active = false;
    PoseData graspBoxPose;
    PoseData holdBoxPose;
    std::array<Eigen::Vector3d, 2> eeTranslationsInBoxFrame{{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}};
    std::array<Eigen::Quaterniond, 2> eeOrientationsInBoxFrame{
        {Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity()}};
  };

  static constexpr double kExecutionCompletionTolerance = 0.05;

  static const char* stateName(PlannerState state) {
    switch (state) {
      case PlannerState::IDLE:
        return "IDLE";
      case PlannerState::EXECUTING_GRASP:
        return "EXECUTING_GRASP";
      case PlannerState::HOLDING_OBJECT:
        return "HOLDING_OBJECT";
      case PlannerState::EXECUTING_PLACE:
        return "EXECUTING_PLACE";
    }
    return "UNKNOWN";
  }

  static bool isFinite(const Eigen::Vector3d& vector) { return vector.array().isFinite().all(); }

  static bool isFinite(const Eigen::Quaterniond& quaternion) { return quaternion.coeffs().array().isFinite().all(); }

  static PoseData poseDataFromMsg(const geometry_msgs::msg::PoseStamped& msg) {
    PoseData pose;
    pose.position.x() = msg.pose.position.x;
    pose.position.y() = msg.pose.position.y;
    pose.position.z() = msg.pose.position.z;
    pose.orientation = Eigen::Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                                          msg.pose.orientation.z);
    return pose;
  }

  static std::string formatVector(const Eigen::Vector3d& vector) {
    std::ostringstream stream;
    stream << "[" << vector.x() << ", " << vector.y() << ", " << vector.z() << "]";
    return stream.str();
  }

  static Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw) {
    Eigen::Quaterniond quaternion =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    quaternion.normalize();
    return quaternion;
  }

  double scaledTime(double time) const { return trajectoryTimeScale_ * time; }

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

  bool computeGraspPoses(const PoseData& boxPose, std::array<PoseData, 2>& graspPoses, std::string& errorMessage) const {
    if (!isFinite(boxPose.position)) {
      errorMessage = "Box pose position contains NaN/Inf.";
      return false;
    }

    Eigen::Quaterniond boxOrientation = boxPose.orientation;
    if (!normalizeQuaternion(boxOrientation, &errorMessage)) {
      errorMessage = "Box pose orientation invalid: " + errorMessage;
      return false;
    }

    const std::array<Eigen::Vector3d, 2> localOffsets{
        Eigen::Vector3d(graspXOffset_, graspEdgeOffsetY_, 0.01 + graspZOffset_),
        Eigen::Vector3d(graspXOffset_, -graspEdgeOffsetY_, 0.01 + graspZOffset_),
    };

    for (size_t armIndex = 0; armIndex < 2; ++armIndex) {
      PoseData graspPose;
      graspPose.orientation = boxOrientation * graspFrameOrientation_ * wristOrientationCompensations_[armIndex];
      if (!normalizeQuaternion(graspPose.orientation, &errorMessage)) {
        errorMessage = armIndex == 0 ? "Left grasp orientation invalid: " + errorMessage
                                     : "Right grasp orientation invalid: " + errorMessage;
        return false;
      }

      // The configured end-effector frames are the hand frames themselves, so we
      // target the hand pose directly instead of back-projecting to wrist3.
      graspPose.position = boxPose.position + boxOrientation * localOffsets[armIndex];

      if (!isFinite(graspPose.position) || !isFinite(graspPose.orientation)) {
        errorMessage = armIndex == 0 ? "Left grasp pose contains NaN/Inf." : "Right grasp pose contains NaN/Inf.";
        return false;
      }
      graspPoses[armIndex] = graspPose;
    }

    return true;
  }

  void clearCarrySessionUnlocked() {
    carrySession_ = ActiveCarrySession{};
    hasPlaceBoxPose_ = false;
  }

  void updatePlannerStateUnlocked(double currentTime) {
    if (plannerState_ == PlannerState::EXECUTING_GRASP &&
        currentTime + kExecutionCompletionTolerance >= activeTrajectoryEndTime_) {
      plannerState_ = PlannerState::HOLDING_OBJECT;
      RCLCPP_INFO(node_->get_logger(), "Transitioned to HOLDING_OBJECT.");
      return;
    }

    if (plannerState_ == PlannerState::EXECUTING_PLACE &&
        currentTime + kExecutionCompletionTolerance >= activeTrajectoryEndTime_) {
      plannerState_ = PlannerState::IDLE;
      clearCarrySessionUnlocked();
      RCLCPP_INFO(node_->get_logger(), "Place trajectory complete. Transitioned to IDLE.");
    }
  }

  void warnIgnoredTrigger(const std::string& triggerName, PlannerState requiredState) const {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Ignoring %s while planner state is %s. Required state: %s.", triggerName.c_str(),
                         stateName(plannerState_), stateName(requiredState));
  }

  bool readGraspSnapshotUnlocked(GraspSnapshot& snapshot, std::string& errorMessage) {
    if (!hasObservation_) {
      errorMessage = "No mobile_manipulator_mpc_observation received yet.";
      return false;
    }
    if (!hasBoxPose_) {
      errorMessage = "No box PoseStamped received yet.";
      return false;
    }
    if (latestObservation_.input.size() <= 0) {
      errorMessage = "Observation input dimension is empty.";
      return false;
    }
    if (boxPoseMsg_.header.frame_id != planningFrame_) {
      std::ostringstream stream;
      stream << "Box PoseStamped input must use frame_id '" << planningFrame_ << "'.";
      errorMessage = stream.str();
      return false;
    }

    snapshot.observation = latestObservation_;
    snapshot.boxPose = poseDataFromMsg(boxPoseMsg_);
    if (!normalizeQuaternion(snapshot.boxPose.orientation, &errorMessage)) {
      errorMessage = "Box pose orientation invalid: " + errorMessage;
      return false;
    }
    return computeGraspPoses(snapshot.boxPose, snapshot.graspPoses, errorMessage);
  }

  bool readPlaceSnapshotUnlocked(PlaceSnapshot& snapshot, std::string& errorMessage) {
    if (!hasObservation_) {
      errorMessage = "No mobile_manipulator_mpc_observation received yet.";
      return false;
    }
    if (!carrySession_.active) {
      errorMessage = "No active carry session. Publish box_pose and wait for HOLDING_OBJECT first.";
      return false;
    }
    if (!hasPlaceBoxPose_) {
      errorMessage = "No place_box_pose PoseStamped received yet.";
      return false;
    }
    if (latestObservation_.input.size() <= 0) {
      errorMessage = "Observation input dimension is empty.";
      return false;
    }
    if (placeBoxPoseMsg_.header.frame_id != planningFrame_) {
      std::ostringstream stream;
      stream << "place_box_pose PoseStamped input must use frame_id '" << planningFrame_ << "'.";
      errorMessage = stream.str();
      return false;
    }

    snapshot.observation = latestObservation_;
    snapshot.placeBoxPose = poseDataFromMsg(placeBoxPoseMsg_);
    if (!normalizeQuaternion(snapshot.placeBoxPose.orientation, &errorMessage)) {
      errorMessage = "place_box_pose orientation invalid: " + errorMessage;
      return false;
    }
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

  Eigen::Vector3d getApproachDirection(size_t armIndex, const PoseData& boxPose, const PoseData& graspPose) const {
    Eigen::Vector3d radial = graspPose.position - boxPose.position;
    if (!isFinite(radial)) {
      return Eigen::Vector3d::Zero();
    }
    if (radial.norm() > 1e-6) {
      return (-radial).normalized();
    }

    const Eigen::Vector3d boxLocalY = boxPose.orientation.toRotationMatrix().col(1);
    return armIndex == 0 ? -boxLocalY : boxLocalY;
  }

  PoseData composeEndEffectorPose(const PoseData& boxPose, size_t armIndex, const ActiveCarrySession& carrySession) const {
    PoseData pose;
    pose.orientation = boxPose.orientation * carrySession.eeOrientationsInBoxFrame[armIndex];
    pose.orientation.normalize();
    pose.position = boxPose.position + boxPose.orientation * carrySession.eeTranslationsInBoxFrame[armIndex];
    return pose;
  }

  bool validateWaypointSet(const std::array<std::vector<PoseData>, 2>& armWaypoints, std::string& errorMessage) const {
    for (size_t armIndex = 0; armIndex < 2; ++armIndex) {
      for (const auto& pose : armWaypoints[armIndex]) {
        if (!isFinite(pose.position) || !isFinite(pose.orientation)) {
          errorMessage = "Generated waypoint contains NaN/Inf values.";
          return false;
        }
        if (pose.position.z() < minTableClearance_ - 1e-9) {
          errorMessage = "Generated waypoint violates min_table_clearance.";
          return false;
        }
      }
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

  bool buildGraspHoldWaypoints(const GraspSnapshot& snapshot, std::array<std::vector<PoseData>, 2>& armWaypoints,
                               ActiveCarrySession& carrySession, std::vector<double>& timeOffsets,
                               std::string& errorMessage) {
    const auto currentPoses = computeCurrentEndEffectorPoses(snapshot.observation);
    const double clearanceBaseZ = std::max(minTableClearance_, snapshot.boxPose.position.z() + minBoxClearance_);

    PoseData holdBoxPose = snapshot.boxPose;
    holdBoxPose.position.z() = std::max(snapshot.boxPose.position.z() + liftDistance_, clearanceBaseZ);

    const Eigen::Quaterniond inverseBoxOrientation = snapshot.boxPose.orientation.conjugate();

    carrySession = ActiveCarrySession{};
    carrySession.active = true;
    carrySession.graspBoxPose = snapshot.boxPose;
    carrySession.holdBoxPose = holdBoxPose;

    for (size_t armIndex = 0; armIndex < 2; ++armIndex) {
      PoseData graspPose = snapshot.graspPoses[armIndex];
      if (!normalizeQuaternion(graspPose.orientation, &errorMessage)) {
        errorMessage = (armIndex == 0 ? "Left grasp pose invalid: " : "Right grasp pose invalid: ") + errorMessage;
        return false;
      }

      const Eigen::Vector3d approachDirection = getApproachDirection(armIndex, snapshot.boxPose, graspPose);
      if (!isFinite(approachDirection) || approachDirection.norm() < 1e-9) {
        errorMessage = armIndex == 0 ? "Left approach direction is invalid." : "Right approach direction is invalid.";
        return false;
      }

      PoseData preGrasp;
      preGrasp.orientation = graspPose.orientation;
      preGrasp.position = graspPose.position - approachDistance_ * approachDirection;

      PoseData via = preGrasp;
      via.position.z() = std::max({currentPoses[armIndex].position.z(), preGrasp.position.z(), clearanceBaseZ}) + viaExtraHeight_;

      PoseData hold = graspPose;
      PoseData retreat = graspPose;
      PoseData lift = composeEndEffectorPose(holdBoxPose, armIndex, carrySession);

      const Eigen::Vector3d translationInBoxFrame =
          inverseBoxOrientation * (graspPose.position - snapshot.boxPose.position);
      Eigen::Quaterniond orientationInBoxFrame = inverseBoxOrientation * graspPose.orientation;
      orientationInBoxFrame.normalize();

      carrySession.eeTranslationsInBoxFrame[armIndex] = translationInBoxFrame;
      carrySession.eeOrientationsInBoxFrame[armIndex] = orientationInBoxFrame;
      lift = composeEndEffectorPose(holdBoxPose, armIndex, carrySession);

      armWaypoints[armIndex] = {currentPoses[armIndex], via, preGrasp, graspPose, hold, retreat, lift};

      if (via.position.z() < clearanceBaseZ - 1e-9 || lift.position.z() < clearanceBaseZ - 1e-9) {
        errorMessage = "Generated via/lift waypoint violates box or table clearance.";
        return false;
      }
    }

    if (!validateWaypointSet(armWaypoints, errorMessage)) {
      return false;
    }

    timeOffsets = {0.0,
                   scaledTime(dtCurrentToVia_),
                   scaledTime(dtCurrentToVia_ + dtViaToPregrasp_),
                   scaledTime(dtCurrentToVia_ + dtViaToPregrasp_ + dtPregraspToGrasp_),
                   scaledTime(dtCurrentToVia_ + dtViaToPregrasp_ + dtPregraspToGrasp_ + graspHoldSec_),
                   scaledTime(dtCurrentToVia_ + dtViaToPregrasp_ + dtPregraspToGrasp_ + graspHoldSec_ +
                              dtGraspToRetreat_),
                   scaledTime(dtCurrentToVia_ + dtViaToPregrasp_ + dtPregraspToGrasp_ + graspHoldSec_ +
                              dtGraspToRetreat_ + dtRetreatToLift_)};
    return true;
  }

  bool buildPlaceWaypoints(const PlaceSnapshot& snapshot, std::array<std::vector<PoseData>, 2>& armWaypoints,
                           std::vector<double>& timeOffsets, std::string& errorMessage) {
    if (!carrySession_.active) {
      errorMessage = "No active carry session.";
      return false;
    }

    const auto currentPoses = computeCurrentEndEffectorPoses(snapshot.observation);
    const double currentHoldHeight = carrySession_.holdBoxPose.position.z();
    const double prePlaceHeight = std::max(currentHoldHeight, snapshot.placeBoxPose.position.z() + prePlaceHeight_);

    PoseData transportBoxPose = snapshot.placeBoxPose;
    transportBoxPose.position.z() = currentHoldHeight;

    PoseData prePlaceBoxPose = snapshot.placeBoxPose;
    prePlaceBoxPose.position.z() = prePlaceHeight;

    for (size_t armIndex = 0; armIndex < 2; ++armIndex) {
      const PoseData transport = composeEndEffectorPose(transportBoxPose, armIndex, carrySession_);
      const PoseData prePlace = composeEndEffectorPose(prePlaceBoxPose, armIndex, carrySession_);
      const PoseData place = composeEndEffectorPose(snapshot.placeBoxPose, armIndex, carrySession_);
      const PoseData release = place;

      const Eigen::Vector3d postReleaseDirection = -getApproachDirection(armIndex, snapshot.placeBoxPose, release);
      if (!isFinite(postReleaseDirection) || postReleaseDirection.norm() < 1e-9) {
        errorMessage = armIndex == 0 ? "Left post-release retreat direction is invalid."
                                     : "Right post-release retreat direction is invalid.";
        return false;
      }

      PoseData postReleaseRetreat;
      postReleaseRetreat.orientation = release.orientation;
      postReleaseRetreat.position = release.position + postReleaseRetreatDistance_ * postReleaseDirection;
      postReleaseRetreat.position.z() =
          std::max({release.position.z() + postReleaseRetreatHeight_,
                    snapshot.placeBoxPose.position.z() + minBoxClearance_, minTableClearance_});

      armWaypoints[armIndex] = {currentPoses[armIndex], transport, prePlace, place, release, postReleaseRetreat};
    }

    if (!validateWaypointSet(armWaypoints, errorMessage)) {
      return false;
    }

    timeOffsets = {0.0,
                   scaledTime(dtLiftToTransport_),
                   scaledTime(dtLiftToTransport_ + dtTransportToPrePlace_),
                   scaledTime(dtLiftToTransport_ + dtTransportToPrePlace_ + dtPrePlaceToPlace_),
                   scaledTime(dtLiftToTransport_ + dtTransportToPrePlace_ + dtPrePlaceToPlace_ + dtPlaceToRelease_),
                   scaledTime(dtLiftToTransport_ + dtTransportToPrePlace_ + dtPrePlaceToPlace_ + dtPlaceToRelease_ +
                              dtReleaseToPostReleaseRetreat_)};
    return true;
  }

  TargetTrajectories buildTargetTrajectories(const SystemObservation& observation,
                                             const std::array<std::vector<PoseData>, 2>& armWaypoints,
                                             const std::vector<double>& timeOffsets) const {
    if (armWaypoints[0].size() != timeOffsets.size() || armWaypoints[1].size() != timeOffsets.size()) {
      throw std::runtime_error("[DualArmGraspWaypointPlanner] Waypoint count does not match time offsets.");
    }

    scalar_array_t timeTrajectory;
    vector_array_t stateTrajectory;
    vector_array_t inputTrajectory;

    timeTrajectory.reserve(timeOffsets.size());
    stateTrajectory.reserve(timeOffsets.size());
    inputTrajectory.reserve(timeOffsets.size());

    const vector_t zeroInput = vector_t::Zero(observation.input.size());

    for (size_t stage = 0; stage < timeOffsets.size(); ++stage) {
      vector_t state(14);
      state.segment<3>(0) = armWaypoints[0][stage].position;
      state.segment<4>(3) = armWaypoints[0][stage].orientation.coeffs();
      state.segment<3>(7) = armWaypoints[1][stage].position;
      state.segment<4>(10) = armWaypoints[1][stage].orientation.coeffs();

      timeTrajectory.push_back(observation.time + timeOffsets[stage]);
      stateTrajectory.push_back(std::move(state));
      inputTrajectory.push_back(zeroInput);
    }

    return {timeTrajectory, stateTrajectory, inputTrajectory};
  }

  bool planAndPublishGrasp(const std::string& triggerSource, std::string& successMessage, std::string& errorMessage) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hasObservation_) {
      updatePlannerStateUnlocked(latestObservation_.time);
    }
    if (plannerState_ != PlannerState::IDLE) {
      std::ostringstream stream;
      stream << "Cannot start grasp from " << triggerSource << " while state is " << stateName(plannerState_) << ".";
      errorMessage = stream.str();
      return false;
    }

    GraspSnapshot snapshot;
    if (!readGraspSnapshotUnlocked(snapshot, errorMessage)) {
      return false;
    }

    std::array<std::vector<PoseData>, 2> armWaypoints;
    ActiveCarrySession nextCarrySession;
    std::vector<double> timeOffsets;
    if (!buildGraspHoldWaypoints(snapshot, armWaypoints, nextCarrySession, timeOffsets, errorMessage)) {
      return false;
    }

    const auto targetTrajectories = buildTargetTrajectories(snapshot.observation, armWaypoints, timeOffsets);
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);

    carrySession_ = nextCarrySession;
    plannerState_ = PlannerState::EXECUTING_GRASP;
    activeTrajectoryEndTime_ = targetTrajectories.timeTrajectory.back();
    hasPlaceBoxPose_ = false;

    std::ostringstream stream;
    stream << "Published grasp-hold trajectory from " << triggerSource << ". Hold box center="
           << formatVector(carrySession_.holdBoxPose.position) << ", left grasp="
           << formatVector(snapshot.graspPoses[0].position) << ", right grasp="
           << formatVector(snapshot.graspPoses[1].position) << ".";
    successMessage = stream.str();
    return true;
  }

  bool planAndPublishPlace(const std::string& triggerSource, std::string& successMessage, std::string& errorMessage) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hasObservation_) {
      updatePlannerStateUnlocked(latestObservation_.time);
    }
    if (plannerState_ != PlannerState::HOLDING_OBJECT) {
      std::ostringstream stream;
      stream << "Cannot start place from " << triggerSource << " while state is " << stateName(plannerState_) << ".";
      errorMessage = stream.str();
      return false;
    }

    PlaceSnapshot snapshot;
    if (!readPlaceSnapshotUnlocked(snapshot, errorMessage)) {
      return false;
    }

    std::array<std::vector<PoseData>, 2> armWaypoints;
    std::vector<double> timeOffsets;
    if (!buildPlaceWaypoints(snapshot, armWaypoints, timeOffsets, errorMessage)) {
      return false;
    }

    const auto targetTrajectories = buildTargetTrajectories(snapshot.observation, armWaypoints, timeOffsets);
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);

    plannerState_ = PlannerState::EXECUTING_PLACE;
    activeTrajectoryEndTime_ = targetTrajectories.timeTrajectory.back();

    std::ostringstream stream;
    stream << "Published place trajectory from " << triggerSource << ". place_box_pose="
           << formatVector(snapshot.placeBoxPose.position) << ".";
    successMessage = stream.str();
    return true;
  }

  void handleObservationMsg(const ocs2_msgs::msg::MpcObservation& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(msg);
    hasObservation_ = true;
    updatePlannerStateUnlocked(latestObservation_.time);
  }

  void handleBoxPoseMsg(const geometry_msgs::msg::PoseStamped& msg) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (hasObservation_) {
        updatePlannerStateUnlocked(latestObservation_.time);
      }
      if (plannerState_ != PlannerState::IDLE) {
        warnIgnoredTrigger("box_pose", PlannerState::IDLE);
        return;
      }
      boxPoseMsg_ = msg;
      hasBoxPose_ = true;
    }

    std::string successMessage;
    std::string errorMessage;
    if (planAndPublishGrasp("box_pose topic", successMessage, errorMessage)) {
      RCLCPP_INFO(node_->get_logger(), "%s", successMessage.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "%s", errorMessage.c_str());
    }
  }

  void handlePlaceBoxPoseMsg(const geometry_msgs::msg::PoseStamped& msg) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (hasObservation_) {
        updatePlannerStateUnlocked(latestObservation_.time);
      }
      if (plannerState_ != PlannerState::HOLDING_OBJECT) {
        warnIgnoredTrigger("place_box_pose", PlannerState::HOLDING_OBJECT);
        return;
      }
      placeBoxPoseMsg_ = msg;
      hasPlaceBoxPose_ = true;
    }

    std::string successMessage;
    std::string errorMessage;
    if (planAndPublishPlace("place_box_pose topic", successMessage, errorMessage)) {
      RCLCPP_INFO(node_->get_logger(), "%s", successMessage.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "%s", errorMessage.c_str());
    }
  }

  void handlePlanRequest(std_srvs::srv::Trigger::Response& response) {
    std::string successMessage;
    std::string errorMessage;
    if (!planAndPublishGrasp("service", successMessage, errorMessage)) {
      response.success = false;
      response.message = errorMessage;
      RCLCPP_WARN(node_->get_logger(), "%s", errorMessage.c_str());
      return;
    }

    response.success = true;
    response.message = successMessage;
    RCLCPP_INFO(node_->get_logger(), "%s", successMessage.c_str());
  }

  rclcpp::Node::SharedPtr node_;
  PinocchioInterface pinocchioInterface_;
  ManipulatorModelInfo modelInfo_;
  MobileManipulatorPinocchioMapping pinocchioMapping_;
  std::string topicPrefix_;

  std::string planningFrame_;
  std::string boxPoseTopic_;
  std::string placeBoxPoseTopic_;
  double boxSizeX_ = 0.1978;
  double boxSizeY_ = 0.2966;
  double boxSizeZ_ = 0.1464;
  double graspEdgeInsetY_ = 0.0;
  double graspEdgeOffsetY_ = 0.0;
  double graspXOffset_ = 0.0;
  double graspZOffset_ = 0.0;
  Eigen::Quaterniond graspFrameOrientation_ = Eigen::Quaterniond::Identity();
  std::array<Eigen::Quaterniond, 2> wristOrientationCompensations_{
      {Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity()}};

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
  double graspHoldSec_ = 2.0;
  double dtGraspToRetreat_ = 0.8;
  double dtRetreatToLift_ = 1.0;
  bool enableTransportStage_ = false;
  bool enablePlaceStage_ = false;
  bool maintainRigidGraspAfterContact_ = true;
  bool transportOffsetIsAbsolute_ = false;
  double dtLiftToTransport_ = 1.0;
  double dtTransportToPrePlace_ = 0.5;
  double dtPrePlaceToPlace_ = 1.0;
  double dtPlaceToRelease_ = 0.5;
  double dtReleaseToPostReleaseRetreat_ = 1.0;
  double dtPostReleaseRetreatToHome_ = 1.0;
  double trajectoryTimeScale_ = 1.0;
  double prePlaceHeight_ = 0.10;
  double postReleaseRetreatDistance_ = 0.15;
  double postReleaseRetreatHeight_ = 0.05;
  Eigen::Vector3d transportOffset_ = Eigen::Vector3d::Zero();

  std::array<pinocchio::FrameIndex, 2> eeFrameIds_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSubscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr boxPoseSubscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr placeBoxPoseSubscriber_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr planService_;

  std::mutex mutex_;
  PlannerState plannerState_ = PlannerState::IDLE;
  double activeTrajectoryEndTime_ = -1.0;
  SystemObservation latestObservation_;
  bool hasObservation_ = false;
  geometry_msgs::msg::PoseStamped boxPoseMsg_;
  bool hasBoxPose_ = false;
  geometry_msgs::msg::PoseStamped placeBoxPoseMsg_;
  bool hasPlaceBoxPose_ = false;
  ActiveCarrySession carrySession_;
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
