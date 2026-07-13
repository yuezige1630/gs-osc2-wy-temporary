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
#include <ocs2_mobile_manipulator_ros/CarryHomePoseHelpers.h>
#include <ocs2_mobile_manipulator_ros/GraspLiftPoseHelpers.h>
#include <ocs2_mobile_manipulator_ros/PostReleaseInitialReturnHelpers.h>
#include <ocs2_msgs/srv/evaluate_box_pose.hpp>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_msgs/msg/mpc_observation.hpp>

#include "rclcpp/rclcpp.hpp"

namespace ocs2 {
namespace mobile_manipulator {

class DualArmGraspWaypointPlanner final {
 public:
  DualArmGraspWaypointPlanner(const rclcpp::Node::SharedPtr& node, const std::string& topicPrefix,
                              PinocchioInterface pinocchioInterface, ManipulatorModelInfo modelInfo,
                              vector_t stablePosture)
      : node_(node),
        pinocchioInterface_(std::move(pinocchioInterface)),
        modelInfo_(std::move(modelInfo)),
        pinocchioMapping_(modelInfo_),
        topicPrefix_(topicPrefix),
        stablePosture_(std::move(stablePosture)) {
    if (modelInfo_.eeFrames.size() != 2) {
      throw std::runtime_error("[DualArmGraspWaypointPlanner] Exactly two ee frames are required.");
    }

    const auto& model = pinocchioInterface_.getModel();
    eeFrameIds_[0] = model.getBodyId(modelInfo_.eeFrames[0]);
    eeFrameIds_[1] = model.getBodyId(modelInfo_.eeFrames[1]);

    if (stablePosture_.size() != model.nq || stablePosture_.size() != 19 || !stablePosture_.array().isFinite().all()) {
      throw std::runtime_error(
          "[DualArmGraspWaypointPlanner] initialState.arm must contain exactly 19 finite joint values.");
    }
    for (int i = 0; i < stablePosture_.size(); ++i) {
      const double lowerLimit = model.lowerPositionLimit(i);
      const double upperLimit = model.upperPositionLimit(i);
      if ((std::isfinite(lowerLimit) && stablePosture_(i) < lowerLimit + 1e-3) ||
          (std::isfinite(upperLimit) && stablePosture_(i) > upperLimit - 1e-3)) {
        throw std::runtime_error(
            "[DualArmGraspWaypointPlanner] initialState.arm contains a joint value too close to a URDF limit.");
      }
    }
    stablePostureEndEffectorPoses_ = computeEndEffectorPosesFromState(stablePosture_);

    planningFrame_ = node_->declare_parameter<std::string>("planning_frame", modelInfo_.baseFrame);
    boxPoseTopic_ = node_->declare_parameter<std::string>("box_pose_topic", "box_pose");
    placeBoxPoseTopic_ = node_->declare_parameter<std::string>("place_box_pose_topic", "place_box_pose");
    boxSizeX_ = node_->declare_parameter<double>("box_size_x", 0.45);
    boxSizeY_ = node_->declare_parameter<double>("box_size_y", 0.72);
    boxSizeZ_ = node_->declare_parameter<double>("box_size_z", 0.12);
    graspEdgeInsetY_ = node_->declare_parameter<double>("grasp_edge_inset_y", 0.01);
    graspXOffset_ = node_->declare_parameter<double>("grasp_x_offset", 0.0);
    graspZOffset_ = node_->declare_parameter<double>("grasp_z_offset", -0.01);
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

    approachDistance_ = node_->declare_parameter<double>("approach_distance", 0.20);
    retreatDistance_ = node_->declare_parameter<double>("retreat_distance", 0.15);
    liftDistance_ = node_->declare_parameter<double>("lift_distance", 0.15);
    viaExtraHeight_ = node_->declare_parameter<double>("via_extra_height", 0.10);
    minBoxClearance_ = node_->declare_parameter<double>("min_box_clearance", 0.05);
    minTableClearance_ = node_->declare_parameter<double>("min_table_clearance", 0.05);
    tableTopZ_ = node_->declare_parameter<double>("table_top_z", 0.80);
    tableClearance_ = node_->declare_parameter<double>("table_clearance", 0.12);
    approachBoxClearance_ = node_->declare_parameter<double>("approach_box_clearance", 0.10);
    minGraspSeparation_ = node_->declare_parameter<double>("min_grasp_separation", 0.05);

    dtCurrentToVia_ = node_->declare_parameter<double>("dt_current_to_via", 1.0);
    dtViaToPregrasp_ = node_->declare_parameter<double>("dt_via_to_pregrasp", 1.0);
    dtPregraspToGrasp_ = node_->declare_parameter<double>("dt_pregrasp_to_grasp", 0.8);
    graspHoldSec_ = node_->declare_parameter<double>("grasp_hold_sec", 0.5);
    dtGraspToRetreat_ = node_->declare_parameter<double>("dt_grasp_to_retreat", 0.8);
    dtRetreatToLift_ = node_->declare_parameter<double>("dt_retreat_to_lift", 1.0);
    dtLiftToCarryUpright_ = node_->declare_parameter<double>("dt_lift_to_carry_upright", 0.8);
    dtCarryUprightToHome_ = node_->declare_parameter<double>("dt_carry_upright_to_home", 1.0);
    carryHomeAfterGrasp_ = node_->declare_parameter<bool>("carry_home_after_grasp", true);
    enableTransportStage_ = node_->declare_parameter<bool>("enable_transport_stage", false);
    enablePlaceStage_ = node_->declare_parameter<bool>("enable_place_stage", false);
    maintainRigidGraspAfterContact_ = node_->declare_parameter<bool>("maintain_rigid_grasp_after_contact", true);
    transportOffsetIsAbsolute_ = node_->declare_parameter<bool>("transport_offset_is_absolute", false);
    dtLiftToTransport_ = node_->declare_parameter<double>("dt_lift_to_transport", 1.0);
    dtTransportToPrePlace_ = node_->declare_parameter<double>("dt_transport_to_pre_place", 0.5);
    dtPrePlaceToPlace_ = node_->declare_parameter<double>("dt_pre_place_to_place", 1.0);
    dtPlaceToRelease_ = node_->declare_parameter<double>("dt_place_to_release", 0.5);
    dtReleaseToPostReleaseRetreat_ = node_->declare_parameter<double>("dt_release_to_post_release_retreat", 1.0);
    dtPostReleaseRetreatToInitial_ = node_->declare_parameter<double>("dt_post_release_retreat_to_initial", 1.0);
    stablePostureHoldSec_ = node_->declare_parameter<double>("stable_posture_hold_sec", 1.0);
    trajectoryTimeScale_ = node_->declare_parameter<double>("trajectory_time_scale", 0.5);
    carryHomeBoxX_ = node_->declare_parameter<double>("carry_home_box_x", 0.6);
    carryHomeBoxY_ = node_->declare_parameter<double>("carry_home_box_y", 0.0);
    carryHomeBoxZ_ = node_->declare_parameter<double>("carry_home_box_z", 1.0);
    carryHomeFrontClearance_ = node_->declare_parameter<double>("carry_home_front_clearance", 0.05);
    carryHomeTableClearance_ = node_->declare_parameter<double>("carry_home_table_clearance", minTableClearance_);
    prePlaceHeight_ = node_->declare_parameter<double>("pre_place_height", 0.10);
    postReleaseRetreatDistance_ = node_->declare_parameter<double>("post_release_retreat_distance", 0.15);
    postReleaseRetreatHeight_ = node_->declare_parameter<double>("post_release_retreat_height", 0.05);
    transportOffset_.x() = node_->declare_parameter<double>("transport_offset_x", 0.0);
    transportOffset_.y() = node_->declare_parameter<double>("transport_offset_y", 0.0);
    transportOffset_.z() = node_->declare_parameter<double>("transport_offset_z", 0.0);
    dryRunMode_ = node_->declare_parameter<bool>("dry_run_mode", false);

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
    if (!std::isfinite(stablePostureHoldSec_) || stablePostureHoldSec_ <= 0.0) {
      throw std::runtime_error("[DualArmGraspWaypointPlanner] stable_posture_hold_sec must be finite and positive.");
    }
    if (!std::isfinite(approachDistance_) || approachDistance_ <= 0.0 || !std::isfinite(tableTopZ_) ||
        !std::isfinite(tableClearance_) || tableClearance_ < 0.0 || !std::isfinite(approachBoxClearance_) ||
        approachBoxClearance_ < 0.0) {
      throw std::runtime_error(
          "[DualArmGraspWaypointPlanner] approach distance and obstacle clearances must be finite and valid.");
    }
    if (!std::isfinite(dtLiftToCarryUpright_) || dtLiftToCarryUpright_ <= 0.0 ||
        !std::isfinite(dtCarryUprightToHome_) || dtCarryUprightToHome_ <= 0.0) {
      throw std::runtime_error("[DualArmGraspWaypointPlanner] carry-home timing parameters must be finite and positive.");
    }
    if (!std::isfinite(carryHomeBoxX_) || !std::isfinite(carryHomeBoxY_) || !std::isfinite(carryHomeBoxZ_)) {
      throw std::runtime_error("[DualArmGraspWaypointPlanner] carry_home_box_* parameters must be finite.");
    }
    if (!std::isfinite(carryHomeFrontClearance_) || carryHomeFrontClearance_ < 0.0 ||
        !std::isfinite(carryHomeTableClearance_) || carryHomeTableClearance_ < 0.0) {
      throw std::runtime_error("[DualArmGraspWaypointPlanner] carry_home clearances must be finite and non-negative.");
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
    if (!carryHomeAfterGrasp_) {
      RCLCPP_INFO(node_->get_logger(),
                  "carry_home_after_grasp=false: box_pose trajectories will not append upright/home waypoints.");
    }
    if (dryRunMode_) {
      RCLCPP_INFO(node_->get_logger(),
                  "Dry-run mode enabled: box_pose topic updates will be cached, and trajectory evaluation will not "
                  "publish motions.");
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
    continueReturnToInitialService_ = node_->create_service<std_srvs::srv::Trigger>(
        "continue_return_to_initial_pose",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          handleContinueReturnToInitialRequest(*response);
        });
    evaluateService_ = node_->create_service<ocs2_msgs::srv::EvaluateBoxPose>(
        "evaluate_box_pose",
        [this](const std::shared_ptr<ocs2_msgs::srv::EvaluateBoxPose::Request>& request,
               std::shared_ptr<ocs2_msgs::srv::EvaluateBoxPose::Response> response) {
          handleEvaluateBoxPose(*request, *response);
        });

    RCLCPP_INFO(node_->get_logger(),
                "Dual-arm two-stage planner ready. box_pose_topic=%s, place_box_pose_topic=%s, publishing to "
                "%s_mpc_target.",
                boxPoseTopic_.c_str(), placeBoxPoseTopic_.c_str(), topicPrefix_.c_str());
  }

  void spin() { rclcpp::spin(node_); }

 private:
  enum class PlannerState {
    IDLE,
    EXECUTING_GRASP,
    HOLDING_OBJECT,
    EXECUTING_PLACE,
    WAITING_FOR_INITIAL_RETURN,
    EXECUTING_INITIAL_RETURN
  };

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
    PoseData carryUprightBoxPose;
    PoseData carryHomeBoxPose;
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
      case PlannerState::WAITING_FOR_INITIAL_RETURN:
        return "WAITING_FOR_INITIAL_RETURN";
      case PlannerState::EXECUTING_INITIAL_RETURN:
        return "EXECUTING_INITIAL_RETURN";
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

  bool buildGraspSnapshotFromMsg(const geometry_msgs::msg::PoseStamped& msg, GraspSnapshot& snapshot,
                                 std::string& errorMessage) const {
    if (!hasObservation_) {
      errorMessage = "No mobile_manipulator_mpc_observation received yet.";
      return false;
    }
    if (msg.header.frame_id != planningFrame_) {
      std::ostringstream stream;
      stream << "Box PoseStamped input must use frame_id '" << planningFrame_ << "'.";
      errorMessage = stream.str();
      return false;
    }

    snapshot.observation = latestObservation_;
    snapshot.boxPose = poseDataFromMsg(msg);
    if (!normalizeQuaternion(snapshot.boxPose.orientation, &errorMessage)) {
      errorMessage = "Box pose orientation invalid: " + errorMessage;
      return false;
    }
    return computeGraspPoses(snapshot.boxPose, snapshot.graspPoses, errorMessage);
  }

  bool buildGraspPlan(const GraspSnapshot& snapshot, TargetTrajectories& targetTrajectories,
                      ActiveCarrySession& nextCarrySession, std::string& errorMessage) {
    std::array<std::vector<PoseData>, 2> armWaypoints;
    std::vector<double> timeOffsets;
    if (!buildGraspHoldWaypoints(snapshot, armWaypoints, nextCarrySession, timeOffsets, errorMessage)) {
      return false;
    }

    targetTrajectories = buildTargetTrajectories(snapshot.observation, armWaypoints, timeOffsets);
    return true;
  }

  std::string formatGraspSuccessMessage(const std::string& leadIn, const std::string& triggerSource,
                                        const GraspSnapshot& snapshot, const ActiveCarrySession& carrySession) const {
    std::ostringstream stream;
    stream << leadIn << " " << triggerSource << ". ";
    if (carryHomeAfterGrasp_) {
      stream << "Lift box center=" << formatVector(carrySession.holdBoxPose.position)
             << ", carry-upright center=" << formatVector(carrySession.carryUprightBoxPose.position)
             << ", carry-home center=" << formatVector(carrySession.carryHomeBoxPose.position);
    } else {
      stream << "Holding at box center=" << formatVector(carrySession.holdBoxPose.position);
    }
    stream << ", left grasp=" << formatVector(snapshot.graspPoses[0].position) << ", right grasp="
           << formatVector(snapshot.graspPoses[1].position) << ".";
    return stream.str();
  }

  static std::string formatVector(const Eigen::Vector3d& vector) {
    std::ostringstream stream;
    stream << "[" << vector.x() << ", " << vector.y() << ", " << vector.z() << "]";
    return stream.str();
  }

  static std::string formatJointVector(const vector_t& vector) {
    std::ostringstream stream;
    stream << "[";
    for (int i = 0; i < vector.size(); ++i) {
      if (i != 0) {
        stream << ", ";
      }
      stream << vector(i);
    }
    stream << "]";
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
        Eigen::Vector3d(graspXOffset_, graspEdgeOffsetY_, contactGraspLocalZ(graspZOffset_)),
        Eigen::Vector3d(graspXOffset_, -graspEdgeOffsetY_, contactGraspLocalZ(graspZOffset_)),
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
    hasPostReleaseRetreatEndEffectorPoses_ = false;
    postReleaseRetreatEndEffectorPoses_ = {};
  }

  void cacheStablePosturePoseIfNeededUnlocked() {
    if (hasStablePostureEndEffectorPoses_) {
      return;
    }
    hasStablePostureEndEffectorPoses_ = true;
    RCLCPP_INFO(node_->get_logger(), "Cached fixed pre-transport end-effector poses for the return stage.");
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
      plannerState_ = PlannerState::WAITING_FOR_INITIAL_RETURN;
      hasPlaceBoxPose_ = false;
      RCLCPP_INFO(node_->get_logger(),
                  "Post-release retreat complete. Waiting for continue_return_to_initial_pose service.");
      return;
    }

    if (plannerState_ == PlannerState::EXECUTING_INITIAL_RETURN &&
        currentTime + kExecutionCompletionTolerance >= activeTrajectoryEndTime_) {
      plannerState_ = PlannerState::IDLE;
      clearCarrySessionUnlocked();
      RCLCPP_INFO(node_->get_logger(), "Initial return complete. Transitioned to IDLE.");
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
    return buildGraspSnapshotFromMsg(boxPoseMsg_, snapshot, errorMessage);
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

  std::array<PoseData, 2> computeEndEffectorPosesFromState(const vector_t& state) {
    std::array<PoseData, 2> currentPoses;

    vector_t q = pinocchioMapping_.getPinocchioJointPosition(state);
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

  std::array<PoseData, 2> computeCurrentEndEffectorPoses(const SystemObservation& observation) {
    return computeEndEffectorPosesFromState(observation.state);
  }

  Eigen::Vector3d getApproachDirection(size_t armIndex, const PoseData& boxPose, const PoseData& /*graspPose*/) const {
    // Approach strictly along the box-local y axis.  This keeps the handboards
    // outside the box while they descend, and reserves contact for the final
    // short translation into the side slots.
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
    const double safeApproachHeight = computeSafeApproachHeight(
        snapshot.boxPose.position.z(), boxSizeZ_, tableTopZ_, approachBoxClearance_, tableClearance_);
    const double safeTransitHeight = safeApproachHeight + viaExtraHeight_;

    PoseData holdBoxPose = snapshot.boxPose;
    PoseData carryUprightBoxPose = snapshot.boxPose;
    PoseData carryHomeBoxPose = snapshot.boxPose;

    PoseData liftBoxPose = snapshot.boxPose;
    liftBoxPose.position.z() =
        computeLiftBoxCenterZ(snapshot.boxPose.position.z(), liftDistance_, minBoxClearance_, minTableClearance_);
    holdBoxPose = liftBoxPose;

    if (carryHomeAfterGrasp_) {
      carryUprightBoxPose = holdBoxPose;
      carryUprightBoxPose.orientation = baseLinkAlignedBoxOrientation();
      carryHomeBoxPose.position = Eigen::Vector3d(carryHomeBoxX_, carryHomeBoxY_, carryHomeBoxZ_);
      carryHomeBoxPose.orientation = baseLinkAlignedBoxOrientation();
      if (!isCarryHomeBoxPoseSafe(carryHomeBoxPose.position, boxSizeX_, boxSizeZ_, carryHomeFrontClearance_,
                                  carryHomeTableClearance_, &errorMessage)) {
        errorMessage = "Carry-home box pose invalid: " + errorMessage;
        return false;
      }
    }

    const Eigen::Quaterniond inverseBoxOrientation = snapshot.boxPose.orientation.conjugate();

    carrySession = ActiveCarrySession{};
    carrySession.active = true;
    carrySession.graspBoxPose = snapshot.boxPose;
    carrySession.holdBoxPose = holdBoxPose;
    carrySession.carryUprightBoxPose = carryUprightBoxPose;
    carrySession.carryHomeBoxPose = carryHomeBoxPose;

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

      // Split the transit into a vertical lift and a horizontal move.  The
      // previous single diagonal segment could sweep an arm/handboard through
      // the table or the box even when its endpoint was above the clearance.
      PoseData clearanceLift = currentPoses[armIndex];
      clearanceLift.position.z() = std::max(currentPoses[armIndex].position.z(), safeTransitHeight);

      PoseData via = preGrasp;
      via.position.z() = std::max({currentPoses[armIndex].position.z(), preGrasp.position.z(), safeTransitHeight});

      // Expand both arms outside the box footprint before descending.  This
      // keeps the approach from crossing through the box center when the
      // pre-grasp pose is reached from the high-clearance transit plane.
      const double yExpansion = 0.5 * boxSizeY_ + approachBoxClearance_;
      PoseData yExpanded = via;
      yExpanded.position.y() = snapshot.boxPose.position.y() + (armIndex == 0 ? yExpansion : -yExpansion);

      PoseData hold = graspPose;
      PoseData lift;

      // Use the original local offsets as the translation in box frame.
      // This avoids the x-alignment hack corrupting the y/z components when the box is rotated,
      // which would cause lateral movement (and hand flipping) during the lift.
      const Eigen::Vector3d translationInBoxFrame(
          graspXOffset_, armIndex == 0 ? graspEdgeOffsetY_ : -graspEdgeOffsetY_, contactGraspLocalZ(graspZOffset_));
      Eigen::Quaterniond orientationInBoxFrame = inverseBoxOrientation * graspPose.orientation;
      orientationInBoxFrame.normalize();

      carrySession.eeTranslationsInBoxFrame[armIndex] = translationInBoxFrame;
      carrySession.eeOrientationsInBoxFrame[armIndex] = orientationInBoxFrame;

      // Lift: same x/y/orientation as grasp, only z increases (pure vertical motion).
      lift = graspPose;
      lift.position.z() = holdBoxPose.position.z() + contactGraspLocalZ(graspZOffset_);

      // Break the lift phase into multiple sub-steps so MPC tracks small increments.
      const int numLiftSubSteps = 3;
      std::vector<PoseData> liftSubSteps(numLiftSubSteps);
      for (int s = 0; s < numLiftSubSteps; ++s) {
        const double frac = static_cast<double>(s + 1) / static_cast<double>(numLiftSubSteps + 1);
        liftSubSteps[s] = graspPose;
        liftSubSteps[s].position.z() =
            graspPose.position.z() + frac * (lift.position.z() - graspPose.position.z());
      }

      RCLCPP_INFO(node_->get_logger(),
                  "Arm %zu: grasp=[%.3f, %.3f, %.3f] hold=[%.3f, %.3f, %.3f] lift=[%.3f, %.3f, %.3f]",
                  armIndex,
                  graspPose.position.x(), graspPose.position.y(), graspPose.position.z(),
                  hold.position.x(), hold.position.y(), hold.position.z(),
                  lift.position.x(), lift.position.y(), lift.position.z());

      if (carryHomeAfterGrasp_) {
        const PoseData carryUpright = composeEndEffectorPose(carrySession.carryUprightBoxPose, armIndex, carrySession);
        const PoseData carryHome = composeEndEffectorPose(carryHomeBoxPose, armIndex, carrySession);
        armWaypoints[armIndex] = {currentPoses[armIndex], clearanceLift, yExpanded, via, preGrasp, graspPose, hold,
                                  liftSubSteps[0], liftSubSteps[1], liftSubSteps[2],
                                  lift, carryUpright, carryHome};
      } else {
        armWaypoints[armIndex] = {currentPoses[armIndex], clearanceLift, yExpanded, via, preGrasp, graspPose, hold,
                                  liftSubSteps[0], liftSubSteps[1], liftSubSteps[2], lift};
      }

      if (via.position.z() < safeTransitHeight - 1e-9 || yExpanded.position.z() < safeTransitHeight - 1e-9 ||
          clearanceLift.position.z() < safeTransitHeight - 1e-9 ||
          lift.position.z() < minTableClearance_ - 1e-9) {
        errorMessage = "Generated via/lift waypoint violates box or table clearance.";
        return false;
      }
    }

    if (!validateWaypointSet(armWaypoints, errorMessage)) {
      return false;
    }

    const double dtCurrentToClearance = 0.5 * dtCurrentToVia_;
    const double dtCurrentToYExpanded = 0.75 * dtCurrentToVia_;
    const double contactTime = dtCurrentToVia_ + dtViaToPregrasp_ + dtPregraspToGrasp_;
    const double holdEndTime = contactTime + graspHoldSec_;
    const double liftEndTime = holdEndTime + dtRetreatToLift_;
    const int numLiftSubSteps = 3;
    const double liftStepDuration = dtRetreatToLift_ / static_cast<double>(numLiftSubSteps + 1);
    if (carryHomeAfterGrasp_) {
      timeOffsets = {0.0,
                     scaledTime(dtCurrentToClearance),
                     scaledTime(dtCurrentToYExpanded),
                     scaledTime(dtCurrentToVia_),
                     scaledTime(dtCurrentToVia_ + dtViaToPregrasp_),
                     scaledTime(contactTime),
                     scaledTime(holdEndTime),
                     scaledTime(holdEndTime + liftStepDuration),
                     scaledTime(holdEndTime + 2.0 * liftStepDuration),
                     scaledTime(holdEndTime + 3.0 * liftStepDuration),
                     scaledTime(liftEndTime),
                     scaledTime(liftEndTime + dtLiftToCarryUpright_),
                     scaledTime(liftEndTime + dtLiftToCarryUpright_ + dtCarryUprightToHome_)};
    } else {
      timeOffsets = {0.0,
                     scaledTime(dtCurrentToClearance),
                     scaledTime(dtCurrentToYExpanded),
                     scaledTime(dtCurrentToVia_),
                     scaledTime(dtCurrentToVia_ + dtViaToPregrasp_),
                     scaledTime(contactTime),
                     scaledTime(holdEndTime),
                     scaledTime(holdEndTime + liftStepDuration),
                     scaledTime(holdEndTime + 2.0 * liftStepDuration),
                     scaledTime(holdEndTime + 3.0 * liftStepDuration),
                     scaledTime(liftEndTime)};
    }
    return true;
  }

  bool buildPlaceWaypoints(const PlaceSnapshot& snapshot, std::array<std::vector<PoseData>, 2>& armWaypoints,
                           std::vector<double>& timeOffsets, std::string& errorMessage) {
    if (!carrySession_.active) {
      errorMessage = "No active carry session.";
      return false;
    }

    const auto currentPoses = computeCurrentEndEffectorPoses(snapshot.observation);
    const double currentHoldHeight = carrySession_.carryHomeBoxPose.position.z();
    const double prePlaceHeight = std::max(currentHoldHeight, snapshot.placeBoxPose.position.z() + prePlaceHeight_);

    PoseData transportBoxPose = snapshot.placeBoxPose;
    transportBoxPose.position.z() = currentHoldHeight;

    PoseData prePlaceBoxPose = snapshot.placeBoxPose;
    prePlaceBoxPose.position.z() = prePlaceHeight;

    std::array<PoseData, 2> postReleaseRetreatPoses;
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

      postReleaseRetreatPoses[armIndex] = postReleaseRetreat;
      armWaypoints[armIndex] = {currentPoses[armIndex], transport, prePlace, place, release, postReleaseRetreat};
    }

    if (!validateWaypointSet(armWaypoints, errorMessage)) {
      return false;
    }

    postReleaseRetreatEndEffectorPoses_ = postReleaseRetreatPoses;
    hasPostReleaseRetreatEndEffectorPoses_ = true;

    timeOffsets = {0.0,
                   scaledTime(dtLiftToTransport_),
                   scaledTime(dtLiftToTransport_ + dtTransportToPrePlace_),
                   scaledTime(dtLiftToTransport_ + dtTransportToPrePlace_ + dtPrePlaceToPlace_),
                   scaledTime(dtLiftToTransport_ + dtTransportToPrePlace_ + dtPrePlaceToPlace_ + dtPlaceToRelease_),
                   scaledTime(dtLiftToTransport_ + dtTransportToPrePlace_ + dtPrePlaceToPlace_ + dtPlaceToRelease_ +
                              dtReleaseToPostReleaseRetreat_)};
    return true;
  }

  bool buildInitialReturnWaypoints(std::array<std::vector<PoseData>, 2>& armWaypoints, std::vector<double>& timeOffsets,
                                   std::string& errorMessage) {
    if (!hasStablePostureEndEffectorPoses_) {
      errorMessage = "No fixed stable-posture end-effector pose cached yet.";
      return false;
    }
    if (!hasPostReleaseRetreatEndEffectorPoses_) {
      errorMessage = "No post-release retreat pose cached yet.";
      return false;
    }

    const auto currentPoses = computeCurrentEndEffectorPoses(latestObservation_);
    const auto trajectory = buildPostReleaseInitialReturnTrajectory(
        currentPoses, postReleaseRetreatEndEffectorPoses_, stablePostureEndEffectorPoses_,
        dtPostReleaseRetreatToInitial_, trajectoryTimeScale_, stablePostureHoldSec_);

    armWaypoints = trajectory.armWaypoints;
    timeOffsets = trajectory.timeOffsets;
    return validateWaypointSet(armWaypoints, errorMessage);
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

    ActiveCarrySession nextCarrySession;
    TargetTrajectories targetTrajectories;
    if (!buildGraspPlan(snapshot, targetTrajectories, nextCarrySession, errorMessage)) {
      return false;
    }

    if (!dryRunMode_) {
      targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
    }

    if (!dryRunMode_) {
      carrySession_ = nextCarrySession;
      plannerState_ = PlannerState::EXECUTING_GRASP;
      activeTrajectoryEndTime_ = targetTrajectories.timeTrajectory.back();
      hasPlaceBoxPose_ = false;
      hasPostReleaseRetreatEndEffectorPoses_ = false;
      RCLCPP_INFO(node_->get_logger(),
                  "Published grasp target: start=%.6f end=%.6f duration=%.3f s.",
                  targetTrajectories.timeTrajectory.front(), targetTrajectories.timeTrajectory.back(),
                  targetTrajectories.timeTrajectory.back() - targetTrajectories.timeTrajectory.front());
    }

    successMessage = formatGraspSuccessMessage("Published grasp-hold trajectory from", triggerSource, snapshot,
                                               nextCarrySession);
    return true;
  }

  bool evaluateBoxPoseRequest(const geometry_msgs::msg::PoseStamped& msg, std::string& successMessage,
                              std::string& errorMessage) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hasObservation_) {
      updatePlannerStateUnlocked(latestObservation_.time);
    }
    if (!dryRunMode_ && plannerState_ != PlannerState::IDLE) {
      std::ostringstream stream;
      stream << "Cannot evaluate box pose while state is " << stateName(plannerState_)
             << ". Stop the active trajectory or enable dry_run_mode.";
      errorMessage = stream.str();
      return false;
    }
    if (latestObservation_.input.size() <= 0) {
      errorMessage = "Observation input dimension is empty.";
      return false;
    }

    GraspSnapshot snapshot;
    if (!buildGraspSnapshotFromMsg(msg, snapshot, errorMessage)) {
      return false;
    }

    TargetTrajectories targetTrajectories;
    ActiveCarrySession nextCarrySession;
    if (!buildGraspPlan(snapshot, targetTrajectories, nextCarrySession, errorMessage)) {
      return false;
    }

    std::ostringstream stream;
    stream << "Grasp evaluation succeeded for " << msg.header.frame_id << ". Hold box center="
           << formatVector(nextCarrySession.holdBoxPose.position) << ", left grasp="
           << formatVector(snapshot.graspPoses[0].position) << ", right grasp="
           << formatVector(snapshot.graspPoses[1].position) << ", trajectory_end_time="
           << targetTrajectories.timeTrajectory.back() << ".";
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
           << formatVector(snapshot.placeBoxPose.position) << ", retreat-left="
           << formatVector(postReleaseRetreatEndEffectorPoses_[0].position) << ", retreat-right="
           << formatVector(postReleaseRetreatEndEffectorPoses_[1].position);
    stream << ".";
    successMessage = stream.str();
    return true;
  }

  bool planAndPublishContinueReturnToInitial(const std::string& triggerSource, std::string& successMessage,
                                             std::string& errorMessage) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hasObservation_) {
      updatePlannerStateUnlocked(latestObservation_.time);
    }
    if (plannerState_ != PlannerState::WAITING_FOR_INITIAL_RETURN) {
      std::ostringstream stream;
      stream << "Cannot continue return from " << triggerSource << " while state is " << stateName(plannerState_)
             << ".";
      errorMessage = stream.str();
      return false;
    }
    if (!hasStablePostureEndEffectorPoses_) {
      errorMessage = "No fixed stable-posture end-effector pose cached yet.";
      return false;
    }
    if (!hasPostReleaseRetreatEndEffectorPoses_) {
      errorMessage = "No post-release retreat pose cached yet.";
      return false;
    }
    if (latestObservation_.input.size() <= 0) {
      errorMessage = "Observation input dimension is empty.";
      return false;
    }

    std::array<std::vector<PoseData>, 2> armWaypoints;
    std::vector<double> timeOffsets;
    if (!buildInitialReturnWaypoints(armWaypoints, timeOffsets, errorMessage)) {
      return false;
    }

    const auto targetTrajectories = buildTargetTrajectories(latestObservation_, armWaypoints, timeOffsets);
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);

    plannerState_ = PlannerState::EXECUTING_INITIAL_RETURN;
    activeTrajectoryEndTime_ = targetTrajectories.timeTrajectory.back();

    std::ostringstream stream;
    stream << "Published continue-return trajectory from " << triggerSource << ". retreat-left="
           << formatVector(postReleaseRetreatEndEffectorPoses_[0].position) << ", retreat-right="
           << formatVector(postReleaseRetreatEndEffectorPoses_[1].position) << ", return-stable-posture left="
           << formatVector(stablePostureEndEffectorPoses_[0].position) << ", return-stable-posture right="
           << formatVector(stablePostureEndEffectorPoses_[1].position)
           << ", joints=" << formatJointVector(stablePosture_) << ".";
    successMessage = stream.str();
    return true;
  }

  void handleContinueReturnToInitialRequest(std_srvs::srv::Trigger::Response& response) {
    std::string successMessage;
    std::string errorMessage;
    if (!planAndPublishContinueReturnToInitial("service", successMessage, errorMessage)) {
      response.success = false;
      response.message = errorMessage;
      RCLCPP_WARN(node_->get_logger(), "%s", errorMessage.c_str());
      return;
    }

    response.success = true;
    response.message = successMessage;
    RCLCPP_INFO(node_->get_logger(), "%s", successMessage.c_str());
  }

  void handleObservationMsg(const ocs2_msgs::msg::MpcObservation& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(msg);
    hasObservation_ = true;
    cacheStablePosturePoseIfNeededUnlocked();
    updatePlannerStateUnlocked(latestObservation_.time);
  }

  void handleBoxPoseMsg(const geometry_msgs::msg::PoseStamped& msg) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (hasObservation_) {
        updatePlannerStateUnlocked(latestObservation_.time);
      }
      if (dryRunMode_) {
        boxPoseMsg_ = msg;
        hasBoxPose_ = true;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "Cached box_pose in dry-run mode: frame_id=%s, position=[%.4f, %.4f, %.4f].",
                             msg.header.frame_id.c_str(), msg.pose.position.x, msg.pose.position.y,
                             msg.pose.position.z);
        return;
      }
      // Allow retrying after the timed grasp trajectory finished. Otherwise a
      // failed tracking attempt leaves the planner in HOLDING_OBJECT and all
      // subsequent box_pose messages are discarded with the old target time.
      if (plannerState_ == PlannerState::HOLDING_OBJECT) {
        RCLCPP_WARN(node_->get_logger(),
                    "Replanning grasp from the latest observation after the previous grasp trajectory finished.");
        clearCarrySessionUnlocked();
        plannerState_ = PlannerState::IDLE;
        activeTrajectoryEndTime_ = -1.0;
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
    if (dryRunMode_) {
      if (hasObservation_) {
        updatePlannerStateUnlocked(latestObservation_.time);
      }
      if (!hasBoxPose_) {
        response.success = false;
        response.message = "No cached box pose available in dry_run_mode.";
        RCLCPP_WARN(node_->get_logger(), "%s", response.message.c_str());
        return;
      }

      GraspSnapshot snapshot;
      if (!readGraspSnapshotUnlocked(snapshot, errorMessage)) {
        response.success = false;
        response.message = errorMessage;
        RCLCPP_WARN(node_->get_logger(), "%s", errorMessage.c_str());
        return;
      }

      TargetTrajectories targetTrajectories;
      ActiveCarrySession nextCarrySession;
      if (!buildGraspPlan(snapshot, targetTrajectories, nextCarrySession, errorMessage)) {
        response.success = false;
        response.message = errorMessage;
        RCLCPP_WARN(node_->get_logger(), "%s", errorMessage.c_str());
        return;
      }

      successMessage = formatGraspSuccessMessage("Grasp evaluation succeeded for", "service", snapshot,
                                                 nextCarrySession);
      response.success = true;
      response.message = successMessage;
      RCLCPP_INFO(node_->get_logger(), "%s", successMessage.c_str());
      return;
    }

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

  void handleEvaluateBoxPose(const ocs2_msgs::srv::EvaluateBoxPose::Request& request,
                             ocs2_msgs::srv::EvaluateBoxPose::Response& response) {
    std::string successMessage;
    std::string errorMessage;
    if (!evaluateBoxPoseRequest(request.box_pose, successMessage, errorMessage)) {
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
  vector_t stablePosture_;

  std::string planningFrame_;
  std::string boxPoseTopic_;
  std::string placeBoxPoseTopic_;
  double boxSizeX_ = 0.45;
  double boxSizeY_ = 0.72;
  double boxSizeZ_ = 0.12;
  double graspEdgeInsetY_ = 0.01;
  double graspEdgeOffsetY_ = 0.0;
  double graspXOffset_ = 0.0;
  double graspZOffset_ = -0.01;
  Eigen::Quaterniond graspFrameOrientation_ = Eigen::Quaterniond::Identity();
  std::array<Eigen::Quaterniond, 2> wristOrientationCompensations_{
      {Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity()}};

  double approachDistance_ = 0.20;
  double retreatDistance_ = 0.15;
  double liftDistance_ = 0.15;
  double viaExtraHeight_ = 0.10;
  double minBoxClearance_ = 0.05;
  double minTableClearance_ = 0.05;
  double tableTopZ_ = 0.80;
  double tableClearance_ = 0.12;
  double approachBoxClearance_ = 0.10;
  double minGraspSeparation_ = 0.05;

  double dtCurrentToVia_ = 1.0;
  double dtViaToPregrasp_ = 1.0;
  double dtPregraspToGrasp_ = 0.8;
  double graspHoldSec_ = 2.0;
  double dtGraspToRetreat_ = 0.8;
  double dtRetreatToLift_ = 1.0;
  double dtLiftToCarryUpright_ = 0.8;
  double dtCarryUprightToHome_ = 1.0;
  bool carryHomeAfterGrasp_ = true;
  bool enableTransportStage_ = false;
  bool enablePlaceStage_ = false;
  bool maintainRigidGraspAfterContact_ = true;
  bool transportOffsetIsAbsolute_ = false;
  double dtLiftToTransport_ = 1.0;
  double dtTransportToPrePlace_ = 0.5;
  double dtPrePlaceToPlace_ = 1.0;
  double dtPlaceToRelease_ = 0.5;
  double dtReleaseToPostReleaseRetreat_ = 1.0;
  double dtPostReleaseRetreatToInitial_ = 1.0;
  double stablePostureHoldSec_ = 1.0;
  double trajectoryTimeScale_ = 1.0;
  double carryHomeBoxX_ = 0.65;
  double carryHomeBoxY_ = 0.0;
  double carryHomeBoxZ_ = 1.20;
  double carryHomeFrontClearance_ = 0.05;
  double carryHomeTableClearance_ = 0.05;
  double prePlaceHeight_ = 0.10;
  double postReleaseRetreatDistance_ = 0.15;
  double postReleaseRetreatHeight_ = 0.05;
  Eigen::Vector3d transportOffset_ = Eigen::Vector3d::Zero();
  bool dryRunMode_ = false;

  std::array<pinocchio::FrameIndex, 2> eeFrameIds_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSubscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr boxPoseSubscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr placeBoxPoseSubscriber_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr planService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr continueReturnToInitialService_;
  rclcpp::Service<ocs2_msgs::srv::EvaluateBoxPose>::SharedPtr evaluateService_;

  std::mutex mutex_;
  PlannerState plannerState_ = PlannerState::IDLE;
  double activeTrajectoryEndTime_ = -1.0;
  SystemObservation latestObservation_;
  bool hasObservation_ = false;
  std::array<PoseData, 2> stablePostureEndEffectorPoses_{};
  bool hasStablePostureEndEffectorPoses_ = false;
  std::array<PoseData, 2> postReleaseRetreatEndEffectorPoses_{};
  bool hasPostReleaseRetreatEndEffectorPoses_ = false;
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
  ocs2::vector_t stablePosture = ocs2::vector_t::Zero(pinocchioInterface.getModel().nq);
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState.arm", stablePosture);

  ocs2::mobile_manipulator::DualArmGraspWaypointPlanner planner(node, robotName, std::move(pinocchioInterface),
                                                                 std::move(modelInfo), std::move(stablePosture));
  planner.spin();
  return 0;
}
