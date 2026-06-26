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
	    boxSizeX_ = node_->declare_parameter<double>("box_size_x", 0.1978);
	    boxSizeY_ = node_->declare_parameter<double>("box_size_y", 0.3029);
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
	    graspEdgeOffsetY_ = std::max(0.0, 0.5 * boxSizeY_ - graspEdgeInsetY_);
	    wristToHandTranslations_ = loadWristToHandTranslations();

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

	    targetTrajectoriesPublisherPtr_ = std::make_unique<TargetTrajectoriesRosPublisher>(node_, topicPrefix_);
    planService_ = node_->create_service<std_srvs::srv::Trigger>(
        "plan_and_send_grasp_trajectory",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) { handlePlanRequest(*response); });

	    RCLCPP_INFO(node_->get_logger(),
	                "Dual-arm grasp waypoint planner ready. Subscribing to %s and publishing to %s_mpc_target. "
	                "Grasp poses are derived from box size (%.4f, %.4f, %.4f) with edge_offset_y=%.4f.",
	                boxPoseTopic_.c_str(), topicPrefix_.c_str(), boxSizeX_, boxSizeY_, boxSizeZ_, graspEdgeOffsetY_);
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

	  static Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw) {
	    Eigen::Quaterniond quaternion =
	        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
	        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
	    quaternion.normalize();
	    return quaternion;
	  }

	  pinocchio::FrameIndex getBodyFrameId(const std::string& bodyName) const {
	    const auto& model = pinocchioInterface_.getModel();
	    if (model.existBodyName(bodyName)) {
	      return model.getBodyId(bodyName);
	    }
	    if (model.existJointName(bodyName)) {
	      return model.getFrameId(bodyName, pinocchio::JOINT);
	    }
	    throw std::runtime_error("[DualArmGraspWaypointPlanner] Body " + bodyName + " does not exist.");
	  }

	  std::array<Eigen::Vector3d, 2> loadWristToHandTranslations() {
	    const auto& model = pinocchioInterface_.getModel();
	    auto& data = pinocchioInterface_.getData();
	    const vector_t q = vector_t::Zero(model.nq);
	    pinocchio::forwardKinematics(model, data, q);
	    pinocchio::updateFramePlacements(model, data);

	    std::array<Eigen::Vector3d, 2> translations;
	    const std::array<std::string, 2> handFrameNames{{"hand_base_link", "hand_base_right_link"}};
	    for (size_t armIndex = 0; armIndex < 2; ++armIndex) {
	      const auto handFrameId = getBodyFrameId(handFrameNames[armIndex]);
	      const pinocchio::SE3 wristToHand = data.oMf[eeFrameIds_[armIndex]].inverse() * data.oMf[handFrameId];
	      translations[armIndex] = wristToHand.translation();
	      if (!translations[armIndex].array().isFinite().all()) {
	        throw std::runtime_error("[DualArmGraspWaypointPlanner] Wrist-to-hand translation contains NaN/Inf.");
	      }
	    }
	    return translations;
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
	        Eigen::Vector3d(graspXOffset_, graspEdgeOffsetY_, graspZOffset_),
	        Eigen::Vector3d(graspXOffset_, -graspEdgeOffsetY_, graspZOffset_),
	    };

	    for (size_t armIndex = 0; armIndex < 2; ++armIndex) {
	      PoseData graspPose;
	      graspPose.orientation = boxOrientation * graspFrameOrientation_ * wristOrientationCompensations_[armIndex];
	      if (!normalizeQuaternion(graspPose.orientation, &errorMessage)) {
	        errorMessage = armIndex == 0 ? "Left grasp orientation invalid: " + errorMessage
	                                     : "Right grasp orientation invalid: " + errorMessage;
	        return false;
	      }

	      const Eigen::Vector3d handPosition = boxPose.position + boxOrientation * localOffsets[armIndex];
	      const Eigen::Vector3d wristOffsetWorld = graspPose.orientation * wristToHandTranslations_[armIndex];
	      graspPose.position = handPosition - wristOffsetWorld;

	      if (!isFinite(graspPose.position) || !isFinite(graspPose.orientation)) {
	        errorMessage = armIndex == 0 ? "Left grasp pose contains NaN/Inf." : "Right grasp pose contains NaN/Inf.";
	        return false;
	      }
	      graspPoses[armIndex] = graspPose;
	    }

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

    // Side grasp in this setup should approach from outside the box toward the box center.
    // Prefer the lateral radial direction inferred from the grasp point itself.
    if (radial.norm() > 1e-6) {
      return (-radial).normalized();
    }

    // Fallback when grasp point is numerically too close to box center:
    // use the box local +/-Y side to keep left/right arms approaching opposite sides.
    const Eigen::Vector3d boxLocalY = boxPose.orientation.toRotationMatrix().col(1);
    return (armIndex == 0) ? -boxLocalY : boxLocalY;
  }

  bool buildWaypoints(const PlannerSnapshot& snapshot, std::array<std::vector<PoseData>, 2>& armWaypoints, std::string& errorMessage) {
    auto currentPoses = computeCurrentEndEffectorPoses(snapshot.observation);

    const double clearanceBaseZ = std::max(minTableClearance_, snapshot.boxPose.position.z() + minBoxClearance_);
    std::array<PoseData, 2> viaPoses;
    std::array<PoseData, 2> preGraspPoses;
    std::array<PoseData, 2> graspPoses;
    std::array<PoseData, 2> retreatPoses;
    std::array<PoseData, 2> liftPoses;
    std::array<PoseData, 2> transportPoses;
    std::array<Eigen::Vector3d, 2> graspOffsets;

    for (size_t armIndex = 0; armIndex < 2; ++armIndex) {
      auto graspPose = snapshot.graspPoses[armIndex];
      if (!normalizeQuaternion(graspPose.orientation, &errorMessage)) {
        errorMessage = (armIndex == 0 ? "Left grasp pose invalid: " : "Right grasp pose invalid: ") + errorMessage;
        return false;
      }
      const Eigen::Vector3d approachDirection = getApproachDirection(armIndex, snapshot.boxPose, graspPose);
      if (!isFinite(approachDirection) || approachDirection.norm() < 1e-9) {
        errorMessage = armIndex == 0 ? "Left approach direction is invalid." : "Right approach direction is invalid.";
        return false;
      }

      PoseData current = currentPoses[armIndex];
      PoseData preGrasp;
      preGrasp.orientation = graspPose.orientation;
      preGrasp.position = graspPose.position - approachDistance_ * approachDirection;

      PoseData grasp = graspPose;

      PoseData via = preGrasp;
      via.position.z() = std::max({current.position.z(), preGrasp.position.z(), clearanceBaseZ}) + viaExtraHeight_;

      const double preGraspProjection = (grasp.position - preGrasp.position).dot(approachDirection);
      const double retreatProjection = retreatDistance_;
      if (preGraspProjection <= 0.0 || retreatProjection <= 0.0) {
        errorMessage = armIndex == 0 ? "Left pre-grasp/retreat direction does not move toward the box grasp point."
                                     : "Right pre-grasp/retreat direction does not move toward the box grasp point.";
        return false;
      }

      viaPoses[armIndex] = via;
      preGraspPoses[armIndex] = preGrasp;
      graspPoses[armIndex] = grasp;
      graspOffsets[armIndex] = grasp.position - snapshot.boxPose.position;

      for (const auto& pose : {current, via, preGrasp, grasp}) {
        if (!isFinite(pose.position) || !isFinite(pose.orientation)) {
          errorMessage = "Generated waypoint contains NaN/Inf values.";
          return false;
        }
        if (pose.position.z() < minTableClearance_ - 1e-9) {
          errorMessage = "Generated waypoint violates min_table_clearance.";
          return false;
        }
      }
      if (via.position.z() < clearanceBaseZ - 1e-9) {
        errorMessage = "Generated via/lift waypoint violates box or table clearance.";
        return false;
      }
    }

    const Eigen::Vector3d liftBoxCenter = snapshot.boxPose.position + Eigen::Vector3d(0.0, 0.0, liftDistance_);
    const Eigen::Vector3d placeBoxCenter =
        transportOffsetIsAbsolute_ ? transportOffset_ : (snapshot.boxPose.position + transportOffset_);
    Eigen::Vector3d transportBoxCenter = liftBoxCenter;
    if (enableTransportStage_ || enablePlaceStage_) {
      transportBoxCenter.x() = placeBoxCenter.x();
      transportBoxCenter.y() = placeBoxCenter.y();
      transportBoxCenter.z() = std::max(liftBoxCenter.z(), placeBoxCenter.z() + prePlaceHeight_);
    }
    const Eigen::Vector3d prePlaceBoxCenter = transportBoxCenter;

    for (size_t armIndex = 0; armIndex < 2; ++armIndex) {
      PoseData retreat;
      PoseData lift;
      PoseData transport;
      PoseData prePlace;
      PoseData place;
      PoseData release;
      PoseData postReleaseRetreat;

      if (maintainRigidGraspAfterContact_) {
        retreat.orientation = graspPoses[armIndex].orientation;
        retreat.position = snapshot.boxPose.position + graspOffsets[armIndex];

        lift.orientation = graspPoses[armIndex].orientation;
        lift.position = liftBoxCenter + graspOffsets[armIndex];

        transport.orientation = graspPoses[armIndex].orientation;
        transport.position = transportBoxCenter + graspOffsets[armIndex];

        prePlace.orientation = graspPoses[armIndex].orientation;
        prePlace.position = prePlaceBoxCenter + graspOffsets[armIndex];

        place.orientation = graspPoses[armIndex].orientation;
        place.position = placeBoxCenter + graspOffsets[armIndex];

        release = place;

        postReleaseRetreat = currentPoses[armIndex];
      } else {
        const Eigen::Vector3d approachDirection = getApproachDirection(armIndex, snapshot.boxPose, graspPoses[armIndex]);
        retreat.orientation = graspPoses[armIndex].orientation;
        retreat.position = graspPoses[armIndex].position - retreatDistance_ * approachDirection;

        lift = retreat;
        lift.position.z() = std::max(retreat.position.z() + liftDistance_, clearanceBaseZ);

        transport = lift;
        if (enableTransportStage_ || enablePlaceStage_) {
          transport.position.x() = transportBoxCenter.x() + graspOffsets[armIndex].x();
          transport.position.y() = transportBoxCenter.y() + graspOffsets[armIndex].y();
          transport.position.z() = transportBoxCenter.z() + graspOffsets[armIndex].z();
        }

        prePlace = transport;

        place = transport;
        place.position.x() = placeBoxCenter.x() + graspOffsets[armIndex].x();
        place.position.y() = placeBoxCenter.y() + graspOffsets[armIndex].y();
        place.position.z() = std::max(placeBoxCenter.z() + graspOffsets[armIndex].z(), minTableClearance_);

        release = place;

        postReleaseRetreat = currentPoses[armIndex];
      }

      retreatPoses[armIndex] = retreat;
      liftPoses[armIndex] = lift;
      transportPoses[armIndex] = transport;
      if (enablePlaceStage_) {
        if (prePlace.position.z() < clearanceBaseZ - 1e-9) {
          prePlace.position.z() = clearanceBaseZ;
        }
      }

      std::vector<PoseData> poses{currentPoses[armIndex], viaPoses[armIndex], preGraspPoses[armIndex], graspPoses[armIndex], retreat, lift};
      if (enableTransportStage_) {
        poses.push_back(transport);
      }
      if (enablePlaceStage_) {
        poses.push_back(prePlace);
        poses.push_back(place);
        poses.push_back(release);
        poses.push_back(postReleaseRetreat);
      }

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
      if (lift.position.z() < clearanceBaseZ - 1e-9) {
        errorMessage = "Generated via/lift waypoint violates box or table clearance.";
        return false;
      }

      armWaypoints[armIndex] = std::move(poses);
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
    std::vector<double> timeOffsets{0.0,
                                    dtCurrentToVia_,
                                    dtCurrentToVia_ + dtViaToPregrasp_,
                                    dtCurrentToVia_ + dtViaToPregrasp_ + dtPregraspToGrasp_,
                                    dtCurrentToVia_ + dtViaToPregrasp_ + dtPregraspToGrasp_ + dtGraspToRetreat_,
                                    dtCurrentToVia_ + dtViaToPregrasp_ + dtPregraspToGrasp_ + dtGraspToRetreat_ + dtRetreatToLift_};
    if (enableTransportStage_) {
      timeOffsets.push_back(timeOffsets.back() + dtLiftToTransport_);
    }
    if (enablePlaceStage_) {
      timeOffsets.push_back(timeOffsets.back() + dtTransportToPrePlace_);
      timeOffsets.push_back(timeOffsets.back() + dtPrePlaceToPlace_);
      timeOffsets.push_back(timeOffsets.back() + dtPlaceToRelease_);
      timeOffsets.push_back(timeOffsets.back() + dtReleaseToPostReleaseRetreat_);
    }

    if (armWaypoints[0].size() != timeOffsets.size() || armWaypoints[1].size() != timeOffsets.size()) {
      throw std::runtime_error("[DualArmGraspWaypointPlanner] Waypoint count does not match time offsets.");
    }

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
	  double boxSizeX_ = 0.1978;
	  double boxSizeY_ = 0.3029;
	  double boxSizeZ_ = 0.1464;
	  double graspEdgeInsetY_ = 0.0;
	  double graspEdgeOffsetY_ = 0.0;
	  double graspXOffset_ = 0.0;
	  double graspZOffset_ = 0.0;
	  Eigen::Quaterniond graspFrameOrientation_ = Eigen::Quaterniond::Identity();
	  std::array<Eigen::Quaterniond, 2> wristOrientationCompensations_{
	      {Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity()}};
	  std::array<Eigen::Vector3d, 2> wristToHandTranslations_{{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}};

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
  bool enableTransportStage_ = false;
  bool enablePlaceStage_ = false;
  bool maintainRigidGraspAfterContact_ = true;
  bool transportOffsetIsAbsolute_ = false;
  double dtLiftToTransport_ = 1.0;
  double dtTransportToPrePlace_ = 0.5;
  double dtPrePlaceToPlace_ = 1.0;
  double dtPlaceToRelease_ = 0.5;
  double dtReleaseToPostReleaseRetreat_ = 1.0;
  double prePlaceHeight_ = 0.10;
  double postReleaseRetreatDistance_ = 0.15;
  double postReleaseRetreatHeight_ = 0.05;
  Eigen::Vector3d transportOffset_ = Eigen::Vector3d::Zero();

  std::array<pinocchio::FrameIndex, 2> eeFrameIds_;

	  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;
	  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSubscriber_;
	  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr boxPoseSubscriber_;
	  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr planService_;

  std::mutex mutex_;
	  SystemObservation latestObservation_;
	  bool hasObservation_ = false;
	  geometry_msgs::msg::PoseStamped boxPoseMsg_;
	  bool hasBoxPose_ = false;
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
