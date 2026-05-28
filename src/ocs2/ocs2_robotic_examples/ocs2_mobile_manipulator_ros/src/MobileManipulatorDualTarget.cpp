/****************************************************************************
Copyright (c) 2026.
****************************************************************************/

#include <array>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mobile_manipulator/FactoryFunctions.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_msgs/msg/mpc_observation.hpp>

#include "rclcpp/rclcpp.hpp"

namespace ocs2 {
namespace mobile_manipulator {

class DualTargetInteractiveMarker final {
 public:
  DualTargetInteractiveMarker(const rclcpp::Node::SharedPtr& node, const std::string& topicPrefix,
                              PinocchioInterface pinocchioInterface, ManipulatorModelInfo modelInfo)
      : node_(node),
        server_("dual_goal_marker", node_),
        pinocchioInterface_(std::move(pinocchioInterface)),
        modelInfo_(std::move(modelInfo)),
        pinocchioMapping_(modelInfo_) {
    if (modelInfo_.eeFrames.size() != 2) {
      throw std::runtime_error("[DualTargetInteractiveMarker] Exactly two ee frames are required.");
    }

    const auto& model = pinocchioInterface_.getModel();
    eeFrameIds_[0] = model.getBodyId(modelInfo_.eeFrames[0]);
    eeFrameIds_[1] = model.getBodyId(modelInfo_.eeFrames[1]);

    auto observationCallback = [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg) {
      const auto observation = ros_msg_conversions::readObservationMsg(*msg);
      std::lock_guard<std::mutex> lock(mutex_);
      latestObservation_ = observation;
      hasObservation_ = true;
      initializeGoalsFromObservation();
    };
    observationSubscriber_ = node_->create_subscription<ocs2_msgs::msg::MpcObservation>(topicPrefix + "_mpc_observation", 1,
                                                                                          observationCallback);

    targetTrajectoriesPublisherPtr_ = std::make_unique<TargetTrajectoriesRosPublisher>(node_, topicPrefix);

    auto feedbackCb = [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
      processFeedback(feedback);
    };
    menuHandler_.insert("Send target pose", feedbackCb);

    auto leftMarker = createInteractiveMarker("left_goal", "Left Goal", Eigen::Vector3d(0.6, 0.3, 1.0));
    auto rightMarker = createInteractiveMarker("right_goal", "Right Goal", Eigen::Vector3d(0.6, -0.3, 1.0));

    server_.insert(leftMarker);
    menuHandler_.apply(server_, leftMarker.name);

    server_.insert(rightMarker);
    menuHandler_.apply(server_, rightMarker.name);

    server_.applyChanges();
  }

  void spin() { rclcpp::spin(node_); }

 private:
  static geometry_msgs::msg::Pose toPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = position.x();
    pose.position.y = position.y();
    pose.position.z = position.z();
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
    return pose;
  }

  visualization_msgs::msg::InteractiveMarker createInteractiveMarker(const std::string& name, const std::string& description,
                                                                     const Eigen::Vector3d& position) const {
    visualization_msgs::msg::InteractiveMarker interactiveMarker;
    interactiveMarker.header.frame_id = "world";
    interactiveMarker.header.stamp = node_->now();
    interactiveMarker.name = name;
    interactiveMarker.scale = 0.2;
    interactiveMarker.description = description + " (right click to send)";
    interactiveMarker.pose.position.x = position.x();
    interactiveMarker.pose.position.y = position.y();
    interactiveMarker.pose.position.z = position.z();

    visualization_msgs::msg::Marker boxMarker;
    boxMarker.type = visualization_msgs::msg::Marker::CUBE;
    boxMarker.scale.x = 0.1;
    boxMarker.scale.y = 0.1;
    boxMarker.scale.z = 0.1;
    boxMarker.color.r = (name == "left_goal") ? 0.2f : 0.8f;
    boxMarker.color.g = (name == "left_goal") ? 0.8f : 0.2f;
    boxMarker.color.b = 0.5f;
    boxMarker.color.a = 0.7f;

    visualization_msgs::msg::InteractiveMarkerControl boxControl;
    boxControl.always_visible = true;
    boxControl.markers.push_back(boxMarker);
    boxControl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
    interactiveMarker.controls.push_back(boxControl);

    visualization_msgs::msg::InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(control);

    return interactiveMarker;
  }

  void initializeGoalsFromObservation() {
    if (!hasObservation_) {
      return;
    }

    vector_t q = pinocchioMapping_.getPinocchioJointPosition(latestObservation_.state);
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    for (size_t i = 0; i < 2; ++i) {
      if (goalInitialized_[i]) {
        continue;
      }
      goalPositions_[i] = data.oMf[eeFrameIds_[i]].translation();
      goalOrientations_[i] = Eigen::Quaterniond(data.oMf[eeFrameIds_[i]].rotation());
      goalOrientations_[i].normalize();
      goalInitialized_[i] = true;

      const std::string markerName = (i == 0) ? "left_goal" : "right_goal";
      server_.setPose(markerName, toPose(goalPositions_[i], goalOrientations_[i]));
    }

    server_.applyChanges();
  }

  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
    if (feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!hasObservation_) {
      RCLCPP_WARN(node_->get_logger(), "No MPC observation received yet, command ignored.");
      return;
    }
    initializeGoalsFromObservation();

    size_t index = 0;
    if (feedback->marker_name == "left_goal") {
      index = 0;
    } else if (feedback->marker_name == "right_goal") {
      index = 1;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Unknown marker name: %s", feedback->marker_name.c_str());
      return;
    }

    goalPositions_[index] = Eigen::Vector3d(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
    goalOrientations_[index] = Eigen::Quaterniond(feedback->pose.orientation.w, feedback->pose.orientation.x,
                                                  feedback->pose.orientation.y, feedback->pose.orientation.z);
    goalOrientations_[index].normalize();
    goalInitialized_[index] = true;

    vector_t target(14);
    target.segment<3>(0) = goalPositions_[0];
    target.segment<4>(3) = goalOrientations_[0].coeffs();
    target.segment<3>(7) = goalPositions_[1];
    target.segment<4>(10) = goalOrientations_[1].coeffs();

    const scalar_array_t timeTrajectory{latestObservation_.time};
    const vector_array_t stateTrajectory{target};
    const vector_array_t inputTrajectory{vector_t::Zero(latestObservation_.input.size())};
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories({timeTrajectory, stateTrajectory, inputTrajectory});
  }

  rclcpp::Node::SharedPtr node_;
  interactive_markers::MenuHandler menuHandler_;
  interactive_markers::InteractiveMarkerServer server_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSubscriber_;

  PinocchioInterface pinocchioInterface_;
  ManipulatorModelInfo modelInfo_;
  MobileManipulatorPinocchioMapping pinocchioMapping_;
  std::array<pinocchio::FrameIndex, 2> eeFrameIds_;

  std::mutex mutex_;
  SystemObservation latestObservation_;
  bool hasObservation_ = false;
  std::array<Eigen::Vector3d, 2> goalPositions_{};
  std::array<Eigen::Quaterniond, 2> goalOrientations_{};
  std::array<bool, 2> goalInitialized_{{false, false}};
};

}  // namespace mobile_manipulator
}  // namespace ocs2

int main(int argc, char* argv[]) {
  const std::string robotName = "mobile_manipulator";
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(robotName + "_dual_target");

  const std::string taskFile = node->declare_parameter<std::string>("taskFile", "");
  const std::string urdfFile = node->declare_parameter<std::string>("urdfFile", "");
  if (taskFile.empty() || urdfFile.empty()) {
    throw std::runtime_error("[MobileManipulatorDualTarget] Parameters 'taskFile' and 'urdfFile' are required.");
  }
  if (!std::ifstream(taskFile).good()) {
    throw std::runtime_error("[MobileManipulatorDualTarget] Task file not found: " + taskFile);
  }
  if (!std::ifstream(urdfFile).good()) {
    throw std::runtime_error("[MobileManipulatorDualTarget] URDF file not found: " + urdfFile);
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
    throw std::runtime_error("[MobileManipulatorDualTarget] 'model_information.eeFrames' must contain exactly 2 frames.");
  }

  auto pinocchioInterface = ocs2::mobile_manipulator::createPinocchioInterface(urdfFile, modelType, removeJointNames);
  auto modelInfo =
      ocs2::mobile_manipulator::createManipulatorModelInfo(pinocchioInterface, modelType, baseFrame, eeFrames);

  ocs2::mobile_manipulator::DualTargetInteractiveMarker dualTarget(node, robotName, std::move(pinocchioInterface),
                                                                    std::move(modelInfo));
  dualTarget.spin();
  return 0;
}
