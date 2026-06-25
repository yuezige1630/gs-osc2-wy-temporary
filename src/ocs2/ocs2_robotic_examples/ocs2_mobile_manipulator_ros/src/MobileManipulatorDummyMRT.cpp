/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator_ros/MobileManipulatorDummyVisualization.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

using namespace ocs2;
using namespace mobile_manipulator;

int main(int argc, char** argv) {
  const std::string robotName = "mobile_manipulator";

  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared(robotName + "_mrt");

  const std::string taskFile =
      node->declare_parameter<std::string>("taskFile", "");
  const std::string libFolder =
      node->declare_parameter<std::string>("libFolder", "");
  const std::string urdfFile =
      node->declare_parameter<std::string>("urdfFile", "");
  if (taskFile.empty() || libFolder.empty() || urdfFile.empty()) {
    throw std::runtime_error(
        "[MobileManipulatorDummyMRT] Parameters 'taskFile', 'libFolder', and "
        "'urdfFile' are required.");
  }
  std::cerr << "Loading task file: " << taskFile << std::endl;
  std::cerr << "Loading library folder: " << libFolder << std::endl;
  std::cerr << "Loading urdf file: " << urdfFile << std::endl;
  // Robot Interface
  mobile_manipulator::MobileManipulatorInterface interface(taskFile, libFolder,
                                                           urdfFile);

  // MRT
  MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&interface.getRollout());
  mrt.launchNodes(node);

  // Visualization
  auto dummyVisualization =
      std::make_shared<mobile_manipulator::MobileManipulatorDummyVisualization>(
          node, interface);

  // Dummy MRT
  MRT_ROS_Dummy_Loop dummy(mrt, interface.mpcSettings().mrtDesiredFrequency_,
                           interface.mpcSettings().mpcDesiredFrequency_);
  dummy.subscribeObservers({dummyVisualization});

  // initial state
  SystemObservation initObservation;
  initObservation.state = interface.getInitialState();
  initObservation.input.setZero(interface.getManipulatorModelInfo().inputDim);
  initObservation.time = 0.0;

  // Initialize the target at the current end-effector pose so the robot stays
  // at its configured initial state after loading.
  const auto& modelInfo = interface.getManipulatorModelInfo();
  const auto& model = interface.getPinocchioInterface().getModel();
  auto& data =
      const_cast<PinocchioInterface&>(interface.getPinocchioInterface()).getData();
  const auto& eeFrames =
      modelInfo.eeFrames.empty() ? std::vector<std::string>{modelInfo.eeFrame}
                                 : modelInfo.eeFrames;
  const size_t numEndEffectors = eeFrames.size();

  pinocchio::forwardKinematics(model, data, initObservation.state);
  pinocchio::updateFramePlacements(model, data);

  vector_t initTarget(7 * numEndEffectors);
  for (size_t i = 0; i < numEndEffectors; ++i) {
    const auto eeIndex = model.getBodyId(eeFrames[i]);
    initTarget.segment<3>(7 * i) = data.oMf[eeIndex].translation();
    Eigen::Quaternion<scalar_t> eeOrientation(data.oMf[eeIndex].rotation());
    eeOrientation.normalize();
    initTarget.segment<4>(7 * i + 3) = eeOrientation.coeffs();
  }
  const vector_t zeroInput =
      vector_t::Zero(interface.getManipulatorModelInfo().inputDim);
  const TargetTrajectories initTargetTrajectories({initObservation.time},
                                                  {initTarget}, {zeroInput});

  // Run dummy (loops while ros is ok)
  dummy.run(initObservation, initTargetTrajectories);

  // Successful exit
  return 0;
}
