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

#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>
#include <ocs2_mobile_manipulator/constraint/EndEffectorConstraint.h>

#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorConstraint::EndEffectorConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                             const ReferenceManager& referenceManager)
    : StateConstraint(ConstraintOrder::Linear),
      numEndEffectors_(endEffectorKinematics.getIds().size()),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      referenceManagerPtr_(&referenceManager) {
  if (numEndEffectors_ == 0) {
    throw std::runtime_error("[EndEffectorConstraint] endEffectorKinematics has no end effector IDs.");
  }
  pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t EndEffectorConstraint::getNumConstraints(scalar_t time) const {
  return 6 * numEndEffectors_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto desiredPose = interpolateEndEffectorPose(time);
  std::vector<quaternion_t> desiredOrientations;
  desiredOrientations.reserve(numEndEffectors_);
  for (const auto& pose : desiredPose) {
    desiredOrientations.emplace_back(pose.orientation);
  }

  const auto position = endEffectorKinematicsPtr_->getPosition(state);
  const auto orientationError = endEffectorKinematicsPtr_->getOrientationError(state, desiredOrientations);

  vector_t constraint(6 * numEndEffectors_);
  for (size_t i = 0; i < numEndEffectors_; ++i) {
    constraint.segment<3>(6 * i) = position.at(i) - desiredPose.at(i).position;
    constraint.segment<3>(6 * i + 3) = orientationError.at(i);
  }
  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                const PreComputation& preComputation) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto desiredPose = interpolateEndEffectorPose(time);
  std::vector<quaternion_t> desiredOrientations;
  desiredOrientations.reserve(numEndEffectors_);
  for (const auto& pose : desiredPose) {
    desiredOrientations.emplace_back(pose.orientation);
  }

  auto approximation = VectorFunctionLinearApproximation(6 * numEndEffectors_, state.rows(), 0);

  const auto eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(state);
  const auto eeOrientationError = endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(state, desiredOrientations);
  for (size_t i = 0; i < numEndEffectors_; ++i) {
    approximation.f.segment<3>(6 * i) = eePosition.at(i).f - desiredPose.at(i).position;
    approximation.dfdx.middleRows<3>(6 * i) = eePosition.at(i).dfdx;

    approximation.f.segment<3>(6 * i + 3) = eeOrientationError.at(i).f;
    approximation.dfdx.middleRows<3>(6 * i + 3) = eeOrientationError.at(i).dfdx;
  }

  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto EndEffectorConstraint::interpolateEndEffectorPose(scalar_t time) const -> std::vector<EndEffectorPose> {
  const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
  const auto& timeTrajectory = targetTrajectories.timeTrajectory;
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;
  if (stateTrajectory.empty()) {
    throw std::runtime_error("[EndEffectorConstraint] Target trajectory is empty.");
  }

  const size_t expectedDim = 7 * numEndEffectors_;
  for (size_t i = 0; i < stateTrajectory.size(); ++i) {
    if (stateTrajectory[i].size() != expectedDim) {
      throw std::runtime_error("[EndEffectorConstraint] Target trajectory state dimension mismatch.");
    }
  }

  auto unpackPose = [&](const vector_t& sample) {
    std::vector<EndEffectorPose> pose(numEndEffectors_);
    for (size_t i = 0; i < numEndEffectors_; ++i) {
      const size_t offset = 7 * i;
      pose[i].position = sample.segment<3>(offset);
      pose[i].orientation.coeffs() = sample.segment<4>(offset + 3);
    }
    return pose;
  };

  if (stateTrajectory.size() > 1) {
    // Normal interpolation case
    int index;
    scalar_t alpha;
    std::tie(index, alpha) = LinearInterpolation::timeSegment(time, timeTrajectory);

    const auto lhsPose = unpackPose(stateTrajectory[index]);
    const auto rhsPose = unpackPose(stateTrajectory[index + 1]);
    std::vector<EndEffectorPose> interpolated(numEndEffectors_);
    for (size_t i = 0; i < numEndEffectors_; ++i) {
      interpolated[i].position = alpha * lhsPose[i].position + (1.0 - alpha) * rhsPose[i].position;
      interpolated[i].orientation = lhsPose[i].orientation.slerp((1.0 - alpha), rhsPose[i].orientation);
    }
    return interpolated;
  }

  return unpackPose(stateTrajectory.front());
}

}  // namespace mobile_manipulator
}  // namespace ocs2
