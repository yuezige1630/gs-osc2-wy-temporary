#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Eigenvalues>

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/reference/TargetTrajectories.h>

namespace ocs2 {
namespace mobile_manipulator {

class StablePostureCost final : public StateInputCost {
 public:
  StablePostureCost(vector_t posture, vector_t stableEndEffectorTarget, matrix_t Q,
                    scalar_t positionTolerance, scalar_t orientationTolerance)
      : posture_(std::move(posture)),
        stableEndEffectorTarget_(std::move(stableEndEffectorTarget)),
        Q_(std::move(Q)),
        positionTolerance_(positionTolerance),
        orientationTolerance_(orientationTolerance) {
    if (posture_.size() == 0 || stableEndEffectorTarget_.size() != 14 || Q_.rows() != posture_.size() ||
        Q_.cols() != posture_.size()) {
      throw std::invalid_argument("[StablePostureCost] Posture, end-effector target, and Q dimensions are invalid.");
    }
    if (!posture_.array().isFinite().all() || !stableEndEffectorTarget_.array().isFinite().all() ||
        !Q_.array().isFinite().all()) {
      throw std::invalid_argument("[StablePostureCost] Constructor inputs must be finite.");
    }
    if (!std::isfinite(positionTolerance_) || positionTolerance_ <= 0.0 ||
        !std::isfinite(orientationTolerance_) || orientationTolerance_ <= 0.0) {
      throw std::invalid_argument("[StablePostureCost] Activation tolerances must be finite and positive.");
    }
    if (!Q_.isApprox(Q_.transpose(), 1e-12)) {
      throw std::invalid_argument("[StablePostureCost] Q must be symmetric.");
    }
    const Eigen::SelfAdjointEigenSolver<matrix_t> eigenSolver(Q_);
    if (eigenSolver.info() != Eigen::Success || eigenSolver.eigenvalues().minCoeff() < -1e-9) {
      throw std::invalid_argument("[StablePostureCost] Q must be positive semidefinite.");
    }
  }

  StablePostureCost* clone() const override { return new StablePostureCost(*this); }

  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& /*input*/,
                   const TargetTrajectories& targetTrajectories,
                   const PreComputation& /*preComputation*/) const override {
    validateStateDimension(state);
    if (!isActiveAt(time, targetTrajectories)) {
      return 0.0;
    }

    const vector_t deviation = state - posture_;
    return 0.5 * deviation.dot(Q_ * deviation);
  }

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(
      scalar_t time, const vector_t& state, const vector_t& input,
      const TargetTrajectories& targetTrajectories,
      const PreComputation& /*preComputation*/) const override {
    validateStateDimension(state);
    auto approximation = ScalarFunctionQuadraticApproximation::Zero(state.size(), input.size());
    if (!isActiveAt(time, targetTrajectories)) {
      return approximation;
    }

    const vector_t deviation = state - posture_;
    approximation.dfdxx = Q_;
    approximation.dfdx = Q_ * deviation;
    approximation.f = 0.5 * deviation.dot(approximation.dfdx);
    return approximation;
  }

  static bool isStableEndEffectorTarget(const vector_t& target, const vector_t& stableTarget,
                                        scalar_t positionTolerance, scalar_t orientationTolerance) {
    if (target.size() != 14 || stableTarget.size() != 14 || !target.array().isFinite().all() ||
        !stableTarget.array().isFinite().all() || !std::isfinite(positionTolerance) || positionTolerance <= 0.0 ||
        !std::isfinite(orientationTolerance) || orientationTolerance <= 0.0) {
      return false;
    }

    for (size_t endEffector = 0; endEffector < 2; ++endEffector) {
      const size_t offset = 7 * endEffector;
      if ((target.segment<3>(offset) - stableTarget.segment<3>(offset)).norm() > positionTolerance) {
        return false;
      }

      Eigen::Quaterniond targetOrientation(target(offset + 6), target(offset + 3), target(offset + 4),
                                            target(offset + 5));
      Eigen::Quaterniond stableOrientation(stableTarget(offset + 6), stableTarget(offset + 3),
                                            stableTarget(offset + 4), stableTarget(offset + 5));
      if (targetOrientation.norm() < 1e-9 || stableOrientation.norm() < 1e-9) {
        return false;
      }
      targetOrientation.normalize();
      stableOrientation.normalize();
      const double dot = std::clamp(std::abs(targetOrientation.dot(stableOrientation)), 0.0, 1.0);
      if (2.0 * std::acos(dot) > orientationTolerance) {
        return false;
      }
    }
    return true;
  }

 private:
  StablePostureCost(const StablePostureCost&) = default;

  void validateStateDimension(const vector_t& state) const {
    if (state.size() != posture_.size()) {
      throw std::invalid_argument("[StablePostureCost] State dimension does not match posture dimension.");
    }
  }

  bool isActiveAt(scalar_t time, const TargetTrajectories& targetTrajectories) const {
    return isStableEndEffectorTarget(targetTrajectories.getDesiredState(time), stableEndEffectorTarget_,
                                     positionTolerance_, orientationTolerance_);
  }

  vector_t posture_;
  vector_t stableEndEffectorTarget_;
  matrix_t Q_;
  scalar_t positionTolerance_;
  scalar_t orientationTolerance_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
