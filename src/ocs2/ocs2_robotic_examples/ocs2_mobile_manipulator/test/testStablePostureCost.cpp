#include <gtest/gtest.h>

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mobile_manipulator/cost/StablePostureCost.h>

using namespace ocs2;
using namespace ocs2::mobile_manipulator;

namespace {

vector_t stableEndEffectorTarget() {
  vector_t target = vector_t::Zero(14);
  target(6) = 1.0;
  target(13) = 1.0;
  return target;
}

}  // namespace

TEST(StablePostureCost, ActivatesOnlyForTheStableDualEndEffectorTarget) {
  const vector_t stableEe = stableEndEffectorTarget();
  const vector_t exactTarget = stableEe;
  vector_t movedTarget = stableEe;
  movedTarget(0) = 0.05;

  EXPECT_TRUE(StablePostureCost::isStableEndEffectorTarget(exactTarget, stableEe, 0.01, 0.05));
  EXPECT_FALSE(StablePostureCost::isStableEndEffectorTarget(movedTarget, stableEe, 0.01, 0.05));
}

TEST(StablePostureCost, ProducesQuadraticJointDeviationWhenActive) {
  const vector_t stableEe = stableEndEffectorTarget();
  vector_t stateTarget = vector_t::Zero(19);
  stateTarget(3) = 1.0;
  stateTarget(10) = 1.0;
  vector_t state = stateTarget;
  state(5) = 0.2;

  const matrix_t Q = matrix_t::Identity(19, 19);
  StablePostureCost cost(stateTarget, stableEe, Q, 0.01, 0.05);
  const TargetTrajectories target({0.0}, {stableEe}, {vector_t::Zero(19)});
  const PreComputation preComputation;

  EXPECT_GT(cost.getValue(0.0, state, vector_t::Zero(19), target, preComputation), 0.0);
  const auto approximation =
      cost.getQuadraticApproximation(0.0, state, vector_t::Zero(19), target, preComputation);
  EXPECT_NEAR(approximation.dfdx(5), 0.2, 1e-12);
  EXPECT_DOUBLE_EQ(approximation.dfdxx(5, 5), 1.0);
}

TEST(StablePostureCost, ReturnsZeroWhenReferenceIsNotStable) {
  const vector_t stableEe = stableEndEffectorTarget();
  vector_t taskTarget = stableEe;
  taskTarget(0) = 0.2;
  const vector_t posture = vector_t::Zero(19);
  StablePostureCost cost(posture, stableEe, matrix_t::Identity(19, 19), 0.01, 0.05);
  const TargetTrajectories target({0.0}, {taskTarget}, {vector_t::Zero(19)});
  const PreComputation preComputation;

  EXPECT_DOUBLE_EQ(cost.getValue(0.0, vector_t::Ones(19), vector_t::Zero(19), target, preComputation), 0.0);
}

