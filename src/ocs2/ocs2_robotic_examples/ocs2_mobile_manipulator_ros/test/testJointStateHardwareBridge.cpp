#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <ocs2_mobile_manipulator_ros/JointStateHardwareBridgeHelpers.h>

using namespace ocs2::mobile_manipulator;

TEST(JointStateHardwareBridgeHelpers, MapsControlledJointPositionsByName) {
  sensor_msgs::msg::JointState msg;
  msg.name = {"joint_c", "extra_joint", "joint_a", "joint_b"};
  msg.position = {3.0, 99.0, 1.0, 2.0};

  Eigen::VectorXd state;
  std::string error;
  const bool ok = mapJointStateToControlledPositions(msg, {"joint_a", "joint_b", "joint_c"}, &state, &error);

  ASSERT_TRUE(ok) << error;
  ASSERT_EQ(state.size(), 3);
  EXPECT_DOUBLE_EQ(state(0), 1.0);
  EXPECT_DOUBLE_EQ(state(1), 2.0);
  EXPECT_DOUBLE_EQ(state(2), 3.0);
}

TEST(JointStateHardwareBridgeHelpers, RejectsMissingControlledJoint) {
  sensor_msgs::msg::JointState msg;
  msg.name = {"joint_a", "joint_c"};
  msg.position = {1.0, 3.0};

  Eigen::VectorXd state;
  std::string error;
  const bool ok = mapJointStateToControlledPositions(msg, {"joint_a", "joint_b", "joint_c"}, &state, &error);

  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("joint_b"), std::string::npos);
}

TEST(JointStateHardwareBridgeHelpers, BuildsCommandMessageUsingOutputJointOrder) {
  const Eigen::VectorXd controlledState = (Eigen::VectorXd(3) << 1.5, -2.0, 0.25).finished();
  const auto msg = buildCommandJointState(
      std_msgs::msg::Header(), controlledState, {"joint_a", "joint_b", "joint_c"},
      {"fixed_zero", "joint_c", "joint_a", "joint_b", "unused_joint"});

  ASSERT_EQ(msg.name.size(), 5u);
  ASSERT_EQ(msg.position.size(), 5u);
  EXPECT_EQ(msg.name[0], "fixed_zero");
  EXPECT_EQ(msg.name[1], "joint_c");
  EXPECT_EQ(msg.name[2], "joint_a");
  EXPECT_EQ(msg.name[3], "joint_b");
  EXPECT_EQ(msg.name[4], "unused_joint");
  EXPECT_DOUBLE_EQ(msg.position[0], 0.0);
  EXPECT_DOUBLE_EQ(msg.position[1], 0.25);
  EXPECT_DOUBLE_EQ(msg.position[2], 1.5);
  EXPECT_DOUBLE_EQ(msg.position[3], -2.0);
  EXPECT_DOUBLE_EQ(msg.position[4], 0.0);
}

TEST(JointStateHardwareBridgeHelpers, DetectsTimedOutObservation) {
  EXPECT_FALSE(isObservationTimedOut(10.0, 9.95, 0.1));
  EXPECT_TRUE(isObservationTimedOut(10.0, 9.89, 0.1));
}

TEST(JointStateHardwareBridgeHelpers, RejectsQueryTimeOutsidePolicyCoverage) {
  const std::vector<double> timeTrajectory{10.0, 10.2, 10.4};

  EXPECT_TRUE(isQueryTimeCoveredByPolicy(timeTrajectory, 10.0));
  EXPECT_TRUE(isQueryTimeCoveredByPolicy(timeTrajectory, 10.35));
  EXPECT_FALSE(isQueryTimeCoveredByPolicy(timeTrajectory, 9.99));
  EXPECT_FALSE(isQueryTimeCoveredByPolicy(timeTrajectory, 10.41));
}

TEST(JointStateHardwareBridgeHelpers, RejectsEmptyPolicyCoverage) {
  const std::vector<double> timeTrajectory;
  EXPECT_FALSE(isQueryTimeCoveredByPolicy(timeTrajectory, 10.0));
}

TEST(JointStateHardwareBridgeHelpers, ClampsQueryTimeToPolicyBoundaryWithinTolerance) {
  const std::vector<double> timeTrajectory{10.0, 10.2, 10.4};
  double queryTime = 0.0;
  std::string error;

  const bool ok = computeBoundedPolicyQueryTime(timeTrajectory, 10.4005, 0.001, &queryTime, &error);

  ASSERT_TRUE(ok) << error;
  EXPECT_DOUBLE_EQ(queryTime, 10.4);
}

TEST(JointStateHardwareBridgeHelpers, RejectsQueryTimeTooFarBeyondPolicyBoundary) {
  const std::vector<double> timeTrajectory{10.0, 10.2, 10.4};
  double queryTime = 0.0;
  std::string error;

  const bool ok = computeBoundedPolicyQueryTime(timeTrajectory, 10.405, 0.001, &queryTime, &error);

  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("outside active policy"), std::string::npos);
}

TEST(JointStateHardwareBridgeHelpers, RefreshesExpiredPolicyWhenNoRequestIsPending) {
  EXPECT_TRUE(shouldRefreshExpiredPolicy(false));
}

TEST(JointStateHardwareBridgeHelpers, DetectsPolicySynchronizedToObservationTime) {
  EXPECT_TRUE(isPolicySynchronizedForObservationTime(10.0008, 10.0000, 100.0));
  EXPECT_FALSE(isPolicySynchronizedForObservationTime(10.0200, 10.0000, 100.0));
}

TEST(JointStateHardwareBridgeHelpers, AcceptsPolicyThatAlreadyCoversPendingObservation) {
  EXPECT_TRUE(isPolicyFreshEnoughForObservationTime({10.0, 10.2, 10.4}, 10.15, 100.0));
}

TEST(JointStateHardwareBridgeHelpers, AcceptsPolicyThatAdvancedPastPendingObservation) {
  EXPECT_TRUE(isPolicyFreshEnoughForObservationTime({10.05, 10.2, 10.4}, 10.0, 100.0));
}

TEST(JointStateHardwareBridgeHelpers, RejectsPolicyThatIsStillEntirelyBeforePendingObservation) {
  EXPECT_FALSE(isPolicyFreshEnoughForObservationTime({9.7, 9.8, 9.9}, 10.0, 100.0));
}

TEST(JointStateHardwareBridgeHelpers, SkipsNewObservationRequestsWhilePolicyIsPending) {
  EXPECT_TRUE(shouldRequestPolicyUpdate(0, 4, false));
  EXPECT_FALSE(shouldRequestPolicyUpdate(1, 4, false));
  EXPECT_FALSE(shouldRequestPolicyUpdate(4, 4, true));
}

TEST(JointStateHardwareBridgeHelpers, RequestsPolicyEveryOtherTickForTwentyHertzObservations) {
  EXPECT_TRUE(shouldRequestPolicyUpdate(0, 2, false));
  EXPECT_FALSE(shouldRequestPolicyUpdate(1, 2, false));
  EXPECT_TRUE(shouldRequestPolicyUpdate(2, 2, false));
}

TEST(JointStateHardwareBridgeHelpers, RetriesPendingPolicyRequestAfterRepeatedTicks) {
  EXPECT_FALSE(shouldRetryPendingPolicyRequest(1, 2));
  EXPECT_FALSE(shouldRetryPendingPolicyRequest(3, 2));
  EXPECT_TRUE(shouldRetryPendingPolicyRequest(4, 2));
}

TEST(JointStateHardwareBridgeHelpers, DetectsNewObservationSamplesForCommandPublication) {
  EXPECT_TRUE(shouldPublishCommandForObservationTime(10.0, std::nullopt));
  EXPECT_TRUE(shouldPublishCommandForObservationTime(10.1, 10.0));
  EXPECT_FALSE(shouldPublishCommandForObservationTime(10.1, 10.1));
}

TEST(JointStateHardwareBridgeHelpers, DetectsDuplicateMpcRosGraphEndpoints) {
  std::string error;
  EXPECT_FALSE(hasDuplicateMpcRosGraphEndpoints(1, 0, 1, &error));
  EXPECT_TRUE(hasDuplicateMpcRosGraphEndpoints(2, 0, 1, &error));
  EXPECT_NE(error.find("observation publishers"), std::string::npos);
  EXPECT_TRUE(hasDuplicateMpcRosGraphEndpoints(1, 2, 1, &error));
  EXPECT_NE(error.find("policy publishers"), std::string::npos);
  EXPECT_TRUE(hasDuplicateMpcRosGraphEndpoints(1, 1, 2, &error));
  EXPECT_NE(error.find("reset services"), std::string::npos);
}

TEST(JointStateHardwareBridgeHelpers, FiltersCommandPositionsWithBlendAndStepLimit) {
  const Eigen::VectorXd desired = (Eigen::VectorXd(3) << 1.0, -1.0, 0.02).finished();
  const Eigen::VectorXd last = Eigen::VectorXd::Zero(3);

  const Eigen::VectorXd filtered = filterCommandPositions(desired, last, 0.2, 0.05);

  ASSERT_EQ(filtered.size(), 3);
  EXPECT_DOUBLE_EQ(filtered(0), 0.05);
  EXPECT_DOUBLE_EQ(filtered(1), -0.05);
  EXPECT_DOUBLE_EQ(filtered(2), 0.004);
}

TEST(JointStateHardwareBridgeHelpers, UsesDesiredCommandDirectlyBeforeFirstPublish) {
  const Eigen::VectorXd desired = (Eigen::VectorXd(2) << 0.3, -0.2).finished();
  const Eigen::VectorXd filtered = filterCommandPositions(desired, std::nullopt, 0.2, 0.05);

  ASSERT_EQ(filtered.size(), 2);
  EXPECT_DOUBLE_EQ(filtered(0), 0.3);
  EXPECT_DOUBLE_EQ(filtered(1), -0.2);
}

TEST(JointStateHardwareBridgeHelpers, UsesHeaderStampWhenItMatchesNodeTimebase) {
  EXPECT_DOUBLE_EQ(resolveObservationTime(100.0, 99.95, true, 0.3), 99.95);
}

TEST(JointStateHardwareBridgeHelpers, FallsBackToNodeTimeWhenHeaderStampIsTooOld) {
  EXPECT_DOUBLE_EQ(resolveObservationTime(100.0, 90.0, true, 0.3), 100.0);
}

TEST(JointStateHardwareBridgeHelpers, FallsBackToNodeTimeWhenHeaderStampIsTooFarInFuture) {
  EXPECT_DOUBLE_EQ(resolveObservationTime(100.0, 110.0, true, 0.3), 100.0);
}

TEST(JointStateHardwareBridgeHelpers, FallsBackToNodeTimeWhenHeaderStampIsInvalid) {
  EXPECT_DOUBLE_EQ(resolveObservationTime(100.0, 0.0, false, 0.3), 100.0);
}
