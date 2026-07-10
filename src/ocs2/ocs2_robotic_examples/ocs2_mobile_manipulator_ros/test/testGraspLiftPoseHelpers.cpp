#include <gtest/gtest.h>

#include <ocs2_mobile_manipulator_ros/GraspLiftPoseHelpers.h>

namespace ocs2::mobile_manipulator {

TEST(GraspLiftPoseHelpers, DefaultContactOffsetPlacesHandAtBoxCenterHeight) {
  constexpr double kBoxCenterZ = 0.91074;

  EXPECT_DOUBLE_EQ(kBoxCenterZ + contactGraspLocalZ(-0.01), kBoxCenterZ);
}

TEST(GraspLiftPoseHelpers, LiftUsesRequestedDistanceWhenItExceedsClearance) {
  EXPECT_DOUBLE_EQ(computeLiftBoxCenterZ(0.91074, 0.15, 0.05, 0.05), 1.06074);
}

TEST(GraspLiftPoseHelpers, LiftHonorsBoxAndTableClearance) {
  EXPECT_DOUBLE_EQ(computeLiftBoxCenterZ(0.10, 0.01, 0.20, 0.15), 0.30);
}

TEST(GraspLiftPoseHelpers, BuildsSixStageTrajectoryThroughContactAndLift) {
  const auto offsets = makeGraspLiftTimeOffsets(1.0, 1.0, 0.8, 2.0, 1.0, 4.0);

  ASSERT_EQ(offsets.size(), 6u);
  EXPECT_DOUBLE_EQ(offsets[0], 0.0);
  EXPECT_DOUBLE_EQ(offsets[1], 4.0);
  EXPECT_DOUBLE_EQ(offsets[2], 8.0);
  EXPECT_DOUBLE_EQ(offsets[3], 11.2);
  EXPECT_DOUBLE_EQ(offsets[4], 19.2);
  EXPECT_DOUBLE_EQ(offsets[5], 23.2);
}

}  // namespace ocs2::mobile_manipulator
