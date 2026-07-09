#include <gtest/gtest.h>

#include <array>

#include <ocs2_mobile_manipulator_ros/PostReleaseInitialReturnHelpers.h>

using namespace ocs2::mobile_manipulator;

TEST(PostReleaseInitialReturnHelpers, BuildsThreeStageContinueTrajectory) {
  std::array<PoseLike, 2> currentPoses{};
  std::array<PoseLike, 2> retreatPoses{};
  std::array<PoseLike, 2> taskStartPoses{};

  currentPoses[0].position = Eigen::Vector3d(1.0, 0.2, 0.8);
  currentPoses[0].orientation = Eigen::Quaterniond(0.9, 0.1, 0.2, 0.3).normalized();
  currentPoses[1].position = Eigen::Vector3d(1.0, -0.2, 0.8);
  currentPoses[1].orientation = Eigen::Quaterniond(0.8, -0.2, 0.1, 0.5).normalized();

  retreatPoses[0].position = Eigen::Vector3d(1.1, 0.2, 0.9);
  retreatPoses[0].orientation = Eigen::Quaterniond(0.4, 0.5, 0.6, 0.7).normalized();
  retreatPoses[1].position = Eigen::Vector3d(1.1, -0.2, 0.9);
  retreatPoses[1].orientation = Eigen::Quaterniond(0.7, -0.5, 0.1, 0.4).normalized();

  taskStartPoses[0].position = Eigen::Vector3d(0.4, 0.1, 1.2);
  taskStartPoses[0].orientation = Eigen::Quaterniond::Identity();
  taskStartPoses[1].position = Eigen::Vector3d(0.4, -0.1, 1.2);
  taskStartPoses[1].orientation = Eigen::Quaterniond::Identity();

  const auto trajectory = buildPostReleaseInitialReturnTrajectory(currentPoses, retreatPoses, taskStartPoses, 1.2, 0.5);

  ASSERT_EQ(trajectory.armWaypoints[0].size(), 3u);
  ASSERT_EQ(trajectory.armWaypoints[1].size(), 3u);
  ASSERT_EQ(trajectory.timeOffsets.size(), 3u);
  EXPECT_DOUBLE_EQ(trajectory.timeOffsets[0], 0.0);
  EXPECT_DOUBLE_EQ(trajectory.timeOffsets[1], 0.3);
  EXPECT_DOUBLE_EQ(trajectory.timeOffsets[2], 0.6);

  EXPECT_TRUE(trajectory.armWaypoints[0][0].position.isApprox(currentPoses[0].position));
  EXPECT_TRUE(trajectory.armWaypoints[0][1].position.isApprox(retreatPoses[0].position));
  EXPECT_TRUE(trajectory.armWaypoints[0][2].position.isApprox(taskStartPoses[0].position));
  EXPECT_NEAR(trajectory.armWaypoints[0][1].orientation.x(), 0.0, 1e-12);
  EXPECT_NEAR(trajectory.armWaypoints[0][1].orientation.y(), 0.0, 1e-12);
  EXPECT_NEAR(trajectory.armWaypoints[0][1].orientation.z(), 0.0, 1e-12);
  EXPECT_NEAR(trajectory.armWaypoints[0][1].orientation.w(), 1.0, 1e-12);
}
