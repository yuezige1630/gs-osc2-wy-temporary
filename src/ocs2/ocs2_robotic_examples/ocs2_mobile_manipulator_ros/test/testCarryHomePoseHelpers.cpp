#include <gtest/gtest.h>

#include <ocs2_mobile_manipulator_ros/CarryHomePoseHelpers.h>

using namespace ocs2::mobile_manipulator;

TEST(CarryHomePoseHelpers, BaseLinkAlignedOrientationIsIdentity) {
  const auto q = baseLinkAlignedBoxOrientation();
  EXPECT_NEAR(q.x(), 0.0, 1e-12);
  EXPECT_NEAR(q.y(), 0.0, 1e-12);
  EXPECT_NEAR(q.z(), 0.0, 1e-12);
  EXPECT_NEAR(q.w(), 1.0, 1e-12);
}

TEST(CarryHomePoseHelpers, RejectsHomePoseWhenBoxLengthWouldHitFrontClearance) {
  std::string error;
  const bool ok = isCarryHomeBoxPoseSafe(Eigen::Vector3d(0.20, 0.0, 1.10), 0.45, 0.12, 0.05, 0.05, &error);
  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("front face"), std::string::npos);
}

TEST(CarryHomePoseHelpers, AcceptsHomePoseWithEnoughFrontAndTableClearance) {
  std::string error;
  const bool ok = isCarryHomeBoxPoseSafe(Eigen::Vector3d(0.50, 0.0, 1.10), 0.45, 0.12, 0.05, 0.05, &error);
  EXPECT_TRUE(ok) << error;
}
