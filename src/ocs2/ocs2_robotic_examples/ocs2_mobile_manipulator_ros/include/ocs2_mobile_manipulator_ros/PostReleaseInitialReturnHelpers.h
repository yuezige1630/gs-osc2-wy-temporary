/******************************************************************************
Copyright (c) 2026.
******************************************************************************/

#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include <Eigen/Geometry>

namespace ocs2 {
namespace mobile_manipulator {

struct PoseLike {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

template <typename PoseT>
struct PostReleaseInitialReturnTrajectory {
  std::array<std::vector<PoseT>, 2> armWaypoints;
  std::vector<double> timeOffsets;
};

template <typename PoseT>
inline PoseT makeBaseLinkAlignedPose(PoseT pose) {
  pose.orientation = Eigen::Quaterniond::Identity();
  return pose;
}

template <typename PoseT>
inline PostReleaseInitialReturnTrajectory<PoseT> buildPostReleaseInitialReturnTrajectory(
    const std::array<PoseT, 2>& currentPoses, const std::array<PoseT, 2>& retreatPoses,
    const std::array<PoseT, 2>& taskStartPoses, double dtPostReleaseRetreatToInitial, double trajectoryTimeScale) {
  PostReleaseInitialReturnTrajectory<PoseT> trajectory;
  const double totalDuration = trajectoryTimeScale * dtPostReleaseRetreatToInitial;
  trajectory.timeOffsets = {0.0, 0.5 * totalDuration, totalDuration};

  for (std::size_t armIndex = 0; armIndex < 2; ++armIndex) {
    trajectory.armWaypoints[armIndex] = {currentPoses[armIndex], makeBaseLinkAlignedPose(retreatPoses[armIndex]),
                                         taskStartPoses[armIndex]};
  }

  return trajectory;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
