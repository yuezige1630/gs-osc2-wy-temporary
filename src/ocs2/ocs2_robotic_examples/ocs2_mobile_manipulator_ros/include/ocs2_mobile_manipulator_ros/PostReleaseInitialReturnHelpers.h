/******************************************************************************
Copyright (c) 2026.
******************************************************************************/

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
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
    const std::array<PoseT, 2>& stablePoses, double dtPostReleaseRetreatToInitial, double trajectoryTimeScale,
    double stablePoseHoldSec = 1.0) {
  if (!std::isfinite(dtPostReleaseRetreatToInitial) || dtPostReleaseRetreatToInitial <= 0.0 ||
      !std::isfinite(trajectoryTimeScale) || trajectoryTimeScale <= 0.0 || !std::isfinite(stablePoseHoldSec) ||
      stablePoseHoldSec <= 0.0) {
    throw std::invalid_argument(
        "[PostReleaseInitialReturnHelpers] Return transition and stable hold durations must be finite and positive.");
  }

  PostReleaseInitialReturnTrajectory<PoseT> trajectory;
  const double transitionDuration = trajectoryTimeScale * dtPostReleaseRetreatToInitial;
  const double holdDuration = trajectoryTimeScale * stablePoseHoldSec;
  trajectory.timeOffsets = {0.0, 0.5 * transitionDuration, transitionDuration, transitionDuration + holdDuration};

  for (std::size_t armIndex = 0; armIndex < 2; ++armIndex) {
    trajectory.armWaypoints[armIndex] = {currentPoses[armIndex], makeBaseLinkAlignedPose(retreatPoses[armIndex]),
                                         stablePoses[armIndex], stablePoses[armIndex]};
  }

  return trajectory;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
