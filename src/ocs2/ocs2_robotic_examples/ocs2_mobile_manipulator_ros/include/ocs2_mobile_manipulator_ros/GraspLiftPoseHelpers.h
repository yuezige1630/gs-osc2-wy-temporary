/******************************************************************************
Copyright (c) 2026.
******************************************************************************/

#pragma once

#include <algorithm>
#include <vector>

namespace ocs2 {
namespace mobile_manipulator {

constexpr double kHandFrameContactLocalZ = 0.01;

inline double contactGraspLocalZ(double graspZOffset) {
  return kHandFrameContactLocalZ + graspZOffset;
}

inline double computeLiftBoxCenterZ(double boxCenterZ, double liftDistance, double minBoxClearance,
                                    double minTableClearance) {
  return std::max(boxCenterZ + liftDistance, std::max(minTableClearance, boxCenterZ + minBoxClearance));
}

inline std::vector<double> makeGraspLiftTimeOffsets(double currentToVia, double viaToPregrasp,
                                                     double pregraspToGrasp, double holdDuration,
                                                     double graspToLift, double timeScale) {
  const double contactTime = currentToVia + viaToPregrasp + pregraspToGrasp;
  const double holdEndTime = contactTime + holdDuration;
  const double liftEndTime = holdEndTime + graspToLift;
  return {0.0, timeScale * currentToVia, timeScale * (currentToVia + viaToPregrasp), timeScale * contactTime,
          timeScale * holdEndTime, timeScale * liftEndTime};
}

}  // namespace mobile_manipulator
}  // namespace ocs2
