/******************************************************************************
Copyright (c) 2026.
******************************************************************************/

#pragma once

#include <Eigen/Geometry>

#include <cmath>
#include <string>

namespace ocs2 {
namespace mobile_manipulator {

inline Eigen::Quaterniond baseLinkAlignedBoxOrientation() {
  return Eigen::Quaterniond::Identity();
}

inline bool isCarryHomeBoxPoseSafe(const Eigen::Vector3d& boxCenter, double boxSizeX, double boxSizeZ,
                                   double frontClearance, double tableClearance, std::string* errorMessage) {
  if (!boxCenter.array().isFinite().all() || !std::isfinite(boxSizeX) || !std::isfinite(boxSizeZ) ||
      !std::isfinite(frontClearance) || !std::isfinite(tableClearance)) {
    if (errorMessage != nullptr) {
      *errorMessage = "Carry-home box pose parameters must be finite.";
    }
    return false;
  }

  if (boxCenter.x() - 0.5 * boxSizeX < frontClearance) {
    if (errorMessage != nullptr) {
      *errorMessage = "Carry-home box front face violates the front clearance.";
    }
    return false;
  }

  if (boxCenter.z() - 0.5 * boxSizeZ < tableClearance) {
    if (errorMessage != nullptr) {
      *errorMessage = "Carry-home box bottom violates the table clearance.";
    }
    return false;
  }

  return true;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
