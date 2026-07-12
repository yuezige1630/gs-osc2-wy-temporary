/******************************************************************************
Copyright (c) 2026.
******************************************************************************/

#pragma once

#include <algorithm>

#include <Eigen/Core>

#include <sensor_msgs/msg/joint_state.hpp>
#include <urdf/model.h>

#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace ocs2 {
namespace mobile_manipulator {

namespace detail {

inline void collectMovableJointNames(const urdf::LinkConstSharedPtr& link, std::vector<std::string>& jointNames) {
  if (!link) {
    return;
  }

  for (const auto& childJoint : link->child_joints) {
    if (childJoint && childJoint->type != urdf::Joint::FIXED) {
      jointNames.push_back(childJoint->name);
    }
  }

  for (const auto& childLink : link->child_links) {
    collectMovableJointNames(childLink, jointNames);
  }
}

}  // namespace detail

inline std::vector<std::string> loadMovableJointNamesFromUrdf(const std::string& urdfFile) {
  urdf::Model urdfModel;
  if (!urdfModel.initFile(urdfFile)) {
    throw std::runtime_error("[JointStateHardwareBridgeHelpers] Failed to parse URDF file: " + urdfFile);
  }

  std::vector<std::string> jointNames;
  detail::collectMovableJointNames(urdfModel.getRoot(), jointNames);
  return jointNames;
}

inline bool mapJointStateToControlledPositions(const sensor_msgs::msg::JointState& msg,
                                               const std::vector<std::string>& controlledJointNames,
                                               Eigen::VectorXd* controlledPositions, std::string* errorMessage) {
  if (controlledPositions == nullptr) {
    throw std::invalid_argument("[JointStateHardwareBridgeHelpers] controlledPositions must not be null.");
  }

  if (msg.name.size() != msg.position.size()) {
    if (errorMessage != nullptr) {
      *errorMessage = "JointState name and position arrays must have the same length.";
    }
    return false;
  }

  std::unordered_map<std::string, double> positionByName;
  positionByName.reserve(msg.name.size());
  for (size_t i = 0; i < msg.name.size(); ++i) {
    positionByName[msg.name[i]] = msg.position[i];
  }

  controlledPositions->resize(static_cast<Eigen::Index>(controlledJointNames.size()));
  for (size_t i = 0; i < controlledJointNames.size(); ++i) {
    const auto it = positionByName.find(controlledJointNames[i]);
    if (it == positionByName.end()) {
      if (errorMessage != nullptr) {
        *errorMessage = "Missing controlled joint in observation: " + controlledJointNames[i];
      }
      controlledPositions->resize(0);
      return false;
    }
    (*controlledPositions)(static_cast<Eigen::Index>(i)) = it->second;
  }

  return true;
}

inline sensor_msgs::msg::JointState buildCommandJointState(const std_msgs::msg::Header& header,
                                                           const Eigen::VectorXd& controlledPositions,
                                                           const std::vector<std::string>& controlledJointNames,
                                                           const std::vector<std::string>& outputJointNames) {
  if (controlledPositions.size() != static_cast<Eigen::Index>(controlledJointNames.size())) {
    throw std::invalid_argument("[JointStateHardwareBridgeHelpers] controlledPositions size must match controlledJointNames size.");
  }

  std::unordered_map<std::string, double> positionByName;
  positionByName.reserve(controlledJointNames.size());
  for (size_t i = 0; i < controlledJointNames.size(); ++i) {
    positionByName.emplace(controlledJointNames[i], controlledPositions(static_cast<Eigen::Index>(i)));
  }

  sensor_msgs::msg::JointState msg;
  msg.header = header;
  msg.name = outputJointNames;
  msg.position.resize(outputJointNames.size(), 0.0);
  for (size_t i = 0; i < outputJointNames.size(); ++i) {
    const auto it = positionByName.find(outputJointNames[i]);
    if (it != positionByName.end()) {
      msg.position[i] = it->second;
    }
  }

  return msg;
}

inline bool isObservationTimedOut(double nowSec, double observationSec, double timeoutSec) {
  if (!std::isfinite(nowSec) || !std::isfinite(observationSec) || !std::isfinite(timeoutSec) || timeoutSec < 0.0) {
    throw std::invalid_argument("[JointStateHardwareBridgeHelpers] Observation timeout inputs must be finite and timeout non-negative.");
  }
  return (nowSec - observationSec) > timeoutSec;
}

inline bool isQueryTimeCoveredByPolicy(const std::vector<double>& timeTrajectory, double queryTime, double toleranceSec = 1.0e-6) {
  if (!std::isfinite(queryTime) || !std::isfinite(toleranceSec) || toleranceSec < 0.0) {
    throw std::invalid_argument("[JointStateHardwareBridgeHelpers] Policy query inputs must be finite and tolerance non-negative.");
  }

  if (timeTrajectory.empty()) {
    return false;
  }

  return queryTime >= (timeTrajectory.front() - toleranceSec) && queryTime <= (timeTrajectory.back() + toleranceSec);
}

inline bool computeBoundedPolicyQueryTime(const std::vector<double>& timeTrajectory, double desiredTime, double toleranceSec,
                                          double* queryTime, std::string* errorMessage) {
  if (queryTime == nullptr) {
    throw std::invalid_argument("[JointStateHardwareBridgeHelpers] queryTime must not be null.");
  }
  if (!std::isfinite(desiredTime) || !std::isfinite(toleranceSec) || toleranceSec < 0.0) {
    throw std::invalid_argument("[JointStateHardwareBridgeHelpers] Policy query inputs must be finite and tolerance non-negative.");
  }
  if (timeTrajectory.empty()) {
    if (errorMessage != nullptr) {
      *errorMessage = "active policy is empty";
    }
    return false;
  }

  const double front = timeTrajectory.front();
  const double back = timeTrajectory.back();
  if (!isQueryTimeCoveredByPolicy(timeTrajectory, desiredTime, toleranceSec)) {
    if (errorMessage != nullptr) {
      *errorMessage = "observation time is outside active policy coverage";
    }
    return false;
  }

  *queryTime = std::max(front, std::min(desiredTime, back));
  return true;
}

inline bool isPolicySynchronizedForObservationTime(double policyStartTime, double observationTime, double mpcDesiredFrequency,
                                                   double toleranceFraction = 0.1) {
  if (!std::isfinite(policyStartTime) || !std::isfinite(observationTime) || !std::isfinite(mpcDesiredFrequency) ||
      !std::isfinite(toleranceFraction) || mpcDesiredFrequency <= 0.0 || toleranceFraction < 0.0) {
    throw std::invalid_argument(
        "[JointStateHardwareBridgeHelpers] Policy synchronization inputs must be finite, with positive frequency and non-negative tolerance.");
  }

  return std::abs(policyStartTime - observationTime) < (toleranceFraction / mpcDesiredFrequency);
}

inline bool isPolicyFreshEnoughForObservationTime(const std::vector<double>& timeTrajectory, double observationTime,
                                                  double mpcDesiredFrequency, double toleranceFraction = 0.5) {
  if (!std::isfinite(observationTime) || !std::isfinite(mpcDesiredFrequency) || !std::isfinite(toleranceFraction) ||
      mpcDesiredFrequency <= 0.0 || toleranceFraction < 0.0) {
    throw std::invalid_argument(
        "[JointStateHardwareBridgeHelpers] Policy freshness inputs must be finite, with positive frequency and non-negative tolerance.");
  }

  if (timeTrajectory.empty()) {
    return false;
  }

  const double toleranceSec = toleranceFraction / mpcDesiredFrequency;
  return isQueryTimeCoveredByPolicy(timeTrajectory, observationTime, toleranceSec) ||
         (timeTrajectory.back() >= (observationTime - toleranceSec));
}

inline bool shouldRequestPolicyUpdate(size_t loopCounter, size_t mpcUpdateRatio, bool hasPendingPolicyRequest) {
  if (mpcUpdateRatio == 0u) {
    throw std::invalid_argument("[JointStateHardwareBridgeHelpers] mpcUpdateRatio must be positive.");
  }

  return !hasPendingPolicyRequest && (loopCounter % mpcUpdateRatio == 0u);
}

inline bool shouldRetryPendingPolicyRequest(size_t pendingPolicyAgeTicks, size_t mpcUpdateRatio) {
  if (mpcUpdateRatio == 0u) {
    throw std::invalid_argument("[JointStateHardwareBridgeHelpers] mpcUpdateRatio must be positive.");
  }

  return pendingPolicyAgeTicks >= std::max<size_t>(2u * mpcUpdateRatio, 2u);
}

// An expired policy cannot produce a command. Request a replacement as soon
// as possible unless an observation is already awaiting a policy update.
inline bool shouldRefreshExpiredPolicy(bool hasPendingPolicyRequest) { return !hasPendingPolicyRequest; }

inline bool shouldPublishCommandForObservationTime(double observationTime, const std::optional<double>& lastPublishedObservationTime) {
  if (!std::isfinite(observationTime)) {
    throw std::invalid_argument("[JointStateHardwareBridgeHelpers] observationTime must be finite.");
  }

  return !lastPublishedObservationTime.has_value() || observationTime != *lastPublishedObservationTime;
}

// The bootstrap MPC policy is initialized from end-effector poses only and can
// choose a different redundant joint configuration before a real target arrives.
inline bool shouldHoldInitialPose(bool externalTargetReceived) { return !externalTargetReceived; }

inline bool hasDuplicateMpcRosGraphEndpoints(size_t observationPublishers, size_t policyPublishers, size_t resetServices,
                                             std::string* errorMessage) {
  if (observationPublishers > 1u) {
    if (errorMessage != nullptr) {
      *errorMessage = "duplicate MRT observation publishers detected on the MPC observation topic";
    }
    return true;
  }

  if (policyPublishers > 1u) {
    if (errorMessage != nullptr) {
      *errorMessage = "duplicate MPC policy publishers detected on the MPC policy topic";
    }
    return true;
  }

  if (resetServices > 1u) {
    if (errorMessage != nullptr) {
      *errorMessage = "duplicate MPC reset services detected on the MPC reset service";
    }
    return true;
  }

  return false;
}

inline Eigen::VectorXd filterCommandPositions(const Eigen::VectorXd& desiredPositions,
                                              const std::optional<Eigen::VectorXd>& lastPublishedPositions, double blendAlpha,
                                              double maxDeltaPerCommand) {
  if (!std::isfinite(blendAlpha) || blendAlpha < 0.0 || blendAlpha > 1.0 || !std::isfinite(maxDeltaPerCommand) ||
      maxDeltaPerCommand < 0.0) {
    throw std::invalid_argument(
        "[JointStateHardwareBridgeHelpers] Command filtering inputs must be finite, with blendAlpha in [0, 1] and non-negative maxDeltaPerCommand.");
  }

  if (!lastPublishedPositions.has_value()) {
    return desiredPositions;
  }

  if (lastPublishedPositions->size() != desiredPositions.size()) {
    throw std::invalid_argument(
        "[JointStateHardwareBridgeHelpers] lastPublishedPositions size must match desiredPositions size.");
  }

  const Eigen::VectorXd blendedPositions = ((1.0 - blendAlpha) * (*lastPublishedPositions)) + (blendAlpha * desiredPositions);
  const Eigen::VectorXd unclampedDelta = blendedPositions - *lastPublishedPositions;
  const Eigen::VectorXd clampedDelta =
      unclampedDelta.array().max(-maxDeltaPerCommand).min(maxDeltaPerCommand).matrix();
  return *lastPublishedPositions + clampedDelta;
}

inline double resolveObservationTime(double nodeNowSec, double headerStampSec, bool hasValidHeaderStamp, double maxAllowedSkewSec) {
  if (!std::isfinite(nodeNowSec) || !std::isfinite(headerStampSec) || !std::isfinite(maxAllowedSkewSec) || maxAllowedSkewSec < 0.0) {
    throw std::invalid_argument("[JointStateHardwareBridgeHelpers] Observation time inputs must be finite and skew non-negative.");
  }

  if (!hasValidHeaderStamp) {
    return nodeNowSec;
  }

  return (std::abs(nodeNowSec - headerStampSec) <= maxAllowedSkewSec) ? headerStampSec : nodeNowSec;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
