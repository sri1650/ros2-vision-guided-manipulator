#include "vgm_motion/motion_policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace vgm_motion
{
namespace
{

bool isFinite(const Point3 & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool isFinite(const Quaternion & orientation)
{
  return std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
         std::isfinite(orientation.z) && std::isfinite(orientation.w);
}

bool isFinite(const ObjectSize & size)
{
  return std::isfinite(size.x) && std::isfinite(size.y) && std::isfinite(size.z);
}

double quaternionNorm(const Quaternion & orientation)
{
  return std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
}

}  // namespace

MotionPolicy::MotionPolicy(PolicyLimits limits)
: limits_(std::move(limits))
{
  const auto & minimum = limits_.workspace.minimum;
  const auto & maximum = limits_.workspace.maximum;
  if (!isFinite(minimum) || !isFinite(maximum) || minimum.x >= maximum.x ||
    minimum.y >= maximum.y || minimum.z >= maximum.z)
  {
    throw std::invalid_argument("workspace bounds must be finite and strictly ordered");
  }
  if (limits_.planning_frame.empty()) {
    throw std::invalid_argument("planning_frame must not be empty");
  }
  if (!std::isfinite(limits_.minimum_confidence) || limits_.minimum_confidence < 0.0 ||
    limits_.minimum_confidence > 1.0)
  {
    throw std::invalid_argument("minimum_confidence must be in [0, 1]");
  }
  if (!std::isfinite(limits_.maximum_object_dimension) ||
    limits_.maximum_object_dimension <= 0.0)
  {
    throw std::invalid_argument("maximum_object_dimension must be positive");
  }
  if (!std::isfinite(limits_.quaternion_norm_tolerance) ||
    limits_.quaternion_norm_tolerance < 0.0)
  {
    throw std::invalid_argument("quaternion_norm_tolerance must be finite and non-negative");
  }
}

PolicyDecision MotionPolicy::evaluate(const PolicyRequest & request) const
{
  if (request.object_id.empty()) {
    return {PolicyError::kInvalidRequest, "target object_id is empty"};
  }
  if (request.target_frame != limits_.planning_frame ||
    request.place_frame != limits_.planning_frame)
  {
    return {
      PolicyError::kInvalidRequest,
      "target and place poses must be expressed in planning frame '" +
      limits_.planning_frame + "'"};
  }
  if (!std::isfinite(request.confidence) || request.confidence < limits_.minimum_confidence ||
    request.confidence > 1.0)
  {
    return {PolicyError::kInvalidRequest, "target confidence is outside the accepted range"};
  }
  if (!isFinite(request.target_position) || !isFinite(request.place_position) ||
    !isFinite(request.target_orientation) || !isFinite(request.place_orientation) ||
    !isFinite(request.object_size))
  {
    return {PolicyError::kInvalidRequest, "request contains a non-finite numeric value"};
  }
  if (request.object_size.x <= 0.0 || request.object_size.y <= 0.0 ||
    request.object_size.z <= 0.0 ||
    std::max({request.object_size.x, request.object_size.y, request.object_size.z}) >
    limits_.maximum_object_dimension)
  {
    return {PolicyError::kInvalidRequest, "object dimensions are invalid or exceed the limit"};
  }

  const auto target_norm = quaternionNorm(request.target_orientation);
  const auto place_norm = quaternionNorm(request.place_orientation);
  if (std::abs(target_norm - 1.0) > limits_.quaternion_norm_tolerance ||
    std::abs(place_norm - 1.0) > limits_.quaternion_norm_tolerance)
  {
    return {PolicyError::kInvalidRequest, "pose quaternion is not normalized"};
  }
  if (!objectFitsAt(request.target_position, request.object_size) ||
    !objectFitsAt(request.place_position, request.object_size))
  {
    return {PolicyError::kOutsideWorkspace, "target or place object bounds leave the workspace"};
  }
  if (request.execute_requested && !limits_.execution_enabled) {
    return {
      PolicyError::kExecutionDisabled,
      "execution was requested but execution_enabled is false"};
  }
  return {PolicyError::kNone, "request accepted"};
}

bool MotionPolicy::objectFitsAt(const Point3 & centre, const ObjectSize & size) const
{
  const auto & minimum = limits_.workspace.minimum;
  const auto & maximum = limits_.workspace.maximum;
  return centre.x - size.x / 2.0 >= minimum.x &&
         centre.y - size.y / 2.0 >= minimum.y &&
         centre.z - size.z / 2.0 >= minimum.z &&
         centre.x + size.x / 2.0 <= maximum.x &&
         centre.y + size.y / 2.0 <= maximum.y &&
         centre.z + size.z / 2.0 <= maximum.z;
}

}  // namespace vgm_motion
