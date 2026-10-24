#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "vgm_motion/motion_policy.hpp"

namespace vgm_motion
{
namespace
{

PolicyLimits defaultLimits(bool execution_enabled = false)
{
  PolicyLimits limits;
  limits.workspace = {{-0.8, -0.8, 0.0}, {0.8, 0.8, 1.2}};
  limits.planning_frame = "world";
  limits.minimum_confidence = 0.65;
  limits.maximum_object_dimension = 0.5;
  limits.execution_enabled = execution_enabled;
  return limits;
}

PolicyRequest validRequest()
{
  PolicyRequest request;
  request.object_id = "part-042";
  request.target_frame = "world";
  request.place_frame = "world";
  request.confidence = 0.92;
  request.target_position = {0.4, 0.1, 0.2};
  request.target_orientation = {0.0, 0.0, 0.0, 1.0};
  request.object_size = {0.06, 0.04, 0.10};
  request.place_position = {0.3, -0.3, 0.2};
  request.place_orientation = {0.0, 0.0, 0.0, 1.0};
  return request;
}

TEST(MotionPolicy, AcceptsValidPlanningOnlyRequest)
{
  const MotionPolicy policy(defaultLimits());
  const auto decision = policy.evaluate(validRequest());
  EXPECT_TRUE(decision.accepted());
  EXPECT_EQ(decision.error, PolicyError::kNone);
}

TEST(MotionPolicy, RejectsMissingObjectIdentifier)
{
  const MotionPolicy policy(defaultLimits());
  auto request = validRequest();
  request.object_id.clear();
  EXPECT_EQ(policy.evaluate(request).error, PolicyError::kInvalidRequest);
}

TEST(MotionPolicy, RejectsUntransformedFrame)
{
  const MotionPolicy policy(defaultLimits());
  auto request = validRequest();
  request.target_frame = "camera_color_optical_frame";
  EXPECT_EQ(policy.evaluate(request).error, PolicyError::kInvalidRequest);
}

TEST(MotionPolicy, RejectsLowConfidenceDetection)
{
  const MotionPolicy policy(defaultLimits());
  auto request = validRequest();
  request.confidence = 0.64;
  EXPECT_EQ(policy.evaluate(request).error, PolicyError::kInvalidRequest);
}

TEST(MotionPolicy, RejectsNonFiniteInput)
{
  const MotionPolicy policy(defaultLimits());
  auto request = validRequest();
  request.target_position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(policy.evaluate(request).error, PolicyError::kInvalidRequest);
}

TEST(MotionPolicy, RejectsNonNormalizedQuaternion)
{
  const MotionPolicy policy(defaultLimits());
  auto request = validRequest();
  request.place_orientation.w = 0.5;
  EXPECT_EQ(policy.evaluate(request).error, PolicyError::kInvalidRequest);
}

TEST(MotionPolicy, AccountsForObjectExtentsAtWorkspaceBoundary)
{
  const MotionPolicy policy(defaultLimits());
  auto request = validRequest();
  request.target_position.x = 0.79;
  EXPECT_EQ(policy.evaluate(request).error, PolicyError::kOutsideWorkspace);
}

TEST(MotionPolicy, RejectsExecutionUnlessServerGateIsEnabled)
{
  const MotionPolicy policy(defaultLimits(false));
  auto request = validRequest();
  request.execute_requested = true;
  EXPECT_EQ(policy.evaluate(request).error, PolicyError::kExecutionDisabled);
}

TEST(MotionPolicy, AcceptsExecutionWhenBothGatesAllowIt)
{
  const MotionPolicy policy(defaultLimits(true));
  auto request = validRequest();
  request.execute_requested = true;
  EXPECT_TRUE(policy.evaluate(request).accepted());
}

TEST(MotionPolicy, RejectsInvalidWorkspaceConfiguration)
{
  auto limits = defaultLimits();
  limits.workspace.maximum.x = limits.workspace.minimum.x;
  EXPECT_THROW(MotionPolicy policy(limits), std::invalid_argument);
}

}  // namespace
}  // namespace vgm_motion
