#ifndef VGM_MOTION__MOTION_POLICY_HPP_
#define VGM_MOTION__MOTION_POLICY_HPP_

#include <array>
#include <string>

namespace vgm_motion
{

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct ObjectSize
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct AxisAlignedWorkspace
{
  Point3 minimum;
  Point3 maximum;
};

struct PolicyLimits
{
  AxisAlignedWorkspace workspace;
  std::string planning_frame;
  double minimum_confidence{0.65};
  double maximum_object_dimension{0.50};
  double quaternion_norm_tolerance{1.0e-2};
  bool execution_enabled{false};
};

struct PolicyRequest
{
  std::string object_id;
  std::string target_frame;
  std::string place_frame;
  double confidence{0.0};
  Point3 target_position;
  Quaternion target_orientation;
  ObjectSize object_size;
  Point3 place_position;
  Quaternion place_orientation;
  bool execute_requested{false};
};

enum class PolicyError
{
  kNone,
  kInvalidRequest,
  kOutsideWorkspace,
  kExecutionDisabled,
};

struct PolicyDecision
{
  PolicyError error{PolicyError::kNone};
  std::string reason;

  [[nodiscard]] bool accepted() const noexcept
  {
    return error == PolicyError::kNone;
  }
};

/// Pure, deterministic request policy. This class deliberately has no ROS or MoveIt dependency.
class MotionPolicy
{
public:
  explicit MotionPolicy(PolicyLimits limits);

  [[nodiscard]] PolicyDecision evaluate(const PolicyRequest & request) const;

private:
  [[nodiscard]] bool objectFitsAt(const Point3 & centre, const ObjectSize & size) const;

  PolicyLimits limits_;
};

}  // namespace vgm_motion

#endif  // VGM_MOTION__MOTION_POLICY_HPP_
