#include "vgm_motion/motion_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <utility>

#include <builtin_interfaces/msg/duration.hpp>
#include <moveit/robot_state/robot_state.hpp>

namespace vgm_motion
{
namespace
{

uint16_t toActionError(PolicyError error)
{
  switch (error) {
    case PolicyError::kNone:
      return vgm_interfaces::action::PickPlace::Result::NONE;
    case PolicyError::kInvalidRequest:
      return vgm_interfaces::action::PickPlace::Result::INVALID_REQUEST;
    case PolicyError::kOutsideWorkspace:
      return vgm_interfaces::action::PickPlace::Result::OUTSIDE_WORKSPACE;
    case PolicyError::kExecutionDisabled:
      return vgm_interfaces::action::PickPlace::Result::EXECUTION_DISABLED;
  }
  return vgm_interfaces::action::PickPlace::Result::INVALID_REQUEST;
}

builtin_interfaces::msg::Duration durationMessage(std::chrono::nanoseconds duration)
{
  builtin_interfaces::msg::Duration message;
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
  message.sec = static_cast<int32_t>(seconds.count());
  message.nanosec = static_cast<uint32_t>((duration - seconds).count());
  return message;
}

class ActiveGoalReset
{
public:
  explicit ActiveGoalReset(std::atomic_bool & active)
  : active_(active) {}

  ~ActiveGoalReset()
  {
    active_.store(false);
  }

private:
  std::atomic_bool & active_;
};

}  // namespace

MotionServer::MotionServer(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  const auto planning_group = node_->declare_parameter<std::string>("planning_group", "manipulator");
  planning_frame_ = node_->declare_parameter<std::string>("planning_frame", "world");
  end_effector_link_ =
    node_->declare_parameter<std::string>("end_effector_link", "tool0");
  const auto action_name = node_->declare_parameter<std::string>("action_name", "pick_place");
  const auto execution_enabled = node_->declare_parameter<bool>("execution_enabled", false);
  const auto minimum_confidence = node_->declare_parameter<double>("minimum_confidence", 0.65);
  const auto maximum_object_dimension =
    node_->declare_parameter<double>("maximum_object_dimension", 0.50);
  approach_distance_ = node_->declare_parameter<double>("approach_distance", 0.12);
  retreat_distance_ = node_->declare_parameter<double>("retreat_distance", 0.14);
  const auto planning_time = node_->declare_parameter<double>("planning_time", 5.0);
  const auto velocity_scaling = node_->declare_parameter<double>("velocity_scaling", 0.20);
  const auto acceleration_scaling =
    node_->declare_parameter<double>("acceleration_scaling", 0.20);
  const auto workspace_min = node_->declare_parameter<std::vector<double>>(
    "workspace.min", {-0.80, -0.80, 0.00});
  const auto workspace_max = node_->declare_parameter<std::vector<double>>(
    "workspace.max", {0.80, 0.80, 1.20});

  if (planning_group.empty() || planning_frame_.empty() || end_effector_link_.empty()) {
    throw std::invalid_argument(
            "planning_group, planning_frame, and end_effector_link must not be empty");
  }
  if (workspace_min.size() != 3U || workspace_max.size() != 3U) {
    throw std::invalid_argument("workspace.min and workspace.max must each contain three values");
  }
  if (!std::isfinite(approach_distance_) || !std::isfinite(retreat_distance_) ||
    approach_distance_ <= 0.0 || retreat_distance_ <= 0.0)
  {
    throw std::invalid_argument("approach_distance and retreat_distance must be positive");
  }
  if (!std::isfinite(planning_time) || planning_time <= 0.0 ||
    !std::isfinite(velocity_scaling) || !std::isfinite(acceleration_scaling))
  {
    throw std::invalid_argument("planning time and scaling factors must be finite");
  }

  PolicyLimits limits;
  limits.workspace = {
    {workspace_min[0], workspace_min[1], workspace_min[2]},
    {workspace_max[0], workspace_max[1], workspace_max[2]}};
  limits.planning_frame = planning_frame_;
  limits.minimum_confidence = minimum_confidence;
  limits.maximum_object_dimension = maximum_object_dimension;
  limits.execution_enabled = execution_enabled;
  policy_ = std::make_unique<MotionPolicy>(std::move(limits));

  move_group_ =
    std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, planning_group);
  move_group_->setPlanningTime(planning_time);
  move_group_->setMaxVelocityScalingFactor(std::clamp(velocity_scaling, 0.01, 1.0));
  move_group_->setMaxAccelerationScalingFactor(std::clamp(acceleration_scaling, 0.01, 1.0));
  move_group_->setPoseReferenceFrame(planning_frame_);
  if (!move_group_->setEndEffectorLink(end_effector_link_)) {
    throw std::invalid_argument("end_effector_link is not part of the configured robot model");
  }

  using std::placeholders::_1;
  using std::placeholders::_2;
  action_server_ = rclcpp_action::create_server<PickPlace>(
    node_, action_name,
    std::bind(&MotionServer::handleGoal, this, _1, _2),
    std::bind(&MotionServer::handleCancel, this, _1),
    std::bind(&MotionServer::handleAccepted, this, _1));

  RCLCPP_INFO(
    node_->get_logger(),
    "Motion server ready on '%s' (planning frame: %s, execution: %s)",
    action_name.c_str(), planning_frame_.c_str(), execution_enabled ? "enabled" : "disabled");
}

MotionServer::~MotionServer()
{
  move_group_->stop();
  std::lock_guard<std::mutex> lock(worker_mutex_);
  if (worker_.valid()) {
    worker_.wait();
  }
}

rclcpp_action::GoalResponse MotionServer::handleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const PickPlace::Goal>)
{
  bool expected = false;
  if (!goal_active_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN(node_->get_logger(), "Rejecting pick/place goal: another goal is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionServer::handleCancel(
  const std::shared_ptr<GoalHandle>)
{
  move_group_->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionServer::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  worker_ = std::async(
    std::launch::async, [this, goal_handle]() {runGoal(goal_handle);});
}

void MotionServer::runGoal(const std::shared_ptr<GoalHandle> goal_handle)
{
  ActiveGoalReset reset(goal_active_);
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<PickPlace::Result>();
  const auto planning_started = std::chrono::steady_clock::now();

  try {
    publishFeedback(
      goal_handle, PickPlace::Feedback::VALIDATING, 0.05F, "Validating request policy");
    const auto decision = policy_->evaluate(makePolicyRequest(*goal));
    if (!decision.accepted()) {
      result->error_code = toActionError(decision.error);
      result->message = decision.reason;
      goal_handle->abort(result);
      return;
    }
    if (goal_handle->is_canceling()) {
      result->error_code = PickPlace::Result::CANCELLED;
      result->message = "Goal cancelled before planning";
      goal_handle->canceled(result);
      return;
    }

    std::vector<PlannedStage> plans;
    std::string error_message;
    if (!planSequence(goal_handle, makePoseSequence(*goal), plans, error_message)) {
      result->planning_time = durationMessage(std::chrono::steady_clock::now() - planning_started);
      if (goal_handle->is_canceling()) {
        result->error_code = PickPlace::Result::CANCELLED;
        result->message = "Goal cancelled during planning";
        goal_handle->canceled(result);
      } else {
        result->error_code = PickPlace::Result::PLANNING_FAILED;
        result->message = error_message;
        goal_handle->abort(result);
      }
      return;
    }

    result->planning_time = durationMessage(std::chrono::steady_clock::now() - planning_started);
    publishFeedback(goal_handle, PickPlace::Feedback::READY, 0.72F, "All stages planned");

    if (goal_handle->is_canceling()) {
      result->error_code = PickPlace::Result::CANCELLED;
      result->message = "Goal cancelled after planning";
      goal_handle->canceled(result);
      return;
    }
    if (goal->execute && !executeSequence(goal_handle, plans, error_message)) {
      if (goal_handle->is_canceling()) {
        result->error_code = PickPlace::Result::CANCELLED;
        result->message = "Goal cancelled during execution";
        goal_handle->canceled(result);
      } else {
        result->error_code = PickPlace::Result::EXECUTION_FAILED;
        result->message = error_message;
        goal_handle->abort(result);
      }
      return;
    }

    result->success = true;
    result->error_code = PickPlace::Result::NONE;
    result->message = goal->execute ?
      "Arm trajectory executed; gripper actuation remains an integration boundary" :
      "Planning completed; no robot motion was executed";
    publishFeedback(goal_handle, PickPlace::Feedback::COMPLETE, 1.0F, result->message);
    goal_handle->succeed(result);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(node_->get_logger(), "Unhandled motion goal error: %s", exception.what());
    result->error_code = PickPlace::Result::PLANNING_FAILED;
    result->message = std::string("Motion server exception: ") + exception.what();
    result->planning_time = durationMessage(std::chrono::steady_clock::now() - planning_started);
    goal_handle->abort(result);
  }
}

PolicyRequest MotionServer::makePolicyRequest(const PickPlace::Goal & goal) const
{
  PolicyRequest request;
  request.object_id = goal.target.object_id;
  request.target_frame = goal.target.header.frame_id;
  request.place_frame = goal.place_pose.header.frame_id;
  request.confidence = goal.target.confidence;
  request.target_position = {
    goal.target.pose.pose.position.x,
    goal.target.pose.pose.position.y,
    goal.target.pose.pose.position.z};
  request.target_orientation = {
    goal.target.pose.pose.orientation.x,
    goal.target.pose.pose.orientation.y,
    goal.target.pose.pose.orientation.z,
    goal.target.pose.pose.orientation.w};
  request.object_size = {goal.target.size.x, goal.target.size.y, goal.target.size.z};
  request.place_position = {
    goal.place_pose.pose.position.x,
    goal.place_pose.pose.position.y,
    goal.place_pose.pose.position.z};
  request.place_orientation = {
    goal.place_pose.pose.orientation.x,
    goal.place_pose.pose.orientation.y,
    goal.place_pose.pose.orientation.z,
    goal.place_pose.pose.orientation.w};
  request.execute_requested = goal.execute;
  return request;
}

std::vector<MotionServer::NamedPose> MotionServer::makePoseSequence(
  const PickPlace::Goal & goal) const
{
  geometry_msgs::msg::PoseStamped pick_pose;
  pick_pose.header = goal.target.header;
  pick_pose.pose = goal.target.pose.pose;

  auto pre_pick = pick_pose;
  pre_pick.pose.position.z += approach_distance_;
  auto post_pick = pick_pose;
  post_pick.pose.position.z += retreat_distance_;
  auto pre_place = goal.place_pose;
  pre_place.pose.position.z += approach_distance_;
  auto post_place = goal.place_pose;
  post_place.pose.position.z += retreat_distance_;

  return {
    {"pre_pick", pre_pick},
    {"pick", pick_pose},
    {"post_pick", post_pick},
    {"pre_place", pre_place},
    {"place", goal.place_pose},
    {"post_place", post_place}};
}

bool MotionServer::planSequence(
  const std::shared_ptr<GoalHandle> & goal_handle,
  const std::vector<NamedPose> & targets,
  std::vector<PlannedStage> & plans,
  std::string & error_message)
{
  auto planned_state = move_group_->getCurrentState(5.0);
  if (!planned_state) {
    error_message = "Current robot state was unavailable";
    return false;
  }

  plans.clear();
  plans.reserve(targets.size());
  for (std::size_t index = 0; index < targets.size(); ++index) {
    if (goal_handle->is_canceling()) {
      error_message = "Planning cancelled";
      return false;
    }
    const auto & target = targets[index];
    const auto planning_stage = index < 3U ?
      PickPlace::Feedback::PLANNING_APPROACH : PickPlace::Feedback::PLANNING_TRANSFER;
    publishFeedback(
      goal_handle, planning_stage,
      0.10F + static_cast<float>(index) * 0.10F,
      "Planning stage: " + target.name);

    move_group_->setStartState(*planned_state);
    if (!move_group_->setPoseTarget(target.pose, end_effector_link_)) {
      error_message = "MoveIt rejected the pose target for stage '" + target.name + "'";
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_result = move_group_->plan(plan);
    move_group_->clearPoseTargets();
    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
      error_message = "MoveIt could not plan stage '" + target.name + "'";
      return false;
    }
    if (plan.trajectory_.joint_trajectory.points.empty()) {
      error_message = "MoveIt returned an empty trajectory for stage '" + target.name + "'";
      return false;
    }

    const auto & joint_names = plan.trajectory_.joint_trajectory.joint_names;
    const auto & final_positions = plan.trajectory_.joint_trajectory.points.back().positions;
    if (joint_names.size() != final_positions.size()) {
      error_message = "Trajectory joint metadata is inconsistent for stage '" + target.name + "'";
      return false;
    }
    planned_state->setVariablePositions(joint_names, final_positions);
    planned_state->update();
    plans.push_back({target.name, std::move(plan)});
  }
  return true;
}

bool MotionServer::executeSequence(
  const std::shared_ptr<GoalHandle> & goal_handle,
  const std::vector<PlannedStage> & plans,
  std::string & error_message)
{
  for (std::size_t index = 0; index < plans.size(); ++index) {
    if (goal_handle->is_canceling()) {
      move_group_->stop();
      error_message = "Execution cancelled";
      return false;
    }
    const auto & stage = plans[index];
    publishFeedback(
      goal_handle, PickPlace::Feedback::EXECUTING,
      0.75F + 0.20F * static_cast<float>(index + 1U) / static_cast<float>(plans.size()),
      "Executing stage: " + stage.name);
    if (move_group_->execute(stage.plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      error_message = "MoveIt failed while executing stage '" + stage.name + "'";
      return false;
    }
  }
  return true;
}

void MotionServer::publishFeedback(
  const std::shared_ptr<GoalHandle> & goal_handle,
  uint8_t stage,
  float progress,
  const std::string & detail) const
{
  auto feedback = std::make_shared<PickPlace::Feedback>();
  feedback->stage = stage;
  feedback->progress = std::clamp(progress, 0.0F, 1.0F);
  feedback->detail = detail;
  goal_handle->publish_feedback(feedback);
}

}  // namespace vgm_motion
