#ifndef VGM_MOTION__MOTION_SERVER_HPP_
#define VGM_MOTION__MOTION_SERVER_HPP_

#include <atomic>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <vgm_interfaces/action/pick_place.hpp>

#include "vgm_motion/motion_policy.hpp"

namespace vgm_motion
{

class MotionServer
{
public:
  using PickPlace = vgm_interfaces::action::PickPlace;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PickPlace>;

  explicit MotionServer(const rclcpp::Node::SharedPtr & node);
  ~MotionServer();

  MotionServer(const MotionServer &) = delete;
  MotionServer & operator=(const MotionServer &) = delete;

private:
  struct NamedPose
  {
    std::string name;
    geometry_msgs::msg::PoseStamped pose;
  };

  struct PlannedStage
  {
    std::string name;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
  };

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickPlace::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
  void runGoal(const std::shared_ptr<GoalHandle> goal_handle);

  [[nodiscard]] PolicyRequest makePolicyRequest(const PickPlace::Goal & goal) const;
  [[nodiscard]] std::vector<NamedPose> makePoseSequence(const PickPlace::Goal & goal) const;
  [[nodiscard]] bool planSequence(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::vector<NamedPose> & targets,
    std::vector<PlannedStage> & plans,
    std::string & error_message);
  [[nodiscard]] bool executeSequence(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::vector<PlannedStage> & plans,
    std::string & error_message);

  void publishFeedback(
    const std::shared_ptr<GoalHandle> & goal_handle,
    uint8_t stage,
    float progress,
    const std::string & detail) const;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<MotionPolicy> policy_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Server<PickPlace>::SharedPtr action_server_;

  std::string planning_frame_;
  std::string end_effector_link_;
  double approach_distance_{0.12};
  double retreat_distance_{0.14};
  std::atomic_bool goal_active_{false};
  std::mutex worker_mutex_;
  std::future<void> worker_;
};

}  // namespace vgm_motion

#endif  // VGM_MOTION__MOTION_SERVER_HPP_
