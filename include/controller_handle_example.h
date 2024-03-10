#pragma once

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <memory>
#include <moveit_ros_control_interface/ControllerHandle.h>

namespace example
{
class controller_handle_example : public moveit_controller_manager::MoveItControllerHandle
{
private:
  // Idle or done executing trajectory
  bool done_;

  // Connects to Action Server exposed by the controller
  std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> actionClient_;

public:
  controller_handle_example(const std::string& name)
  {
    std::string actionName = name + "/follow_joint_trajectory";

    actionClient_ =
        std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(actionName, true);

    actionClient_->waitForServer(ros::Duration(20.0));

    if (!actionClient_->isServerConnected())
    {
      ROS_ERROR_NAMED(getName().c_str(), "Action client failed to connect to %s", actionName.c_str());

      actionClient_.reset();
    }
  }

public:
  // MoveIt calls this method when it wants to send a trajectory goal to execute
  bool sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory) override
  {
    if (!actionClient_)
    {
      ROS_ERROR_NAMED(getName().c_str(), "Action client not connected, could not send trajectory");

      return false;
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory.joint_trajectory;

    actionClient_->sendGoal(
        goal,
        [this](const auto& state, const auto& result) {
          // Completed trajectory
          done_ = true;
        },
        [this] {
          // Beginning trajectory
        },
        [this](const auto& feedback) {
          // Trajectory feedback
        });

    done_ = false;

    return true;
  }

  // MoveIt calls this method when it wants a blocking call until done
  bool waitForExecution(const ros::Duration& timeout = ros::Duration(0)) override
  {
    if (actionClient_ && !done_)
      return actionClient_->waitForResult(ros::Duration(5.0));

    return true;
  }

  // MoveIt calls this method to get status updates
  moveit_controller_manager::ExecutionStatus getLastExecutionStatus() override
  {
    // comment
    return moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  }

  // MoveIt calls this method to abort trajectory goal execution
  bool cancelExecution() override
  {
    if (!actionClient_)
      return false;

    actionClient_->cancelGoal();
    done_ = true;

    return true;
  }
};
}  // namespace example