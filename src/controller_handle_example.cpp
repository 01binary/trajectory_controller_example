/*
    controller_handle_example.cpp

    MoveIt Controller Handle Example
*/

#include "controller_handle_example.h"

using namespace example;

controller_handle_example::controller_handle_example(const std::string& name, const std::string& action_ns)
  : moveit_controller_manager::MoveItControllerHandle(name)
  , done_(true)
{
  std::string actionName = name + "/" + action_ns;

  actionClient_ = std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(
    actionName, true);

  actionClient_->waitForServer(ros::Duration(20.0));

  if (!actionClient_->isServerConnected())
  {
    ROS_ERROR_NAMED(
      getName().c_str(),
      "Action client failed to connect to %s",
      actionName.c_str());

    actionClient_.reset();
  }
}

bool controller_handle_example::sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory)
{
  if (!actionClient_)
  {
    ROS_ERROR_NAMED(
      getName().c_str(),
      "Action client not connected, could not send trajectory");

    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory.joint_trajectory;

  actionClient_->sendGoal(
    goal,
    [this](const auto& state, const auto& result)
    {
      ROS_DEBUG_NAMED(getName().c_str(), "Trajectory Controller Completed Trajectory");
      done_ = true;
    },
    [this]
    {
      ROS_DEBUG_NAMED(getName().c_str(), "Trajectory Controller Beginning Trajectory");
    },
    [this](const auto& feedback)
    {
      ROS_DEBUG_NAMED(getName().c_str(), "Trajectory Controller Received Feedback");
    });

  done_ = false;

  return true;
}

bool controller_handle_example::waitForExecution(const ros::Duration& timeout)
{
  if (actionClient_ && !done_)
    return actionClient_->waitForResult(ros::Duration(5.0));

  return true;
}

moveit_controller_manager::ExecutionStatus controller_handle_example::getLastExecutionStatus()
{
  return moveit_controller_manager::ExecutionStatus::SUCCEEDED;
}

bool controller_handle_example::cancelExecution()
{
  if (!actionClient_)
    return false;

  actionClient_->cancelGoal();
  done_ = true;

  return true;
}
