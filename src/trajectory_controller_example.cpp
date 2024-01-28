/*
    trajectory_controller_example.cpp
    Custom ROS Trajectory Controller for MoveIt
*/

#include "trajectory_controller_example.h"

using namespace example;

bool trajectory_controller_example::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& controllerManagerNode, ros::NodeHandle& node)
{
    ROS_INFO("trajectory_controller_example::init");
    hw_ = hw;
    node.getParam("joints", joint_names_);

    // start the action server that our MoveIt wrapper will use to communicate with this controller
    actionServer_.reset(new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>(
        node,
        "follow_joint_trajectory",
        std::bind(&trajectory_controller_example::trajectoryCallback, this, std::placeholders::_1),
        std::bind(&trajectory_controller_example::trajectoryCancelCallback, this, std::placeholders::_1),
        false
    ));

    actionServer_->start();

    // call getHandle() for each joint for the first time to "claim" them
    for (const std::string& joint_name: joint_names_)
    {
        hw_->getHandle(joint_name);
    }

    return true;
}

void trajectory_controller_example::starting(const ros::Time& time)
{
    // start or re-start after initialization
}

void trajectory_controller_example::stopping(const ros::Time&)
{
    // stop (should abort trajectory execution and notify MoveIt we aborted)
    if (goal_.isValid()) goal_.setAborted();
}

void trajectory_controller_example::update(const ros::Time& time, const ros::Duration& period)
{
    // execute current trajectory if any...
    if (goal_.isValid())
    {
        // for each controlled joint
        for (const std::string& joint_name: joint_names_)
        {
            // get the joint hardware interface
            auto handle = hw_->getHandle(joint_name);

            // get current joint state from hardware
            double position = handle.getPosition();

            // send joint command to hardware
            handle.setCommand(0.0);
        }

        // if done executing, notify MoveIt goal has succeeded
        // goal_.setSucceeded();

        // or notify that it failed because of timeout, etc
        // goal_.setAborted();
    }
}

void trajectory_controller_example::trajectoryCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal)
{
    if (!this->isRunning())
    {
        // notify MoveIt if the goal was rejected
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
        goal.setRejected(result);
    }
    else
    {
        // notify MoveIt we accepted goal and start executing it
        goal.setAccepted();
        ROS_INFO("we got it!");
        // store goal handle so we can call goal_.setSucceeded() when done
        goal_ = goal;
        // goal trajectory contains waypoints to execute, specifies joint states for each way point
        const trajectory_msgs::JointTrajectory trajectory = goal.getGoal()->trajectory;
        // use trajectory...
    }
}

void trajectory_controller_example::trajectoryCancelCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal)
{
    // notify MoveIt we canceled the goal
    goal.setCanceled();
}
