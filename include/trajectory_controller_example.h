/*
    trajectory_controller_example.h

    Custom ROS Trajectory Controller for MoveIt

    Can be specialized for:
    - VelocityJointInterface if you are using PWM-controlled DC motors, etc
    - EffortJointInterface if you are using BDLC servos in MIT mode, etc
    - PositionJointInterface if you are using servos or stepper motors, etc
*/

#pragma once

#include <string>
#include <vector>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <controller_interface/controller.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace example
{
    class trajectory_controller_example: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
    {
    private:
        // Joints claimed by this controller
        std::vector<std::string> joint_names_;

        // Hardware interface for reading joint positions and sending commands
        hardware_interface::VelocityJointInterface* hw_;

        // Action server for receiving trajectory commands from MoveIt
        std::shared_ptr<actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>> actionServer_;

        // Current goal to execute
        actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_;

    public:
        bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& controllerManagerNode, ros::NodeHandle& node);
        void starting(const ros::Time& time);
        void stopping(const ros::Time&);
        void update(const ros::Time& time, const ros::Duration& period);
        void trajectoryCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal);
        void trajectoryCancelCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal);
    };
}

PLUGINLIB_EXPORT_CLASS(example::trajectory_controller_example, controller_interface::ControllerBase);
