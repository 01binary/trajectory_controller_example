/*
    controller_handle_example.h

    Wraps a ROS Trajectory Controller into a MoveIt interface,
    to make it usable by MoveIt motion planning and execution pipeline
*/

#pragma once

#include <memory>
#include <moveit_ros_control_interface/ControllerHandle.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace example
{
    class controller_handle_example: public moveit_controller_manager::MoveItControllerHandle
    {
    private:
        // Idle or done executing trajectory
        bool done_;

        // Connects to Action Server we implemented in previous ROS controller example
        std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> actionClient_;

    public:
        controller_handle_example(const std::string& name, const std::string& action_ns);

    public:
        // MoveIt calls this method when it wants to send a trajectory goal to execute
        bool sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory) override;

        // MoveIt calls this method when it wants a blocking call that returns when done
        bool waitForExecution(const ros::Duration& timeout = ros::Duration(0)) override;

        // MoveIt calls this method to get status updates
        moveit_controller_manager::ExecutionStatus getLastExecutionStatus() override;

        // MoveIt calls this method to abort trajectory goal execution
        bool cancelExecution() override;
    };
}
