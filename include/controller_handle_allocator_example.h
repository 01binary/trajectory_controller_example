#pragma once

#include "controller_handle_example.h"
#include <pluginlib/class_list_macros.h>

namespace example
{
class controller_handle_allocator_example : public moveit_ros_control_interface::ControllerHandleAllocator
{
public:
  moveit_controller_manager::MoveItControllerHandlePtr alloc(const std::string& name,
                                                             const std::vector<std::string>& resources) override
  {
    return std::make_shared<controller_handle_example>(name, std::string("follow_joint_trajectory"));
  }
};
}  // namespace example

PLUGINLIB_EXPORT_CLASS(example::controller_handle_allocator_example,
                       moveit_ros_control_interface::ControllerHandleAllocator);
