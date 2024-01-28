/*
    controller_handle_allocator_example.cpp

    Controller Handle Allocator implementation
*/

#include "controller_handle_example.h"
#include "controller_handle_allocator_example.h"

using namespace example;

moveit_controller_manager::MoveItControllerHandlePtr controller_handle_allocator_example::alloc(
  const std::string& name, const std::vector<std::string>& resources)
{
  return std::make_shared<controller_handle_example>(
    name, std::string("follow_joint_trajectory"));
}
