# MoveIt Controller Integration Example

This topic provides an example of a custom ROS Joint Trajectory Controller integrated with MoveIt! Simple Controller Manager, which is a default choice for new MoveIt Configuration packages created by MoveIt Setup Assistant.

MoveIt supports the following [ROS controllers](https://github.com/ros-controls/ros_controllers) out of the box:

- [JointTrajectoryController](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/include/joint_trajectory_controller/joint_trajectory_controller.h) (see [implementation](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/include/joint_trajectory_controller/joint_trajectory_controller_impl.h)) specialized for the three types of hardware interfaces supported by [ROS Control](https://wiki.ros.org/ros_control):

  - [velocity_controllers/JointTrajectoryController](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/src/joint_trajectory_controller.cpp#L53)
  - [position_controllers/JointTrajectoryController](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/src/joint_trajectory_controller.cpp#L36)
  - [effort_controllers/JointTrajectoryController](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/src/joint_trajectory_controller.cpp#L64)
- [GripperActionController](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/gripper_action_controller/include/gripper_action_controller/gripper_action_controller.h) (see [implementation](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/gripper_action_controller/include/gripper_action_controller/gripper_action_controller_impl.h)) specialized for the two types of hardware interfaces commonly used to control end effectors:
  - [position_controllers/GripperActionController](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/gripper_action_controller/src/gripper_action_controller.cpp#L42)
  - [effort_controllers/GripperActionController](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/gripper_action_controller/src/gripper_action_controller.cpp#L52)

These controllers are supported because MoveIt exports plugins that implement the `moveit_ros_control_interface::ControllerHandleAllocator` interface defined in [ControllerHandle.h](https://github.com/ros-planning/moveit/blob/master/moveit_plugins/moveit_ros_control_interface/include/moveit_ros_control_interface/ControllerHandle.h), which integrate these controllers with the MoveIt pipeline:

- `ControllerHandleAllocator`
    - When MoveIt is ready to execute a trajectory, the MoveIt Simple Controller Manager will look for a registered plugin that implements this interface, with a name that matches the ROS controller name. If such a plugin is found, MoveIt will call its `alloc` method, which is expected to return a `MoveItControllerHandle`.
- `MoveItControllerHandle`
    - Starts a ROS `actionlib::SimpleActionClient` for `control_msgs::FollowJointTrajectoryAction` and implements methods like `sendTrajectory`, `waitForExecution`, and `cancelExecution` which forward trajectory commands to the underlying ROS controller through this action client.

The following configuration is required to use a ROS controller with MoveIt Simple Controller Manager - either a stock controller like [JointTrajectoryController](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/include/joint_trajectory_controller/joint_trajectory_controller.h) which is supported by MoveIt out of the box, or a custom one that you write.

First, the `ros_controllers.yaml` file generated by MoveIt Setup Assistant configures ROS Controllers for your project:

```
yourcontroller:
  type: effort_controllers/JointTrajectoryController
  joints:
    - ...
  constraints:
    ...
  gains:
    ...

```

The `type` setting must specify the *type name* of a ROS controller registered with [pluginlib](https://wiki.ros.org/pluginlib), which implements a `controller_interface::Controller` from `ros_control`, templated by the type of hardware interface this controller can talk to:

* `hardware_interface::EffortJointInterface`
* `hardware_interface::PositionJointInterface`
* `hardware_interface::VelocityJointInterface`

The ROS controller is expected to start a `actionlib::ActionServer` that responds to requests defined by `control_msgs::FollowJointTrajectoryAction` to start or cancel a trajectory.

Next, the `simple_moveit_controllers.yaml` file also generated by MoveIt Setup Assistant maps ROS Controllers specified in `ros_controllers.yaml` to corresponding MoveIt wrappers called *controller handles* for integration with MoveIt Simple Controller Manager:

```
controller_list:
  - name: yourcontroller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: True
    joints:
      - ...
```

* `name`: the ROS controller to wrap with a MoveIt controller handle
* `action_ns` the name of the action server started by the ROS controller
* `type`: specifies how the MoveIt wrapper will communicate with this ROS controller:
  * `FollowJointTrajectory`: ROS controller will start an action server that will service [control_msgs::FollowJointTrajectoryAction](https://docs.ros.org/en/jade/api/control_msgs/html/action/FollowJointTrajectory.html) requests on `action_ns` namespace
  * `GripperCommand`: ROS controller will start an action server that will service [control_msgs::GripperCommand](https://docs.ros.org/en/jade/api/control_msgs/html/action/GripperCommand.html) requests on `action_ns` namespace

For each controller defined under `controller_list` in `simple_moveit_controllers.yaml`, MoveIt Simple Controller manager will look for a plugin registered with `pluginlib` that implements the `moveit_ros_control_interface::ControllerHandleAllocator` interface, and a name that matches the ROS controller name.
* If the plugin is not found, attempting to execute a trajectory will emit an error message: *Unable to identify any set of controllers that can actuate the specified joints*
* If the plugin is found and loaded successfully, the ROS controller should claim the specified joints in its `init` or `starting` methods by calling the `getHandle` method of the hardware interface it receives as a parameter to `init` (see code examples below).

## Example

Here's an example of how to implement a custom Joint Trajectory Controller that works with a *Velocity Joint Interface*, and integrates with MoveIt Simple Controller Manager.

> The steps to integrate a custom **Gripper Action Controller** with MoveIt are similar (the only difference is in the type of Action Server/Action Client), so following this example should help you implement either type of controller.

We will go through the following steps:

- Add package dependencies
- Implement a ROS Trajectory Controller
- Register the ROS Trajectory Controller with ROS Controller Manager
- Register the ROS Trajectory Controller with MoveIt Simple Controller Manager
- Configure the new ROS Controller
- Test the new ROS Controller with MoveIt

Once the steps are complete, you should be left with a boilerplate custom ROS Controller which can be loaded and used by MoveIt, both simulated in Gazebo and with real robot hardware.

### Add package dependencies

If your custom trajectory controller for MoveIt will be in its own ROS package:

```
# Change into your ROS workspace
cd ~/catkin_ws/src

# Create a new package with dependencies
catkin_create_pkg trajectory_controller_example \
roscpp \
actionlib \
pluginlib \
moveit_ros_control_interface \
controller_interface \
control_msgs
```

Otherwise, add the above dependencies to your package:

* `roscpp`: because you are using ROS C++ interface
* `pluginlib`: because you are registering a ROS Controller plugin and a MoveIt Controller wrapper plugin
* `controller_interface`: because you are exposing your controller to ROS Controller Manager
* `moveit_ros_control_interface`: because you are exposing your controller to MoveIt Controller Manager
* `actionlib`: because your controller needs an Action Server that will receive trajectory commands
* `control_msgs`: includes the trajectory command message

Install any missing packages:

```
cd ~/catkin_ws/src
rosdep install -y --from-paths . --ignore-src --rosdistro $ROS_DISTRO
```

Build your workspace to ensure everything is working up to this point.

### Implement a ROS Trajectory Controller

Create `trajectory_controller_example.h` in the include directory of your package:

```
/*
    trajectory_controller_example.h

    Custom ROS Joint Trajectory Controller that can be used with MoveIt
    This example specialized for VelocityJointInterface

    Can be specialized for:
    - PositionJointInterface if you are using servos or stepper motors, etc
    - EffortJointInterface if you are using BDLC servos in MIT mode, etc
    - VelocityJointInterface if you are using PWM-controlled DC motors, etc
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
```

Create `trajectory_controller_example.cpp` in the source directory of your package:

```
/*
    trajectory_controller_example.cpp
    Implementing custom ROS Joint Trajectory Controller for MoveIt
*/

#include "trajectory_controller_example.h"

using namespace example;

bool trajectory_controller_example::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& controllerManagerNode, ros::NodeHandle& node)
{
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
```

Make the necessary changes to `CMakeLists.txt` to get this to compile:

* Uncomment the `catkin_package` macro to activate include directories, libraries, and dependencies:

```
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_controller_example
  CATKIN_DEPENDS actionlib control_msgs control_msgs controller_interface moveit_ros_control_interface pluginlib roscpp
)
```

* Uncomment the `include_directories` macro so that the header you created in your package include directory can be found along with all other headers from ROS Controllers, Move It, etc:

```
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
```

* Uncomment the `add_library` macro to specify source files for building a library that contains your ROS controller, so that it can be loaded by the plugin system:

```
add_library(${PROJECT_NAME}
   src/trajectory_controller_example.cpp
)
```

* Uncomment the `target_link_libraries` macro to link the libraries you specified as package dependencies into your library:

```
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)
```

Build the workspace to verify your package compiles successfully. If you get any errors, compare your work with the [trajectory_controller_example](https://github.com/01binary/trajectory_controller_example) repository.

### Register the ROS Trajectory Controller with ROS Controller Manager

The next step is to register the new ROS controller with `pluginlib` so that it can be loaded by ROS Controller Manager.

This requires including the following header at the top of the `.h` or the `.cpp` file:

```
#include <pluginlib/class_list_macros.h>
```

Then the `PLUGINLIB_EXPORT_CLASS` macro defined in that header must be called after your class has been defined, at the end of the `.h` or the `.cpp` file:

```
PLUGINLIB_EXPORT_CLASS(example::trajectory_controller_example, controller_interface::ControllerBase);
```

Next, create a plugin description file. In this example we called it `trajectory_controller_plugin.xml` and placed it at the root of the package:

```
<library path="lib/libtrajectory_controller_example">
  <class
    name="example/trajectory_controller_example"
    type="example::trajectory_controller_example"
    base_class_type="controller_interface::ControllerBase"
  >
    <description>
      Your custom trajectory controller description
    </description>
  </class>
</library>
```

Next, your `package.xml` must be updated to export the new controller plugin. If you created a new package with `catkin_create_pkg`, the `<export>` section will already be there. Add a `controller_interface` entry with a `plugin` attribute pointing to your plugin description file:

```
<!-- The export tag contains other, unspecified, tags -->
<export>
  <!-- Other tools can request additional information be placed here -->
  <controller_interface plugin="${prefix}/trajectory_controller_plugin.xml"/>
</export>
```

Build your workspace again to ensure the changes have been made correctly.

Now your controller can be listed and loaded by the ROS Controller Manager. Try editing `ros_controllers.yaml` for a package generated by MoveIt Setup Assistant and specifying the type name of your custom controller instead of the stock Joint Trajectory Controller exported by ROS:

```
your_controller:
  type: example/trajectory_controller_example
  joints:
    ...
  gains:
    ...
```

Launching the Gazebo simulation at this point should successfully find and load your new controller:

```
roslaunch your_moveit_config demo_gazebo.launch
```

If you haven't created a MoveIt Configuration package for your own project yet, you can try the code in this example with a mock robot arm defined by these two packages:

* [basic_velocity_control](https://github.com/01binary/basic_velocity_control)
* [basic_velocity_control_moveit_config](https://github.com/01binary/basic_velocity_control_moveit_config)

In that case, after cloning both packages into `~/catkin_ws/src`, you can modify `ros_controllers.yaml` in `basic_velocity_control_moveit_config` package and launch Gazebo with RViz with this command:

```
roslaunch basic_velocity_control_moveit_config demo_gazebo.launch
```

If your controller library was not compiled or linked correctly, the `gzserver` ROS node that hosts the ROS Controller Manager will likely crash at startup while trying to load your controller. Compare your work with the working example in [trajectory_controller_example](https://github.com/01binary/trajectory_controller_example) repository if necessary.

You can verify your new controller loads by adding a `ROS_INFO()` call to log a message in the constructor. However, at this point, attempting to execute a trajectory with MoveIt will emit the following error:

```
Unable to identify any set of controllers that can actuate the specified joints: ...
```

This error is emitted because MoveIt Simple Controller Manager attempts to find a registered plugin with the same name as your ROS controller (i.e. `example/trajectory_controller_example`) but a different base class (`moveit_ros_control_interface::ControllerHandleAllocator`).

In the next section, will create another plugin description file to export a trajectory controller wrapper class already implemented by MoveIt under a different name (the name of your custom ROS controller) so that it can be found and loaded. As long as your controller starts an action server on the right namespace, the existing MoveIt wrapper used for the standard ROS joint trajectory controller will work with your custom controller as well.

### Register the ROS Trajectory Controller with MoveIt Simple Controller Manager

Create a plugin definition file `trajectory_controller_moveit_plugin.xml` at the root of the package:

```
<library path="libmoveit_ros_control_interface_trajectory_plugin">
  <class
    name="example/trajectory_controller_example"
    type="moveit_ros_control_interface::JointTrajectoryControllerAllocator"
    base_class_type="moveit_ros_control_interface::ControllerHandleAllocator"
  >
    <description>
      Joint Trajectory Controller Example Exported for MoveIt!
    </description>
  </class>
</library>
```

> Notice that the `name` of the exported class doesn't match the type. Instead the name matches the name of the ROS Controller being used (our custom trajectory controller).

Reference the new plugin definition in the package manifest file `package.xml` by placing it below the `<controller_interface ... />` entry created earlier:

```
<!-- The export tag contains other, unspecified, tags -->
<export>
  <!-- Other tools can request additional information be placed here -->
  <controller_interface plugin="${prefix}/trajectory_controller_plugin.xml"/>
  <moveit_ros_control_interface plugin="${prefix}/controller_handle_allocator_plugin.xml"/>
</export>
```

In this case, the entry tag is `moveit_ros_control_interface` because that's the name of the package that contains the interface `moveit_ros_control_interface::ControllerHandleAllocator` we are implementing.

Build your workspace again and ensure there are no errors.

### Configure the new ROS Controller

The ROS controller we created in this example should now be ready for use in the MoveIt pipeline.

Create a new MoveIt configuration package by using the MoveIt Setup Assistant, or use an example MoveIt Config package such as `basic_velocity_control`:

* [basic_velocity_control](https://github.com/01binary/basic_velocity_control)
* [basic_velocity_control_moveit_config](https://github.com/01binary/basic_velocity_control_moveit_config).

Modify the `ros_controllers.yaml` configuration created by MoveIt Setup Assistant to substitute the custom controller created in this tutorial for the stock ROS joint trajectory controller:

```
arm_controller:
  type: example/trajectory_controller_example
  joints:
    - shoulder_joint
    - upperarm_joint
    - forearm_joint
  gains:
    shoulder_joint:
      p: 10
      d: 1
      i: 1
      i_clamp: 1
    upperarm_joint:
      p: 10
      d: 1
      i: 1
      i_clamp: 1
    forearm_joint:
      p: 10
      d: 1
      i: 1
      i_clamp: 1
```

The configuration in `simple_moveit_controllers.yaml` doesn't need to be modified since our custom controller uses exactly the same Action Server interface as the stock ROS controller (`FollowJointTrajectory`) and also runs its Action Server on the same namespace (`follow_joint_trajectory`).

Launch the Gazebo simulation with RViz by using the `demo_gazebo.launch` file generated by MoveIt Setup Assistant. For example:

```
roslaunch basic_velocity_control_moveit_config demo_gazebo.launch
```

The ROS controller should be loaded at startup and then the MoveIt wrapper will be activated once you move the joints in RViz and click **Plan & Execute**.
