# Integrating Hardware Controllers with MoveIt!

There are a few paths for integrating a controller that moves robot joints with the MoveIt! framework, which accomodate simple usage scenarios all the way to advanced customization:

* Stock controllers [JointTrajectoryController](http://wiki.ros.org/joint_trajectory_controller) and [GripperActionController](http://wiki.ros.org/gripper_action_controller) from [ROS controllers](http://wiki.ros.org/ros_controllers) package are supported out of the box because MoveIt exports integration plugins that bridge them with MoveIt motion planning pipeline.
* Any other controllers that can be loaded by the [ROS Controller Manager](http://wiki.ros.org/controller_manager) can be used by linking them with an existing MoveIt integration plugin, as long as they support [Follow Joint Trajectory Action](https://docs.ros.org/en/noetic/api/control_msgs/html/action/FollowJointTrajectory.html) or [Gripper Command Action](https://docs.ros.org/en/noetic/api/control_msgs/html/action/GripperCommand.html) interfaces.
* Custom controllers that can be loaded by the [ROS Controller Manager](http://wiki.ros.org/controller_manager) but don't support [Follow Joint Trajectory Action](https://docs.ros.org/en/noetic/api/control_msgs/html/action/FollowJointTrajectory.html) or [Gripper Command Action](https://docs.ros.org/en/noetic/api/control_msgs/html/action/GripperCommand.html) can be bridged with MoveIt by writing a MoveIt integration plugin, as long as they can be made to fit into the [MoveIt Controller Handle](https://docs.ros.org/en/noetic/api/moveit_core/html/classmoveit__controller__manager_1_1MoveItControllerHandle.html) interface.
* Custom controllers that cannot be recognized and loaded by the [ROS Controller Manager](http://wiki.ros.org/controller_manager), or for which the [MoveIt Controller Handle](https://docs.ros.org/en/noetic/api/moveit_core/html/classmoveit__controller__manager_1_1MoveItControllerHandle.html) is a poor fit, can be integrated by writing a custom MoveIt Controller Manager.

We will look at each of these options in more detail.

## Stock ROS Controllers

The [JointTrajectoryController](http://wiki.ros.org/joint_trajectory_controller) and [GripperActionController](http://wiki.ros.org/gripper_action_controller) from [ROS controllers](http://wiki.ros.org/ros_controllers) package are supported out of the box for simple usage scenarios, and can be easily configured by using the [MoveIt Setup Assistant](https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html) (*MSA*) on the *Controllers* page.

These controllers are integrated with MoveIt by using existing [MoveIt Controller Handles](https://docs.ros.org/en/noetic/api/moveit_core/html/classmoveit__controller__manager_1_1MoveItControllerHandle.html):

* Stock [JointTrajectoryController](http://wiki.ros.org/joint_trajectory_controller) is integrated through [Joint Trajectory Controller Handle](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_plugins/moveit_simple_controller_manager/include/moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h).
* Stock [GripperActionController](http://wiki.ros.org/gripper_action_controller) is integrated through [Gripper Controller Handle](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_plugins/moveit_simple_controller_manager/include/moveit_simple_controller_manager/gripper_controller_handle.h).

The [ROS Controller Manager](http://wiki.ros.org/controller_manager) recognizes and loads these controllers from `ros_controllers.yaml` configuration file created by MSA because they are [exported](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/ros_control_plugins.xml) as plugins by [ROS Controllers](http://wiki.ros.org/ros_controllers) package using their `type` names:

```
arm_controller:
  type: velocity_controllers/JointTrajectoryController
  joints:
    - shoulder_joint
    - upperarm_joint
    - forearm_joint
  gains:
    shoulder_joint:
      p: 100
      d: 1
      i: 1
      i_clamp: 1
    upperarm_joint:
      p: 100
      d: 1
      i: 1
      i_clamp: 1
    forearm_joint:
      p: 100
      d: 1
      i: 1
      i_clamp: 1
gripper_controller:
  type: position_controllers/GripperActionController
  joint: gripper
```

The *MoveIt Simple Controller Manager* configured by MSA [as the default](https://github.com/ros-planning/moveit/blob/master/moveit_setup_assistant/templates/moveit_config_pkg_template/launch/move_group.launch#L17) will load the *handles* that bridge these controllers with MoveIt by reading the `simple_moveit_controllers.yaml` configuration file, for example:

```
controller_list:
  - name: arm_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: True
    joints:
      - shoulder_joint
      - upperarm_joint
      - forearm_joint
  - name: gripper_controller
    action_ns: gripper_cmd
    type: GripperCommand
    default: True
    joints:
      - gripper
```

The mapping from ROS controller `name` to MoveIt *controller handle* `type` is done by using pre-defined *simple integration types*:

* `FollowJointTrajectory`: specifying this integration type will bridge the ROS controller specified by the `name` setting to the MoveIt pipeline through the [Joint Trajectory Controller Handle](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_plugins/moveit_simple_controller_manager/include/moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h).
* `GripperCommand`: specifying this integration type will bridge the ROS controller specified by the `name` setting to the MoveIt pipeline through the [Gripper Controller Handle](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_plugins/moveit_simple_controller_manager/include/moveit_simple_controller_manager/gripper_controller_handle.h).

In both cases, `action_ns` specifies the Action Server name exposed by the ROS controller. The full topic name consists of the ROS controller `name` and the `action_ns`. If you were to list topics by using `rostopic list` with the above two ROS controllers loaded, you would see something like the following:

```
/arm_controller/command
/arm_controller/follow_joint_trajectory/cancel
/arm_controller/follow_joint_trajectory/feedback
/arm_controller/follow_joint_trajectory/goal
/arm_controller/follow_joint_trajectory/result
/arm_controller/follow_joint_trajectory/status
/arm_controller/gains/forearm_joint/parameter_descriptions
/arm_controller/gains/forearm_joint/parameter_updates
/arm_controller/gains/shoulder_joint/parameter_descriptions
/arm_controller/gains/shoulder_joint/parameter_updates
/arm_controller/gains/upperarm_joint/parameter_descriptions
/arm_controller/gains/upperarm_joint/parameter_updates
/arm_controller/state
/gripper_controller/gripper_cmd/cancel
/gripper_controller/gripper_cmd/feedback
/gripper_controller/gripper_cmd/goal
/gripper_controller/gripper_cmd/result
/gripper_controller/gripper_cmd/status
```

To test simple controller integration with *MoveIt Simple Controller Manager*, launch the package generanted by MSA by using the `move_group.launch` file. This will load your robot description and the MoveIt motion planning pipeline hosted in `move_group` node from `moveit_ros_move_group` package.

> This launch file assumes that your robot's [hardware interface](http://wiki.ros.org/ros_control/Tutorials/Create%20your%20own%20hardware%20interface) is already running, since any ROS controllers you use will attempt to connect to this interface and send commands. It does not not include any [visualization](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) and does not [simulate the hardware interface](https://classic.gazebosim.org/tutorials?tut=ros_control&cat=connect_ros). In the absence of visualization and/or simulation tools, you can use the [C++](https://moveit.picknik.ai/main/doc/examples/moveit_cpp/moveitcpp_tutorial.html), [Python](https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html), or [Command Line](https://ros-planning.github.io/moveit_tutorials/doc/moveit_commander_scripting/moveit_commander_scripting_tutorial.html) interface to interact with MoveIt.

## Other ROS Controllers

The *MoveIt ROS Control Controller Manager* which is the default configured by MSA for visualizing and/or simulating the robot does not use the configuration in `simple_moveit_controllers.yaml` to discover controllers, and instead queries [ROS Controller Manager](http://wiki.ros.org/controller_manager) for loaded and active controllers.

Since this discovery process does not use the pre-defined types `FollowJointTrajectory` and `GripperCommand` (only supported by *Simple Controller Manager*), [Controller Handle Allocator](https://github.com/ros-planning/moveit/blob/master/moveit_plugins/moveit_ros_control_interface/include/moveit_ros_control_interface/ControllerHandle.h) plugins also need to be exported for each controller used in this fashion in order to link ROS controllers to MoveIt *Controller Handles*.

> Note: while [JointTrajectoryController](http://wiki.ros.org/joint_trajectory_controller) from [ROS controllers](http://wiki.ros.org/ros_controllers) is supported by *MoveIt ROS Control Controller Manager* out of the box because its [Controller Handle Allocator](https://github.com/ros-planning/moveit/blob/master/moveit_plugins/moveit_ros_control_interface/src/joint_trajectory_controller_plugin.cpp) is [exported](https://github.com/ros-planning/moveit/blob/master/moveit_plugins/moveit_ros_control_interface/moveit_ros_control_interface_plugins.xml) as a plugin, the [GripperActionController](http://wiki.ros.org/gripper_action_controller) is not, because it's only needed for simple usage scenarios. Even though it has a *Controller Handle*, it does not implement or export the corresponding *Controller Handle Allocator* plugin that enables the controller handle to be dynamically created from ROS controller type name. The next few sections will explain how to implement and export such a plugin.

Any other ROS controllers can be used after linking them with *controller handle allocator* plugins exported by MoveIt, as long as they support the [Follow Joint Trajectory Action](https://docs.ros.org/en/noetic/api/control_msgs/html/action/FollowJointTrajectory.html) interface.

*Controller Handles* bridge ROS Controllers with MoveIt motion planning pipeline by means of an [Action Client](http://wiki.ros.org/actionlib), as long as the controller starts an *Action Server* that handles one of the two types of supported action interfaces:

* The [Joint Trajectory Controller Handle](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_plugins/moveit_simple_controller_manager/include/moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h) plugin can be used for controllers that support [Follow Joint Trajectory Action](https://docs.ros.org/en/noetic/api/control_msgs/html/action/FollowJointTrajectory.html).
* The [Gripper Controller Handle](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_plugins/moveit_simple_controller_manager/include/moveit_simple_controller_manager/gripper_controller_handle.h) plugin can be used for controllers that support [Gripper Command Action](https://docs.ros.org/en/jade/api/control_msgs/html/action/GripperCommand.html).

The *MoveIt ROS Control Controller Manager* can be configured by changing the `moveit_controller_manager` setting to `ros_control`. The MoveIt configuration package auto-generated by MSA includes the [demo_gazebo.launch](https://github.com/ros-planning/moveit/blob/master/moveit_setup_assistant/templates/moveit_config_pkg_template/launch/demo_gazebo.launch#L19) file that configures this manager type to launch [Gazebo](https://classic.gazebosim.org/tutorials?tut=ros_control&cat=connect_ros) simulation and visualize the robot state in [RViz](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html).

The *MoveIt ROS Control Controller Manager* will regard any controllers loaded by ROS Controller Manager as *managed* if it finds a plugin registration that links the `type` of the ROS controller with a MoveIt Controller Handle Allocator. If no such registration is found, the controller is regarded as *unmanaged* (merely *active*) and cannot be used to receive trajectory commands from MoveIt.

For example, see the stock Joint Trajectory Controller [plugin registration](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_plugins/moveit_ros_control_interface/moveit_ros_control_interface_plugins.xml), which links several variations of this controller exported from `ros_controllers` package with the corresponding MoveIt Controller Handle that supports [Follow Joint Trajectory Action](https://docs.ros.org/en/noetic/api/control_msgs/html/action/FollowJointTrajectory.html) via an exported MoveIt *Controller Handle Allocator* plugin.

The same pattern can be followed to link any other ROS controller with a MoveIt *Controller Handle* so that it can receive trajectory commands.

First, create a plugin description file:

```
<library path="libmoveit_ros_control_interface_trajectory_plugin">
  <class
    name="controller_package_name/controller_type_name"
    type="moveit_ros_control_interface::JointTrajectoryControllerAllocator"
    base_class_type="moveit_ros_control_interface::ControllerHandleAllocator"
  >
    <description>
      Controller description
    </description>
  </class>
</library>
```

> Replace `controller_package_name/controller_type_name` and `Controller description` with values appropriate for your project.

Reference the plugin description in your package `export` section:

```
<export>
  <moveit_ros_control_interface plugin="${prefix}/controller_moveit_plugin.xml"/>
</export>
```

Replace `/controller_moveit_plugin.xml` with a relative path of the plugin description file created in the previous step.

After building the package, any controllers in `ros_controllers.yaml` that reference `controller_package_name/controller_type_name` will become available for use with MoveIt.

To test ROS controller integration with *MoveIt ROS Control Controller Manager*, launch the package generanted by MSA by using the `demo_gazebo.launch` file. This will load your robot description and the MoveIt motion planning pipeline hosted in `move_group` node from `moveit_ros_move_group` package, in addition to visualizing the robot state in [RViz](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) and simulating the robot hardware interface in [Gazebo](https://classic.gazebosim.org/tutorials?tut=ros_control&cat=connect_ros).

> Since `position_controllers/GripperActionController` is not supported by MoveIt ROS Control Controller Manager, it can be replaced in the above example by `position_controllers/JointTrajectoryController`:
>```
> gripper_controller:
>  type: position_controllers/JointTrajectoryController
>  joints:
>    - gripper
>```

## Custom Controllers

Custom controllers not based on ROS Control can be integrated by writing and exporting a custom MoveIt *Controller Handle* plugin that conforms to the MoveIt interface and can be loaded by MoveIt Simple or ROS Controller Manager. If that interface is a poor fit for your custom controller, a custom MoveIt Controller Manager can be written that will take care of loading or unloading the controller, and managing its state and lifecycle.

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
  DEPENDS system_lib
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
