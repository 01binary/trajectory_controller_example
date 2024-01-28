# MoveIt Controller Integration Example

Example of a custom ROS Joint Trajectory Controller integrated with MoveIt! motion planning and execution pipeline to complement the [MoveIt Plugin Interfaces](https://moveit.ros.org/documentation/plugins/) help topic.

MoveIt supports the following [ROS controllers](https://github.com/ros-controls/ros_controllers) out of the box:

- [JointTrajectoryController](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/include/joint_trajectory_controller/joint_trajectory_controller.h) (see [implementation](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/include/joint_trajectory_controller/joint_trajectory_controller_impl.h)) specialized for the three types of hardware interfaces supported by [ROS Control](https://wiki.ros.org/ros_control):

  - [velocity_controllers/JointTrajectoryController](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/src/joint_trajectory_controller.cpp#L53)
  - [position_controllers/JointTrajectoryController](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/src/joint_trajectory_controller.cpp#L36)
  - [effort_controllers/JointTrajectoryController](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/src/joint_trajectory_controller.cpp#L64)
- [GripperActionController](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/gripper_action_controller/include/gripper_action_controller/gripper_action_controller.h) (see [implementation](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/gripper_action_controller/include/gripper_action_controller/gripper_action_controller_impl.h)) specialized for the two types of hardware interfaces commonly used to control end effectors:
  - [position_controllers/GripperActionController](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/gripper_action_controller/src/gripper_action_controller.cpp#L42)
  - [effort_controllers/GripperActionController](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/gripper_action_controller/src/gripper_action_controller.cpp#L52)

These controllers are supported because MoveIt implements `moveit_controller_manager::MoveItControllerHandle` and `moveit_ros_control_interface::ControllerHandleAllocator` interfaces defined in [ControllerHandle.h](https://github.com/ros-planning/moveit/blob/master/moveit_plugins/moveit_ros_control_interface/include/moveit_ros_control_interface/ControllerHandle.h), that integrate these controllers with the MoveIt pipeline:

- `MoveItControllerHandle`
    - Starts a ROS `actionlib::SimpleActionClient` for `control_msgs::FollowJointTrajectoryAction` and implements methods like `sendTrajectory`, `waitForExecution`, and `cancelExecution` which simply forward the MoveIt trajectory start/stop commands to a ROS controller specified by the `name` configuration setting through this action client.
- `ControllerHandleAllocator`
    - **The entry point for integrating a ROS controller with MoveIt**. When MoveIt is ready to start sending trajectory commands (i.e. through the MoveGroup C++ interface or by clicking Plan & Execute in RViz) the MoveIt controller manager will look for a registered plugin that implements this interface, with a name that precisely matches the ROS controller name. If such a plugin is found, MoveIt will call the `alloc` method, which is expected to return a `MoveItControllerHandle` (just like the one described above).

The following steps are therefore required to integrate a ROS controller with MoveIt - either a stock controller like `JointTrajectoryController` or a custom one that you write.

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

The `type` setting must specify the type name of a controller plugin registered with [pluginlib](https://wiki.ros.org/pluginlib), which implements a `controller_interface::Controller` interface, templated by the type of hardware interface this controller can talk to:

* `hardware_interface::EffortJointInterface`
* `hardware_interface::PositionJointInterface`
* `hardware_interface::VelocityJointInterface`

The ROS controller is expected to start a `actionlib::ActionServer` that responds to requests defined by `control_msgs::FollowJointTrajectoryAction` - mainly the requests to start and stop a trajectory.

Upon receiving these trajectory requests, the ROS controller can repeatedly determine the low-level position, velocity, or effort commands to execute on the hardware in each call to its `update` method, and use the hardware interface pointer it received in its `init` method to forward the commands to the underlying hardware interface.

> For more information on working with `hardware_interface` on both client and server side, see [ROS Control](https://wiki.ros.org/ros_control).

Next, the `simple_moveit_controllers.yaml` file also generated by MoveIt Setup Assistant maps ROS Controllers specified in `ros_controllers.yaml` to corresponding MoveIt interfaces:

```
controller_list:
  - name: yourcontroller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: True
    joints:
      - ...
```

Notice that `yourcontroller` ROS controller defined in `ros_controllers.yaml` (which implements `controller_interface::Controller` interface) is mapped to `FollowJointTrajectory` MoveIt controller interface (which implements `moveit_controller_manager::MoveItControllerHandle` and `moveit_ros_control_interface::ControllerHandleAllocator`).

It also tells MoveIt which joints will be controlled by the ROS controller, so that it knows which joints to include in trajectory messages that it asks this controller to execute.

The `action_ns` setting must specify the name of the action server started by the ROS Controller specified by the `name` setting (see above).

The `type` setting specifies one of the two kinds of interfaces MoveIt can integrate with:
* `FollowJointTrajectory`: this means the action server started by the referenced ROS controller must service [control_msgs::FollowJointTrajectoryAction](https://docs.ros.org/en/jade/api/control_msgs/html/action/FollowJointTrajectory.html) requests
* `GripperCommand`: this means the action server started by the ROS controller must service [control_msgs::GripperCommand](https://docs.ros.org/en/jade/api/control_msgs/html/action/GripperCommand.html) requests

Finally, the MoveIt Controller manager which runs inside the `move_group` node configured by `move_group.launch` file (also created by MoveIt Setup Assistant) will load controller integrations defined in `simple_moveit_controllers.yaml`.

For each controller defined under `controller_list` setting:

* MoveIt Controller Manager looks for a plugin registered with `pluginlib` that implements `moveit_ros_control_interface::ControllerHandleAllocator` interface, with a name that matches the `name` setting (i.e. must be exactly the same as the ROS controller type name)
* If the plugin is not found, the controller is loaded but does not claim the joints specified by the `joints` setting. Attempting to execute a trajectory will emit an error message: *Unable to identify any set of controllers that can actuate the specified joints*
* If the plugin is found and loaded successfully, the ROS controller has a chance to claim the specified joints in its `init` or `starting` methods by calling the `getHandle` method of the hardware interface it receives as a parameter to `init` (see code examples below).

> Note the following requirements for writing plugins that are discoverable and loadable by `pluginlib`:
> * Your package must specify `pluginlib` as a dependency
> * Your package must specify the package which exports the plugin interface you are implementing as a dependency. For example, if you are implementing `moveit_ros_control_interface::ControllerHandleAllocator`, you must specify `moveit_ros_control_interface` as dependency
> * Your package must have an `exports` section with `moveit_ros_control_interface` entry that points to your MoveIt interface plugin definition XML, for example see [package.xml](https://github.com/ros-planning/moveit/blob/master/moveit_plugins/moveit_ros_control_interface/package.xml#L24) that exports stock controller interface plugins
> * Your package must have the plugin definition XML file, for example see [stock plugins XML](https://github.com/ros-planning/moveit/blob/master/moveit_plugins/moveit_ros_control_interface/moveit_ros_control_interface_plugins.xml)
> * You must `#include <pluginlib/class_list_macros.h>` and call the `PLUGINLIB_EXPORT_CLASS` macro with the full name of your class, and the full name of the interface you are implementing

## Example

Here's an example of how to implement a **Joint Trajectory Controller** that works with a *Velocity Joint Interface*, and integrates with MoveIt.

> The steps to integrate a custom **Gripper Action Controller** with MoveIt are similar (the only difference is in the type of Action Server/Action Client), so following this example should help you implement either type of controller.

We will go through the following steps:

- Specify relevant package dependencies
- Implement a ROS Trajectory Controller
- Export the ROS Trajectory Controller
- Implement a MoveIt Controller Handle
- Implement a MoveIt Controller Handle Allocator
- Export the MoveIt Controller Handle Allocator
- Configure the new ROS Controller
- Test the new ROS Controller and its MoveIt integration

Once the steps are complete, you should be left with a boilerplate custom ROS Controller which can be loaded and used by MoveIt, both simulated in Gazebo and with real robot hardware.

### Specify relevant package dependencies

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
control_msgs \
trajectory_msgs

# If you are following a tutorial, you may want to track changes made in each step
git init
```

Otherwise, add the above dependencies to your package:

* `roscpp`: because you are using ROS C++ interface
* `pluginlib`: because you are creating a ROS Controller plugin and a MoveIt Controller Interface plugin, and both need to be exported
* `controller_interface`: because you are exposing your controller to ROS Controller Manager as a plug-in
* `moveit_ros_control_interface`: because you are exposing your controller to MoveIt Controller Manager as a plug-in
* `actionlib`: because your controller needs an Action Server that will receive trajectory commands, and your MoveIt controller interface needs an Action Client that talks to this server
* `control_msgs`: includes the trajectory command message your Action Client will send, and your Action Server will receive
* `trajectory_msgs`: includes the "inner" trajectory command details from the trajectory command message in `control_msgs` above

Install any missing packages:

```
cd ~/catkin_ws/src
rosdep install -y --from-paths . --ignore-src --rosdistro $ROS_DISTRO
```

Build your workspace to ensure everything is working up to this point:

```
cd ~/catkin_ws
catkin_make
```

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
  CATKIN_DEPENDS actionlib control_msgs control_msgs controller_interface moveit_ros_control_interface pluginlib roscpp trajectory_msgs
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

No installation should be necessary: `pluginlib` will know where to find your built library after we add the exports in the next section.

Build the workspace to verify your package compiles successfully. If you get any errors, compare your work with the [trajectory_controller_example](https://github.com/01binary/trajectory_controller_example) repository.

### Export the ROS Trajectory Controller

The next step is to register the new ROS Controller with `pluginlib` so that it can be loaded by ROS Controller Manager.

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

> Any libraries you build with `catkin_make` will typically be located in a shared `lib` folder and have `lib` prefix added. This explains the `/lib/libtrajectory_controller_example` used here as the library path when the logical expectation would just be `trajectory_controller_example`.

Finally, your `package.xml` must be updated to export the new controller plugin. If you created a new package with `catkin_create_pkg`, the `<export>` section will already be there. Add a `controller_interface` entry with a `plugin` attribute pointing to your plugin description file:

```
<!-- The export tag contains other, unspecified, tags -->
<export>
  <!-- Other tools can request additional information be placed here -->
  <controller_interface plugin="${prefix}/trajectory_controller_plugin.xml"/>
</export>
```

> Note that `pluginlib` will fail to find your plugin if both `pluginlib` and the package which exports the type specified by `base_class_type` attribute are not specified as dependencies in your own package. In this example the base class type is `controller_interface::ControllerBase`, and it's exported by `controller_interface` package, therefore `controller_interface` must appear in your package dependencies.

Build your workspace again to ensure the changes have been made correctly:

```
cd ~/catkin_ws
catkin_make
```

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

If you haven't created a MoveIt Config package for your own project yet, you can try the code in this example with a mock robot arm defined by these two packages:

* [basic_velocity_control](https://github.com/01binary/basic_velocity_control)
* [basic_velocity_control_moveit_config](https://github.com/01binary/basic_velocity_control_moveit_config)

In that case, after cloning both packages into `~/catkin_ws/src`, you can modify `ros_controllers.yaml` in `basic_velocity_control_moveit_config` package and launch Gazebo with RViz with this command:

```
roslaunch basic_velocity_control_moveit_config demo_gazebo.launch
```

If your controller library was not compiled or linked correctly, the `gzserver` ROS node that hosts the ROS Controller Manager will likely crash at startup while trying to load your controller. No problem! Just compare your work with the working example in [trajectory_controller_example](https://github.com/01binary/trajectory_controller_example) repository.

You can verify your controller loads by adding `ROS_INFO()` calls to log messages in the constructor. However, at this point, attempting to execute a trajectory will emit the following error:

```
Unable to identify any set of controllers that can actuate the specified joints: ...
```

This error is emitted because MoveIt Controller Manager (not to be confused with ROS Controller Manager which already found and loaded your controller successfully) attempts to find a registered plugin with the same name as your ROS controller (i.e. `example/trajectory_controller_example`) but a different base class (`moveit_ros_control_interface::ControllerHandleAllocator`).

We will create and export this new class as well as another class it depends on in the next two sections.

> This error can also be emitted when you don't call `getHandle()` on each joint in the hardware interface that's supposed to be controlled by your controller when it's initialized or started. Calling `getHandle()` for the first time will cause ROS to record that certain joints were "claimed" by your controller.

### Implement a MoveIt Controller Handle

The first step of integrating a ROS controller with MoveIt is wrapping it into a *controller handle* class.

The ROS controller (running inside ROS Controller Manager) starts an Action Server to receive trajectory commands, and the MoveIt controller handle (running inside MoveIt Controller Manager) attempts to connect to this Action Server and forward MoveIt commands to request and cancel trajectory goals.

Create `controller_handle_example.h` in the include directory of your package:

```
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
```

Create `controller_handle_example.cpp` in the source directory of your package:

```
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
```

This is the most basic working implementation of a MoveIt Controller Handle: it doesn't load values like timeouts from settings, and always returns `SUCCEEDED` execution status instead of querying the goal status through the action client.

Include this new module in your library build before or after `trajectory_controller_example.cpp`:

```
## Declare a C++ library
add_library(${PROJECT_NAME}
   src/trajectory_controller_example.cpp
   src/controller_handle_example.cpp
)
```

Build your workspace to ensure there are no errors so far:

```
cd ~/catkin_ws
catkin_make
```

This class does not have to be exported for use by `pluginlib`, rather it will be returned by MoveIt Controller Handle Allocator (which is exported). We are creating this class in the next section.

### Implement a MoveIt Controller Handle Allocator

We're in the home stretch of the effort required to make a ROS Controller work with MoveIt! The last class we have to create is very short: all it does is create a new instance of the MoveIt Controller Handle.

Create `controller_handle_allocator_example.h` in the include directory of your package:

```
/*
    controller_handle_allocator_example.h

    Exposes a wrapped ROS Controller to MoveIt pipeline
*/

#pragma once

#include "controller_handle_example.h"

namespace example
{
    class controller_handle_allocator_example: public moveit_ros_control_interface::ControllerHandleAllocator
    {
    public:
        moveit_controller_manager::MoveItControllerHandlePtr alloc(const std::string& name, const std::vector<std::string>& resources) override;
    };
}
```

Create `controller_handle_allocator_example.cpp` in the source directory of your package:

```
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
```

Include this new module in your library build:

```
## Declare a C++ library
add_library(${PROJECT_NAME}
   src/trajectory_controller_example.cpp
   src/controller_handle_example.cpp
   src/controller_handle_allocator_example.cpp
)
```

Build your workspace to ensure there are no errors so far:

```
cd ~/catkin_ws
catkin_make
```

### Export the MoveIt Controller Handle Allocator

Now we will take the same steps to export the Controller Handle Allocator as we did with the ROS Controller earlier.

Add the following header to the top of `.h` or `.cpp` file for the controller handle allocator class:

```
#include <pluginlib/class_list_macros.h>
```

Add the `PLUGINLIB_EXPORT_CLASS` macro call to the bottom of `.h` or `.cpp` file for the controller handle allocator class:

```
PLUGINLIB_EXPORT_CLASS(example::controller_handle_allocator_example, moveit_ros_control_interface::ControllerHandleAllocator);
```

Create a plugin definition file `controller_handle_allocator_plugin.xml` at the root of the package:

```
<library path="lib/libtrajectory_controller_example">
  <class
    name="example/trajectory_controller_example"
    type="example::controller_handle_allocator_example"
    base_class_type="moveit_ros_control_interface::ControllerHandleAllocator"
  >
    <description>
      Example Controller Handle Allocator for MoveIt!
    </description>
  </class>
</library>
```

> Notice that the `name` of the exported class doesn't match the type. Instead the name matches the name of the ROS Controller being used, in this case our custom trajectory controller.

Finally, create a reference to the plugin definition in the package manifest file `package.xml`. Simply place it below the `<controller_interface ... />` entry we created earlier:

```
<!-- The export tag contains other, unspecified, tags -->
<export>
  <!-- Other tools can request additional information be placed here -->
  <controller_interface plugin="${prefix}/trajectory_controller_plugin.xml"/>
  <moveit_ros_control_interface plugin="${prefix}/controller_handle_allocator_plugin.xml"/>
</export>
```

In this case, the entry is called `moveit_ros_control_interface` because that's the name of the package that contains the interface `moveit_ros_control_interface::ControllerHandleAllocator` we are implementing.

Build your workspace again and ensure there are no errors:

```
cd ~/catkin_ws
catkin_make
```

### Configure the ROS Controller

The ROS controller we created in this example should now be ready for use in the MoveIt pipeline.

Create a new MoveIt configuration package by using the MoveIt Setup Assistant, or use an example MoveIt Config package such as `basic_velocity_control`:

* [basic_velocity_control](https://github.com/01binary/basic_velocity_control)
* [basic_velocity_control_moveit_config](https://github.com/01binary/basic_velocity_control_moveit_config).

Modify the `ros_controllers.yaml` configuration created by MoveIt Setup Assistant to substitute the custom controller created in this tutorial for the stock `velocity_controllers/JointTrajectoryController` exported by ROS Controls package.

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

Run the Gazebo simulation with RViz (in this example we are using a mock robot arm defined in the two `basic_velocity_control` packages):

```
roslaunch basic_velocity_control_moveit_config demo_gazebo.launch
```

The ROS controller should be loaded at startup, and then the MoveIt wrapper you wrote will be activated once you move the joints in RViz and click **Plan & Execute**.

If you haven't implemented the details of the joint trajectory controller and left it blank as in our examples you may get some errors from MoveIt complaining that nothing is getting done, but all the entry points should be working.

You can examine a full [joint trajectory implementation](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/include/joint_trajectory_controller/joint_trajectory_controller_impl.h) in ROS Controls package source code on GitHub.

