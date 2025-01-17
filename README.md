![](https://github.com/open-rmf/free_fleet/workflows/build/badge.svg)

# Free Fleet

## Contents

- **[About](#About)**
- **[Installation Instructions](#installation-instructions)**
  - [Prerequisites](#prerequisites)
  - [Message Generation](#message-generation)
  - [Client in ROS1](#client-in-ros1)
  - [Server in ROS2](#server-in-ros2)
- **[Examples](#examples)**
  - [Barebones Example](#barebones-example)
  - [Turtlebot3 Simulation](#turtlebot3-simulation)
  - [Multi Turtlebot3 Simulation](#multi-turtlebot3-simulation)
  - [Commands and Requests](#commands-and-requests)
  - [RMF Example](#rmf-example)
- **[Plans](#plans)**

</br>
</br>

## About

Welcome to `free_fleet`, an open-source robot fleet management system. 
Sometimes it is called the "Fun Free Fleet For Friends" (F5).

**Note**, this repository is under active development. Things will be quite unstable
for a while. Please open an issue ticket on this repo if you have problems.
Cheers.

</br>
</br>

## Installation Instructions

### Prerequisites

* [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/)
* [ROS1 - Noetic](https://wiki.ros.org/noetic)
* [ROS2 - Foxy](https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/)

Install all non-ROS prerequisite packages,

```bash
sudo apt update && sudo apt install \
  git wget qt5-default \
  python3-rosdep \
  python3-vcstool \
  python3-colcon-common-extensions \
  # maven default-jdk   # Uncomment to install dependencies for message generation
```

</br>

### Message Generation

Message generation via `FleetMessages.idl` is done using `dds_idlc` from `CycloneDDS`. For convenience, the generated mesasges and files has been done offline and committed into the code base. They can be found [here](./free_fleet/src/messages/FleetMessages.idl).

```bash
./dds_idlc -allstructs FleetMessages.idl
```

</br>

### Client in ROS1

Start a new ROS1 workspace, and pull in the necessary repositories,

```bash
mkdir -p ~/client_ws/src
cd ~/client_ws/src
git clone -b noetic-devel-multi-fix https://github.com/siot-decada-robotics/free_fleet.git
git clone https://github.com/eclipse-cyclonedds/cyclonedds
git clone https://github.com/nilseuropa/gazebo_ros_battery.git
```

Install all the dependencies through `rosdep`,

```bash
cd ~/client_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -yr
```

Source ROS1 and build,

```bash
cd ~/client_ws
source /opt/ros/noetic/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE
```

</br>

### Server in ROS2

Start a new ROS2 workspace, and pull in the necessary repositories,

```bash
mkdir -p ~/server_ws/src
cd ~/server_ws/src
git clone -b noetic-devel-multi-fix https://github.com/siot-decada-robotics/free_fleet.git
git clone https://github.com/siot-decada-robotics/rmf_internal_msgs
```

Install all the dependencies through `rosdep`,

```bash
cd ~/server_ws
rosdep install --from-paths src --ignore-src --rosdistro foxy -yr
```

Source ROS2 and build, 

```bash
cd ~/server_ws
source /opt/ros/foxy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE

# Optionally use the command below to only build the relevant packages,
# colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE \
#   --packages-up-to free_fleet ff_examples_ros2 free_fleet_server_ros2
```

</br>
</br>

## Examples

### Barebones Example

This example emulates a running robot and also a running free fleet server,

```bash
source ~/client_ws/install/setup.bash
roslaunch ff_examples_ros1 fake_client.launch
```

The client will then start subscribing to all the necessary topics, and start publishing robot states over DDS to the server. Start the server using

```bash
source ~/server_ws/install/setup.bash
ros2 launch ff_examples_ros2 fake_server.launch.xml
```

To verify that the fake client has been registered, there will be print-outs on the server terminal, otherwise, the ROS2 messages over the `/fleet_states` topic can also be used to verify,

```bash
source ~/server_ws/install/setup.bash
ros2 topic echo /fleet_states
```

Next, to send requests and commands, check out the example scripts and their uses [here](#commands-and-requests).

</br>

### Turtlebot3 Simulation

Before starting these examples, remember to install all the prerequisites according to the [official tutorials](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#install-dependent-ros-1-packages) of using `Turtlebot3`.

Additional packages are required for this example, clone in additional packages into the client workspace and install the required dependencies,

** Note, due to changes in `robot_state_publisher` for `noetic`, the examples will not work with the main packages, hence the links to multiple forks. The instructions will be updated once the merges are complete.

```bash
cd ~/client_ws/src
git clone https://github.com/rhaschke/robot_state_publisher -b noetic-devel
git clone https://github.com/lkw303/turtlebot3_simulations.git -b noetic-devel-multi-fix
git clone https://github.com/lkw303/turtlebot3.git -b noetic-devel-multi-fix

cd robot_state_publisher
git checkout 661628a76bbb

sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-dwa-local-planner

cd ~/client_ws
source /opt/ros/noetic/setup.bash
colcon build --packages-up-to ff_examples_ros1
```

Launch the basic simulation of a single Turtlebot3, with a free fleet client attached to it, by sourcing the client workspace and launching the provided example launch file,

```bash
source ~/client_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger; roslaunch ff_examples_ros1 turtlebot3_world_ff.launch
```

This launch file starts the simulation in `gazebo`, visualization in `rviz`, as well as the simulated navigation stack of the single turtlebot3. Once the simulation and visualization show up, the robot can be commanded as per normal through `rviz` with `2D Nav Goal`.

Now, we start the free fleet server, with the provided launch file,

```bash
source ~/server_ws/install/setup.bash
ros2 launch ff_examples_ros2 turtlebot3_world_ff.launch.xml
```

At this point, the server should have registered the client running on that single simulated robot and a print-out should appear in the server terminal. Another way to check, is to listen in on the `/fleet_states` topic, using `ros2 topic echo /fleet_states`.

Next, to send requests and commands, check out the example scripts and their uses [here](#commands-and-requests).

</br>

### Multi Turtlebot3 Simulation

This example launches three Turtlebot3s in simulation, and registers each robot as a client within a fleet controlled by `free_fleet`. The setup of this simulation is the same as the [single turtlebot3 simulation example](#turtlebot3-simulation).

Similarly to the example above, launch the provided launch file, which will start the simulation, visualization, navigation stacks of 3 turtlebot3s, and free fleet clients for each of the robots,

```bash
source ~/client_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger; roslaunch ff_examples_ros1 multi_turtlebot3_ff.launch
```

Once the simulation and visualization show up, the robots can then be commanded to navigate to different parts of the map by using the tool and panels in the visualization. Fill in the fleet name and robot name, select the navigation goal using `2D Nav Goal`, which will be reflected on the panel, and select `Send Nav Goal`.

![](media/multi_tb3.gif)

Next, to send more specific requests and commands, check out the example scripts and their uses [here](#commands-and-requests).

</br>

### Commands and Requests

Now the fun begins! There are 3 types of commands/requests that can be sent to the simulated robots through `free_fleet`,

Destination requests, which allows single destination commands for the robots,

```bash
ros2 run ff_examples_ros2 send_destination_request.py -f FLEET_NAME -r ROBOT_NAME -x 1.725 -y -0.39 --yaw 0.0 -i UNIQUE_TASK_ID
```

Path requests, which requests that the robot perform a string of destination commands,

```bash
ros2 run ff_examples_ros2 send_path_request.py -f FLEET_NAME -r ROBOT_NAME -i UNIQUE_TASK_ID -p '[{"x": 1.725, "y": -0.39, "yaw": 0.0, "level_name": "B1"}, {"x": 1.737, "y": 0.951, "yaw": 1.57, "level_name": "B1"}, {"x": -0.616, "y": 1.852, "yaw": 3.14, "level_name": "B1"}, {"x": -0.626, "y": -1.972, "yaw": 4.71, "level_name": "B1"}]'
```

Mode requests which only supports `pause` and `resume` at the moment,

```bash
ros2 run ff_examples_ros2 send_mode_request.py -f FLEET_NAME -r ROBOT_NAME -m pause -i UNIQUE_TASK_ID
ros2 run ff_examples_ros2 send_mode_request.py -f FLEET_NAME -r ROBOT_NAME -m resume -i UNIQUE_TASK_ID
```

**Note** that the task IDs need to be unique, if a request is sent using a previously used task ID, the request will be ignored by the free fleet clients.

</br>
</br>

### RMF Example

Below are the steps to set up and run free fleet with the [rmf_demos](https://github.com/siot-decada-robotics/rmf_demos) repository.

The first step is to have RMF installed by following the instructions on the [RMF root repository](https://github.com/siot-decada-robotics/rmf) that we forked.

The reason we forked this repository is beacuse the ```rmf.repos``` file contains the url pointing to Decada Robotics' forked version of ```rmf_demos``` repository. We have added a few launch files of our own in our forked repo.

Once all the prerequisites are installed such as the workspaces of **RMF**, **Free Fleet Server** and **Free Fleet Client**, **[ROS1_bridge](https://github.com/ros2/ros1_bridge)** open a new terminal and run the launch file to the simulation that launches the office environment and two turtlebot3 with their free fleet client running on ROS1.

Additionally before running the launch file, we need to add the path library of the required gazebo plugins from RMF into `LD_LIBRARY_PATH` to run the simulation with doors and lifts.
```
cd ~/client_ws
source /opt/ros/noetic/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/rmf_ws/install/rmf_door_msgs/lib:~/rmf_ws/install/rmf_building_sim_gazebo_plugins/lib:~/rmf_ws/install/rmf_building_sim_common/lib:/opt/ros/foxy/lib
roslaunch ff_examples_ros1 multi_turtlebot3_office_ff.launch
```
<br/>

In another terminal run the free fleet server for the turtlebot3 fleet and the robots tb3_0 and tb3_1 should be registered by the server.
```
cd ~/server_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch ff_examples_ros2 turtlebot3_server.launch.xml
```
<br/>

Before running the Fleet Adapters, Visualisers, API server from RMF, we need to run ros1_bridge to bridge the ```/clock``` topic so the nodes of RMF running on ROS2 can sync with the ```/clock``` topic with the simulation running on ROS1.
```
cd ros1_bridge_ws
source /opt/ros/foxy/setup.bash
source ./install/setup.bash
ros2 run ros1_bridge dynamic_bridge
```
<br/>

Now run RMF with the turtlebot3 fleet adapter, visuliser node and API server without the simulation.
```
cd ~/rmf_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch rmf_demos office_no_sim.launch.xml
```
<br/>

You now should be able to send loop tasks to the turtlebot3 fleet by dispatching a loop task via the command line
```
ros2 run rmf_demos_tasks dispatch_loop -s coe -f pantry -n 1 --use_sim_time
```
![](media/rmf_free_fleet.gif)

Alternatively you can dispatch a task via the dashboard. Do take note that the turtlebot3 fleet adapter can only take in loop tasks.

![](media/rmf_free_fleet_dashboard.gif)

## Plans

* Significant changes incoming from the `develop` branch, however it is still a work in progress.
