# A Crazyflie SITL-Simulator for ROS

This ROS2 package runs the crazyflie-firmware locally and couples it with a simple quadcopter model.
Its aim is to have very little dependencies and does not require heavy simulators like Webots or Gazebo.
Using the ROS2-component system multiple instances can be run simultaneously.
Positions can be published to tf-Graph or to a PointCloud2 topic.

This repo will be part of the [ds-crazyflies](https://github.com/DynamicSwarms/ds-crazyflies) softwarestack. 
It can however also be used more-or-less standalone.

## Quadcopter Model

The model used is derived from 

 - Song, Y., Naji, S., Kaufmann, E., Loquercio, A., & Scaramuzza, D. (2020).  
   *Flightmare: A Flexible Quadrotor Simulator.*  
   Conference on Robot Learning (CoRL) 2020.

 Original project and source code:

 - <https://github.com/uzh-rpg/flightmare>

As some modifications where necessary those were implemented here: [quadcopter_model](https://github.com/DynamicSwarms/quadcopter_model)

# Standalone build

Tested on Ubuntu 22.04, ROS2 Humble

## Build the crazyflie_sitl package 

The package includes the firmware and provides a does it all `ros2 run` entry point.

### 1. Necessary dependencies

   Because the firmware is build by colcon you will need to:

   ```bash
   sudo apt install cmake build-essential
   ```

   If you installed `ros-lyrical-desktop`, there are no additional dependencies.  
   If you installed a lightweight version, you might need the following packages:

   - `libeigen3-dev`
   - `libpcl-common1.12`
   - `libpcl-dev`
   - `libpcl-io1.12`

### 2. Clone this repository

  ```
  git clone https://github.com/DynamicSwarms/crazyflie_sitl.git --recurse
  ```

### 3. Build it

  ```bash
  cd crazyflie_sitl
  source /opt/ros/lyrical/setup.bash
  colcon build
  ```

### 4. Start a crazyflie

  ```
  source install/setup.bash
  ros2 run crazyflie_sitl crazyflie_sitl --ros-args -p id:=0
  ```

  This will start a crazyflie with the firmware and the corresponding model. 
  It can now be connected to via UDP on port 19850 (for other ids its 19850+id)

## Connect to it with crazyflie-client-python

### 1. Install correct version of the client

  ```
  sudo apt install python3.14-venv
  python3 -m venv crazy_venv
  source crazy_venv/bin/activate

  git clone https://github.com/bitcraze/crazyflie-clients-python
  cd crazyflie-clients-python
  pip install -e .

  cd ..
  git clone https://github.com/bitcraze/crazyflie-lib-python
  cd crazyflie-lib-python
  pip install -e .
  ```

  Although cflib is in pip, currently the version is too old to be used!

### 2. Launch the client

  ```
  source crazy_venv/bin/activate
  python3 crazyflie-clients-python/bin/cfclient
  ```

### 3. Connect to crazyflie

  If you started the crazyflie before the client the client will automatically find the crazyflie. 
  You will just have to click `Connect`. Otherwise maybe `Scan` for your crazyflie.

  Upon clicking `Connect` there should be lots of messages popping up in the terminal you started the client in. 
  If it prints this: `INFO:cflib.crazyflie:Callback->Connection completed [udp://127.0.0.1:19850]` you are all set.

### 4. First flight with client

  > ! The Crazyflie Starts with Complementary Estimator as default. Which will not work, because it cannot use the external position information provided by the Quadcopter Model.

  Because of this you must set the ESTIMATOR to Kalman and the CONTROLLER to Mellinger.
  For this open the Parameters-Tab and select `stabilizer.controller` and set to `2`. 
  Do the same for `stabilizer.estimator` (also set to `2`).

  You can double check by looking at the console tab which should show the appropriate messages.

  > ! Because of the cflib-s connection scheme all previous messenges got lost and are not retrievable.

  #### You can now open the Flight Control Tab and press `Take off`. Hurray!

  Also `Land`, `Up`, `Down` etc. should work properly.

  > `TIP` As you have ROS2 installed anyway, open RVIZ2 and add the tf2-Graph so you have got something to look at as well.

## Connect with ROS2 and DynamicSwarms library

### 1. Install correctly

  Its best to have all repositories in the same workspace.
  
  ```bash
  mkdir ros_ws
  cd ros_ws
  mkdir src
  cd src
  git clone https://github.com/DynamicSwarms/crazyflie_sitl.git --recurse
  git clone https://github.com/DynamicSwarms/crazyflie_hardware.git -b sitl_radio --recurse 
  git clone https://github.com/DynamicSwarms/crazyflie_interfaces.git -b crazyswarm2_alignment
  git clone https://github.com/DynamicSwarms/ros-objecttracker.git --recurse
  cd ..
  ```

  Now colcon build all at once: 

  ```bash
  source /opt/ros/lyrical/setup.bash
  colcon build
  ```

### 2. Launching
  
  You'll now need at least 3 terminals. Its however best to open RVIZ with tf as well.

  In all of them: 
  ```
  cd .../ros_ws
  source install/setup.bash
  ```

  #### First Terminal: `ros2 launch crazyflie_hardware_bringup hardware.launch.py`
  #### Second Terminal: `ros2 run crazyflie_sitl crazyflie_sitl --ros-args -p id:=0`

  #### Third Terminal: 

  This will be used to add to the hardware gateway and then you can also takeoff land etc.

  ##### 1. Add to the Gateway

  Be ware: Markdown messes up these commands, you will need to enter them "by hand".
  The `TAB` key is your friend and will autocomplete most of it.

  ```
  ros2 service call /crazyflie_hardware_gateway/add_crazyflie crazyflie_hardware_gateway_interfaces/srv/AddCrazyflie "channel: 80
      id: 0
      initial_position:
        x: 0.0
        y: 0.0
        z: 0.0
      type: default" 
  ```

  The gateway console will print out lots os messages. (Also crazyflie-log messages).
  You do hope to see: `[cf0]: Successfully configured!`
  It might fail the first time with `batch timed out` (It tries to download log and param toc at first boot)
  Just try again.


  ##### 2. Takeoff

  ```
  ros2 service call /cf231/takeoff crazyflie_interfaces/srv/Takeoff "group_mask: 0
    height: 1.0
    yaw: 0.0         
    duration:
      sec: 3
      nanosec: 0" 
  ```

  > You should now see the crazyflie taking off in RVIZ use `GoTo` etc. for more commands.

  ##### Setting parameters

  Crazyflie parameters can be set with `ros2 param set /cf2/... ...` 

  ##### Its also possible to start log-blocks like in crazyswarm2 dynamic log blocks



## Arguments for the node:

  `id`: the id of the spawned crazyflie (only used in port=19850+id)
  `initial_position`: the position to spawn the crazyflie at (double_array)
  `publish_tf`: Disables tf-publishing (this gets inperformant quickly)

## Using the container:

  Apart from launching single crazyflies the crazyflie_sitl package also provides a container in which multiple crazyflies can be launched in unison.

  ```bash
  ros2 run crazyflie_sitl container 
  ```

  This will spawn 4 preconfigured crazyflies.

  The node has a parameter `crazyflie_configuration`. 
  You can pass a path to a yaml an example of which you can find in `crazyflie_sitl/config`

  You can define: 

    - if crazyflies should publish to tf individually
    - if the container should publish a pointcloud with the crazyflie positions
      - the rate at which the pointcloud should be published
      - the pointCloud topic name
    - ids and inital positions of crazyflies




# Some more information: 


This is currently wip. Bugs will appear...

I originally wanted to use Webots as simulation backend. 
However this is not possible because SITL with the crazyflie-firmware needs the 1000Hz update rate. 
After some testing it seems that even 500Hz is way more unstable.

The system works as follows for now: 
The crazyflie-firmware by llanesc [crazyflie-firmware](https://github.com/llanesc/crazyflie-firmware/tree/3450c0eff7c994c8e0a5a8ab36585e3db7ac1224) can just run on its own on a linux machine (he writes that 24.04 might be an issue). 
I had to do some modifications to run properly.
The CRTP Protocol is transported over a UNIX socket.
The crazyflie_sitl node then models the quadcopter dynamics with the flightmare model and launches and connects to the sitl-firmware.
The node passes IMU (100Hz) and position data (100Hz) to the firmware instance and also receives PWM data to be inputed into the model. (This might be the reason why only Kalman and Mellinger is working)
All other CRTP messages get passed to a second UDP connection, where either the client or our ds-crazyflies crazyradio can connect to.


As in a real swarming setup this leaves crazyradio_node still as a SinglePointOfFailure; however currently this is what we want because this way it is possible to test the full DynamicSwarms software stack.


The PWM to thrust is taken from this [documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/pwm-to-thrust/)



At the current moment we are unsure if we want to fly as external_tracking (like motion-capture).
Or if we want to fly like internal_tracking (like lighthouse)
Ideally both would be possible.

# Building the firmware (probably outdated)

clone the firmware with branch dev-crazysim

git submodule update --init --recursive vendor

sudo apt install cmake build-essential
pip install Jinja2
sudo apt install python-is-python3


git submodule update --init --recursive
git fetch --tags


## Building

From main repo: 

make sitl_defconfig
make PLATFORM=sitl

## Executing

build/cf2