# A Crazyflie SITL 

Quadcopter model from [Flightmare](https://github.com/uzh-rpg/flightmare).
With some modifications regarding propellerdirections etc.

This is currently wip. However a single crazyflie in the high level commander is already working. 

I originally wanted to use Webots as simulation backend. 
However this is not possible because SITL with the crazyflie-firmware needs the 1000Hz update rate. 
After some testing it seems that even 500Hz is way more unstable.

The system works as follows for now: 
The crazyflie-firmware by llanesc [crazyflie-firmware](https://github.com/llanesc/crazyflie-firmware/tree/3450c0eff7c994c8e0a5a8ab36585e3db7ac1224) can just run on its own on a linux machine (he writes that 24.04 might be an issue).
The CRTP Protocol is transported over UDP.

We use ds-crazyflies as backend and modified the radio implementation to also support the UDP radio. 
This leaves crazyradio_node still as a SinglePointOfFailure; however currently this is what we want because this way it is possible to test the full DynamicSwarms software stack.
However each simulated CF needs > 1000Hz Bandwidth and we need to see how fast ROS2 timers will allow us to spin...

The crazyflie_sitl then models the quadcopter dynamics with the above mentioned model. 
It receives PWM data over the CRTP Protocoll.

Currently the SITL node needs to do quite some magic to handle edgecases.
E.g. not falling out of the world but still be able to takeoff.
The PWM to thrust is taken from this [documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/pwm-to-thrust/)


The SITL node then sends back IMU and EXT_POS data at 1000Hz.

# Installation
Create a ros2_ws

clone the following into src folder: 

crazyflie_hardware -b sitl_radio
crazyflie_interfaces
ros-objecttracker
this repo

Then `colcon build`


# Test

3 Terminals with ros sourced
1 Terminal for the firmware

1. Startup the firmware
2. `ros2 launch crazyflie_hardware_bringup hardware.launch.py` 
3. `ros2 run crazyflie_sitl crazyflie_sitl`

You should now see messages from the firmware, hardware launch is still pretty empty

4. Add crazyflie to gateway 

```
ros2 service call /crazyflie_hardware_gateway/add_crazyflie crazyflie_hardware_gateway_interfaces/srv/AddCrazyflie "channel: 80
id: 231
initial_position:
  x: 0.0
  y: 0.0
  z: 0.0
type: default" 
```

This will now startup a ros2 node exactly like when used with real crazyflie. 
You can set Parameters, start log blocks, send commands like takeoff

```
ros2 service call /cf231/takeoff crazyflie_interfaces/srv/Takeoff "group_mask: 0
height: 1.0
yaw: 0.0         
duration:
  sec: 3
  nanosec: 0" 
```

Its adviced to open RVIZ, currently the position is published on the TF-Graph. 
This feature will be removed however in the future. 

We'll see how we handle this. 
At the current moment we are unsure if we want to fly as external_tracking (like motion-capture).
Or if we want to fly like internal_tracking (like lighthouse)
Ideally both would be possible.

# Building the firmware

clone the firmware with branch dev-crazysim

git submodule update --init --recursive vendor

sudo apt install cmake build-essential
pip install Jinja2
sudo apt install python-is-python3

mabye: remove gz from CMakeLists in sitl_make

git submodule update --init --recursive
git fetch --tags


# Building

From main repo: 

make sitl_defconfig
make PLATFORM=sitl

# Executing

build/cf2