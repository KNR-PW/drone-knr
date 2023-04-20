# Running simulation:

## Install
Install Ignition Gazebo Fortress:
````bash
sudo apt install ros-humble-ros-gz
````

Install Ignition Gazebo Fortress dependencies:
````bash
sudo apt install rapidjson-dev libignition-gazebo6-dev ros-humble-image-transport ros-humble-ros-gz-bridge
````

Build with:
````bash
cd $HOME/drone-knr/src/simulation/drone_sim/
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
````

## Setting env vars to include custom models
Set the ignition environment variables in your `.bashrc` or in the terminal used to run gazebo:

### In terminal
Assuming that you have clone the repository in `$HOME/drone-knr/`:
```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$HOME/drone-knr/src/simulation/drone_sim/build:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
export IGN_GAZEBO_RESOURCE_PATH=$HOME/drone-knr/src/simulation/drone_sim/models:$HOME/drone-knr/src/simulation/drone_sim/worlds:$IGN_GAZEBO_RESOURCE_PATH
```

### In .bashrc
Assuming that you have clone the repository in `$HOME/drone-knr/`:
```bash
echo 'export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$HOME/drone-knr/src/simulation/drone_sim/build:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
echo 'export IGN_GAZEBO_RESOURCE_PATH=$HOME/drone-knr/src/simulation/drone_sim/models:$HOME/drone-knr/src/simulation/drone_sim/worlds:$IGN_GAZEBO_RESOURCE_PATH' >> ~/.bashrc
```
Reload your terminal with source ~/.bashrc


## Run Simulation

### Run Ardupilot/SITL
```bash
cd ~/ardupilot
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

### Run Gazebo simulation
```bash
ign gazebo -r -v 1 tree_of_life.sdf
```
- `-r` flag makes the simulation run on startup.
- `-v 1-4` flag sets the logging level (4 for everything, 1 only for errors).
- `tree_of_life.sdf` is the name of the world we want to run, located in our project in $HOME/drone-knr/src/simulation/drone_sim/wolrds/

### Run ROS GZ Bridge
In order to receive camera image in ROS we have to provide a Bridge between simulation and ROS.
```bash
ros2 run ros_gz_image image_bridge camera
```

Alternatively, we can use launch file (only after building package, and sourcing setup script):
```bash
ros2 launch simulation drone_sim_launch.py
```

### Arm and takeoff
In order to controll the drone one may use SITL terminal and MAVLink commands, eg.:

```bash
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 5
GUIDED> mode auto
```