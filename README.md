# KNR Drones quadcopter control
This is KNR repository for control of quadcopter drones based on ROS2 (Humble).

In order to use the repository clone it recursively.
```bash
git clone --recursive https://github.com/qbaaa-0/drone-knr
```

The simulator is implemented as an submodule, thus the `--recursive` flag.

# Prerequisities
ROS2 Humble requires Ubuntu 22.04. Make sure the correct version is installed according to [official tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).


# Running simulation:

## Install
Install Ignition Gazebo Fortress:
````bash
sudo apt install ros-humble-ros-gz
````

Install Ignition Gazebo Fortress development libs and rapidjson:
````bash
sudo apt install rapidjson-dev libignition-gazebo6-dev
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
Assuming that you have clone the repository in `$HOME/drone_sim`:
```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$HOME/drone-knr/src/simulation/drone_sim/build:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
export IGN_GAZEBO_RESOURCE_PATH=$HOME/drone-knr/src/simulation/drone_sim/models:$HOME/drone-knr/src/simulation/drone_sim/worlds:$IGN_GAZEBO_RESOURCE_PATH
```

### In .bashrc
Assuming that you have clone the repository in `$HOME/drone_sim`:
```bash
echo 'export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$HOME/drone-knr/src/simulation/drone_sim/build:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
echo 'export IGN_GAZEBO_RESOURCE_PATH=$HOME/drone-knr/src/simulation/drone_sim/models:$HOME/drone-knr/src/simulation/drone_sim/worlds:$IGN_GAZEBO_RESOURCE_PATH' >> ~/.bashrc
```
Reload your terminal with source ~/.bashrc

## Run Simulation

### Run all needed software
```bash
$HOME/drone-knr/src/simulation/launch/drone_sim.bash
```

### Arm and takeoff
In order to controll the drone one may use SITL terminal and MAVLink commands, eg.:

```bash
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 5
GUIDED> mode auto
```