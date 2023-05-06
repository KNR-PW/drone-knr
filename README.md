# KNR Drones quadcopter control
This is KNR repository for control of quadcopter drones based on ROS2 (Humble).

In order to use the repository clone it recursively.
```bash
git clone --recursive https://github.com/qbaaa-0/drone-knr
```

The simulator is implemented as an submodule, thus the `--recursive` flag.

# Prerequisities

## ROS2 - Humble
ROS2 Humble requires Ubuntu 22.04. Make sure the correct version is installed according to [official tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

## Latest ArduPilot software
The latest ArduPilot software is needed in order to fly the drone, as well as provide software for simulation. Current installtion instructions are based on [official ArduPilot documentation](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)

First, clone official repository:
````bash
git clone --recurse-submodules https://github.com/your-github-userid/ardupilot
cd ardupilot
````
Then, install required packages using provided script:
````bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
````
Lastly, reload the path (log-out and log-in to make permanent):
````bash
. ~/.profile
````


## Run Simulation

In order to run the simulator, look  into [Simulation package documenation](/src/simulation/)

## Run Detector

In order to run the detector, look  into [Detector package documenation](/src/drone_detector/)

## Run Detector GUI

In order to run the detector GUI, look  into [GUI package documenation](/src/drone_gui/)

