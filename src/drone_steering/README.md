# KNR-drone-coding

This is an autonomous drone project dedicated to controlling a quadcopter in sitl simulator or irl

To run navigational code in SITL on your computer

Replace `collections.MutableMapping` by `collections.abc.MutableMapping` (in the `dronekit/__init__.py` file).

It's best to just run this by console, running this in VSCode doesn't open console for some reason

`sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console`

Then, open a second console window and run this

`export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$HOME/KNR/drone-knr/src/simulation/drone_sim/build:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH`

And then this

`export IGN_GAZEBO_RESOURCE_PATH=$HOME/KNR/drone-knr/src/simulation/drone_sim/models:$HOME/KNR/drone-knr/src/simulation/drone_sim/worlds:$IGN_GAZEBO_RESOURCE_PATH`

And to run Gazebo

`ign gazebo -r -v 1 tree_of_life.sdf`

## To setup a RPi for running this code

```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install python-pip
sudo apt-get install python-dev
sudo apt install future
sudo apt-get install screen python-wxgtk4.0 python-lxml
sudo pip install pyserial
sudo pip install dronekit
sudo pip install dronekit-sitl -UI
sudo pip install MAVProxy
```

## Setting up RPi for communication

`raspi-config` -> disable UART for console, enable for serial port hardware
`/boot/config.txt` and add `dtoverlay=disable-bt`
if ttyAMA0 isn't in /dev, `enable_uart=1` in `/boot/config`

The rest of setup is via code:

`dronekit-sitl copter`

`python arm_test.py --connect /dev/ttyAMAO` - run the script to fly irl

## Useful commands

```bash
dronekit-sitl copter -h # help
print("Vehicle attribute values:")
print("GPS: %s" % vehicle.gps_0)
print("Battery: %s" % vehicle.battery)
print("Last heartbeat: %s" % vehicle.last_heartbeat)
print("is armable?: %s" % vehicle.is_armable)
print("System status: %s" % vehicle.system_status.state)
print("Mode: %s" % vehicle.mode.name)
```

## In case of losing your drone

Type this in console:
To come back home `mode rtl`, to land `mode land`
