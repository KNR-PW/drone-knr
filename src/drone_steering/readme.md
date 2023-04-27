To setup the RPi for code guided flight

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

```bash
dronekit-sitl copter
```
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
