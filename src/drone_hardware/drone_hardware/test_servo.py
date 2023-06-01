import dronekit
import argparse
from pymavlink import mavutil
import time


# parser = argparse.ArgumentParser(description='commands')
# parser.add_argument('--connect', default='127.0.0.1:14550')
# args = parser.parse_args()

# connection_string = args.connect
connection_string = "/dev/serial0"
baud_rate = 921600

# vehicle = dronekit.connect('/dev/serial0', wait_ready=False) #doesnt work with wait_ready=True
vehicle = dronekit.connect(connection_string, baud=baud_rate, wait_ready=False)

def get_attitude():
    roll=vehicle.attitude.roll
    pitch=vehicle.attitude.pitch
    yaw=vehicle.attitude.yaw
    print(f"-- Get attitude service called --")
    print(f"Roll: {roll}")
    print(f"Pitch: {pitch}")
    print(f"Yaw: {yaw}")

def set_servo(servo_id, pwm):
    msg = vehicle.message_factory.command_long_encode(
        0,          # time_boot_ms (not used)
        0,   # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
        0,          #not used
        servo_id,   #number of servo instance
        pwm,        #pwm value for servo control
        0,0,0,0,0) #not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


get_attitude()


load =1500
shoot = 1200
left = 2000
right = 800
mid = 1400
stop = 1000

# set_servo(9,left)
# input("left")

# set_servo(9,mid)
# input("mid")

# set_servo(9,right)
# input("right")

def shoot_servo():
    input(dir)
    if dir == 'left':
        site = 2000
    else:
        site = 800
    set_servo(10,stop)
    set_servo(11,stop)
    time.sleep(1)
    set_servo(9,site)
    set_servo(10,load)
    set_servo(11,load)
    time.sleep(2)
    set_servo(10,shoot)
    set_servo(11,shoot)
    set_servo(9,mid - 300 if dir == 'left' else mid+300)
    time.sleep(1)
    set_servo(10,stop)
    set_servo(11,stop)
    set_servo(9,mid)

def config():
    set_servo(10,stop)
    set_servo(11,stop)
    time.sleep(1)

config()


while True:
    shoot_servo()
    # shoot_servo('right')

print("Script end")


# prawy = 800
# lewy = 2000
#srodek = 1400


#low 1000

#high 1500

## low - arm servo
## choose color (l/p)
## high (1500) 2s
## medium (1200) 1s
## return color (1400)
## low (1000)
##ready for next