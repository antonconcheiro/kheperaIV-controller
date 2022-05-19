import random
from enum import Enum
import numpy as np

from controller import Robot, Motor, DistanceSensor

TIME_STEP = 32
INFINITY = 999999
MAX_SPEED = 27.6

ultrasonic_sensors_names = ["left ultrasonic sensor", "front left ultrasonic sensor", "front ultrasonic sensor", "front right ultrasonic sensor", "right ultrasonic sensor"]
infrared_sensors_names = [
    # turret sensors
    "rear left infrared sensor", "left infrared sensor", "front left infrared sensor", "front infrared sensor",
    "front right infrared sensor", "right infrared sensor", "rear right infrared sensor", "rear infrared sensor",
    # ground sensors
    "ground left infrared sensor", "ground front left infrared sensor", "ground front right infrared sensor",
    "ground right infrared sensor"]

robot = Robot()

f_camera = robot.getDevice("camera")
f_camera.enable(TIME_STEP)

u_sensors = []
for sensor in ultrasonic_sensors_names:
    u_sens = robot.getDevice(sensor)
    u_sens.enable(TIME_STEP)
    u_sensors.append(u_sens)

ir_sensors = []
for sensor in infrared_sensors_names:
    ir_sens = robot.getDevice(sensor)
    ir_sens.enable(TIME_STEP)
    ir_sensors.append(ir_sens)

leds = [robot.getDevice("front left led"), robot.getDevice("front right led"), robot.getDevice("rear led")]

leftWheel = robot.getDevice("left wheel motor")
rightWheel = robot.getDevice("right wheel motor")
leftWheel.getPositionSensor().enable(TIME_STEP)
rightWheel.getPositionSensor().enable(TIME_STEP)
leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)

last_display_second = 0




mat_q = np.zeros((3,3))

class Estado(Enum):
    S1 = 1
    S2 = 2
    S3 = 3
    
estado_actual = Estado.S3
    
def check_estado():
    if ir_sensors[9].getValue() > 750 and ir_sensors[11].getValue() < 500:
        return Estado.S1
    elif ir_sensors[10].getValue() > 750 and ir_sensors[8].getValue() < 500:
        return Estado.S2
    return Estado.S3

def go_straight():
    leftWheel.setVelocity(MAX_SPEED)
    rightWheel.setVelocity(MAX_SPEED)
    
def turn_left():
    leftWheel.setVelocity(-MAX_SPEED)
    rightWheel.setVelocity(MAX_SPEED)
    
def turn_right():
    leftWheel.setVelocity(MAX_SPEED)
    rightWheel.setVelocity(-MAX_SPEED)
    

while robot.step(TIME_STEP) != -1:
    display_second = robot.getTime()
    if display_second != last_display_second:
        last_display_second = display_second

        #print(ir_sensors[3].getValue())
        #print(f'time = {display_second} [s]')
        #for i in range(len(u_sensors)):
        #    print(f'- ultrasonic sensor({ultrasonic_sensors_names[i]}) = {u_sensors[i].getValue()}')
        #for i in range(len(ir_sensors)):
        #    print(f' infrared sensor({infrared_sensors_names[i]}) = {ir_sensors[i].getValue()}')

        for led in leds:
            led.set(0xFFFFFF & random.randrange(20000))

        speed_offset = 0.2 * (MAX_SPEED - 0.03 * ir_sensors[3].getValue());
        speed_delta = 0.03 * ir_sensors[2].getValue() - 0.03 * ir_sensors[4].getValue()
        #leftWheel.setVelocity(speed_offset + speed_delta)
        #rightWheel.setVelocity(speed_offset - speed_delta)
        go_straight()
        estado_actual = check_estado()
        print(estado_actual)
		

#wb_robot_cleanup()
