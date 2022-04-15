from controller import Robot, Motor, DistanceSensor
import numpy as np

TIME_STEP = 32
MAX_SPEED = 15
SQUARE_SIZE = 250
WHEEL_RADIUS = 21
FORWARD_ONE = SQUARE_SIZE / WHEEL_RADIUS

INITIAL_POS = (0,3)

robot = Robot()

leftWheel = robot.getDevice("left wheel motor")
rightWheel = robot.getDevice("right wheel motor")
leftWheel.getPositionSensor().enable(TIME_STEP)
rightWheel.getPositionSensor().enable(TIME_STEP)
encoderL = robot.getDevice("left wheel sensor")
encoderR = robot.getDevice("right wheel sensor") 

posL = encoderL.getValue() 
posR = encoderR.getValue()

l_ir_sensor = robot.getDevice("left infrared sensor")
r_ir_sensor = robot.getDevice("right infrared sensor")
f_ir_sensor = robot.getDevice("front infrared sensor")
l_ir_sensor.enable(TIME_STEP)
r_ir_sensor.enable(TIME_STEP)
f_ir_sensor.enable(TIME_STEP)

stopped = 0

map = np.zeros((12,12))
map[INITIAL_POS]  = 1
current_pos_map = INITIAL_POS
direction = 0

# 0-No obstacle, 1-Visited, 2-Obstacle
def mark_left(pos, mark):
    map[pos[0],pos[1]+1] = mark
    
def mark_right(pos, mark):
    map[pos[0],pos[1]-1] = mark

def mark_fwd(pos, mark):
    map[pos[0]+1,pos[1]] = mark

def mark_current(pos, mark):
    map[pos] = mark

def stop_robot():
    leftWheel.setVelocity(0) 
    rightWheel.setVelocity(0)
    
def move_fwd(current_pos_map):
    if direction == 0:
        current_pos_map = (current_pos_map[0] + 1, current_pos_map[1])
    elif direction == 1:
        current_pos_map = (current_pos_map[0], current_pos_map[1] + 1)
    elif direction == 2:
        current_pos_map = (current_pos_map[0] - 1, current_pos_map[1])
    elif direciton == 3:
        current_pos_map = (current_pos_map[0], current_pos_map[1] - 1)
    incrementL = FORWARD_ONE
    incrementR = FORWARD_ONE
    leftWheel.setVelocity(MAX_SPEED) 
    rightWheel.setVelocity(MAX_SPEED)
    leftWheel.setPosition(encoderL.getValue() + incrementL)
    rightWheel.setPosition(encoderL.getValue() + incrementR)
    return current_pos_map
    
def measure_ir():
    return[l_ir_sensor.getValue(),
        r_ir_sensor.getValue(),
        f_ir_sensor.getValue()]
        
def turn_left(direction):
    stop_robot()
    direction += 1
    leftWheel.setVelocity(MAX_SPEED) 
    rightWheel.setVelocity(MAX_SPEED)  
    leftWheel.setPosition(encoderL.getValue()-4.05) 
    rightWheel.setPosition(encoderR.getValue()+4.05)
    
def turn_right(direction):
    stop_robot()
    direction -= 1
    leftWheel.setVelocity(MAX_SPEED) 
    rightWheel.setVelocity(MAX_SPEED)  
    leftWheel.setPosition(encoderL.getValue()+4.05) 
    rightWheel.setPosition(encoderR.getValue()-4.05)

    
stop_robot()

leftWheel.setVelocity(MAX_SPEED) 
rightWheel.setVelocity(MAX_SPEED)

initial_pos = encoderL.getValue()

while robot.step(TIME_STEP) != -1:
    print(map)
    current_pos = encoderL.getValue()
    if (current_pos - initial_pos >= FORWARD_ONE):
        stopped = 0
    if stopped == 0:
        mark_current(current_pos_map, 1)
        ir_values = measure_ir()
        if(ir_values[0] > 200):
            mark_left(current_pos_map, 2)
        if(ir_values[1] > 200):
            mark_right(current_pos_map, 2)
        if(ir_values[2] > 200):
            mark_fwd(current_pos_map, 2)
        if(ir_values[0] < 200):
            stopped = 1
            direction = turn_left(direction)        
        elif(ir_values[2] < 200):
            stopped = 1
            initial_pos = encoderL.getValue()
            current_pos_map = move_fwd(current_pos_map)