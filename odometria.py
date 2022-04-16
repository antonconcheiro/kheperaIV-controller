from controller import Robot, Motor, DistanceSensor
import numpy as np

TIME_STEP = 32
MAX_SPEED = 15
SQUARE_SIZE = 250
WHEEL_RADIUS = 21
FORWARD_ONE = SQUARE_SIZE / WHEEL_RADIUS
TURN_VAL = 4.05

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

map = np.ones((12,12))
map[INITIAL_POS]  = 1
current_pos_map = INITIAL_POS
direction = 0

# 0-No obstacle, 1-No visited, 2-Visited, 3-Obstacle
def mark_pos(pos, ir):
    if ir > 200:
        map[pos] = 3
    elif map[pos] != 2:
        map[pos] = 0

def mark_current(pos):
    map[pos] = 2
    
def mark_near(pos, direction, ir):
    if direction == 0:
        mark_pos((pos[0],pos[1]+1), ir[0]) # Left
        mark_pos((pos[0],pos[1]-1), ir[1]) # Right
        mark_pos((pos[0]+1,pos[1]), ir[2]) # Fwd
    elif direction == 1:
        mark_pos((pos[0]-1,pos[1]), ir[0]) # Left
        mark_pos((pos[0]+1,pos[1]), ir[1]) # Right
        mark_pos((pos[0],pos[1]+1), ir[2]) # Fwd
    elif direction == 2:
        mark_pos((pos[0],pos[1]-1), ir[0]) # Left
        mark_pos((pos[0],pos[1]+1), ir[1]) # Right
        mark_pos((pos[0]-1,pos[1]), ir[2]) # Fwd
    elif direction == 3:
        mark_pos((pos[0]+1,pos[1]), ir[0]) # Left
        mark_pos((pos[0]-1,pos[1]), ir[1]) # Right
        mark_pos((pos[0],pos[1]-1), ir[2]) # Fwd
        
def decide_dir(pos, dir):
    if direction == 0:
        array_directions = [
            map[(pos[0],pos[1]+1)],
            map[(pos[0]+1,pos[1])],
            map[(pos[0],pos[1]-1)]
        ] # Left, fwd, right
    elif direction == 1:
        array_directions = [
            map[(pos[0]-1,pos[1])],
            map[(pos[0],pos[1]+1)],
            map[(pos[0]+1,pos[1])]
        ] # Left, fwd, right
    elif direction == 2:
        array_directions = [
            map[(pos[0],pos[1]-1)],
            map[(pos[0]-1,pos[1])],
            map[(pos[0],pos[1]+1)]
        ] # Left, fwd, right
    elif direction == 3:
        array_directions = [
            map[(pos[0]+1,pos[1])],
            map[(pos[0],pos[1]-1)],
            map[(pos[0]-1,pos[1])]
        ] # Left, fwd, right
    index = np.argmin(array_directions)
    if array_directions[index] < 3:
        return index
    else:
        return -1

def stop_robot():
    leftWheel.setVelocity(0) 
    rightWheel.setVelocity(0)
    
def move_fwd(current_pos_map, direction):
    #print(map)
    #print(current_pos_map)
    #print(direction)
    if direction == 0:
        current_pos_map = (current_pos_map[0] + 1, current_pos_map[1])
    elif direction == 1:
        current_pos_map = (current_pos_map[0], current_pos_map[1] + 1)
    elif direction == 2:
        current_pos_map = (current_pos_map[0] - 1, current_pos_map[1])
    elif direction == 3:
        current_pos_map = (current_pos_map[0], current_pos_map[1] - 1)
    incrementL = FORWARD_ONE
    incrementR = FORWARD_ONE
    leftWheel.setVelocity(MAX_SPEED) 
    rightWheel.setVelocity(MAX_SPEED)
    leftWheel.setPosition(encoderL.getValue() + incrementL)
    rightWheel.setPosition(encoderR.getValue() + incrementR)
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
    leftWheel.setPosition(encoderL.getValue()-TURN_VAL) 
    rightWheel.setPosition(encoderR.getValue()+TURN_VAL)
    return direction
    
def turn_right(direction):
    stop_robot()
    direction -= 1
    leftWheel.setVelocity(MAX_SPEED) 
    rightWheel.setVelocity(MAX_SPEED)  
    leftWheel.setPosition(encoderL.getValue()+TURN_VAL) 
    rightWheel.setPosition(encoderR.getValue()-TURN_VAL)
    return direction

    
stop_robot()

leftWheel.setVelocity(MAX_SPEED) 
rightWheel.setVelocity(MAX_SPEED)

initial_pos = encoderL.getValue()

movement = 0

while robot.step(TIME_STEP) != -1:
    current_pos = encoderL.getValue()
    #print(f"current: {current_pos}, initial: {initial_pos}")
    #print(current_pos - initial_pos)
    if (movement == 0 and current_pos - initial_pos >= FORWARD_ONE):
        stopped = 0
    elif (movement == 1 and initial_pos - current_pos >= TURN_VAL -0.001):
        stopped = 0
    elif (movement == 2 and current_pos - initial_pos >= TURN_VAL -0.001):
        stopped = 0
    if stopped == 0:
        mark_current(current_pos_map)
        ir_values = measure_ir()
        mark_near(current_pos_map, direction, ir_values)
        print(map)
        print(current_pos_map)
        next_dir = decide_dir(current_pos_map, direction) # 0-Left, 1-Fwd, 2-Right, -1-Back
        print(next_dir)
        if(next_dir == 0 and movement != 1):
            movement = 1
            stopped = 1
            initial_pos = encoderL.getValue()
            direction = turn_left(direction)        
        elif(next_dir == 1):
            movement = 0
            stopped = 1
            initial_pos = encoderL.getValue()
            #print(direction)
            current_pos_map = move_fwd(current_pos_map, direction)
        elif(next_dir == 2):
            movement = 2
            stopped = 1
            initial_pos = encoderL.getValue()
            direction = turn_right(direction)