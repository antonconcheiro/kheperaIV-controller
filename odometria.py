from controller import Robot, Motor, DistanceSensor
import numpy as np
from collections import deque

TIME_STEP = 32
MAX_SPEED = 5
SQUARE_SIZE = 250
WHEEL_RADIUS = 21
FORWARD_ONE = SQUARE_SIZE / WHEEL_RADIUS
TURN_VAL = 4.05

INITIAL_POS = (1,4)

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

f_camera = robot.getDevice("camera")
f_camera.enable(TIME_STEP)

found_goal = False

stopped = 0

map = np.ones((14,14))
map[0],map[-1],map[:,0],map[:,-1] = 3,3,3,3
map[INITIAL_POS]  = 1
current_pos_map = INITIAL_POS
direction = 0

# 0-No obstacle, 1-No visited, 2-Visited, 3-Obstacle
def mark_pos(pos, ir):
    if map[pos]!=3:
        if ir > 190:
            map[pos] = 3
        elif map[pos] != 2:
            map[pos] = 0

def mark_current(pos):
    map[pos] = 2
    
def mark_near(pos, direction, ir):
    if direction%4 == 0:
        mark_pos((pos[0],pos[1]+1), ir[0]) # Left
        mark_pos((pos[0],pos[1]-1), ir[1]) # Right
        mark_pos((pos[0]+1,pos[1]), ir[2]) # Fwd
    elif direction%4 == 1:
        mark_pos((pos[0]-1,pos[1]), ir[0]) # Left
        mark_pos((pos[0]+1,pos[1]), ir[1]) # Right
        mark_pos((pos[0],pos[1]+1), ir[2]) # Fwd
    elif direction%4 == 2:
        mark_pos((pos[0],pos[1]-1), ir[0]) # Left
        mark_pos((pos[0],pos[1]+1), ir[1]) # Right
        mark_pos((pos[0]-1,pos[1]), ir[2]) # Fwd
    elif direction%4 == 3:
        mark_pos((pos[0]+1,pos[1]), ir[0]) # Left
        mark_pos((pos[0]-1,pos[1]), ir[1]) # Right
        mark_pos((pos[0],pos[1]-1), ir[2]) # Fwd
    #print(map)
        
def decide_dir(pos, dir):
    #print(pos, dir%4)
    if dir%4 == 0:
        array_directions = [
            map[(pos[0],pos[1]+1)],
            map[(pos[0]+1,pos[1])],
            map[(pos[0],pos[1]-1)]
        ] # Left, fwd, right
    elif dir%4 == 1:
        array_directions = [
            map[(pos[0]-1,pos[1])],
            map[(pos[0],pos[1]+1)],
            map[(pos[0]+1,pos[1])]
        ] # Left, fwd, right
    elif dir%4 == 2:
        array_directions = [
            map[(pos[0],pos[1]-1)],
            map[(pos[0]-1,pos[1])],
            map[(pos[0],pos[1]+1)]
        ] # Left, fwd, right
    elif dir%4 == 3:
        array_directions = [
            map[(pos[0]+1,pos[1])],
            map[(pos[0],pos[1]-1)],
            map[(pos[0]-1,pos[1])]
        ] # Left, fwd, right
    #print(array_directions)
    index = np.argmin(array_directions)
    if array_directions[0]==array_directions[1] and array_directions[1]==array_directions[2] and array_directions[0]==2:
        return -1
    elif array_directions[index] < 3:
        return index
    else:
        return 2
        
def get_nearest_empty(pos):
    aux=np.asarray(np.where(map==0))
    near = None
    min_dist = None
    for x in range(aux.shape[1]):
        zero_loc = (aux[:,x])
        dist = np.sqrt(pow((pos[0] - zero_loc[0]), 2) + pow(pos[1] - zero_loc[1], 2))
        if min_dist is None or dist<min_dist:
            min_dist = dist
            near = (zero_loc[0],zero_loc[1])
    return near
    
def bfs(start,goal):
    queue = deque([[start]])
    seen = set([start])
    while queue:
        path = queue.popleft()
        x, y = path[-1]
        #print((x,y),goal)
        if (y,x) == goal:
            return path
        for x2, y2 in ((x+1,y), (x-1,y), (x,y+1), (x,y-1)):
            if 0 <= x2 < 14 and 0 <= y2 < 14 and map[y2][x2] != 3 and (x2, y2) not in seen:
                queue.append(path + [(x2, y2)])
                seen.add((x2, y2))
                
def create_path(path):
    for cell in path:
        new_cell=(cell[1],cell[0])
        map[new_cell]=0

def stop_robot():
    leftWheel.setVelocity(0) 
    rightWheel.setVelocity(0)
    
def move_fwd(current_pos_map, direction):
    if direction%4 == 0:
        current_pos_map = (current_pos_map[0] + 1, current_pos_map[1])
    elif direction%4 == 1:
        current_pos_map = (current_pos_map[0], current_pos_map[1] + 1)
    elif direction%4 == 2:
        current_pos_map = (current_pos_map[0] - 1, current_pos_map[1])
    elif direction%4 == 3:
        current_pos_map = (current_pos_map[0], current_pos_map[1] - 1)
    incrementL = FORWARD_ONE
    incrementR = FORWARD_ONE
    leftWheel.setVelocity(MAX_SPEED) 
    rightWheel.setVelocity(MAX_SPEED)
    leftWheel.setPosition(encoderL.getValue() + incrementL)
    rightWheel.setPosition(encoderR.getValue() + incrementR)
    return current_pos_map
    
def measure_ir():
    front_add_value = 0
    if check_goal():
        front_add_value = 200
    return[l_ir_sensor.getValue(),
        r_ir_sensor.getValue(),
        f_ir_sensor.getValue() + front_add_value]
        
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
    

def check_goal():
    image = np.array(f_camera.getImageArray())
    red = int(image[266:486,72:408,0].mean())
    green = int(image[266:486,72:408,1].mean())
    blue = int(image[266:486,72:408,2].mean())
    #print("R: "+str(red)+" G: "+str(green)+" B: "+str(blue))
    if (red <= 110 and green >= 120 and blue <= 80):
        return True
    else:
        return False

    
stop_robot()

leftWheel.setVelocity(MAX_SPEED) 
rightWheel.setVelocity(MAX_SPEED)

initial_pos = encoderL.getValue()

movement = 0
mapping = 1

while robot.step(TIME_STEP) != -1:
    current_pos = encoderL.getValue()
    if (movement == 0 and current_pos - initial_pos >= FORWARD_ONE):
        stopped = 0
    elif (movement == 1 and initial_pos - current_pos >= TURN_VAL -0.004):
        stopped = 0
    elif (movement == 2 and current_pos - initial_pos >= TURN_VAL -0.004):
        stopped = 0
    if stopped == 0:
        found_goal = check_goal()
        mark_current(current_pos_map)
        ir_values = measure_ir()
        #print(ir_values)
        mark_near(current_pos_map, direction, ir_values)
        next_dir = decide_dir(current_pos_map, direction) # 0-Left, 1-Fwd, 2-Right
        if next_dir == -1:
            nearest = get_nearest_empty(current_pos_map)
            path=bfs(current_pos_map,nearest)
            create_path(path)
            if decide_dir(current_pos_map, direction) == -1:
                next_dir = 2
        print(map)
        #if mapping==1 or found_goal == False:
        if(next_dir == 0 and movement != 1):
            movement = 1
            stopped = 1
            initial_pos = encoderL.getValue()
            direction = turn_left(direction)        
        elif(next_dir == 1):
            movement = 0
            stopped = 1
            initial_pos = encoderL.getValue()
            current_pos_map = move_fwd(current_pos_map, direction)
        elif(next_dir == 2):
            movement = 2
            stopped = 1
            initial_pos = encoderL.getValue()
            direction = turn_right(direction)