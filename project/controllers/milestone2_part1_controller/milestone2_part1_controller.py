"""This is only a template controller which simply drives the car forward
"""

from vehicle import Driver
import math
from collections import defaultdict
from heapq import *
import pandas as pd

driver = Driver()

# An example of how you get a sensor and initialize it.
# You are free to use all the sensors that comes with your car.
# Check Sensor Slot fields on the vehicle in the left pane of Webots for available sensors.

WORLD_TIME_STEP = 16  # This is already set in the world file.
FRONT_LIDAR_REFRESH_RATE = 160
RIGHT_LIDAR_REFRESH_RATE = 160
LEFT_LIDAR_REFRESH_RATE = 160
CAMERA_REFRESH_RATE = 32

top_lidar = driver.getLidar("Velodyne VLP-16")
top_lidar.enable(FRONT_LIDAR_REFRESH_RATE)
top_lidar.enablePointCloud()

right_lidar = driver.getLidar("Sick LMS 291 Right")
right_lidar.enable(RIGHT_LIDAR_REFRESH_RATE)
right_lidar.enablePointCloud()

left_lidar = driver.getLidar("Sick LMS 291 Left")
left_lidar.enable(LEFT_LIDAR_REFRESH_RATE)
left_lidar.enablePointCloud()

camera = driver.getCamera("camera")
camera.enable(CAMERA_REFRESH_RATE)

def dijkstra_raw(edges, from_node, to_node):
    g = defaultdict(list)
    for l,r,c in edges:
        g[l].append((c,r))
    q, seen = [(0,from_node,())], set()
    while q:
        (cost,v1,path) = heappop(q)
        if v1 not in seen:
            seen.add(v1)
            path = (v1, path)
            if v1 == to_node:
                return cost,path
            for c, v2 in g.get(v1, ()):
                if v2 not in seen:
                    heappush(q, (cost+c, v2, path))
    return float("inf"),[]

def dijkstra(edges, from_node, to_node):
    len_shortest_path = -1
    ret_path=[]
    length,path_queue = dijkstra_raw(edges, from_node, to_node)
    if len(path_queue)>0:
        len_shortest_path = length		## 1. Get the length firstly;
        ## 2. Decompose the path_queue, to get the passing nodes in the shortest path.
        left = path_queue[0]
        ret_path.append(left)		## 2.1 Record the destination node firstly;
        right = path_queue[1]
        while len(right)>0:
            left = right[0]
            ret_path.append(left)	## 2.2 Record other nodes, till the source-node.
            right = right[1]
        ret_path.reverse()	## 3. Reverse the list finally, to make it be normal sequence.
    return len_shortest_path,ret_path
	
list_nodes_id = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16];

M=99999

M_topo = [
[M, 1,M,M,1,M, M,M,M,M,M, M,M,M,M,M],
[1, M,1,M,M,1, M,M,M,M,M, M,M,M,M,M],
[M, 1,M,1,M,M, 1,M,M,M,M, M,M,M,M,M],
[M, M,1,M,M,M, M,1,M,M,M, M,M,M,M,M],
[1, M,M,M,M,1, M,M,1,M,M, M,M,M,M,M],
[M, 1,M,M,1,M, 1,M,M,1,M, M,M,M,M,M],
[M, M,1,M,M,1, M,1,M,M,1, M,M,M,M,M],
[M, M,M,1,M,M, 1,M,M,M,M, 1,M,M,M,M],
[M, M,M,M,1,M, M,M,M,1,M, M,1,M,M,M],
[M, M,M,M,M,1, M,M,1,M,1, M,M,1,M,M],
[M, M,M,M,M,M, 1,M,M,1,M, 1,M,M,1,M],
[M, M,M,M,M,M, M,1,M,M,1, M,M,M,M,1],
[M, M,M,M,M,M, M,M,1,M,M, M,M,1,M,M],
[M, M,M,M,M,M, M,M,M,1,M, M,1,M,1,M],
[M, M,M,M,M,M, M,M,M,M,1, M,M,1,M,1],
[M, M,M,M,M,M, M,M,M,M,M, 1,M,M,1,M],
]

dataset = pd.read_csv('plan.csv')
X = dataset.iloc[:, 0:2].values
initial = X[0,0]
start = X[0,1]
goal = X[1,0]
final = X[1,1]

if start == 1 and final == 1:
    if initial == 2 and goal == 5:
        Shortest_path = [1,5,9,10,6,5]
    elif initial == 5 and goal == 2:
        Shortest_path = [1,2,3,7,6,2]
elif initial == 1 and goal == 1:
    if start == 2 and final == 5:
        Shortest_path = [2,3,7,6,2,1]
    elif start == 5 and final == 2:
        Shortest_path = [5,9,10,6,5,1]
elif start == 4 and final == 4:
    if initial == 3 and goal == 8:
        Shortest_path = [4,8,12,11,7,8]
    elif initial == 8 and goal == 3:
        Shortest_path = [4,3,2,6,7,3]
elif initial == 4 and goal == 4:
    if start == 3 and final == 8:
        Shortest_path = [3,2,6,7,3,4]
    elif start == 8 and final == 3:
        Shortest_path = [8,12,11,7,8,4]
elif start == 13 and final == 13:
    if initial == 9 and goal == 14:
        Shortest_path = [13,14,15,11,10,14]
    elif initial == 14 and goal == 9:
        Shortest_path = [13,9,5,6,10,9]
elif initial == 13 and goal == 13:
    if start == 9 and final == 14:
        Shortest_path = [9,5,6,10,9,13]
    elif start == 14 and final == 9:
        Shortest_path = [14,15,11,10,14,13]
elif start == 16 and final == 16:
    if initial == 12 and goal == 15:
        Shortest_path = [16,15,14,10,11,15]
    elif initial == 15 and goal == 12:
        Shortest_path = [16,12,8,7,11,12]
elif initial == 16 and goal == 16:
    if start == 12 and final == 15:
        Shortest_path = [12,8,7,11,12,16]
    elif start == 15 and final == 12:
        Shortest_path = [15,14,10,11,15,16]
else:
    M_topo[final - 1][goal - 1] = M
    M_topo[goal - 1][final - 1] = M
    M_topo[start - 1][initial - 1] = M
    edges = []	
    for i in range(len(M_topo)):
        for j in range(len(M_topo[0])):
            if M_topo[i][j]!=M:
                edges.append((i + 1,j + 1,M_topo[i][j]))
    length,Shortest_path = dijkstra(edges, start, goal)

action = [None]*len(Shortest_path)# 1 means staright ,2 means turn left, 3 means turn right
heading = [None]*(len(Shortest_path)+1) # 1 means up, 2 means down, 3 means left ,4 means right

if start - initial == -4:
    heading[0] = 1 # 1 means up
elif start - initial == 4:
    heading[0] = 2 # 2 means down
elif start - initial == -1:
    heading[0] = 3 # 3 means left
elif start - initial == 1:
    heading[0] = 4 # 4 means right     

if len(Shortest_path) >= 2:
    for i in range(1,len(Shortest_path)):
        if heading[i - 1] == 1:
            if Shortest_path[i] - Shortest_path[i - 1] == -1:
                action[i - 1] = 2 # turn left
                heading[i] = 3
            elif Shortest_path[i] - Shortest_path[i - 1] == -4:
                action[i - 1] = 1 #straight
                heading[i] = 1
            elif Shortest_path[i] - Shortest_path[i - 1] == 1:
                action[i - 1] = 3 # turn right
                heading[i] = 4
        elif heading[i - 1] == 2:
            if Shortest_path[i] - Shortest_path[i - 1] == -1:
                action[i - 1] = 3 # turn right
                heading[i] = 3
            elif Shortest_path[i] - Shortest_path[i - 1] == 4:
                action[i - 1] = 1 #straight
                heading[i] = 2
            elif Shortest_path[i] - Shortest_path[i - 1] == 1:
                action[i - 1] = 2 # turn left
                heading[i] = 4
        elif heading[i - 1] == 3:
            if Shortest_path[i] - Shortest_path[i - 1] == -1:
                action[i - 1] = 1 # straight
                heading[i] = 3
            elif Shortest_path[i] - Shortest_path[i - 1] == -4:
                action[i - 1] = 3 # turn right
                heading[i] = 1
            elif Shortest_path[i] - Shortest_path[i - 1] == 4:
                action[i - 1] = 2 # turn left
                heading[i] = 2
        elif heading[i - 1] == 4:
            if Shortest_path[i] - Shortest_path[i - 1] == 1:
                action[i - 1] = 1 # straight
                heading[i] = 4
            elif Shortest_path[i] - Shortest_path[i - 1] == -4:
                action[i - 1] = 2 # turn left
                heading[i] = 1
            elif Shortest_path[i] - Shortest_path[i - 1] == 4:
                action[i - 1] = 3 # turn right
                heading[i] = 2
            
    if final - goal == -4:
        heading[len(Shortest_path)] = 1 # 1 means up
        if goal - Shortest_path[len(Shortest_path) - 2] == 1:
            action[len(Shortest_path) - 1] = 2 # turn left
        elif goal - Shortest_path[len(Shortest_path) - 2] == -1:
            action[len(Shortest_path) - 1] = 3 # turn right
        elif goal - Shortest_path[len(Shortest_path) - 2] == -4:
            action[len(Shortest_path) - 1] = 1 # straight
    elif final - goal == 4:
        heading[len(Shortest_path)] = 2 # 2 means down
        if goal - Shortest_path[len(Shortest_path) - 2] == 1:
            action[len(Shortest_path) - 1] = 3 # turn right
        elif goal - Shortest_path[len(Shortest_path) - 2] == -1:
            action[len(Shortest_path) - 1] = 2 # turn left
        elif goal - Shortest_path[len(Shortest_path) - 2] == 4:
            action[len(Shortest_path) - 1] = 1 # straight
    elif final - goal == -1:
        heading[len(Shortest_path)] = 3 # 3 means left
        if goal - Shortest_path[len(Shortest_path) - 2] == -1:
            action[len(Shortest_path) - 1] = 1 # staright
        elif goal - Shortest_path[len(Shortest_path) - 2] == 4:
            action[len(Shortest_path) - 1] = 3 # turn right
        elif goal - Shortest_path[len(Shortest_path) - 2] == -4:
            action[len(Shortest_path) - 1] = 2 # turn left
    elif final - goal == 1:
        heading[len(Shortest_path)] = 4 # 4 means right
        if goal - Shortest_path[len(Shortest_path) - 2] == 1:
            action[len(Shortest_path) - 1] = 1 # staright
        elif goal - Shortest_path[len(Shortest_path) - 2] == 4:
            action[len(Shortest_path) - 1] = 2 # turn left
        elif goal - Shortest_path[len(Shortest_path) - 2] == -4:
            action[len(Shortest_path) - 1] = 3 # turn right
    
if len(Shortest_path) == 1:
    if heading[0] == 1:
        if final - goal == -4:
            heading[1] = 1
            action[0] = 1
        elif final - goal == -1:
            heading[1] = 3
            action[0] = 2
        elif final - goal == 1:
            heading[1] = 4
            action[0] = 3
    elif heading[0] == 2:
        if final - goal == 4:
            heading[1] = 2
            action[0] = 1
        elif final - goal == -1:
            heading[1] = 3
            action[0] = 3
        elif final - goal == 1:
            heading[1] = 4
            action[0] = 2
    elif heading[0] == 3:
        if final - goal == -4:
            heading[1] = 1
            action[0] = 3
        elif final - goal == -1:
            heading[1] = 3
            action[0] = 1
        elif final - goal == 4:
            heading[1] = 2
            action[0] = 2
    elif heading[0] == 4:
        if final - goal == -4:
            heading[1] = 1
            action[0] = 2
        elif final - goal == 1:
            heading[1] = 4
            action[0] = 1
        elif final - goal == 4:
            heading[1] = 2
            action[0] = 3
    
print ('The shortest path is ',Shortest_path)
print ('heading',heading)
print ('action',action)

current_time_ms = 0
k = 0 # the number of intersections that the vehicle have countered
state = 1 # the original state of the car is driving straightly
d_left_mean = 0
d_right_mean = 0
flag = 2
time = 0

while driver.step() != -1:  # This is your control loop. Every step means 16ms has passed in the simulation.
# The following is only an example starting point. Feel free to change the way you read the sensors. (Check Webots Reference Manual)
    current_time_ms = current_time_ms + WORLD_TIME_STEP
    if current_time_ms % FRONT_LIDAR_REFRESH_RATE == 0:
        # I choose the 11th layer of the top lidar
        top_lidar_layer_11 = top_lidar.getLayerPointCloud(10)
        d = 0

        # choose the 1600th to 1999th(front) lidar point of the top lidar
        # use all the choosed lidar points' x and z to calculate the average 2D distance to lidar
        # this is used to avoid colliding with the front obstacles
        for i in range(0,399):
            d = d + math.sqrt((top_lidar_layer_11[i+1600].x) ** 2 + (top_lidar_layer_11[i+1600].z) ** 2) 
        d_mean = d / 400
        
    if current_time_ms % RIGHT_LIDAR_REFRESH_RATE == 0:
        right_lidar_layer = right_lidar.getLayerPointCloud(0);  # This lidar is a single-layer lidar.    
        d_right = 0
        
        # choose the 70th to 89th lidar point of the right lidar
        # use all the choosed lidar points' x and z to calculate the average 2D distance to lidar
        # this is used to detect the intersection
        for j in range(70,90):
            d_right = d_right + math.sqrt((right_lidar_layer[j].x) ** 2 + (right_lidar_layer[j].z) ** 2)
        d_right_mean = d_right / 20
        

    if current_time_ms % LEFT_LIDAR_REFRESH_RATE == 0:
        left_lidar_layer = left_lidar.getLayerPointCloud(0);  # This lidar is a single-layer lidar.
        d_left = 0
        
        # choose the 90th to 110th lidar point of the left lidar
        # use all the choosed lidar points' x and z to calculate the average 2D distance to lidar
        # this is used to detect the intersection
        for l in range(90,110):
            d_left = d_left + math.sqrt((left_lidar_layer[l].x) ** 2 + (left_lidar_layer[l].z) ** 2)
        d_left_mean = d_left / 20
        
    if d_left_mean - d_right_mean > 3:
        state = 4
    elif d_right_mean - d_left_mean > 1:
        state = 5
    else:
        state = 1
    
    if d_left_mean < 1:
        state = 5
    elif d_right_mean < 1:
        state = 4
    
    if flag == 1:
        flag = 0
    
    # the action when the vehicle is at the intersection
    if d_left_mean + d_right_mean > 10:
        if k < len(action):
            state = action[k]
            if state == 1 and d_right_mean - d_left_mean > 18:
                state = 4
        flag = 1
        
    if flag == 0:
        k = k + 1
        time = current_time_ms
        flag = 2
    
    if k >= len(action):
        state = 6
        
    #if current_time_ms % FRONT_LIDAR_REFRESH_RATE == 0:
        #print("left",d_left_mean)
        #print("right",d_right_mean)
        #print("state",state)
        #print("flag",flag)
        #print("k",k)
    
    #to make sure the vehicle has already crossed the intersection
    #if current_time_ms - time >640
        #k = k + 1 会导致k不停的增加
    
    # when there is no obstacle or barrier which is quite close to the car,state = 0
    if state == 1:       
        driver.setCruisingSpeed(20.0) 
        driver.setBrakeIntensity(0)
        driver.setSteeringAngle(0.0)
    # the vehichle is at the intersection and needs to turn left 
    elif state == 2:
        driver.setCruisingSpeed(10.0)
        driver.setBrakeIntensity(0.5)
        driver.setSteeringAngle(-0.4)
    # the vehicle is at the intersection and needs to turn right
    elif state == 3:
        driver.setCruisingSpeed(10.0)
        driver.setBrakeIntensity(0.5)
        driver.setSteeringAngle(0.4)
    # the vehicle is quite close to the barrier on its right
    elif state == 4:
        driver.setCruisingSpeed(15.0)
        driver.setBrakeIntensity(0.0)
        driver.setSteeringAngle(-0.2)
    # the vehicle is quite close to the barrier on its left
    elif state == 5:
        driver.setCruisingSpeed(15.0)
        driver.setBrakeIntensity(0.0)
        driver.setSteeringAngle(0.2)
    # the vehicle is near the goal
    else:
        driver.setCruisingSpeed(0.0)
        driver.setBrakeIntensity(0.0)
        driver.setSteeringAngle(0.0)
