from robot_class import *
import random
import math
from collections import defaultdict

# Functions
# Initializations of robots
def multiple_robots(no_of_robots):
    list1 = []  
    for i in range(no_of_robots):
        rbt = init_robot(i)
        list1.append(rbt)
    for rbt in list1:
        for r in list1:
            if rbt.ID!=r.ID:
                rbt.comm_and_update(r.ID)
                temp=rbt.max_bounds[r.ID]
                bounds[r.ID]=[[temp[0],temp[2]], [temp[0],temp[3]], [temp[1],temp[2]], [temp[1],temp[3]]]

    return list1 
    
def init_robot(i):
    x = random.randint(0,10)
    # x = 10
    # y = random.randint(5,85)
    y = (i*35) % 80
    # x, y = 5,95
    rbt = robot([x,y], i, 1)
    rbt.get_sensor_readings_and_update()
    return rbt

# Explorations - levy walk with modifications. More comments in main section of code
# Three different implementations:
    # 1. Selects random point on frontier
max_counter = 100
def explore_frontier_random(rbt):
    return random_index(rbt)
    
    # 2. Selects random point on frontier which is close to current position
def explore_frontier_closeby(rbt):
    counter = 0
    while True:
        x = random_index(rbt)
        distance = euc_dist(rbt1=rbt, x=x)
        if distance <= 7:
            return x
        counter += 1
        if counter >= max_counter:
            return explore_frontier_random(rbt)

    # 3. Selects random point in the map which are far from current position.
        # This is implemented by choosing points that are at a distance greater than x and less than y from robots current positions
        # The distance of these points are found from the min and max (on x-axis and y-axis) of ellipsoid (is the map for explored area of other robots)
def explore_frontier_far(rbt, allrobots, minmax):
    listofindexforpoints = list()
    numberofpoints = 10
    costlist = defaultdict()
    
    mapxy = list()

    for nop in range(numberofpoints):
        counter = 0
        check = True
        while check:
            m_x = random.randint(1,95)
            m_y = random.randint(1,95)
            distance = euc_dist(rbt1=rbt, m_x=m_x, m_y=m_y)

            # if distance >= 25 and distance <50:
            if distance >= 25 and distance <45:
                mapxy.append([m_x, m_y])
                listofindexforpoints.append(nop)
                check = False
            
    for i, val in enumerate(allrobots):
        if val == rbt:
            continue
        else:
            for r in range(numberofpoints):
                minlist = list()
                
                for j in range(4):
                    minlist.append(euc_dist(minmax=minmax[rbt.ID][j], map=mapxy[r]))

                min_in_minlist = min(minlist)
                try:
                    costlist[listofindexforpoints[r]] += min_in_minlist
                except Exception:
                    costlist[listofindexforpoints[r]] = min_in_minlist

    maxdistancevalue = max(costlist.values())
    return mapxy[(list(costlist.values())).index(maxdistancevalue)]

# Returns random index from the list of frontiers
def random_index(rbt):
    return random.randint(0, len(rbt.map.frontiers)-1)

# Euclidean distance
def euc_dist(rbt1=None, x=None, minmax=None, m_x=None, m_y=None, map=None):
    if minmax is None and m_x is None and map is None:
        return math.sqrt(((rbt1.map.frontiers[x][0] - rbt1.cur_pos[0]) ** 2 ) + ((rbt1.map.frontiers[x][1] - rbt1.cur_pos[1]) ** 2 ))
    elif minmax is not None and m_x is None and map is None:
        return math.sqrt(((rbt1.map.frontiers[x][0] - minmax[0]) ** 2 ) + ((rbt1.map.frontiers[x][1] - minmax[1]) ** 2 ))
    elif (m_x is not None and m_y is not None) and rbt1 is not None:
        return math.sqrt(((m_x - rbt1.cur_pos[0]) ** 2 ) + ((m_y - rbt1.cur_pos[1]) ** 2 ))
    elif map is not None and minmax is not None:
        return math.sqrt(((map[0] - minmax[0]) ** 2 ) + ((map[1] - minmax[1]) ** 2 ))
    else:
        a = 0
        ################# Yet to be implemented
        print("THIS FUNC IS NOT COMPLETE")
        return a

# Robot actions
def move_to_goal(rbt, robots, x=None, farpoint=None):
    copy = rbt
    if farpoint is None:
        rbt.update_goal(rbt.map.frontiers[x])
    else:
        rbt.update_goal(farpoint)

    rbt.find_path_to_goal(True)
    try:
        rbt.drive_along_path()
        rbt.get_sensor_readings_and_update()
    except Exception:
        rbt = copy
    
    for r in robots:
            if r.ID != rbt.ID:
                rbt.comm_and_update(r.ID)
                temp=rbt.max_bounds[r.ID]
                bounds[r.ID]=[[temp[0],temp[2]], [temp[0],temp[3]], [temp[1],temp[2]], [temp[1],temp[3]]] 

    return True

########################################
################# MAIN #################
########################################

# Initializing number of robots
bounds={}
robots = multiple_robots(3)

# # First movement of robots
for _ in range(1):
    for r in range(len(robots)):
        move_to_goal(robots[r], robots, x=explore_frontier_random(robots[r]))

for r in range(len(robots)):
    plt.subplot(1,len(robots),r+1) 
    robots[r].map.display_robot_map()
plt.savefig("Initialpoints.png")
# plt.show()

# # Exploration 
i, p = 0, 0
while i < 50:
# while i < 30:
    for r in range(len(robots)):
        if p == 0:
            try:
                move_to_goal(robots[r], robots, farpoint=explore_frontier_far(robots[r], robots, bounds))   
            except Exception:
                continue
        else:
            move_to_goal(robots[r], robots, x=explore_frontier_closeby(robots[r]))
        
        i += 1
        
        # condition for fast wider exploration in the beginning
        if i > 10:
            p = random.randint(0,4)
        else:
            p = random.randint(0,1)
        
        print("Iteration # ", i)

# Displaying explored areas
for r in range(len(robots)):
    plt.subplot(1,len(robots),r+1) 
    robots[r].map.display_robot_map()
plt.savefig("Finalpoints.png")
# plt.show()