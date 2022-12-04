from robot_class import *
import random
import math

# Test
# Initializations
def multiple_robots(no_of_robots):
    list1 = []  
    for i in range(no_of_robots):
        rbt = init_robot()
        list1.append(rbt)
    return list1 
    
def init_robot():
    # x = random.randint(8,8)
    x = 11
    y = random.randint(0,60)
    rbt = robot([x,y],1,1)
    rbt.get_sensor_readings_and_update()
    return rbt

# Explorations - levy walk with modifications. 
# Three different implementations:
    # 1. Selects random point on frontier
    # 2. Selects random point on frontier which is close to current position
    # 3. Selects random point on frontier which is far to current position
max_counter = 100
def explore_frontier_random(rbt):
    return random_index(rbt)

def explore_frontier_far(rbt):
    counter = 0
    while True:
        x = random_index(rbt)
        distance = euc_dist(rbt, x)
        if distance >= 25:
            print("Max counter NOT triggered")
            return x
        counter += 1
        if counter >= max_counter:
            print("Max counter triggered")
            return explore_frontier_random(rbt)

def explore_frontier_closeby(rbt):
    counter = 0
    while True:
        x = random_index(rbt)
        distance = euc_dist(rbt, x)
        if distance <= 7:
            print("Max counter NOT triggered")
            return x
        counter += 1
        if counter >= max_counter:
            print("Max counter triggered")
            return explore_frontier_random(rbt)

# Returns random index from the list of frontiers
def random_index(rbt):
    return random.randint(0, len(rbt.map.frontiers)-1)

############### WORK HERE ################

# Euclidean distance
def euc_dist(rbt1, x=None, rbt2=None):
    if rbt2 is None:
        return math.sqrt(((rbt1.map.frontiers[x][0] - rbt1.cur_pos[0]) ** 2 ) + ((rbt1.map.frontiers[x][1] - rbt1.cur_pos[1]) ** 2 ))
    else:
        a = 0
        ################# Yes to be implemented
        print("THIS FUNC IS NOT COMPLETE")
        return a

############### WORK HERE ENDS ################

# Robot actions
def move_to_goal(rbt, x):
    rbt.update_goal(rbt.map.frontiers[x])
    rbt.find_path_to_goal(True)
    rbt.drive_along_path()
    rbt.get_sensor_readings_and_update()
    return True


################# MAIN #################

# Initializing number of robots
robots = multiple_robots(2)

# First movement of robots
for r in range(len(robots)):
    move_to_goal(robots[r], explore_frontier_random(robots[r]))

# Exploration 
i, p = 0, 0
while i < 30:
    for r in range(len(robots)):
        if p == 0:
            move_to_goal(robots[r], explore_frontier_far(robots[r]))   
            print("Long walk")
        else:
        # x, dist = explore_frontier_random(robots[0])
            move_to_goal(robots[r], explore_frontier_closeby(robots[r]))
        i += 1
        
        # condition for fast wider exploration in the beginning
        if i > 10:
            p = random.randint(0,4)
        else:
            p = random.randint(0,1)

# Displaying explored areas
for r in range(len(robots)):
    plt.subplot(1,2,r+1) 
    robots[r].map.display_robot_map()
plt.show()