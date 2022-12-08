from robot_class import *
import threading
import signal
import sys
import pylab as pl
import matplotlib
from collections import defaultdict

matplotlib.use('TkAgg')


num_of_robots=2

x = 20
y = 50
r1_start=[x,y]
r1 = robot(r1_start,1,1, True)

x = 40
y = 52
r2_start=[x,y]
r2 = robot(r2_start,2,1, True)

display_local_maps=False
sim="nmpc"
prev_pos_list=[]

def get_distance(pos1, pos2):
    return np.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)


def plot_circle(x, y, size, color="-b"):  # pragma: no cover
    deg = list(range(0, 360, 5))
    deg.append(0)
    print(x,y,size)
    xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
    yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
    pl.plot(xl, yl, color)

def check_for_major_change_map(prev_pos_list, cur_pos_list):
    major_change=False
    for ind in range(len(prev_pos_list)):
        p=prev_pos_list[ind]
        c=cur_pos_list[ind]
        if get_distance(p,c) >3:
            major_change=True
    return major_change


def execute_simulation(robots):

    global prev_pos_list
    
    object_found=False
    i, p = 0, 0
    pl.figure()
    while i < 1 or  (not object_found):
#         if p == 0:
#             # move_to_goal(robots[r], explore_frontier_far(robots[r]))   
#             try:
#                 move_to_goal(robots[r], explore_frontier_far(robots[r], robots, minmax))   
#             except Exception:
#                 continue
#             print("Long walk")
#         else:
#             move_to_goal(robots[r], explore_frontier_closeby(robots[r]))
#         i += 1
        iter=0
        robots[0].update_goal(robots[1].cur_pos)
        robots[1].update_goal(robots[0].cur_pos)
        i=i+1
        # rob1.find_path_to_goal(True)
        if display_local_maps:
            pl.subplot(131)
        else:
            pl.axis([0,100,0,100])
        # pl.plot(rob1.path_x, rob1.path_y,'-k')
        # print(" robot 1 path x:", rob1.path_x)
        # print(" robot 1 path y:", rob1.path_y)
        # plt.show()
        # plt.show()
        # rob2.find_path_to_goal(True)
        # pl.plot(rob2.path_x, rob2.path_y, '-g')
        # print(" robot 2 path x:", rob2.path_x)
        # print(" robot 2 path y:", rob2.path_y)

        plt.pause(1)
        t=0
        if not (sim=="nmpc"):
            for r in robots:
                r.state = State(x=r.path_x[0], y=r.path_y[0], yaw=r.path_yaw[0], v=0.1)
            # rob2.state = State(x=rob2.path_x[0], y=rob2.path_y[0], yaw=rob2.path_yaw[0], v=0.1)
        else:
            for r in robots:
                r.state=[r.cur_pos[0], r.cur_pos[1], 0, 0]
                r.state=[r.cur_pos[0], r.cur_pos[1], 0, 0]

        rob1=robots[0]
        rob2=robots[1]
        while(True):
            t+=1
            r_ind=0

            for iter in range(len(robots)):
                main_rob=robots[iter]
                print("Rob 1 ID: ", main_rob.ID)
                for r in robots:
                    if r.ID!=main_rob.ID:
                        print("Rob 2 ID", r.ID)
                        if int(get_distance(main_rob.cur_pos, r.cur_pos))<50:
                            robots[iter].other_rob_pos[r.ID]=[r.cur_pos[0],r.cur_pos[1],0,0]
            flag=True
            for iter in range(len(robots)):
                rob=robots[iter]
                if rob.get_sensor_readings_and_update():
                    print("OBJECT FOUNDD!!")
                    return                
                rob.get_obstacles_fit()

                if not rob.reached_goal:
                    if not (sim=="nmpc"):
                        rob.drive_along_path()
                    else:
                        # rob.other_rob_pos[rob2.ID]=[rob2.state[0], rob2.state[1],rob2.state[2], rob2.state[3]]
                        rob.simulate()
                # if not rob2.reached_goal:
                #     if not (sim=="nmpc"):

                #         rob2.drive_along_path()
                #     else:
                #         rob2.other_rob_pos[rob1.ID]=[rob1.state[0], rob1.state[1],rob1.state[2], rob1.state[3]]
                #         rob2.simulate()    
                flag=(flag and rob.reached_goal)

            if flag:
                break 

            # if not rob1.reached_goal:
            #     if not (sim=="nmpc"):
            #         rob1.drive_along_path()
            #     else:
            #         rob1.other_rob_pos[rob2.ID]=[rob2.state[0], rob2.state[1],rob2.state[2], rob2.state[3]]
            #         rob1.simulate()
            # if not rob2.reached_goal:
            #     if not (sim=="nmpc"):

            #         rob2.drive_along_path()
            #     else:
            #         rob2.other_rob_pos[rob1.ID]=[rob1.state[0], rob1.state[1],rob1.state[2], rob1.state[3]]
            #         rob2.simulate()    
            # if rob1.reached_goal and rob2.reached_goal:
            #     break

            
            if sim=="nmpc":
                if t%2==0:
                    if display_local_maps:
                        pl.subplot(131)
                        pl.cla()
                        pl.axis([0,100,0,100])
                    for ob in g_obstacles:
                        plot_circle(ob[0], ob[1], ob[2])

                    print("sim step", rob1.time)
                    # print("Other robot positions :",rob1.other_rob_pos)
                    # pos=rob1.other_rob_pos[rob2.ID]
                    pl.plot(rob1.state[0], rob1.state[1],'ob')
                    pl.plot(rob2.state[0], rob2.state[1],'oc')
                    plt.pause(0.5)

            if t%20==0:
                if display_local_maps:
                    pl.subplot(131)
                    pl.cla()
                    pl.axis([0,100,0,100])
                for ob in g_obstacles:
                    plot_circle(ob[0], ob[1], ob[2])

                print("sim step", rob1.time)
                print("Other robot positions :",rob1.other_rob_pos)
                pos=rob1.other_rob_pos[rob2.ID]

                if not(sim=="nmpc"):
                    pl.plot(rob1.state.x, rob1.state.y,'ob')
                    pl.plot(rob2.state.x, rob2.state.y,'oc')
                    plt.pause(0.5)
                else:
                    pl.plot(rob1.state[0], rob1.state[1],'ob')
                    pl.plot(pos[0], pos[1],'oc')
                    plt.pause(0.5)
                if rob1.get_sensor_readings_and_update():
                    print("OBJECT FOUNDD!!")
                    object_found=True
                    return     
                rob1.get_obstacles_fit()
                if rob2.get_sensor_readings_and_update():
                    print("OBJECT FOUNDD!!")
                    object_found=True
                    return     
                rob2.get_obstacles_fit()
        # rob2.drive_along_path()

            if t%50==0 :
                cur_pos_list=[rob1.cur_pos, rob2.cur_pos]
                # rob1.get_sensor_readings_and_update()
                # rob1.get_obstacles_fit()
                # # print("Obstacles near robot ", self.ID," : ", self.obstacle_list)


                # rob2.get_sensor_readings_and_update()
                # rob2.get_obstacles_fit()

                # print("Obstacles near robot ", self.ID," : ", self.obstacle_list)
                if check_for_major_change_map(prev_pos_list, cur_pos_list):
                    rob1.map.gmap.fit_ellipse_over_frontier(rob1.map.frontiers_x, rob1.map.frontiers_y, rob1.ID)

                    rob2.map.gmap.fit_ellipse_over_frontier(rob2.map.frontiers_x, rob2.map.frontiers_y, rob2.ID)

                    rob1.comm_and_update(2)
                    rob2.comm_and_update(1)
                    print("Max Bounds for 1 are: ", rob1.max_bounds)
                    print("Max Bounds for 2 are : ", rob2.max_bounds)
                    if display_local_maps:
                        pl.subplot(132)
                        rob1.map.display_robot_map()

                        pl.subplot(133)
                        rob2.map.display_robot_map()
                        pl.pause(0.5)
                    prev_pos_list=cur_pos_list

r1.cur_pos=[25,30]
r2.cur_pos=[30,32]
r1.get_sensor_readings_and_update()
r2.get_sensor_readings_and_update()

r1.cur_pos=[20,30]
r2.cur_pos=[35,32]
r1.get_sensor_readings_and_update()
r2.get_sensor_readings_and_update()


r1.comm_and_update(2)
r2.comm_and_update(1)

r1.map.display_robot_map()
pl.show()
r2.map.display_robot_map()
pl.show()
# execute_simulation([r1,r2])