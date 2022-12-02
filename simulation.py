from robot_class import *
import threading
import signal
import sys
import pylab as pl
import matplotlib
matplotlib.use('TkAgg')

num_of_robots=2

dx=True

g_obstacles=[[15,16,5],[20,21,2],[30,40,4]]
sim="nmpc"
sim="pot"


r1=robot([20,25],1,1, True)
r2=robot([15,37],2,1, True)

def sigint_handler(signal, frame):
    print('Interrupted')
    sys.exit(0)
signal.signal(signal.SIGINT, sigint_handler)

r1.get_sensor_readings_and_update()
r1.get_obstacles_fit()

r2.get_sensor_readings_and_update()
r2.get_obstacles_fit()

prev_pos_list=[r1.cur_pos, r2.cur_pos]
# r1.map.display_robot_map()

display_local_maps=False

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


def execute_simulation(rob1, rob2):
    global prev_pos_list

    robots=[rob1, rob2]
    rand=random.sample(range(0, len(rob1.map.frontiers)), 1)        
    rob1.update_goal(rob1.map.frontiers[rand[0]])
    rob1.update_goal(rob2.cur_pos)
    rob1.find_path_to_goal(True)
    end_flag=False
    if display_local_maps:
        pl.subplot(131)
    else:
        pl.figure()
    pl.axis([0,100,0,100])
    pl.plot(rob1.path_x, rob1.path_y,'-k')
    print(" robot 1 path x:", rob1.path_x)
    print(" robot 1 path y:", rob1.path_y)
    # plt.show()
    rand=random.sample(range(0, len(rob2.map.frontiers)), 1)        
    rob2.update_goal(rob2.map.frontiers[rand[0]])
    rob2.update_goal(rob1.cur_pos)
    # plt.show()
    rob2.find_path_to_goal(True)
    pl.plot(rob2.path_x, rob2.path_y, '-g')
    print(" robot 2 path x:", rob2.path_x)
    print(" robot 2 path y:", rob2.path_y)

    plt.pause(1)
    t=0
    if not (sim=="nmpc"):
        rob1.state = State(x=rob1.path_x[0], y=rob1.path_y[0], yaw=rob1.path_yaw[0], v=0.1)
        rob2.state = State(x=rob2.path_x[0], y=rob2.path_y[0], yaw=rob2.path_yaw[0], v=0.1)
    else:
        rob1.state=[rob1.cur_pos[0], rob1.cur_pos[1], 0, 0]
        rob2.state=[rob2.cur_pos[0], rob2.cur_pos[1], 0, 0]

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
                        robots[iter].other_rob_pos[r.ID]=r.cur_pos
        if not rob1.reached_goal:
            rob1.drive_along_path()
        if not rob2.reached_goal:
            rob2.drive_along_path()    
        if rob1.reached_goal and rob2.reached_goal:
            break
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
    # rob2.drive_along_path()

        if t%75==0 :
            cur_pos_list=[rob1.cur_pos, rob2.cur_pos]
            rob1.get_sensor_readings_and_update()
            rob1.get_obstacles_fit()
            # print("Obstacles near robot ", self.ID," : ", self.obstacle_list)


            rob2.get_sensor_readings_and_update()
            rob2.get_obstacles_fit()

            # print("Obstacles near robot ", self.ID," : ", self.obstacle_list)
            if check_for_major_change_map(prev_pos_list, cur_pos_list):
                rob1.map.gmap.fit_ellipse_over_frontier(rob1.map.frontiers_x, rob1.map.frontiers_y, rob1.ID)

                rob2.map.gmap.fit_ellipse_over_frontier(rob2.map.frontiers_x, rob2.map.frontiers_y, rob2.ID)

                rob1.comm_and_update(2)
                rob2.comm_and_update(1)
                if display_local_maps:
                    pl.subplot(132)
                    rob1.map.display_robot_map()

                    pl.subplot(133)
                    rob2.map.display_robot_map()
                    pl.pause(0.5)
                prev_pos_list=cur_pos_list


execute_simulation(r1, r2)


# r2.get_sensor_readings_and_update()
# r2.get_obstacles_fit()


# t = threading.Thread(target=execute_simulation, args=(r1,))
# try:
# #     # Start the thread
#     t.start()
# #     # If the child thread is still running
#     while t.is_alive():
#         # Try to join the child thread back to parent for 0.5 seconds
#         t.join(0.5)    
# except KeyboardInterrupt:
#     print(InterruptedError)
#     t.join()
# # r1.map.display_robot_map()
# print(r1.obstacle_list)

# # r1=robot([25,25],1,1)
# # r1.get_sensor_readings_and_update()
# # r1.get_obstacles_fit()
# # r1.map.display_robot_map()
# # print(r1.obstacle_list)
# time.sleep(2)

# # r1.get_sensor_readings_and_update()
# print(r1.obstacle_list)
# r1.map.display_robot_map()

# # r1.get_obstacles_fit()
# r1.update_goal([39,31])
# r1.find_path_to_goal(True)
# r1.drive_along_path()
# # r1.get_sensor_readings_and_update()
# r1.map.display_robot_map()
# print(r1.obstacle_list)


