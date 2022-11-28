from map import *
import numpy as np
from lqr_pid import *
from global_map import *
import pylab as pl
import random
import time
import threading

class robot:
    def __init__(self, start_pos, ID, team_num) -> None:
        self.map=GridMap()
        self.map.gmap=gmap
        self.window_size=[6,6]
        self.map.generate_grid_map(100, 100, 0,0, [start_pos[0], start_pos[1]])
        self.obstacle_found=False
        

        self.start_position=start_pos
        self.ID=ID
        self.team_num=team_num
        self.local_planner_ID=0
        self.cur_pos=start_pos
        self.goal=[0,0]
        self.sensory_radius=10
        self.obstacle_list=[]
        self.time=0
        # self.scanning_thread = threading.Thread(target=self.scan_for_obstacles, args=())
        # self.scanning_thread.start()
        

    def update_goal(self, goal):
        self.goal=goal

    def scan_for_obstacles(self):
        cur_time=self.time
        self.get_sensor_readings_and_update()

        self.get_obstacles_fit()
        try:
            while(True):
                # print("thread inside")
                if cur_time-self.time>1:
                
                    self.get_sensor_readings_and_update()
                    cur_time=self.time
                    self.get_obstacles_fit()
        except:
            print("Ending Thread!")
            return

    def f_2(self,c):
        """ calculate the algebraic distance between the data points and the mean circle centered at c=(xc, yc) """
        Ri = self.calc_r(*c)
        return Ri - Ri.mean()

    def get_obstacles_fit(self):
        
        key_list=[]
        for key in self.map.obstacles:
            key_list.append(key)

        key_list.sort()

        for obs_key in key_list:
            tmp_list=self.map.obstacles[obs_key]
            print(tmp_list)
            x=[]
            y=[]
            randomlist = random.sample(range(0, len(tmp_list)), min(5,len(tmp_list)))

            r_sum=0
            for tmp_x,tmp_y in tmp_list:
                x.append(tmp_x)
                y.append(tmp_y)

            N=len(x)
            x=np.array(x,dtype=float)
            y=np.array(y,dtype=float)
            xmean, ymean = x.mean(), y.mean()
            for ind in randomlist:
                r_sum+=math.sqrt((x[ind]-xmean)**2 + (y[ind]-ymean)**2)               

            r=r_sum/len(x)
            ind=int(obs_key-3)
            print("helloo",ind, len(self.obstacle_list))
            if ind < len(self.obstacle_list):
                self.obstacle_list[ind]=[xmean,ymean,r]
            else:
                print("inserting!")
                self.obstacle_list.append([xmean,ymean,r])

    def find_path_to_goal(self, show_animation):
        obstacle_list=[]  
        fov_r=0.5
        x_path=[]
        y_path=[]

        min_val=min(self.map.cur_map_bound[0])
        max_val=max(self.map.cur_map_bound[1])
        # Set Initial parameters
        print(min_val)
        print(max_val)
        print("obstacles:", self.obstacle_list)
        rrt_star = RRTStar(
            start=self.start_position,
            goal=self.goal,
            rand_area=[min_val, max_val],
            obstacle_list=self.obstacle_list,
            expand_dis=1,
            robot_radius=0.8)
        path = rrt_star.planning(animation=True)

        if path is None:
            print("Cannot find path")
        else:
            print("found path!!")

            # Draw final path
            
            if show_animation:
                print("drawing graph")
                rrt_star.draw_graph()
                iter=0
                for (x,y) in path:
                    if iter: 
                        x_path.append(x)
                        y_path.append(y)
                    iter=1
                plt.plot(x_path, y_path, 'r--')
                plt.grid(True)

        x_path.reverse()
        y_path.reverse()

        goal = [x_path[-1], y_path[-1]]
        cyaw=[]
        cx=[]
        cy=[]
        cx1, cy1, cyaw1, ck, s = calc_spline_course(
                x_path, y_path, ds=0.1)
        cx= [round(num, 1) for num in cx1]
        cy= [round(num, 1) for num in cy1]
        cyaw= [round(num, 1) for num in cyaw1]
        self.path_x=cx
        self.path_y=cy
        self.path_yaw=cyaw
        self.path_k=ck
    
    def drive_along_path(self):
        target_speed = 10.0 / 3.6  # simulation parameter km/h -> m/s

        sp = calc_speed_profile(self.path_yaw, target_speed)

        t, x, y, yaw, v = self.do_simulation(self.path_x, self.path_y, self.path_yaw, self.path_k, sp, self.goal)

    def plot_circle(self, x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        print(x,y,size)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    def do_simulation(self, cx, cy, cyaw, ck, speed_profile, goal):
        T = 500.0  # max simulation time
        goal_dis = 0.8
        stop_speed = 0.05
        show_animation=True
        print("doing sim!!")
        state = State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)

        state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.1)

        self.time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]

        e, e_th = 0.0, 0.0

        update_iter=0
        while T >= self.time:

            dl, target_ind, e, e_th, ai = lqr_speed_steering_control(
                state, cx, cy, cyaw, ck, e, e_th, speed_profile, lqr_Q, lqr_R)

            state = update(state, ai, dl)

            if abs(state.v) <= stop_speed:
                target_ind += 1

            self.time = self.time + dt

            # check goal
            dx = state.x - goal[0]
            dy = state.y - goal[1]
            if math.hypot(dx, dy) <= goal_dis:
                print("Goal")
                break

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(self.time)
            self.cur_pos=[int(state.x), int(state.y)]
            update_iter+=1
            if update_iter%5==0:
                self.get_sensor_readings_and_update()
                self.get_obstacles_fit()
            if target_ind % 1 == 0 and show_animation:
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                plt.plot(cx, cy, "-r", label="course")
                plt.plot(x, y, "ob", label="trajectory")
                plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                plt.axis("equal")
                plt.xlim([0,100])
                plt.ylim([0,100])
                for obs in self.obstacle_list:
                    print("hello theree!",obs)
                    self.plot_circle(obs[0],obs[1],obs[2])
                plt.grid(True)
                plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2))
                        + ",target index:" + str(target_ind))
                plt.pause(0.0001)
                
        return t, x, y, yaw, v

    def get_sensor_readings_and_update(self):
        self.map.rob_position=self.cur_pos
        self.map.get_sensor_boundary(self.sensory_radius)

    def get_obstacles_inside_window(self):
        step=self.window_size/2
        self.get_obstacles_fit()
        for obs in self.obstacle_list:
            x=obs[0]
            y=obs[1]
            r=obs[2]



