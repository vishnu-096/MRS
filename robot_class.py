from map import *
import numpy as np
from lqr_pid import *
from global_map import *
import pylab as pl
import random
import time
import threading
import cv2
from nmpc import *

ROBOT_RADIUS = 1.5
VMAX = 2
VMIN = 0.1

# collision cost parameters
Qc = 5.
kappa = 4.

# nmpc parameters
HORIZON_LENGTH = int(4)
NMPC_TIMESTEP = 0.3
upper_bound = [(1/np.sqrt(2)) * VMAX] * HORIZON_LENGTH * 2
lower_bound = [-(1/np.sqrt(2)) * VMAX] * HORIZON_LENGTH * 2


class robot:
    def __init__(self, start_pos, ID, team_num, simultaneous_plot) -> None:
        self.map=GridMap()
        self.map.gmap=gmap
        self.window_size=[6,6]
        self.map.generate_grid_map(100, 100, 0,0, [start_pos[0], start_pos[1]])
        self.obstacle_found=False
        

        self.start_position=start_pos
        self.local_map_pos=[start_pos[1],  start_pos[0]]
        self.ID=ID
        self.team_num=team_num
        self.local_planner_ID=0
        self.cur_pos=start_pos
        self.goal=[0,0]
        self.sensory_radius=10
        self.obstacle_list=[]
        self.time=0
        self.state=[self.cur_pos[0], self.cur_pos[1], 0, 0]
        self.simultaneous_plot=simultaneous_plot
        self.reached_goal=False

        self.path_x=[]
        self.path_y=[]
        self.path_k=[]
        self.path_yaw=[]
        self.other_rob_pos={}
        self.max_bounds={}

        # self.scanning_thread = threading.Thread(target=self.scan_for_obstacles, args=())
        # self.scanning_thread.start()
        

    def update_goal(self, goal):
        self.goal=[goal[1],  goal[0]]

    def convert_to_local_coord(self, x, y):
        tmp_x=[]
        tmp_y=[]
        for x1,y1 in zip(x,y):
            tmp_x.append(y1)
            tmp_y.append( x1)

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
                    print("Obstacles near robot ", self.ID," : ", self.obstacle_list)
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
            # print(tmp_list)
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

            r=r_sum/len(randomlist)
            ind=int(obs_key-3)
            # print("helloo",ind, len(self.obstacle_list))
            tmp=xmean
            xmean=ymean
            ymean= xmean
            if ind < len(self.obstacle_list):
                self.obstacle_list[ind]=[xmean,ymean,r]
            else:
                # print("inserting!")
                self.obstacle_list.append([xmean,ymean,r])

    def find_path_to_goal(self, show_animation):
        obstacle_list=[]  
        fov_r=0.5
        x_path=[]
        y_path=[]
        show_animation=not(self.simultaneous_plot)

        min_val=min(self.map.cur_map_bound[0])
        max_val=max(self.map.cur_map_bound[1])
        # Set Initial parameters
        # print(min_val)
        # print(max_val)
        # print("obstacles:", self.obstacle_list)
        rrt_star = RRTStar(
            start=self.local_map_pos,
            goal=self.goal,
            rand_area=[min_val, max_val],
            obstacle_list=self.obstacle_list,
            expand_dis=1,
            robot_radius=0.8)
        path = rrt_star.planning(animation=show_animation)

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
                pl.plot(x_path, y_path, 'r--')
                pl.grid(True)
        for (x,y) in path:
            x_path.append(x)
            y_path.append(y)
        x_path.reverse()
        y_path.reverse()
        print("first path_x ", x_path)
        print("first path_y ", y_path)
        x_path=x_path[:-1]
        y_path=y_path[:-1]
        goal = [x_path[-1], y_path[-1]]

        cyaw=[]
        cx=[]
        cy=[]
        cx1, cy1, cyaw1, ck, s = calc_spline_course(
                x_path, y_path, ds=0.5)
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

    def simulate(self):

        start = np.array([self.state[0], self.state[1]])
        p_desired = np.array([self.goal[1], self.goal[0]])
        NUMBER_OF_TIMESTEPS=1
        goal_thresh=0.5
        robot_state = start
        robot_state_history = np.empty((4, NUMBER_OF_TIMESTEPS))
        pl.plot(self.goal[1],self.goal[0],'xk')

        for i in range(NUMBER_OF_TIMESTEPS):
            # predict the obstacles' position in future
            obstacle_predictions=np.zeros((len(self.other_rob_pos),4))
            ind=0
            for rob_id in self.other_rob_pos:

                pos=self.other_rob_pos[rob_id]
                print("hellooo",pos, ind)
                obstacle_predictions[ind][0] = pos[0]
                obstacle_predictions[ind][1] = pos[1]
                obstacle_predictions[ind][2] = pos[2]
                obstacle_predictions[ind][3] = pos[3]         

                ind+=1
            # ob_iter=0
            # if ind==0:
            #     ind=1
            # for ob in self.obstacle_list:
            #     obstacle_predictions[ind-1][0]=ob[0]
            #     obstacle_predictions[ind-1][1]=ob[1]
            #     obstacle_predictions[ind-1][2]=0
            #     obstacle_predictions[ind-1][3]=0
            #     ind+=1
            obstacle_predictions = predict_obstacle_positions(obstacle_predictions)    
            xref = compute_xref(robot_state, p_desired,
                                HORIZON_LENGTH, NMPC_TIMESTEP)
            # compute velocity using nmpc
            vel, velocity_profile = compute_velocity(
                robot_state, obstacle_predictions, xref)
            robot_state = update_state(robot_state, vel, TIMESTEP)
            robot_state_history[:2, i] = robot_state
            self.state=[robot_state[0],robot_state[1], vel[0], vel[1]]
            self.cur_pos=self.state[:2]
        if np.linalg.norm(np.array([self.cur_pos[0], self.cur_pos[1]])- p_desired)<0.2:
            self.reached_goal=True


    def plot_circle(self, x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        print(x,y,size)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        pl.plot(xl, yl, color)

    def do_simulation(self, cx, cy, cyaw, ck, speed_profile, goal):
        T = 100.0  # max simulation time
        dt=0.1
        goal_dis = 0.8
        stop_speed = 0.05
        show_animation=not(self.simultaneous_plot)
        print("doing sim!!")


        x = [self.state.x]
        y = [self.state.y]
        yaw = [self.state.yaw]
        v = [self.state.v]
        t = [0.0]

        e, e_th = 0.0, 0.0

        update_iter=0
        # while T >= self.time:
        if self.time >=T:
            print("Max simulation time reached! ")
            self.reached_goal=True 
            return
        dl, target_ind, e, e_th, ai = lqr_speed_steering_control(
            self.state, cx, cy, cyaw, ck, e, e_th, speed_profile, lqr_Q, lqr_R)

        self.state = update(self.state, ai, dl)


        if abs(self.state.v) <= stop_speed:
            target_ind += 1

        self.time = self.time + dt

        # check goal
        dx = self.state.x - goal[0]
        dy = self.state.y - goal[1]
        if math.hypot(dx, dy) <= goal_dis:
            print("Goal")
            self.reached_goal=True

        x.append(self.state.x)
        y.append(self.state.y)
        yaw.append(self.state.yaw)
        v.append(self.state.v)
        t.append(self.time)
        # print("Current state : x ",self.state.x," y ",self.state.y)
        self.cur_pos=[int(self.state.x), int(self.state.y)]
        update_iter+=1
        if self.time%1==0:
            self.get_sensor_readings_and_update()
            self.get_obstacles_fit()
            print("Obstacles near robot ", self.ID," : ", self.obstacle_list)

            self.map.gmap.fit_ellipse_over_frontier(self.map.frontiers_x, self.map.frontiers_y, self.ID)


        if target_ind % 1 == 0 and show_animation:
            # pl.cla()
            # for stopping simulation with the esc key.

            pl.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            pl.plot(cx, cy, "-r", label="course")
            pl.plot(x, y, "ob", label="trajectory")
            pl.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            # pl.axis("equal")
            # pl.xlim([0,100])
            # pl.ylim([0,100])
            for obs in self.obstacle_list:
                # print("hello theree!",obs)
                self.plot_circle(obs[0],obs[1],obs[2])
            pl.grid(True)
            # pl.title("speed[km/h]:" + str(round(state.v * 3.6, 2))
            #         + ",target index:" + str(target_ind))
            pl.pause(0.0001)
            
        return t, x, y, yaw, v

    def comm_and_update(self, robo_ID):
        tform=self.map.gmap.com_params[robo_ID][0]
        tt = np.linspace(0, 2*np.pi, 100)
        circle = np.stack((np.cos(tt), np.sin(tt)))
        xmean=self.map.gmap.com_params[robo_ID][1]
        ymean=self.map.gmap.com_params[robo_ID][2]
        fit = tform.dot(circle) + np.array([[xmean], [ymean]]) 
        fit_x=list(fit[0,:])
        fit_y=list(fit[1,:])

        # np.save("C:\\Users\\vpishara\\Downloads\\MRS\\fit_np"+str(self.ID)+".npy",fit)
        if not self.simultaneous_plot:
            color='xb'
            if self.ID==2:
                color='xk'
            pl.plot(fit_x, fit_y,color)
            cv2.waitKey(0)
            pl.pause(4)
        xmin,xmax,ymin, ymax=self.map.map_fill_boundary(fit_x, fit_y, robo_ID)
        self.max_bounds[robo_ID]=[xmin,xmax,ymin, ymax]


    def get_sensor_readings_and_update(self):
        self.map.rob_position=[int(self.cur_pos[0]), int(self.cur_pos[1])] 
        self.map.gmap=gmap

        # print("other robot", self.map.gmap.grid2D[2][50] )
        self.map.get_sensor_boundary(self.sensory_radius)
        self.map.gmap.fit_ellipse_over_frontier(self.map.frontiers_x, self.map.frontiers_y, self.ID)

        if self.map.object_found:
            return True
        else:
            False

    def get_obstacles_inside_window(self):
        step=self.window_size/2
        self.get_obstacles_fit()
        for obs in self.obstacle_list:
            x=obs[0]
            y=obs[1]
            r=obs[2]



