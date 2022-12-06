from map import *
import numpy as np
from lqr_pid import *

class robot:
    def __init__(self, start_pos, ID, team_num) -> None:
        self.map=GridMap()
        self.map.generate_grid_map(100, 100, 0,0, [start_pos[0], start_pos[1]])
        self.start_position=start_pos
        self.ID=ID
        self.team_num=team_num
        self.local_planner_ID=0
        self.cur_pos=start_pos
        self.goal=[0,0]
        self.sensory_radius=10
        self.max_bounds = {}
        self.map.gmap = gmap

    def comm_and_update(self, robo_ID):
        tform=self.map.gmap.com_params[robo_ID][0]
        tt = np.linspace(0, 2*np.pi, 100)
        circle = np.stack((np.cos(tt), np.sin(tt)))
        xmean=self.map.gmap.com_params[robo_ID][1]
        ymean=self.map.gmap.com_params[robo_ID][2]
        fit = tform.dot(circle) + np.array([[xmean], [ymean]])
        fit_x=list(fit[0,:])
        fit_y=list(fit[1,:])
        xmin,xmax,ymin, ymax=self.map.map_fill_boundary(fit_x, fit_y, robo_ID)
        self.max_bounds[robo_ID]=[xmin,xmax,ymin, ymax]


    def update_goal(self, goal):
        self.goal=goal

    def find_path_to_goal(self, show_animation):
        obstacle_list=[]  
        fov_r=0.5
        x_path=[]
        y_path=[]

        min_val=min(self.map.cur_map_bound[0])
        max_val=max(self.map.cur_map_bound[1])
        # Set Initial parameters
        # print(min_val)
        # print(max_val)
        rrt_star = RRTStar(
            start=self.start_position,
            goal=self.goal,
            rand_area=[min_val, max_val],
            obstacle_list=obstacle_list,
            expand_dis=1,
            robot_radius=0.8)
        path = rrt_star.planning(animation=False)

        if path is None:
            print("Cannot find path")
        else:
            # print("found path!!")
            affan = 0

            # Draw final path
            
            if show_animation:
                # print("drawing graph")
                # rrt_star.draw_graph()
                iter=0
                for (x,y) in path:
                    if iter: 
                        x_path.append(x)
                        y_path.append(y)
                    iter=1
                # plt.plot(x_path, y_path, 'r--')
                # plt.grid(True)

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

    def do_simulation(self, cx, cy, cyaw, ck, speed_profile, goal):
        T = 500.0  # max simulation time
        goal_dis = 1
        stop_speed = 0.05
        show_animation=True
        # print("doing sim!!")
        state = State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)

        state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.1)

        time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]

        e, e_th = 0.0, 0.0

        while T >= time:
            # print("heyy111")
            dl, target_ind, e, e_th, ai = lqr_speed_steering_control(
                state, cx, cy, cyaw, ck, e, e_th, speed_profile, lqr_Q, lqr_R)

            state = update(state, ai, dl)

            if abs(state.v) <= stop_speed:
                target_ind += 1

            time = time + dt

            # check goal
            dx = state.x - goal[0]
            dy = state.y - goal[1]
            if math.hypot(dx, dy) <= goal_dis:
                # print("Goal")
                break

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)
            self.cur_pos=[int(state.x), int(state.y)]

            # if target_ind % 1 == 0 and show_animation:
            #     plt.cla()
            #     # print("heyy")
            #     # for stopping simulation with the esc key.
            #     plt.gcf().canvas.mpl_connect('key_release_event',
            #             lambda event: [exit(0) if event.key == 'escape' else None])
            #     plt.plot(cx, cy, "-r", label="course")
            #     plt.plot(x, y, "ob", label="trajectory")
            #     plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            #     plt.axis("equal")
            #     plt.grid(True)
            #     plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2))
            #             + ",target index:" + str(target_ind))
            #     plt.pause(0.0001)

        return t, x, y, yaw, v

    def get_sensor_readings_and_update(self):
        self.map.rob_position=self.cur_pos

        self.map.gmap=gmap
        # print("other robot", self.map.gmap.grid2D[2][50] )
        self.map.get_sensor_boundary(self.sensory_radius)
        self.map.gmap.fit_ellipse_over_frontier(self.map.frontiers_x, self.map.frontiers_y, self.ID)


