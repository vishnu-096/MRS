from rrt_plan import *

# def get_potential_fun(b_min, b_max, obstacles, goal):


goal=[18,18]
start_position=[2,1]
obstacle_list=[]  
fov_r=0.5
x_path=[]
y_path=[]

min_val=min(-2)
max_val=max(25)
# Set Initial parameters
print(min_val)
print(max_val)
rrt_star = RRTStar(
    start=start_position,
    goal=goal,
    rand_area=[min_val, max_val],
    obstacle_list=obstacle_list,
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
path_x=cx
path_y=cy
path_yaw=cyaw
path_k=ck

obstacle_list=[[3,5,3],[10,15,2]]

