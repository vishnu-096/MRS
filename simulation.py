from robot_class import *
import threading
import signal
import sys

def execute_simulation(rob):
    rand=random.sample(range(0, len(rob.map.frontiers)), 1)        
    rob.update_goal(rob.map.frontiers[rand[0]])
    rand=random.sample(range(0, len(rob.map.frontiers)), 1)        
    rob.update_goal(rob.map.frontiers[rand[0]])

    rob.find_path_to_goal(True)
    rob.drive_along_path()

    plt.close()
    rob.map.display_robot_map()
    plt.pause(4)


r1=robot([20,25],1,1)
r2=robot([45,37],1,1)

def sigint_handler(signal, frame):
    print('Interrupted')
    sys.exit(0)
signal.signal(signal.SIGINT, sigint_handler)

r1.get_sensor_readings_and_update()
r1.get_obstacles_fit()
r2.get_sensor_readings_and_update()
r2.get_obstacles_fit()


t = threading.Thread(target=execute_simulation, args=(r1,))
try:
    # Start the thread
    t.start()
    # If the child thread is still running
    while t.is_alive():
        # Try to join the child thread back to parent for 0.5 seconds
        t.join(0.5)    
except KeyboardInterrupt:
    print(InterruptedError)
    t.join()
# # r1.map.display_robot_map()
# print(r1.obstacle_list)

# # r1=robot([25,25],1,1)
# # r1.get_sensor_readings_and_update()
# # r1.get_obstacles_fit()
# # r1.map.display_robot_map()
# # print(r1.obstacle_list)
# time.sleep(2)

# # r1.get_sensor_readings_and_update()
print(r1.obstacle_list)
r1.map.display_robot_map()

# # r1.get_obstacles_fit()
# r1.update_goal([39,31])
# r1.find_path_to_goal(True)
# r1.drive_along_path()
# # r1.get_sensor_readings_and_update()
# r1.map.display_robot_map()
# print(r1.obstacle_list)


