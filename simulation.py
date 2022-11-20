from robot_class import *

r1=robot([20,25],1,1)
r1.get_sensor_readings_and_update()
r1.map.display_robot_map()
r1.update_goal([29,31])
r1.find_path_to_goal(True)
r1.drive_along_path()
r1.get_sensor_readings_and_update()
r1.map.display_robot_map()

