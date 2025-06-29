from fairis_tools.engjelina_Lab3_Task1_MyRobot import MyRobot
import math
endrit_ngjelina = MyRobot()
maze_file = '../../worlds/Fall24/maze5.xml'
endrit_ngjelina.load_environment(maze_file)
endrit_ngjelina.move_to_start()
speed_to_rotate = 1.9
position_in_order = [ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]
position_in_x = [2.0, -1.5, -2.0, -1.5, 1.5, 2.0, 1.5, 0.0, -2.0, -2.0, 1.5, 2.0, 1.5, -1.0]
position_in_y = [-2.0, -2.0, -1.5, -1.0, -1.0, -0.5, 0.0, 0.0, 0.0, 2.0, 2.0, 1.5, 1.0, 1.0]
position_in_z = [math.pi, math.pi, math.pi/2, 0, 0, math.pi/4, 3*math.pi/4, math.pi, math.pi/2, 
                    math.pi/2, 0, math.pi, math.pi]
time =0
wheel_radius = 0.043
axel_distance = 0.053
axel_length = 0.265
d_mid = 0.1325
min_dist = 0.5
Kp = 9
while endrit_ngjelina.experiment_supervisor.step(endrit_ngjelina.timestep) != -1:
    #recognize objects from RGB camera
    objective_object = endrit_ngjelina.rgb_camera.getRecognitionObjects()
    #rotate if no objects are detected, else otherwise move to goal
    if not objective_object:
        print("No object detected yet, turning robot...")
        endrit_ngjelina.rotate_until_object_detected(speed_to_rotate)
    else:
        endrit_ngjelina.motion_to_goal(objective_object)
