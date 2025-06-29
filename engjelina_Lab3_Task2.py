from fairis_tools.engjelina_Lab3_Task2_MyRobot import MyRobot
import math
endrit_ngjelina = MyRobot()
maze_file = '../../worlds/Fall24/maze6.xml'
endrit_ngjelina.load_environment(maze_file)
endrit_ngjelina.move_to_start()
speed_to_rotate = 3
wall = "L"
distance_to_goal = 0.5

while endrit_ngjelina.experiment_supervisor.step(endrit_ngjelina.timestep) != -1:
    distance_forward = min(endrit_ngjelina.get_lidar_range_image()[360:440])
    right_distance = min(endrit_ngjelina.get_lidar_range_image()[411:590])
    left_distance = min(endrit_ngjelina.get_lidar_range_image()[210:389])
    velocity = endrit_ngjelina.pid(0.1, 8)
    sat_vel = endrit_ngjelina.speed_sataturation(velocity)/2
    obstacles = endrit_ngjelina.rgb_camera.getRecognitionObjects()
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
    wall = "R"
    min_dist = 0.5
    Kp = 9
    if not obstacles:
        if distance_forward < 3.6:
            print("Robot is rotating")
            endrit_ngjelina.stop()
            #rotate based on wall
            rotation_speed = speed_to_rotate if wall == "L" else -speed_to_rotate
            endrit_ngjelina.rotate_until_object_detected(rotation_speed)
        else:
            endrit_ngjelina.another_function_for_wall_following(wall)

    elif left_distance > 2:
        print("Change to Motion-to-goal")
        endrit_ngjelina.algorithm_bug0(obstacles)              