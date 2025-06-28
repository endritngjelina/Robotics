#Endrit Ngjelina U43630372
# Import MyRobot Class
from controller import Supervisor
from fairis_tools.U43630372_MyRobot import MyRobot
from fairis_lib.robot_lib import *
import matplotlib.pyplot as plt
import math
import numpy as np
#2 centimeters margin of error

# Create the robot instance.
robot = MyRobot()

# Loads the environment from the maze file
maze_file = '../../worlds/Fall24/maze1.xml'
robot.load_environment(maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()

totaltime = 0
# Main Control Loop for Robot
while robot.experiment_supervisor.step(robot.timestep) != -1:

    time =0
    wheel_radius = 0.043
    axel_distance = 0.053
    axel_length = 0.265
    d_mid = 0.1325
    left_motor_linear_velocity = 26.5 * (0.5 + 0.1325)
    right_motor_linear_velocity = 26.5 * (0.5 - 0.1325)
    radius = 0.5
    velocity = 26.5
    left_motor_angular_velocity = 26.5
    right_motor_angluar_velocity = 26.5
    position_in_order = [ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]
    position_in_x = [2.0, -1.5, -2.0, -1.5, 1.5, 2.0, 1.5, 0.0, -2.0, -2.0, 1.5, 2.0, 1.5, -1.0]
    position_in_y = [-2.0, -2.0, -1.5, -1.0, -1.0, -0.5, 0.0, 0.0, 0.0, 2.0, 2.0, 1.5, 1.0, 1.0]
    position_in_z = [math.pi, math.pi, math.pi/2, 0, 0, math.pi/4, 3*math.pi/4, math.pi, math.pi/2, 
                     math.pi/2, 0, math.pi, math.pi]
             
    print("Max rotational motor velocity: ", robot.max_motor_velocity)

    # Reads and Prints Distance Sensor Values
    print("Front Left Distance Sensor: ", robot.get_front_left_distance_reading())
    print("Front Right Distance Sensor: ", robot.get_front_right_distance_reading())
    print("Rear Left Distance Sensor: ", robot.get_rear_left_distance_reading())
    print("Rear Right Distance Sensor: ", robot.get_rear_right_distance_reading())

    # Reads and Prints Robot's Encoder Readings
    print("Motor Encoder Readings: ", robot.get_encoder_readings())


    # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
    print("Lidar Front Reading", robot.get_lidar_range_image()[400])
    print("Lidar Right Reading", robot.get_lidar_range_image()[600])
    print("Lidar Rear Reading", robot.get_lidar_range_image()[0])
    print("Lidar Left Reading", robot.get_lidar_range_image()[200])
    print("Simulation Time", robot.experiment_supervisor.getTime())
        
    front_left_distance = robot.get_front_left_distance_reading()
    front_left_distance += front_left_distance

    front_right_distance = robot.get_front_right_distance_reading()
    front_right_distance += front_right_distance

    rear_left_distance = robot.get_rear_left_distance_reading()
    rear_left_distance += rear_left_distance

    rear_right_distance = robot.get_rear_right_distance_reading()
    rear_right_distance += rear_right_distance
    
    
    robot.straight_movement(3.4) 
    robot.curved_movement( 1.63, left_velocity= 8, right_velocity=5)
    robot.straight_movement(3)  
    robot.rotate_in_place(0, 38)
    robot.straight_movement(0.7)    
    robot.rotate_in_place(38 , 125)
    robot.straight_movement(0.7)    
    robot.rotate_in_place(125 , 172.8)
    robot.straight_movement(3.5)    
    robot.rotate_in_place(172.8 , 96)
    robot.straight_movement(1.8)  
    robot.rotate_in_place(96 , 9)
    robot.straight_movement(3.33)    
    robot.curved_movement( 1.81, left_velocity= 9.1, right_velocity=6 )
    robot.straight_movement(2.4)
    
    robot.stop()
    break

print(f"Total travel time from S to G is {robot.experiment_supervisor.getTime()} seconds.")
total_distance  = front_left_distance + front_right_distance + rear_left_distance + rear_right_distance

print(f"Total travel distance from S to G is {total_distance:.2} meters.")
    