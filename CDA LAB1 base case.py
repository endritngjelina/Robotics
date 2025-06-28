# Import MyRobot Class
from fairis_tools.my_robot import MyRobot
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

    wheel_radius = 0.043
    axel_distance = 0.053
    axel_length = 0.265
    d_mid = 0.1325
    radius = 0.5
    velocity = 26.5
    left_motor_front_velocity = 26.5
    right_motor_front_velocity = 26.5
    position_in_order = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
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

    # Sets the robot's motor velocity to 20 rad/sec
    robot.set_right_motors_velocity(20)
    robot.set_left_motors_velocity(20)

    # Calculates distance the wheel has turned since beginning of simulation
    distance_front_left_wheel_traveled = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()
    robot.experiment_supervisor.getTime()

    def forward( velocity):
        if (position_in_x[0] - position_in_x[1] == 0) or (position_in_y [0] - position_in_y[1] == 0):
            distance_front_left_wheel_traveled > 3.27
        else:
            robot.stop()
    
    #def get_front_left_motor_encoder_reading(self):

    # Stops the robot after the robot moves a distance of 1.5 meters
    if distance_front_left_wheel_traveled > 3.27:
        robot.stop()
        break