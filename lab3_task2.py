# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math

#######################################################
# Creates Robot
#######################################################
robot = Robot()


#######################################################
# Sets the time step of the current world
#######################################################
timestep = int(robot.getBasicTimeStep())

#######################################################
# Gets Robots Distance Sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/distancesensor
#######################################################
frontDistanceSensor = robot.getDevice('front distance sensor')
leftDistanceSensor = robot.getDevice('left distance sensor')
rightDistanceSensor = robot.getDevice('right distance sensor')
rearDistanceSensor = robot.getDevice('rear distance sensor')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)
rearDistanceSensor.enable(timestep)

#######################################################
# Gets Robots Lidar Distance Sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/lidar
#######################################################
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar_horizontal_res = lidar.getHorizontalResolution()
lidar_num_layers = lidar.getNumberOfLayers()
lidar_min_dist = lidar.getMinRange()
lidar_max_dist = lidar.getMaxRange()


print("Lidar is enabled. \nLidar has a Horizontal Resolution of: ", lidar_horizontal_res, "\nLidar Image Number of Layers: ", lidar_num_layers)
print("Lidar Range: [",lidar_min_dist," ,", lidar_max_dist,'] in meters')

#######################################################
# Gets Robots Camera
# Documentation:
#  https://cyberbotics.com/doc/reference/camera
#######################################################
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

#######################################################
# Gets Robots Motors
# Documentation:
#  https://cyberbotics.com/doc/reference/motor
#######################################################
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)


#######################################################
# Gets Robot's the position sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/positionsensor
#######################################################
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

#######################################################
# Gets Robot's IMU sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/inertialunit
#######################################################
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

# Main loop:
# perform simulation steps until Webots is stopping the controller
def Wallfollowing(wall,mode,k_value):
    times = 0.00
    while robot.step(timestep) != -1:
        # Read the sensors:
        # Getting full Range Image from Lidar returns a list of 1800 distances = 5 layers X 360 distances
        full_range_image = lidar.getRangeImage()
        # print size of Range Image
        print('#################################################################')
        print("Lidar's Full Range Image Size: ", len(full_range_image))
        # Compare Distance Sensors to Lidar Ranges
        front_dist = (frontDistanceSensor.getValue())*100/2.54
        right_dist = (rightDistanceSensor.getValue())*100/2.54
        rear_dist = (rearDistanceSensor.getValue())*100/2.54
        left_dist = (leftDistanceSensor.getValue())*100/2.54
            
        Lidar_front = (full_range_image[0]-0.027)*100/2.54
        Lidar_rear = (full_range_image[180]-0.047)*100/2.54
        Lidar_left = (full_range_image[270]-0.037)*100/2.54
        Lidar_right = (full_range_image[90]-0.037)*100/2.54
    
        print("Distance Sensor vs Lidar")
        print("\tFront:\t", front_dist, "\t|", Lidar_front )
        print("\tRight:\t", right_dist, "\t|", Lidar_right)
        print("\tRear:\t", rear_dist, "\t|", Lidar_rear)
        print("\tLeft:\t", left_dist, "\t|", Lidar_left)
        

        # Enter here functions to send actuator commands, like:

        if wall == 'r':
         
            e_t = 2 - Lidar_front
            e_t_side = 3- Lidar_right
            
        
            print ("e_t is " + str(e_t))
            print ("e_t side for right wall " + str(e_t_side))
            
            
            if abs(e_t) < 1 and Lidar_left > Lidar_right :
               
                while robot.step(timestep) != -1:
                    if times < 1.1:    
                        times = times + 32/1000
                        leftMotor.setVelocity(-2)
                        rightMotor.setVelocity(2)
                    else:
                        times = 0
                        break

            if abs(e_t) > 1 :
                if mode == 1:
                    print("Mode is :" + str(mode))
                    vel_l ,vel_r = PID_lidar(k_value,Lidar_right,wall)
                if mode == 0:
                    vel_l, vel_r = PID_sensordistance(k_value,right_dist,wall)
                    
                leftMotor.setVelocity(vel_l)
                rightMotor.setVelocity(vel_r)
        
        if wall == 'l':
            
            e_t = 2 - Lidar_front
            
            et_side_left = 3 -Lidar_left
            
        
            print ("e_t is " + str(e_t))
            print ("e_t side for left wall " + str(et_side_left))
            
            
            if abs(e_t) < 1 and Lidar_left < Lidar_right :
               
                while robot.step(timestep) != -1:
                    if times < 1.1:    
                        times = times + 32/1000
                        leftMotor.setVelocity(2)
                        rightMotor.setVelocity(-2)
                    else:
                        times = 0
                        break

            if abs(e_t) > 1 :
                if mode == 1:
                    print("Mode is :" + str(mode))
                    vel_l ,vel_r = PID_lidar(k_value,Lidar_left,wall)
                if mode == 0:
                    vel_l, vel_r = PID_sensordistance(k_value,left_dist,wall)
                    
                leftMotor.setVelocity(vel_l)
                rightMotor.setVelocity(vel_r)

       
def PID_sensordistance(Kp, side_dist,wall):
    e_t = 3 - side_dist #full_range_image[0]-0.027
    u_t = Kp * (e_t)
    if wall =='r':
    
        print("U t for right wall is : " + str(u_t))
        
        if u_t > 0 and u_t < 1 :
            velocity_r = 3.5
            velocity_l = 3.5 - u_t

        if u_t > 1.0 and u_t <1.5 :
            velocity_r = 4-u_t
            velocity_l = 2

        if u_t >1.5:
            velocity_r = 5.5
            velocity_l = 3
         
        if u_t >-1.0 and u_t < 0: 
            velocity_r = 4.0 + u_t
            velocity_l = 3.5 
        
        if u_t < -1.0 and u_t > -1.5:
            velocity_r = 2
            velocity_l = 4+u_t
            
        if u_t <-1.5 :
            velocity_r = 4+u_t
            if velocity_r < 0:
                velocity_r = 4
            velocity_l = 3
            
        if u_t <-3:
            velocity_l = 2
            velocity_r = 5
            
        return velocity_l , velocity_r
        
    if wall =='l':
    
        print("U t for left wall is : " + str(u_t))
        if u_t > 0 and u_t < 1 :
            velocity_l = 3.5
            velocity_r = 3.5 - u_t

        if u_t > 1.0 and u_t <1.5 :
            velocity_l = 5-u_t
            velocity_r = 2

        if u_t >1.5:
            velocity_l = 5.5
            velocity_r = 3-u_t
            if velocity_r < 0:
                velocity_r = 3
       
        if u_t >-1.0 and u_t < 0: 
            velocity_l = 3.5 + u_t
            velocity_r = 3.5 
        
        if u_t < -1.0 and u_t > -1.5:
            velocity_l = 4+u_t  
            velocity_r = 2
            
        if u_t <-1.5 :
            velocity_l = 3+u_t
            if velocity_l < 0:
                velocity_l = 3
            velocity_r = 5.5

        return velocity_l , velocity_r
    
            
def PID_lidar(Kp, side_dist,wall):
    e_t = 3 - side_dist #full_range_image[0]-0.027
    u_t = Kp * (e_t)
    if wall =='r':
    
        print("U t for right wall is : " + str(u_t))
        if u_t > 0 and u_t < 1 :
            velocity_r = 3.5
            velocity_l = 3.5 - u_t

        if u_t > 1.0 and u_t <1.5 :
            velocity_r = 5-u_t
            velocity_l = 2
        if u_t >1.5:
            velocity_r = 5.5
            velocity_l = 3-u_t
            if velocity_l < 0:
                velocity_l = 3
        if u_t >-1.0 and u_t < 0: 
            velocity_r = 3.5 + u_t
            velocity_l = 3.5 
        
        if u_t < -1.0 and u_t > -1.5:
            velocity_r = 2
            velocity_l = 4+u_t
            
        if u_t <-1.5 :
            velocity_r = 3+u_t
            if velocity_r < 0:
                velocity_r = 3
            velocity_l = 6
        
      
            
        return velocity_l , velocity_r
        
    if wall =='l':
    
        print("U t for left wall is : " + str(u_t))
        
        
        if u_t > 0 and u_t < 1 :
            velocity_l = 3.5
            velocity_r = 3.5 - u_t

        if u_t > 1.0 and u_t <1.5 :
            velocity_l = 5-u_t
            velocity_r = 2
        if u_t >1.5:
            velocity_l = 5.5
            velocity_r = 3-u_t
            if velocity_r < 0:
                velocity_r = 3
       
        if u_t >-1.0 and u_t < 0: 
            velocity_l = 3.5 + u_t
            velocity_r = 3.5 
        
        if u_t < -1.0 and u_t > -1.5:
            velocity_l = 4+u_t  
            velocity_r = 2
            
        if u_t <-1.5 :
            velocity_l = 3+u_t
            if velocity_l < 0:
                velocity_l = 3
            velocity_r = 6
       
        
        return velocity_l , velocity_r 

            
Wallfollowing('l',1,2)       
        
