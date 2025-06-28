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
def PID(k_value, mode):
    timecontrol =0
    sentinel =0
    while robot.step(timestep) != -1:
    
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
        
        myangle = int(imu_cleaner(imu.getRollPitchYaw()[2]))
        
       
        
        print("Distance Sensor vs Lidar")
        print("\tFront:\t", front_dist, "\t|", Lidar_front)
        print("\tRight:\t", right_dist, "\t|", Lidar_right)
        print("\tRear:\t", rear_dist, "\t|", Lidar_rear)
        print("\tLeft:\t", left_dist, "\t|", Lidar_left)
        

        print("Angle of robot is " + str(myangle))
        

        print("Sentinel is 0")
        if (right_dist > left_dist):
            print("rotation left is less")
            e_t_left = 3.4 - Lidar_left #full_range_image[0]-0.027
            u_t = k_value * (e_t_left)
            print("U of rotation is t is : " + str(u_t))
            if u_t > 0:
                velocity_l = 3.5
                velocity_r = 3.5 - u_t
            if u_t > -2 and u_t < 0:
                velocity_l = 3.5 - u_t
                velocity_r = 3
                    
            if u_t < -2:
                velocity_l = 3
                velocity_r = 3.5
            print("Velocities are" + str(velocity_l))
                   
            leftMotor.setVelocity(velocity_l)
            rightMotor.setVelocity(velocity_r)
                
    
                
        if (right_dist < left_dist):
            print("rotation right is less")
            e_t_right = 3.4 - Lidar_right #full_range_image[0]-0.027
            u_t = k_value * (e_t_right)
            print("U t rotation right is : " + str(u_t))
            if u_t > 0:
                velocity_l = 3.5 - u_t
                velocity_r = 3.5 
            if u_t > -2 and u_t < 0:
                velocity_l = 3 
                velocity_r = 3 - u_t
                    
            if u_t < -2:
                velocity_l = 3.5
                velocity_r = 3
                  
            leftMotor.setVelocity(velocity_l)
            rightMotor.setVelocity(velocity_r)
                
    
        if myangle == 0 or myangle == 359:
            sentinel = 1
            if Lidar_front > 60:
                leftMotor.setVelocity(5)
                rightMotor.setVelocity(5)  
            else:  
                e_t = 5 - Lidar_front
                
                print ("e_t is " + str(e_t))
                if abs(e_t) < 0.1:
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)

                if abs(e_t) > 0.1:
                    if mode == 1:
                        print("Mode is :" + str(mode))
                        vel = PID_lidar(k_value,Lidar_front)
                    if mode == 0:
                        vel = PID_sensordistance(k_value,Lidar_front)
                    
                    # Enter here functions to send actuator commands, like:
              
                    leftMotor.setVelocity(vel)
                    rightMotor.setVelocity(vel)
          
                    
                print("Velocities are" + str(vel))
                
def PID_sensordistance(Kp, front_dist):
    e_t = 5 - front_dist #full_range_image[0]-0.027
    u_t = Kp * (e_t)

    print("U t is : " + str(u_t))
    if u_t > 0:
        velocity = -u_t
    if u_t > -6.0 and u_t < 0:
        velocity = abs(u_t)
    if u_t < -6.0:
        velocity = 6 
    return velocity
        
def PID_lidar(Kp, front_dist):

    if front_dist > 100:
        velocity = 6
    if front_dist < 100:
        e_t = 5 - front_dist 
       
        u_t = Kp * (e_t)
        print("U t is : " + str(u_t))
        if u_t > 0:
            velocity = -u_t
        if u_t > -6.0 and u_t < 0:
            velocity = abs(u_t)
        if u_t < -6.0:
            velocity = 6 
    return velocity  


def imu_cleaner(imu_reading):
    rad_out = imu_reading
    if rad_out < 0:
        rad_out = rad_out + 2*math.pi
    degrees = math.degrees(rad_out)
   
    return degrees
       

#PID(1,0)  
PID(2,0)  
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)


    
# Enter here exit cleanup code.
