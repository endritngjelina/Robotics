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
# Gets Robots Camera
#######################################################
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

#######################################################
# Gets Robots Motors
#######################################################
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)


#######################################################
# Gets Robot's the position sensors
#######################################################
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)

# Main loop:
# perform simulation steps until Webots is stopping the controller

def robotWaypointNav(A):
    j , nums, condition, turn , sentinel , totaltime, m , Pose_x , Pose_y , Angle_Pose , start_x , start_y , Waypoint, Waypoint_2 = variable_declaration(A)
    
    timesentinel = ((1.14*math.pi)/2) / (0.9*(0.8))
    p=0
    counter = 1
    Waypoint_3 = 0
    Rads = [0.00, 0.00,0.00,0.00]
    point_next_x = A[j+1][0]
    point_next_y = A[j+1][1]
    
    
    for j in range (nums-1):
        condition, ztime,sentinel,diff_x,diff_y,dist_x,dist_y,point_x,point_y = variables_in_loop(A,j)
        
        if dist_y == 0 or dist_x == 0:
           # print ("one of the distances are 0")
            condition = 1

          
        print(" trajectory between point "+str(A[j][0])+" ,"+str(A[j][1])+" and point " + str(A[j+1][0])+" ,"+str(A[j+1][1]))
        timecontrolled = 0.0
        timegoal = 0.0
        K = 0 
  
        if condition == 0:
            print("Begin circular motion")
            print("totaltime is" + str(totaltime))
            turn = 0
            if diff_x < 0 and diff_y < 0:
                if point_x == 5 and point_y == 0:
                    Waypoint = 1
                
                
                map_angle = imu_cleaner(imu.getRollPitchYaw()[2])
                
                if map_angle >180 and map_angle < 270 and Waypoint == 0 and totaltime >= 40:
                
                    print("Circular motion 4 begins")
                    
                    dist_y = 5+0.03
   
                    S = (dist_y)*(math.pi/2)
                           # print("S is :" + str(S))
                    Tsentinel =  2.229
                    RobotLinearV = S/Tsentinel   
                    omega = RobotLinearV / dist_y
                           # print("omega is :" + str(omega))
                           
                    LinearVright = omega*(dist_y + 1.14)
                           # print("Linear V right is " + str(LinearVright))
                           
                    LinearVleft = omega*(dist_y - 1.14)
                           # print("Linear V left is " + str(LinearVleft))
                            
                    angularVleft = LinearVleft / 0.8
                    angularVright = LinearVright / 0.8
                            
                          #  print("Angular left is " + str(angularVleft))
                           # print("Angular right is " + str(angularVright))
                        
                    while robot.step(timestep) != -1:
                    
                        leftMotor.setVelocity(angularVleft)
                        rightMotor.setVelocity(angularVright)
                      
                        #print("IMU: "+str(imu.getRollPitchYaw()[2]))
                        
                        totaltime = (totaltime + (timestep/1000))
                        Rads[0] = Rads[2]
                        Rads[1] = Rads[3]
                        Rads[2]= leftposition_sensor.getValue()
                        Rads[3]= rightposition_sensor.getValue()

                        sentinel = sentinel + timestep
                          #  print ( "sentinel is " + str(sentinel))
                            #vrobot is = 4.09
                        if sentinel/1000 >= Tsentinel:
                            print("Linear V right is " + str(round(LinearVright,3))+" inches/ sec")
                            print("Linear V left is " + str(round(LinearVleft,3))+" inches/ sec") 
                            print("T to perform circular motion 4 is : " + str(round(Tsentinel,3))+" secs") 
                            print("Distance to perform in circular motion is :" + str(round(S,3)) +" inches") 
                            
                            break     
                                    
                if  map_angle > 180 and map_angle <270 and Waypoint == 1:
                    
                    dist_y = 5 + 0.1
                    
                       # print(" Counter is " + str(counter))
                    S = (dist_y)*(math.pi/2)
                           # print("Distance to perform is :" + str(S) +"inches")
                    Tsentinel =  2.214
                    RobotLinearV = S/Tsentinel
                            
                           # print("Robot linear v is "+ str(RobotLinearV))
                            
                    omega = RobotLinearV / dist_y
                           # print("omega is :" + str(omega))
                            
                    LinearVright = omega*(dist_y - 1.14)
                           # print("Linear V right is " + str(LinearVright)+"inches/ sec")
                           
                    LinearVleft = omega*(dist_y + 1.14)
                            #print("Linear V left is " + str(LinearVleft)+"inches/ sec")
                            
                    angularVleft = LinearVleft / 0.8
                    angularVright = LinearVright / 0.8
                            
                          #  print("Angular left is " + str(angularVleft))
                           # print("Angular right is " + str(angularVright))
                            
                    while robot.step(timestep) != -1:
                        totaltime = (totaltime + (timestep/1000))
                         #   print("Time: " +str(robot.getTime()))
                        
                        leftMotor.setVelocity(angularVleft)
                        rightMotor.setVelocity(angularVright)
                        
                        Rads[0] = Rads[2]
                        Rads[1] = Rads[3]
                        Rads[2]= leftposition_sensor.getValue()
                        Rads[3]= rightposition_sensor.getValue()
                        
                       
                    
                        #print("T to perform circular motion is : " + str(Tsentinel))
                            
                        sentinel = sentinel + timestep
                          #  print ( "sentinel is " + str(sentinel))
                            #vrobot is = 4.09
                        if sentinel/1000 >= Tsentinel:
                            print("Linear V right is " + str(round(LinearVright,3))+" inches/ sec")
                            print("Linear V left is " + str(round(LinearVleft,3))+" inches/ sec") 
                            print("T to perform circular motion 3 is : " + str(round(Tsentinel,3))+" secs") 
                            print("Distance to perform in circular motion is :" + str(round(S,3)) +" inches")
                            Waypoint = 0
                            
                            break   
               
                
                if map_angle >180 and map_angle < 270 and Waypoint == 0 and totaltime <= 40:
                
                    print("Circular motion 1 begins")
                    
                    
                    dist_y = 5-0.1
   
                    S = (dist_y)*(math.pi/2)
                           # print("S is :" + str(S))
                    Tsentinel =  2.2
                    RobotLinearV = S/Tsentinel   
                    omega = RobotLinearV / dist_y
                           # print("omega is :" + str(omega))
                           
                    LinearVright = omega*(dist_y + 1.14)
                           # print("Linear V right is " + str(LinearVright))
                           
                    LinearVleft = omega*(dist_y - 1.14)
                           # print("Linear V left is " + str(LinearVleft))
                            
                    angularVleft = LinearVleft / 0.8
                    angularVright = LinearVright / 0.8
                            
                          #  print("Angular left is " + str(angularVleft))
                           # print("Angular right is " + str(angularVright))
                        
                    while robot.step(timestep) != -1:
                    
                        leftMotor.setVelocity(angularVleft)
                        rightMotor.setVelocity(angularVright)
                      
                        #print("IMU: "+str(imu.getRollPitchYaw()[2]))
                        
                        totaltime = (totaltime + (timestep/1000))
                        Rads[0] = Rads[2]
                        Rads[1] = Rads[3]
                        Rads[2]= leftposition_sensor.getValue()
                        Rads[3]= rightposition_sensor.getValue()
                        
                            
                        sentinel = sentinel + timestep
                          #  print ( "sentinel is " + str(sentinel))
                            #vrobot is = 4.09
                        if sentinel/1000 >= Tsentinel:
                            print("Linear V right is " + str(round(LinearVright,3))+" inches/ sec")
                            print("Linear V left is " + str(round(LinearVleft,3))+" inches/ sec") 
                            print("T to perform circular motion 1 is : " + str(round(Tsentinel,3))+" secs") 
                            print("Distance to perform in circular motion is :" + str(round(S,3)) +" inches") 
                            Waypoint = 1
                            break 
            
            
            if diff_x > 0 and diff_y > 0:
                if point_next_x == 5 and point_next_y == 0:
                    Waypoint = 1
                
                
                map_angle = imu_cleaner(imu.getRollPitchYaw()[2])
                
                if map_angle >0 and map_angle < 90 and Waypoint == 0 and totaltime >= 20:
                
                    print("Circular reverse motion 4 begins")
                    
                    dist_y = 5+0.03
   
                    S = (dist_y)*(math.pi/2)
                           # print("S is :" + str(S))
                    Tsentinel =  2.229
                    RobotLinearV = S/Tsentinel   
                    omega = RobotLinearV / dist_y
                           # print("omega is :" + str(omega))
                           
                    LinearVright = omega*(dist_y - 1.14)
                           # print("Linear V right is " + str(LinearVright))
                           
                    LinearVleft = omega*(dist_y + 1.14)
                           # print("Linear V left is " + str(LinearVleft))
                            
                    angularVleft = LinearVleft / 0.8
                    angularVright = LinearVright / 0.8
                            
                          #  print("Angular left is " + str(angularVleft))
                           # print("Angular right is " + str(angularVright))
                        
                    while robot.step(timestep) != -1:
                    
                        leftMotor.setVelocity(angularVleft)
                        rightMotor.setVelocity(angularVright)
                      
                        #print("IMU: "+str(imu.getRollPitchYaw()[2]))
                        
                        totaltime = (totaltime + (timestep/1000))
                        Rads[0] = Rads[2]
                        Rads[1] = Rads[3]
                        Rads[2]= leftposition_sensor.getValue()
                        Rads[3]= rightposition_sensor.getValue()

                            
                        sentinel = sentinel + timestep
                          #  print ( "sentinel is " + str(sentinel))
                            #vrobot is = 4.09
                        if sentinel/1000 >= Tsentinel:
                            print("Linear V right is " + str(round(LinearVright,3))+" inches/ sec")
                            print("Linear V left is " + str(round(LinearVleft,3))+" inches/ sec") 
                            print("T to perform reverse circular motion 4 is : " + str(round(Tsentinel,3))+" secs") 
                            print("Distance to perform in circular motion is :" + str(round(S,3)) +" inches") 
                            
                            break     
                                    
                if  map_angle > 0 and map_angle <90 and Waypoint == 1:
                    
                    dist_y = 5 + 0.1
                    
                       # print(" Counter is " + str(counter))
                    S = (dist_y)*(math.pi/2)
                           # print("Distance to perform is :" + str(S) +"inches")
                    Tsentinel =  2.214
                    RobotLinearV = S/Tsentinel
                            
                           # print("Robot linear v is "+ str(RobotLinearV))
                            
                    omega = RobotLinearV / dist_y
                           # print("omega is :" + str(omega))
                            
                    LinearVright = omega*(dist_y + 1.14)
                           # print("Linear V right is " + str(LinearVright)+"inches/ sec")
                           
                    LinearVleft = omega*(dist_y - 1.14)
                            #print("Linear V left is " + str(LinearVleft)+"inches/ sec")
                            
                    angularVleft = LinearVleft / 0.8
                    angularVright = LinearVright / 0.8
                            
                          #  print("Angular left is " + str(angularVleft))
                           # print("Angular right is " + str(angularVright))
                            
                    while robot.step(timestep) != -1:
                        totaltime = (totaltime + (timestep/1000))
                         #   print("Time: " +str(robot.getTime()))
                        
                        leftMotor.setVelocity(angularVleft)
                        rightMotor.setVelocity(angularVright)
                        
                        Rads[0] = Rads[2]
                        Rads[1] = Rads[3]
                        Rads[2]= leftposition_sensor.getValue()
                        Rads[3]= rightposition_sensor.getValue()
                       
                        sentinel = sentinel + timestep
                          #  print ( "sentinel is " + str(sentinel))
                            #vrobot is = 4.09
                        if sentinel/1000 >= Tsentinel:
                            print("Linear V right is " + str(round(LinearVright,3))+" inches/ sec")
                            print("Linear V left is " + str(round(LinearVleft,3))+" inches/ sec") 
                            print("T to perform reverse circular motion 2 is : " + str(round(Tsentinel,3))+" secs") 
                            print("Distance to perform in circular motion is :" + str(round(S,3)) +" inches")
                            Waypoint = 0
                            
                            break   
               
                
                if map_angle >0 and map_angle < 90 and Waypoint == 0 and totaltime <= 20:
                
                    print("Circular reverse  motion 1 begins")
                    
                    
                    dist_y = 5-0.1
   
                    S = (dist_y)*(math.pi/2)
                           # print("S is :" + str(S))
                    Tsentinel =  2.2
                    RobotLinearV = S/Tsentinel   
                    omega = RobotLinearV / dist_y
                           # print("omega is :" + str(omega))
                           
                    LinearVright = omega*(dist_y - 1.14)
                           # print("Linear V right is " + str(LinearVright))
                           
                    LinearVleft = omega*(dist_y + 1.14)
                           # print("Linear V left is " + str(LinearVleft))
                            
                    angularVleft = LinearVleft / 0.8
                    angularVright = LinearVright / 0.8
                            
                          #  print("Angular left is " + str(angularVleft))
                           # print("Angular right is " + str(angularVright))
                        
                    while robot.step(timestep) != -1:
                    
                        leftMotor.setVelocity(angularVleft)
                        rightMotor.setVelocity(angularVright)
                      
                        #print("IMU: "+str(imu.getRollPitchYaw()[2]))
                        
                        totaltime = (totaltime + (timestep/1000))
                        Rads[0] = Rads[2]
                        Rads[1] = Rads[3]
                        Rads[2]= leftposition_sensor.getValue()
                        Rads[3]= rightposition_sensor.getValue()
   
                        sentinel = sentinel + timestep
                          #  print ( "sentinel is " + str(sentinel))
                            #vrobot is = 4.09
                        if sentinel/1000 >= Tsentinel:
                            print("Linear V right is " + str(round(LinearVright,3))+" inches/ sec")
                            print("Linear V left is " + str(round(LinearVleft,3))+" inches/ sec") 
                            print("T to perform reverse circular motion 1 is : " + str(round(Tsentinel,3))+" secs") 
                            print("Distance to perform in circular motion is :" + str(round(S,3)) +" inches") 
                            Waypoint = 1
                            break 
                                                  
            if diff_x < 0 and diff_y > 0:
                
                dist_y = 5
               
                    #print("New radius is " + str(dist_y))
                S = (dist_y)*(math.pi/2)
                  #  print("S is :" + str(S))
                Tsentinel =  2.3
                RobotLinearV = S/Tsentinel
                  #  print("Robot linear v is "+ str(RobotLinearV))
                    
                omega = RobotLinearV / dist_y
                   # print("omega is :" + str(omega))
                    
                LinearVright = omega*(dist_y + 1.14)
                   # print("Linear V right is " + str(LinearVright))
                   
                LinearVleft = omega*(dist_y - 1.14)
                  #  print("Linear V left is " + str(LinearVleft))
                    
                angularVleft = LinearVleft / 0.8
                angularVright = LinearVright / 0.8
                    
                  #  print("Angular left is " + str(angularVleft))
                  #  print("Angular right is " + str(angularVright))  
                while robot.step(timestep) != -1:
                    leftMotor.setVelocity(angularVleft)
                    rightMotor.setVelocity(angularVright)
                    
                    totaltime = (totaltime + (timestep/1000))
                    Rads[0] = Rads[2]
                    Rads[1] = Rads[3]
                    Rads[2]= leftposition_sensor.getValue()
                    Rads[3]= rightposition_sensor.getValue()
 
                    sentinel = sentinel + timestep
 
                    if sentinel/1000 >= Tsentinel:
                        print("Linear V right is " + str(round(LinearVright,3))+" inches/ sec")
                        print("Linear V left is " + str(round(LinearVleft,3))+" inches/ sec") 
                        print("T to perform circular motion 3 in reverse array is : " + str(round(Tsentinel,3))+" secs") 
                        print("Distance to perform in circular motion is :" + str(round(S,3)) +" inches") 
                        break  
                        
                        
            if diff_x > 0 and diff_y < 0:
                
                dist_y = dist_y 
               
                    #print("New radius is " + str(dist_y))
                S = (dist_y)*(math.pi/2)
                  #  print("S is :" + str(S))
                Tsentinel =  2.3
                RobotLinearV = S/Tsentinel
                  #  print("Robot linear v is "+ str(RobotLinearV))
                    
                omega = RobotLinearV / dist_y
                   # print("omega is :" + str(omega))
                    
                LinearVright = omega*(dist_y - 1.14)
                   # print("Linear V right is " + str(LinearVright))
                   
                LinearVleft = omega*(dist_y + 1.14)
                  #  print("Linear V left is " + str(LinearVleft))
                    
                angularVleft = LinearVleft / 0.8
                angularVright = LinearVright / 0.8
                    
                  #  print("Angular left is " + str(angularVleft))
                  #  print("Angular right is " + str(angularVright))  
                while robot.step(timestep) != -1:
                    leftMotor.setVelocity(angularVleft)
                    rightMotor.setVelocity(angularVright)
                    
                    totaltime = (totaltime + (timestep/1000))
                    Rads[0] = Rads[2]
                    Rads[1] = Rads[3]
                    Rads[2]= leftposition_sensor.getValue()
                    Rads[3]= rightposition_sensor.getValue()
                    
                    
                   # print("T sentinel is : " + str(Tsentinel))
                    
                    sentinel = sentinel + timestep
                   # print ( "sentinel is " + str(sentinel))
                    #vrobot is = 4.09
                    if sentinel/1000 >= Tsentinel:
                        print("Linear V right is " + str(round(LinearVright,3))+" inches/ sec")
                        print("Linear V left is " + str(round(LinearVleft,3))+" inches/ sec") 
                        print("T to perform circular motion 2 is : " + str(round(Tsentinel,3))+" secs") 
                        print("Distance to perform in circular motion is :" + str(round(S,3)) +" inches") 
                        break  
                        
      #  print("turn after movement is  " + str(turn))    
                
        if condition == 1:
            linearvelocity = 4.00*0.8
            if turn == 1:
                if start_x ==-5 and start_y == -15:
                    print("going to start rotation")
                    
                    
                    while robot.step(timestep) != -1:
                     #   print("Time: " +str(robot.getTime()))
                        leftMotor.setVelocity(-0.80)
                        rightMotor.setVelocity(0.80)
                        totaltime = (totaltime + (timestep/1000))
                       # print("total time is " + str(totaltime))
                        
                 
                   
                        sentinel = sentinel + (timestep)
                      #  print ( "sentinel is " + str(sentinel))
                      #  print("IMU: "+str(imu.getRollPitchYaw()[2]))
                        
                        
                        if p == 0 :
                            if  totaltime >= 9.054 :
                               
                                turn = 0
                               # print(" turn after turn is " + str(turn))
                                p = p+1
                                break 
                        if p ==1 :
                            if totaltime >= 21.24:
                              
                                turn = 0
                               # print(" turn after turn is " + str(turn))
                                p = p+1
                                break 
                                
                        if p ==2 :
                            if imu.getRollPitchYaw()[2] >= 0:
                              
                                turn = 0
                                #print(" turn after turn is " + str(turn))
                                p = p+1
                                break 
                                
                if start_x == -15 and start_y == -15:
                    while robot.step(timestep) != -1:
                     #   print("Time: " +str(robot.getTime()))
                        leftMotor.setVelocity(0.80)
                        rightMotor.setVelocity(-0.80)
                        totaltime = (totaltime + (timestep/1000))
              
                        pose_angle = imu_cleaner(imu.getRollPitchYaw()[2])
                    
                        sentinel = sentinel + (timestep)
                      #  print ( "sentinel is " + str(sentinel))
                      #  print("IMU: "+str(imu.getRollPitchYaw()[2]))
                        
                        if p == 0 :
                            if  totaltime >= 9.054 :
                               
                                turn = 0
                               # print(" turn after turn is " + str(turn))
                                p = p+1
                                break 
                        if p ==1 :
                            if totaltime >= 21.24:
                              
                                turn = 0
                               # print(" turn after turn is " + str(turn))
                                p = p+1
                                break 
                                
                        if p ==2 :
                            if imu.getRollPitchYaw()[2] >= 0:
                              
                                turn = 0
                                #print(" turn after turn is " + str(turn))
                                p = p+1
                                break 
                        
                 
            if dist_x > 0:
                timegoal = (dist_x / linearvelocity)
                #print("Time to perform straight motion is: " + str(round(timegoal,2)))
                
                
                
                
            if dist_y > 0:
                timegoal = (dist_y / linearvelocity)
               # print("Time to perform straight motion is: " + str(round(timegoal,2)))
                

            while robot.step(timestep) != -1:
            
                leftMotor.setVelocity(4.00)
                rightMotor.setVelocity(4.00)
            
                 
                #print("Left position sensor: " +str(leftposition_sensor.getValue()))
                
                #print("Right position sensor: " +str(rightposition_sensor.getValue()))
                Rads[0] = Rads[2]
                Rads[1] = Rads[3]
                
                #print("Rads 0 is :"+ str(Rads[0]))
                #print("Rads 1 is :"+ str(Rads[1]))
                Rads[2]= leftposition_sensor.getValue()
                Rads[3]= rightposition_sensor.getValue()
                #print("Rads 2 is :"+ str(Rads[2]))
                #print("Rads 3 is :"+ str(Rads[3]))
  
             #   print("Time: " +str(robot.getTime()))
 
                #print("IMU: "+str(imu.getRollPitchYaw()[2]))
                
                timecontrolled = timestep+timecontrolled
                totaltime = (totaltime + (timestep/1000))
               # print("totaltime is " + str(totaltime))
           
            
               # print("timecontrolled is " + str(timecontrolled/1000))
                K= K+1
                if timecontrolled/1000 >= timegoal:
                
                    
                  #  print(" Rads left is "+ str(round(B[j][1],4)) + " - " + str(round(B[j][0],4)))
                  #  print(" Rads Right is "+ str(round(C[j][1],4)) + " - " + str(round(C[j][0],4)))
                    
                    print("Time to perform straight motion is: " + str(round(timegoal,2)))

                    #print("Linear velocity of left wheel is " + str(round(LinearLeft,4))+" inches/ sec")
                    #print("Linear velocity of right wheel is " + str(round(LinearRight,4))+" inches/ sec")
                                
                   
                    print("linear velocity of robot is " + str(round(linearvelocity,4)) + " inches/ sec")
                    distance = linearvelocity * timecontrolled/1000
                    print("Distance travelled by the robot is : " + str(round(distance,4)) + " inches")
                    j= j+1
                    # print(str(j)+ " Array done")
                    turn = 1
                    
                    print("turn is " + str(turn))
                   
                    break
                    

    
# Cleans the IMU readings so that the are in degrees and in the
# range of [0,359]
def imu_cleaner(imu_reading):
    rad_out = imu_reading
    if rad_out < 0:
        rad_out = rad_out + 2*math.pi
    degrees = math.degrees(rad_out)
   
    return degrees
    
def variable_declaration(A):
    j= 0
    nums= len(A)
    condition = 0
    turn = 0
    sentinel = 0.0
   
    totaltime = 0.00
    m = 0
    Pose_x = A[0][0]
    Pose_y = A[0][1]
    Angle_Pose = 0
    start_x = A[0][0]
    start_y = A[0][1]
    Waypoint = 0
    Waypoint_2 = 0
    
    return j , nums, condition, turn , sentinel , totaltime, m , Pose_x , Pose_y , Angle_Pose , start_x , start_y , Waypoint, Waypoint_2

def variables_in_loop(A,j):
    condition = 0
    ztime = 0
    sentinel = 0.0
  
    diff_x = A[j+1][0] - A[j][0]
    diff_y = A[j+1][1] - A[j][1]

    dist_x = abs(diff_x)
    dist_y = abs(diff_y)
        
    point_x = A[j][0]
    point_y = A[j][1]   
    
    return condition, ztime,sentinel,diff_x,diff_y,dist_x,dist_y,point_x,point_y
    

                            
A = [[-5,-15],[15,-15],[15,15],[-10,15],[-15,10],[-15,5],[0,5],[5,0],[0,-5],[-10,-5],[-15,-10],[-15,-15]]

robotWaypointNav(A)
print("Robot has reach endpoint")
leftMotor.setVelocity(0.00)
rightMotor.setVelocity(0.00)

    
# Enter here exit cleanup code.
