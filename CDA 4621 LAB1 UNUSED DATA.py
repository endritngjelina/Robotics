#lab2 task 1
#print("Front Left Distance Sensor: ", self.get_front_left_distance_reading())

#lab 2 task 2
    # turn until time 
            if sentinel == 0:   #right turn
                self.front_right_motor.setVelocity(4)
                self.front_left_motor.setVelocity(7)

                if time > timesent:
                    break

            elif sentinel == 1: #left turn
                self.front_right_motor.setVelocity(4)
                self.front_left_motor.setVelocity(7)
                if front_dist > 2:
                    break
#lab 2 task 1


#lab 2 task 2
def hardcode(self, sentinel): #time to complete turn
        front_dist = self.get_lidar_range_image()[400]
        right_dist = self.get_lidar_range_image()[600]
        left_dist = self.get_lidar_range_image()[200]

        while True:
        #turn considering distanace of lidar L and R and front bigger than 2    
            if sentinel == 0:   #right turn
                self.front_right_motor.setVelocity(4)
                self.front_left_motor.setVelocity(7)
                if front_dist > 2:
                    break

            elif sentinel == 1: #left turn
                self.front_right_motor.setVelocity(4)
                self.front_left_motor.setVelocity(7)
                if front_dist > 2:
                    break





position_in_order = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
position_in_x = [2.0, -1.5, -2.0, -1.5, 1.5, 2.0, 1.5, 0.0, -2.0, -2.0, 1.5, 2.0, 1.5, -1.0]
position_in_y = [-2.0, -2.0, -1.5, -1.0, -1.0, -0.5, 0.0, 0.0, 0.0, 2.0, 2.0, 1.5, 1.0, 1.0]
position_in_z = [math.pi, math.pi, math.pi/2, 0, 0, math.pi/4, 3*math.pi/4, math.pi, math.pi/2, 
                     math.pi/2, 0, math.pi, math.pi]

t1=9.06
    t2=2.51
    t3=2.48
    t4=8.14
    t5=2.36
    t6=2.93
    t7=9.81
    t8=5.84
    t9=9.59
    t10=2.84
    t11=2.21
    t12=6.39
    t13=1.54

#totaltime= t1+t2
start_theta = self.get_encoder_readings()[0]#the first encoder
def hello(self):
        print("HEllo World!!!")

#def forward( velocity):

    #def right_angular_rotation():

    #def left_angular_rotation():

    #def right_nonangular_rotation():

    #def left_nonangular_rotation():

    #robot.straight_movement(1,velocity=15)
    # robot.straight_movement(1.5)
    #break

    # Sets the robot's motor velocity to 20 rad/sec
    #robot.set_right_motors_velocity(20)
    #robot.set_left_motors_velocity(20)

    
    # def forward( velocity):
    #     if (position_in_x[0] - position_in_x[1] == 0) or (position_in_y [0] - position_in_y[1] == 0):
    #         distance_front_left_wheel_traveled > 3
    #     else:
    #         robot.stop()

    
    
    # #def get_front_left_motor_encoder_reading(self):

    # def turn_180 ():
    #     robot.set_left_motors_velocity(left_motor_linear_velocity)
    #     robot.set_right_motors_velocity(right_motor_linear_velocity)

    # # Stops the robot after the robot moves a distance of 1.5 meters
    # if distance_front_left_wheel_traveled > 3.27:
    #     robot.stop()    

    
    #totaltime overall
    #t1
    #t2
    #t3
    #t4

    # #Calculates distance the wheel has turned since beginning of simulation
    # distance_front_left_wheel_traveled = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()
    # robot.experiment_supervisor.getTime()

    
    #def get_front_left_motor_encoder_reading(self):
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

    #move straight function
    #call it this way robot.straight_movement(3.4)
    def straight_movement(self,distance_input,velocity=12):
        #TODO calculate starting encoer pos
        start_theta = self.get_encoder_readings()[0]#the first encoder


        while self.experiment_supervisor.step(self.timestep) != -1:
            self.set_left_motors_velocity(velocity)
            self.set_front_right_motor_velocity(velocity)
            # Use current encoder and start encoder to calculate distance traveled and use for if
            
            current_theta = self.get_encoder_readings()[0] 
            delta_theta = current_theta - start_theta
            delta_distance = delta_theta * self.wheel_radius
            if delta_distance >= distance_input:
                self.stop()
                break
    #curve function
    #calling it robot.curved_movement( 10, left_velocity= 8, right_velocity=5 )
    def curved_movement(self,distance_input, left_velocity = 10, right_velocity = 7):
        #TODO calculate starting encoer pos
        start_theta = self.get_encoder_readings()[0]#avg of all 4 encoders


        while self.experiment_supervisor.step(self.timestep) != -1:
            self.set_left_motors_velocity(left_velocity)
            self.set_front_right_motor_velocity(right_velocity)
            # Use current encoder and start encoder to calculate distance traveled and use for if
            
            current_theta = self.get_encoder_readings()[0] #left  wheel
            delta_theta = current_theta - start_theta
            delta_distance = delta_theta * self.wheel_radius
            if delta_distance >= distance_input:
                self.stop()
                break