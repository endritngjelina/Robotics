from fairis_lib.robot_lib.rosbot import RosBot

class MyRobot(RosBot):

    def __init__(self):
        RosBot.__init__(self) 
        self.x = 2.0 
        self.y = -2.0
        self.z = 0.0
        self.theta = 0.0

    def algorithm_bug0(self, rec_objects):
        velocity = self.pid(0.1, 8)
        sat_vel = self.speed_sataturation(velocity) / 2

        if rec_objects:
            landmark = rec_objects[0]
            x_pos, y_pos, z_pos = landmark.getPosition()

            if not (-0.1 < y_pos < 0.1):
                self.rotate_until_object_detected(speed=2.5)
            else:
                self.set_left_motors_velocity(sat_vel)
                self.set_right_motors_velocity(sat_vel)

            if x_pos < 0.5:
                print(f"Final destination from objective: {x_pos:.2f} meters. Goal reached!")
                self.stop()
                return


    def curved_movement(self,distance_input, left_velocity = 20, right_velocity = 11.62): #time as another parameter
        #TODO calculate starting encoder pos
        start_theta = self.get_encoder_readings()[0]#avg of all 4 encoders

        while self.experiment_supervisor.step(self.timestep) != -1:

            self.set_left_motors_velocity(left_velocity)
            self.set_front_right_motor_velocity(right_velocity)
            current_theta = self.get_encoder_readings()[0]
            delta_theta = current_theta - start_theta
            delta_distance = delta_theta * self.wheel_radius

            if delta_distance >= distance_input:
                self.stop()
                #time = totaltime
                break

    def rotate(self):
        while self.experiment_supervisor.step(self.timestep) != -1:
            if(self.get_compass_reading() > 91):
                print(self.get_compass_reading())
                self.front_right_motor.setVelocity(-5)
                self.front_left_motor.setVelocity(5)
                self.rear_left_motor.setVelocity(5)
                self.rear_right_motor.setVelocity(-5)
            else:
                break
    
    def straight(self):
        time = 0
        while self.experiment_supervisor.step(self.timestep) != -1:
            if(time < 2):
                time += (self.timestep/1000)
                self.front_right_motor.setVelocity(5)
                self.front_left_motor.setVelocity(5)
                self.front_left_motor.setVelocity(5)
                self.front_right_motor.setVelocity(5)
            else:
                break

    def PID_Lidar(self, k_value, wall):    #reads distance to the wall K value and distace
        velocity = 1
        left_dist = self.get_lidar_range_image()[200]
        right_dist = self.get_lidar_range_image()[600]
        front_dist = self.get_lidar_range_image()[400]
        minimum_left = min(self.get_lidar_range_image()[0:400])     #shortest left distance
        minimum_right = min(self.get_lidar_range_image()[400:800])  #shortest right distance
        minimum_front = min(self.get_lidar_range_image()[200:600])
        print("Minimum Front", minimum_front)
        print("Minimum Left",minimum_left)
        print("Minimum Right",minimum_right)

        if wall == 'L':
            e_t_from_pdf = 0.3 - minimum_left   #how far from goal 0.4
            u_t =  k_value * e_t_from_pdf   #u_t velocity indicator

            if u_t > -0.4 and u_t < 0:
                velocity = abs(u_t)
                self.front_right_motor.setVelocity(velocity)
                self.front_left_motor.setVelocity(5)
                self.rear_left_motor.setVelocity(5)
                self.rear_right_motor.setVelocity(velocity)

            if u_t > -2 and u_t < -0.4:
                velocity = abs(u_t)
                self.front_right_motor.setVelocity(velocity)
                self.front_left_motor.setVelocity(0.3)
                self.rear_left_motor.setVelocity(0.3)
                self.rear_right_motor.setVelocity(velocity)

            if u_t > -3 and u_t < -2:
                velocity = abs(u_t)
                self.front_right_motor.setVelocity(velocity)
                self.front_left_motor.setVelocity(1.7)
                self.rear_left_motor.setVelocity(1.7)
                self.rear_right_motor.setVelocity(velocity)

            if u_t > -4 and u_t < -3:
                velocity = abs(u_t)
                self.front_right_motor.setVelocity(velocity)
                self.front_left_motor.setVelocity(2.7)
                self.rear_left_motor.setVelocity(2.7)
                self.rear_right_motor.setVelocity(velocity)
            
            if u_t> -5.4 and u_t < -4:
                velocity = abs(u_t)
                self.front_right_motor.setVelocity(6)
                self.front_left_motor.setVelocity(velocity)
                self.rear_left_motor.setVelocity(velocity)
                self.rear_right_motor.setVelocity(6)

            if u_t < -5.6:
                velocity = abs(u_t)
                self.front_right_motor.setVelocity(10)
                self.front_left_motor.setVelocity(velocity)
                self.rear_left_motor.setVelocity(velocity)
                self.rear_right_motor.setVelocity(10)

        elif wall == 'R':
            e_t_from_pdf = 0.3  - minimum_right   #how far from right side
            u_t =  k_value * e_t_from_pdf   #u_t  = 10 * -0.3
            
            if u_t > -0.4 and u_t < 0:
                velocity = abs(u_t)
                self.front_right_motor.setVelocity(5)
                self.front_left_motor.setVelocity(velocity)
                self.rear_left_motor.setVelocity(velocity)
                self.rear_right_motor.setVelocity(5)

            if u_t > -2 and u_t < -0.4:
                velocity = abs(u_t)
                self.front_right_motor.setVelocity(0.3)
                self.front_left_motor.setVelocity(velocity)
                self.rear_left_motor.setVelocity(velocity)
                self.rear_right_motor.setVelocity(0.3)

            if u_t > -3 and u_t < -2:
                velocity = abs(u_t)
                self.front_right_motor.setVelocity(1.7)
                self.front_left_motor.setVelocity(velocity)
                self.rear_left_motor.setVelocity(velocity)
                self.rear_right_motor.setVelocity(1.7)

            if u_t > -4 and u_t < -3:
                velocity = abs(u_t)
                self.front_right_motor.setVelocity(2.7)
                self.front_left_motor.setVelocity(velocity)
                self.rear_left_motor.setVelocity(velocity)
                self.rear_right_motor.setVelocity(2.7)
            
            if u_t> -5.4 and u_t < -4:
                velocity = abs(u_t)
                self.front_right_motor.setVelocity(velocity)
                self.front_left_motor.setVelocity(6)
                self.rear_left_motor.setVelocity(6)
                self.rear_right_motor.setVelocity(velocity)

            if u_t < -5.6:
                velocity = abs(u_t)
                self.front_right_motor.setVelocity(velocity)
                self.front_left_motor.setVelocity(10)
                self.rear_left_motor.setVelocity(10)
                self.rear_right_motor.setVelocity(velocity)
        
    def follow_wall_follow(self, mode, wall, k_value):
        time = 0
        flag = 0
        velocity = 3
        front_dist = self.get_lidar_range_image()[400]
        right_dist = self.get_lidar_range_image()[600]
        left_dist = self.get_lidar_range_image()[200]
        minimum_left = min(self.get_lidar_range_image()[0:400])     #shortest left distance
        minimum_right = min(self.get_lidar_range_image()[400:800])  #shortest right distance
        minimum_front = min(self.get_lidar_range_image()[200:600])
        print("Front distance is",front_dist)
        print("Left distance is",left_dist)
        print("Right distance is",right_dist)
        
        if mode == 0:
            if wall == 'L':
                self.PID_Lidar(10, 'L')
                if flag != 0:
                    self.stop()
                    self.front_right_motor.setVelocity(0)
                    self.front_left_motor.setVelocity(0)
                    self.rear_right_motor.setVelocity(0)
                    self.rear_left_motor.setVelocity(0)

                else:
                    if minimum_front < 0.4 and minimum_left < 0.4:
                        while self.experiment_supervisor.step(self.timestep) != -1:
                            if(time < 0.1):
                                time += (self.timestep/1000)
                                self.front_right_motor.setVelocity(4)
                                self.front_left_motor.setVelocity(13)
                            else:
                                break
                    elif front_dist < 0.2 and minimum_right < 0.3:
                        while self.experiment_supervisor.step(self.timestep) != -1:
                            if(time < 0.1):
                                time += (self.timestep/1000)
                                self.front_right_motor.setVelocity(16)
                                self.front_left_motor.setVelocity(4)
                                self.rear_right_motor.setVelocity(16)
                                self.rear_left_motor.setVelocity(4)
                            else:
                                break
                    elif right_dist - minimum_right > 0.35:
                        while self.experiment_supervisor.step(self.timestep) != -1:
                            if(time < 0.15):
                                time += (self.timestep/1000)
                                self.front_right_motor.setVelocity(4)
                                self.front_left_motor.setVelocity(11)
                            else:
                                break
                    
                    if minimum_front< 0.35 and minimum_left < 0.35 and minimum_right < 0.35:
                        flag=1
                        print("flag is",flag)
                        self.front_right_motor.setVelocity(0)
                        self.front_left_motor.setVelocity(0)
                        self.rear_right_motor.setVelocity(0)
                        self.rear_left_motor.setVelocity(0)

            elif wall =='R':
                self.PID_Lidar(10,'R')
                if front_dist < 0.44:
                    while self.experiment_supervisor.step(self.timestep) != -1:
                        if(time < 0.2):
                            time += (self.timestep/1000)
                            self.front_right_motor.setVelocity(13)
                            self.front_left_motor.setVelocity(4)
                        else:
                            break

        if mode == 1:
            if wall == 'L':
                self.PID_Lidar(10, 'L')
                if flag != 0:
                    self.stop()
                    self.front_right_motor.setVelocity(0)
                    self.front_left_motor.setVelocity(0)
                    self.rear_right_motor.setVelocity(0)
                    self.rear_left_motor.setVelocity(0)

                else:
                    if minimum_left > 0.5:
                        while self.experiment_supervisor.step(self.timestep) != -1:
                            if(time < 0.1):
                                time += (self.timestep/1000)
                                self.front_right_motor.setVelocity(14)
                                self.front_left_motor.setVelocity(8)
                                self.rear_right_motor.setVelocity(14)
                                self.rear_left_motor.setVelocity(8)
                            else:
                                break

                    if minimum_left < 0.3:
                        while self.experiment_supervisor.step(self.timestep) != -1:
                            if(time < 0.1):
                                time += (self.timestep/1000)
                                self.front_right_motor.setVelocity(2)
                                self.front_left_motor.setVelocity(6)
                                self.rear_right_motor.setVelocity(2)
                                self.rear_left_motor.setVelocity(6)
                            else:
                                break

                    if minimum_left < 0.22:
                        while self.experiment_supervisor.step(self.timestep) != -1:
                            if(time < 0.1):
                                time += (self.timestep/1000)
                                self.front_right_motor.setVelocity(2)
                                self.front_left_motor.setVelocity(8)
                                self.rear_right_motor.setVelocity(2)
                                self.rear_left_motor.setVelocity(8)
                            else:
                                break
                    if minimum_front < 0.3 and minimum_left < 0.3:
                        while self.experiment_supervisor.step(self.timestep) != -1:
                            if(time < 0.1):
                                time += (self.timestep/1000)
                                self.front_right_motor.setVelocity(8.5)
                                self.front_left_motor.setVelocity(6)
                                self.rear_right_motor.setVelocity(8.5)
                                self.rear_left_motor.setVelocity(6)
                            else:
                                break

                    if minimum_front < 0.4 and minimum_left < 0.4:
                        while self.experiment_supervisor.step(self.timestep) != -1:
                            if(time < 0.1):
                                time += (self.timestep/1000)
                                self.front_right_motor.setVelocity(4)
                                self.front_left_motor.setVelocity(13)
                            else:
                                break

                    elif front_dist < 0.2 and minimum_right < 0.3:
                        while self.experiment_supervisor.step(self.timestep) != -1:
                            if(time < 0.1):
                                time += (self.timestep/1000)
                                self.front_right_motor.setVelocity(16)
                                self.front_left_motor.setVelocity(4)
                                self.rear_right_motor.setVelocity(16)
                                self.rear_left_motor.setVelocity(4)
                            else:
                                break
                    elif right_dist - minimum_right > 0.35:
                        while self.experiment_supervisor.step(self.timestep) != -1:
                            if(time < 0.15):
                                time += (self.timestep/1000)
                                self.front_right_motor.setVelocity(4)
                                self.front_left_motor.setVelocity(11)
                            else:
                                break

            if wall == 'R':
                self.PID_Lidar(10,'R')

                if minimum_right>0.6:
                    while self.experiment_supervisor.step(self.timestep) != -1:
                        if(time < 0.1):
                            time += (self.timestep/1000)
                            self.front_right_motor.setVelocity(19)
                            self.rear_right_motor.setVelocity(19)
                            self.front_left_motor.setVelocity(4)
                            self.rear_left_motor.setVelocity(4)
                            break

                if right_dist > 1.2:
                    while self.experiment_supervisor.step(self.timestep) != -1:
                        if(time < 0.1):
                            time += (self.timestep/1000)
                            self.front_right_motor.setVelocity(19)
                            self.rear_right_motor.setVelocity(19)
                            self.front_left_motor.setVelocity(4)
                            self.rear_left_motor.setVelocity(4)
                        else:
                            break

                if minimum_right < 0.2:
                    while self.experiment_supervisor.step(self.timestep) != -1:
                        if(time < 0.2):
                            time += (self.timestep/1000)
                            self.front_right_motor.setVelocity(13)
                            self.rear_right_motor.setVelocity(13)
                            self.front_left_motor.setVelocity(4)
                            self.rear_left_motor.setVelocity(4)
                        else:
                            break

                if minimum_right < 0.3:
                    while self.experiment_supervisor.step(self.timestep) != -1:
                        if(time < 0.2):
                            time += (self.timestep/1000)
                            self.front_right_motor.setVelocity(13)
                            self.rear_right_motor.setVelocity(13)
                            self.front_left_motor.setVelocity(4)
                            self.rear_left_motor.setVelocity(4)
                        else:
                            break

                if minimum_right < 0.4:
                    while self.experiment_supervisor.step(self.timestep) != -1:
                        if(time < 0.2):
                            time += (self.timestep/1000)
                            self.front_right_motor.setVelocity(13)
                            self.rear_right_motor.setVelocity(13)
                            self.front_left_motor.setVelocity(4)
                            self.rear_left_motor.setVelocity(4)
                        else:
                            break

                if minimum_front < 0.44:
                    while self.experiment_supervisor.step(self.timestep) != -1:
                        if(time < 0.2):
                            time += (self.timestep/1000)
                            self.front_right_motor.setVelocity(13)
                            self.rear_right_motor.setVelocity(13)
                            self.front_left_motor.setVelocity(4)
                            self.rear_left_motor.setVelocity(4)
                        else:
                            break
                        
    def rotate_until_object_detected(self,speed):
        self.set_left_motors_velocity(-speed)
        self.set_right_motors_velocity(speed)

    def speed_sataturation(self, velocity):
        max_velocity = 20
        if velocity >= max_velocity :
            velocity = max_velocity
        elif velocity <= -max_velocity:
            velocity = -max_velocity
        return velocity
    
    def pid(self, target_d=1, Kp=0.1):
        distance_forward = min(self.get_lidar_range_image()[350:450])
        error = distance_forward - target_d
        return self.speed_sataturation(Kp * error)

    def motion_to_goal(self, rec_objects):
        velocity = self.pid(0.01, 8)
        sat_vel = self.speed_sataturation(velocity) / 2

        if len(rec_objects) > 0:
            landmark = rec_objects[0]
            x_pos = landmark.getPosition()[0]
            y_pos = landmark.getPosition()[1]
            z_pos = landmark.getPosition()[2]
            print(f"Position from objective: x = {x_pos:.2f}, y = {y_pos:.2f}, z = {z_pos:.2f}")
            if -0.1 < y_pos < 0.1:
                print("Objective detected.")
                print(f"Velocity: {sat_vel:.2f} m/s")

                self.set_left_motors_velocity(sat_vel)
                self.set_right_motors_velocity(sat_vel)
            if x_pos < 0.5:
                print(f"Final destination from objective: {x_pos:.2f} meters. Goal reached!")
                self.stop()
                return  

    def another_function_for_wall_following(self, wall):
        lspeed, rspeed, = self.wall_following(0.4,8,wall)
        self.set_left_motors_velocity(lspeed)
        self.set_right_motors_velocity(rspeed)

    def wall_following(self,min_dist, Kp , wall ):
        motor_vel = self.pid(0.3,20)
        right_d = min(self.get_lidar_range_image()[411:590]) 
        left_d = min(self.get_lidar_range_image()[210:389]) 
        if wall == 'R':
            error = min_dist - right_d 
            if (error > 0): 
                right_vel = self.speed_sataturation(motor_vel)
                left_vel = self.speed_sataturation(motor_vel - abs(Kp * error))
            elif (error < 0): 
                left_vel = self.speed_sataturation(motor_vel)
                right_vel = self.speed_sataturation(motor_vel - abs(Kp * error))
            elif left_d <= 0.55 :
                error = 0.55 - left_d
                right_vel = self.speed_sataturation(motor_vel - abs(Kp * error))
                left_vel = self.speed_sataturation(0)
            else: 
                left_vel = motor_vel
                right_vel = motor_vel
        else: 
            error = min_dist - left_d
            if(error > 0):
                left_vel = self.speed_sataturation(motor_vel)
                right_vel = self.speed_sataturation(motor_vel - abs(Kp * error))
            elif(error < 0):
                right_vel = self.speed_sataturation(motor_vel)
                left_vel = self.speed_sataturation(motor_vel - abs(Kp * error))
            elif(right_d <= 0.55):
                error = 0.55 - right_d
                right_vel = self.speed_sataturation(0)
                left_vel = self.speed_sataturation(motor_vel - abs(Kp * error))
            else:
                left_vel = motor_vel 
                right_vel = motor_vel
        return left_vel , right_vel
    
    def rotate_in_place(self, speed):
        self.set_left_motors_velocity(-speed)  
        self.set_right_motors_velocity(speed)  

    def PID_Lidar(self, k_value):    
        velocity = 6
        while self.experiment_supervisor.step(self.timestep) != -1:
            print("Front Distance:", self.get_lidar_range_image()[400])
            front_dist = self.get_lidar_range_image()[400]
            e_t = 1 - front_dist   #distance error
            u_t =  k_value * e_t  #motor control velocity
            print("PID Velocity:",-u_t)
            if u_t > 0:
                velocity = -u_t
            if u_t > -19 and u_t < 0:
                velocity = abs(u_t)
            if u_t < -19:
                velocity = 19
            self.front_right_motor.setVelocity(velocity)
            self.front_left_motor.setVelocity(velocity)
            self.rear_left_motor.setVelocity(velocity)
            self.rear_right_motor.setVelocity(velocity)


    def straight_movement(self,distance_input,velocity=9):
        start_theta = self.get_encoder_readings()[0]

        while self.experiment_supervisor.step(self.timestep) != -1:
            print("My angle of direction is ", self.get_compass_reading())
            print("Max rotational motor velocity: ", self.max_motor_velocity)
            print("Front Left Distance Sensor: ", self.get_front_left_distance_reading())
            print("Front Right Distance Sensor: ", self.get_front_right_distance_reading())
            print("Rear Left Distance Sensor: ", self.get_rear_left_distance_reading())
            print("Rear Right Distance Sensor: ", self.get_rear_right_distance_reading())
            print("Motor Encoder Readings: ", self.get_encoder_readings())
            print(f"Front left wheel velocity: {velocity} m/s.")
            print(f"Front right wheel velocity: {velocity} m/s.")
            print(f"Rear left wheel velocity: {velocity} m/s.")
            print(f"Rear right wheel velocity: {velocity} m/s.")
            print("Lidar Front Reading", self.get_lidar_range_image()[400])
            print("Lidar Right Reading", self.get_lidar_range_image()[600])
            print("Lidar Rear Reading", self.get_lidar_range_image()[0])
            print("Lidar Left Reading", self.get_lidar_range_image()[200])
            print("Simulation Time", self.experiment_supervisor.getTime())
            self.set_left_motors_velocity(velocity)
            self.set_front_right_motor_velocity(velocity)
            time = velocity / distance_input
            current_theta = self.get_encoder_readings()[0] 
            delta_theta = current_theta - start_theta
            delta_distance = delta_theta * self.wheel_radius
            print("Current Theta Reading is", current_theta)
            print("Start Theta Reading", start_theta)
            print("Delta Theta Reading", delta_theta)
            print("Delta Distance Reading", delta_distance)
            if delta_distance >= distance_input:
                self.stop()
                break