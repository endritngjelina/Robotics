from fairis_lib.robot_lib.rosbot import RosBot

class MyRobot(RosBot):

    def __init__(self):
        RosBot.__init__(self)
    
    def straight_movement(self,distance_input,velocity=9):
        #TODO calculate starting encoer pos
        start_theta = self.get_encoder_readings()[0]#the first encoder

        while self.experiment_supervisor.step(self.timestep) != -1:
            
            print("My angle of direction is ", self.get_compass_reading())
            print("Max rotational motor velocity: ", self.max_motor_velocity)

            # Reads and Prints Distance Sensor Values
            print("Front Left Distance Sensor: ", self.get_front_left_distance_reading())
            print("Front Right Distance Sensor: ", self.get_front_right_distance_reading())
            print("Rear Left Distance Sensor: ", self.get_rear_left_distance_reading())
            print("Rear Right Distance Sensor: ", self.get_rear_right_distance_reading())

            # Reads and Prints Robot's Encoder Readings
            print("Motor Encoder Readings: ", self.get_encoder_readings())
            print(f"Front left wheel velocity: {velocity} m/s.")
            print(f"Front right wheel velocity: {velocity} m/s.")
            print(f"Rear left wheel velocity: {velocity} m/s.")
            print(f"Rear right wheel velocity: {velocity} m/s.")

            # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
            print("Lidar Front Reading", self.get_lidar_range_image()[400])
            print("Lidar Right Reading", self.get_lidar_range_image()[600])
            print("Lidar Rear Reading", self.get_lidar_range_image()[0])
            print("Lidar Left Reading", self.get_lidar_range_image()[200])
            print("Simulation Time", self.experiment_supervisor.getTime())
            self.set_left_motors_velocity(velocity)
            self.set_front_right_motor_velocity(velocity)
            time = velocity / distance_input
            
            # Use current encoder and start encoder to calculate distance traveled and use for if
            
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
        
    def curved_movement(self,distance_input, left_velocity = 20, right_velocity = 11.62): #time as another parameter
        #TODO calculate starting encoder pos
        start_theta = self.get_encoder_readings()[0]#avg of all 4 encoders
        #totaltime=0
        while self.experiment_supervisor.step(self.timestep) != -1:

            print("My angle of direction is ", self.get_compass_reading())
            print("Max rotational motor velocity: ", self.max_motor_velocity)

            # Reads and Prints Distance Sensor Values
            print("Front Left Distance Sensor: ", self.get_front_left_distance_reading())
            print("Front Right Distance Sensor: ", self.get_front_right_distance_reading())
            print("Rear Left Distance Sensor: ", self.get_rear_left_distance_reading())
            print("Rear Right Distance Sensor: ", self.get_rear_right_distance_reading())

            # Reads and Prints Robot's Encoder Readings
            print("Motor Encoder Readings: ", self.get_encoder_readings())
            print(f"Left wheel velocity: {left_velocity} m/s.")
            print(f"Right wheel velocity: {right_velocity} m/s.")

            # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
            print("Lidar Front Reading", self.get_lidar_range_image()[400])
            print("Lidar Right Reading", self.get_lidar_range_image()[600])
            print("Lidar Rear Reading", self.get_lidar_range_image()[0])
            print("Lidar Left Reading", self.get_lidar_range_image()[200])
            print("Simulation Time", self.experiment_supervisor.getTime())

            #toaltime += timestep/1000
            self.set_left_motors_velocity(left_velocity)
            self.set_front_right_motor_velocity(right_velocity)

            # Use current encoder and start encoder to calculate distance traveled and use for if
            
            current_theta = self.get_encoder_readings()[0] #left  wheel
            delta_theta = current_theta - start_theta
            delta_distance = delta_theta * self.wheel_radius

            print("Current Theta Reading is", current_theta)
            print("Start Theta Reading", start_theta)
            print("Delta Theta Reading", delta_theta)
            print("Delta Distance Reading", delta_distance)
            
            if delta_distance >= distance_input:
                self.stop()
                #time = totaltime
                break

    def rotate_in_place( self, init, final, velocity = 10 ):
        final, init

        print("My angle of direction is ", self.get_compass_reading())
        print("Max rotational motor velocity: ", self.max_motor_velocity)

        # Reads and Prints Distance Sensor Values
        print("Front Left Distance Sensor: ", self.get_front_left_distance_reading())
        print("Front Right Distance Sensor: ", self.get_front_right_distance_reading())
        print("Rear Left Distance Sensor: ", self.get_rear_left_distance_reading())
        print("Rear Right Distance Sensor: ", self.get_rear_right_distance_reading())

        # Reads and Prints Robot's Encoder Readings
        print("Motor Encoder Readings: ", self.get_encoder_readings())
        print(f"Left wheel velocity: {velocity} m/s.")
        print(f"Right wheel velocity: {velocity} m/s.")

        # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
        print("Lidar Front Reading", self.get_lidar_range_image()[400])
        print("Lidar Right Reading", self.get_lidar_range_image()[600])
        print("Lidar Rear Reading", self.get_lidar_range_image()[0])
        print("Lidar Left Reading", self.get_lidar_range_image()[200])
        print("Simulation Time", self.experiment_supervisor.getTime())
         
        if init > final:
            while self.experiment_supervisor.step(self.timestep) != -1:
                    angle =self.get_compass_reading()

                    if angle > final:
                        self.set_left_motors_velocity(velocity)
                        self.set_front_right_motor_velocity(-velocity)
                        print("Angle reading is", angle)
                        print("Velocity reading is", velocity)
                        print("Final angle reading is", final)
                    else:
                        break
         
        if init < final:
            while self.experiment_supervisor.step(self.timestep) != -1:
                    
                    angle =self.get_compass_reading()
                    if angle < final: 
                        self.set_left_motors_velocity(-velocity)
                        self.set_front_right_motor_velocity(velocity)
                        print("Angle reading is", angle)
                        print("Velocity reading is", velocity)
                        print("Final angle reading is", final)
                    else:
                        break   