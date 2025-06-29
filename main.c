from controller import Supervisor
from fairis_tools.lab5_MyRobot import MyRobot
from fairis_lib.robot_lib import *
import math

class RobotNavigator:
    def __init__(self, robot):
        self.robot = robot
        self.wheel_radius = 0.043
        self.axel_distance = 0.053
        self.cell_size = 1.0  # 1 meter per cell
        
    def rotate_to_orientation(self, target_angle):
        """
        Rotate the robot to a specific angle
        Implement precise rotation using compass and motor control
        """
        current_angle = self.robot.get_compass_reading()
        angle_difference = target_angle - current_angle
        
        # Determine rotation direction
        if angle_difference > 180:
            angle_difference -= 360
        elif angle_difference < -180:
            angle_difference += 360
        
        # Set rotation speed based on angle difference
        rotation_speed = 2.0 if abs(angle_difference) > 5 else 0.5
        
        # Rotate robot
        if angle_difference > 0:
            self.robot.set_front_left_motor_velocity(rotation_speed)
            self.robot.set_front_right_motor_velocity(-rotation_speed)
        else:
            self.robot.set_front_left_motor_velocity(-rotation_speed)
            self.robot.set_front_right_motor_velocity(rotation_speed)
        
        # Wait until close to target angle
        while abs(self.robot.get_compass_reading() - target_angle) > 5:
            self.robot.experiment_supervisor.step(self.robot.timestep)
        
        # Stop rotation
        self.robot.stop()
    
    def move_to_next_cell(self, current_cell, next_cell):
        """
        Move robot from current cell to next cell
        Determine required rotation and distance
        """
        # Convert cell numbers to grid coordinates
        current_row = (current_cell - 1) // 5
        current_col = (current_cell - 1) % 5
        next_row = (next_cell - 1) // 5
        next_col = (next_cell - 1) % 5
        
        # Determine movement direction and angle
        dx = next_col - current_col
        dy = next_row - current_row
        
        # Determine target angle based on movement direction
        if dx > 0:  # Move East
            target_angle = 0  # East is 0 degrees
        elif dx < 0:  # Move West
            target_angle = 180  # West is 180 degrees
        elif dy > 0:  # Move South
            target_angle = 270  # South is 270 degrees
        elif dy < 0:  # Move North
            target_angle = 90  # North is 90 degrees
        
        # Rotate to correct orientation
        self.rotate_to_orientation(target_angle)
        
        # Move forward one cell distance
        distance = self.cell_size
        linear_velocity = 0.5  # Adjust as needed
        
        # Calculate time to move one cell
        move_time = distance / linear_velocity
        start_time = self.robot.experiment_supervisor.getTime()
        
        # Move forward
        while self.robot.experiment_supervisor.getTime() - start_time < move_time:
            self.robot.set_front_left_motor_velocity(linear_velocity)
            self.robot.set_front_right_motor_velocity(linear_velocity)
            self.robot.experiment_supervisor.step(self.robot.timestep)
        
        # Stop robot
        self.robot.stop()
        
        return True

def main():
    # Similar to previous implementation, but use RobotNavigator
    robot = MyRobot()
    maze_file = '../../worlds/Fall24/maze8.xml'
    robot.load_environment(maze_file)
    
    navigator = RobotNavigator(robot)
    
    # Path planning and navigation logic
    # ... (similar to previous implementation)

if __name__ == "__main__":
    main()