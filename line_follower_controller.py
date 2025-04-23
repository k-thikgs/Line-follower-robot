#!/usr/bin/env python3

import cv2
import numpy as np
import time
import sys
from enum import Enum

# For Ignition Gazebo 6, you'll need to use the appropriate ROS 2 packages
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class RobotState(Enum):
    FOLLOWING_LINE = 0
    OBSTACLE_DETECTED = 1
    TURNING_AWAY = 2
    MOVING_PAST_OBSTACLE = 3
    TURNING_BACK = 4
    FINDING_LINE = 5


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        
        # Create a publisher for robot velocity commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/world/line_following_world/model/line_follower_robot/link/chassis/sensor/stereo_camera/image',
            self.camera_callback,
            10)
            
        # Subscribe to lidar topic
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10)
            
        self.bridge = CvBridge()
        self.twist = Twist()
        
        # Base movement speeds
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        
        # PID controller parameters
        self.kp = 0.006  # Proportional gain
        self.ki = 0.0  # Integral gain
        self.kd = 0.002  # Derivative gain
        
        self.prev_error = 0
        self.integral = 0
        
        # Line tracking state
        self.frames_without_line = 0
        self.max_frames_without_line = 5  # Stop after this many frames without line
        
        # Robot state management
        self.current_state = RobotState.FOLLOWING_LINE
        self.state_start_time = time.time()
        self.last_state_change_time = time.time()
        
        # Obstacle avoidance parameters
        self.obstacle_distance_threshold = 0.5  # meters
        self.clear_distance_threshold = 0.8  # meters - distance considered clear for avoidance
        self.avoidance_direction = 1  # 1 for right, -1 for left
        self.turn_angle_achieved = False
        self.moving_past_duration = 3.0  # seconds to move past obstacle
        self.turn_back_duration = 1.5  # seconds to turn back toward line
        self.obstacle_detected_in_front = False
        
        # Last known line position (for returning after obstacle avoidance)
        self.last_line_position = 0
        
        # Lidar scan storage
        self.last_lidar_scan = None
        
        # Debug flag
        self.debug = True
        
        # Create a timer to ensure commands are sent regularly
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Last command time for debugging
        self.last_command_time = time.time()
        
        self.get_logger().info('Line follower node with obstacle avoidance initialized')

    def control_loop(self):
        """Timer-based control loop to ensure commands are sent regularly"""
        if self.current_state == RobotState.OBSTACLE_DETECTED:
            # Ensure we're stopped
            self.stop_robot()
            
            # Check if we've been in this state long enough
            if time.time() - self.state_start_time > 0.5:  # 0.5 second pause
                self.get_logger().info('Transitioning to TURNING_AWAY state')
                self.current_state = RobotState.TURNING_AWAY
                self.state_start_time = time.time()
                self.last_state_change_time = time.time()
                
        elif self.current_state == RobotState.TURNING_AWAY:
            # Force turn command to be sent regularly
            self.execute_turning_away()
            
        elif self.current_state == RobotState.MOVING_PAST_OBSTACLE:
            # Force move command to be sent regularly
            self.execute_moving_past_obstacle()
            
        elif self.current_state == RobotState.TURNING_BACK:
            # Force turn back command to be sent regularly
            self.execute_turning_back()
            
        elif self.current_state == RobotState.FINDING_LINE:
            # Force find line command to be sent regularly
            if time.time() - self.last_command_time > 0.5:
                self.find_line_after_avoidance(False)  # Assume no line detected yet
        
        # Log occasional state updates
        if time.time() - self.last_state_change_time > 5.0:
            self.get_logger().info(f'Current state: {self.current_state.name} for {time.time() - self.state_start_time:.1f}s')
            self.last_state_change_time = time.time()

    def lidar_callback(self, msg):
        """Process incoming lidar data to detect obstacles"""
        self.last_lidar_scan = msg
        
        # Check for obstacles in front
        self.obstacle_detected_in_front = self.check_obstacle_in_front(msg)
        
        # Handle different states
        if self.current_state == RobotState.FOLLOWING_LINE and self.obstacle_detected_in_front:
            self.handle_obstacle_detection()
        elif self.current_state == RobotState.TURNING_AWAY:
            # Check if we've turned enough to clear the obstacle
            if not self.check_obstacle_in_front(msg, wider_angle=True):
                self.get_logger().info('Path clear after turning! Moving forward.')
                self.current_state = RobotState.MOVING_PAST_OBSTACLE
                self.state_start_time = time.time()
                self.last_state_change_time = time.time()
    
    def check_obstacle_in_front(self, scan_msg, wider_angle=False):
        """Check if there's an obstacle in front of the robot"""
        if scan_msg is None:
            return False
            
        # Default front angle range
        front_angle_range = 30  # degrees
        
        # Use wider angle if specified (for turning away)
        if wider_angle:
            front_angle_range = 60
            
        front_indices = self.get_indices_for_angle_range(scan_msg, -front_angle_range/2, front_angle_range/2)
        
        # Extract ranges for the front section
        front_ranges = [scan_msg.ranges[i] for i in front_indices if not np.isinf(scan_msg.ranges[i]) and not np.isnan(scan_msg.ranges[i])]
        
        # Check if there are any obstacles within threshold distance
        if front_ranges and min(front_ranges) < self.obstacle_distance_threshold:
            return True
        
        return False
    
    def get_indices_for_angle_range(self, scan_msg, start_angle, end_angle):
        """Convert angle range in degrees to array indices for the scan message"""
        # Convert to radians
        start_rad = np.deg2rad(start_angle)
        end_rad = np.deg2rad(end_angle)
        
        # Calculate indices
        angle_increment = scan_msg.angle_increment
        start_index = int((start_rad - scan_msg.angle_min) / angle_increment)
        end_index = int((end_rad - scan_msg.angle_min) / angle_increment)
        
        # Ensure indices are within range
        start_index = max(0, min(start_index, len(scan_msg.ranges) - 1))
        end_index = max(0, min(end_index, len(scan_msg.ranges) - 1))
        
        return list(range(start_index, end_index + 1))
    
    def stop_robot(self):
        """Helper function to stop the robot"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.publisher.publish(stop_cmd)
        self.last_command_time = time.time()
    
    def handle_obstacle_detection(self):
        """Handle initial obstacle detection"""
        self.get_logger().info('Obstacle detected! Stopping and preparing to avoid.')
        self.current_state = RobotState.OBSTACLE_DETECTED
        self.state_start_time = time.time()
        self.last_state_change_time = time.time()
        
        # Stop the robot immediately
        self.stop_robot()
        
        # Decide which direction to turn
        self.decide_avoidance_direction()
    
    def decide_avoidance_direction(self):
        """Decide which direction to turn to avoid the obstacle"""
        if self.last_lidar_scan is None:
            # Default to right if no scan data
            self.avoidance_direction = 1
            return
            
        # Check left and right sides to find clearer path
        left_angle_range = 60  # degrees
        right_angle_range = 60  # degrees
        
        left_indices = self.get_indices_for_angle_range(
            self.last_lidar_scan, -left_angle_range, -10)
        right_indices = self.get_indices_for_angle_range(
            self.last_lidar_scan, 10, right_angle_range)
        
        # Get minimum distance on each side
        left_ranges = [self.last_lidar_scan.ranges[i] for i in left_indices 
                      if not np.isinf(self.last_lidar_scan.ranges[i]) and not np.isnan(self.last_lidar_scan.ranges[i])]
        right_ranges = [self.last_lidar_scan.ranges[i] for i in right_indices 
                       if not np.isinf(self.last_lidar_scan.ranges[i]) and not np.isnan(self.last_lidar_scan.ranges[i])]
        
        # Calculate average distance on each side
        left_avg = np.mean(left_ranges) if left_ranges else 0
        right_avg = np.mean(right_ranges) if right_ranges else 0
        
        # Choose the direction with more clearance
        if left_avg > right_avg:
            self.avoidance_direction = -1  # Turn left
            self.get_logger().info(f'More space on left (avg: {left_avg:.2f}m). Turning left.')
        else:
            self.avoidance_direction = 1  # Turn right
            self.get_logger().info(f'More space on right (avg: {right_avg:.2f}m). Turning right.')
        
        # Adjust moving past duration based on obstacle size
        min_front_distance = min([r for r in self.last_lidar_scan.ranges 
                                if not np.isinf(r) and not np.isnan(r)] or [self.obstacle_distance_threshold])
        
        # Longer duration for closer obstacles (approximate size estimation)
        self.moving_past_duration = 3.0 + max(0, (self.obstacle_distance_threshold - min_front_distance) * 4)
            
    def camera_callback(self, msg):
        """Process incoming camera image to detect and follow line"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
            
        # Process the image to detect any line in the frame
        processed_image, error, line_detected, line_position = self.detect_line(cv_image)
        
        # Update last known line position when line is detected
        if line_detected:
            self.last_line_position = error
        
        # State machine for robot behavior
        if self.current_state == RobotState.FOLLOWING_LINE:
            self.approach_and_follow_line(error, line_detected, line_position)
        elif self.current_state == RobotState.FINDING_LINE and line_detected:
            # If we detect the line while searching, resume line following
            self.get_logger().info('Line found after obstacle avoidance! Resuming line following.')
            self.current_state = RobotState.FOLLOWING_LINE
            self.last_state_change_time = time.time()
        
        # Display the processed image (for debugging)
        if self.debug:
            # Display current state
            cv2.putText(processed_image, f"State: {self.current_state.name}", (10, 150), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Display obstacle detection status
            obstacle_status = "OBSTACLE DETECTED" if self.obstacle_detected_in_front else "Path Clear"
            cv2.putText(processed_image, obstacle_status, (10, 180), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255) if self.obstacle_detected_in_front else (0, 255, 0), 2)
            
            # Display turn direction
            direction_text = "LEFT" if self.avoidance_direction == -1 else "RIGHT"
            cv2.putText(processed_image, f"Turn: {direction_text}", (10, 210),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            cv2.imshow("Line Detection", processed_image)
            cv2.waitKey(1)
    
    def execute_turning_away(self):
        """Turn away from the obstacle until path is clear"""
        # Make sure we turn with a strong enough angular velocity
        self.twist.linear.x = 0.0
        self.twist.angular.z = self.angular_speed * 1.5 * self.avoidance_direction  # Increased turn rate
        self.publisher.publish(self.twist)
        self.last_command_time = time.time()
        
        self.get_logger().debug(f'Turning away with angular.z = {self.twist.angular.z}')
        
        # Log that we're turning and which direction
        if time.time() - self.state_start_time < 0.2:  # Only log once at the start
            direction = "LEFT" if self.avoidance_direction < 0 else "RIGHT"
            self.get_logger().info(f'Executing turn {direction} to avoid obstacle')
        
        # If we've been turning for too long, force move to next state
        if time.time() - self.state_start_time > 3.0:  # 3 second timeout
            self.get_logger().warning('Turning timeout reached. Moving forward anyway.')
            self.current_state = RobotState.MOVING_PAST_OBSTACLE
            self.state_start_time = time.time()
            self.last_state_change_time = time.time()
    
    def execute_moving_past_obstacle(self):
        """Move forward past the obstacle"""
        self.twist.linear.x = -self.linear_speed  # Forward
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist)
        self.last_command_time = time.time()
        
        # Check if we should go back to turning if obstacle appears again
        if self.obstacle_detected_in_front:
            self.get_logger().warning('Obstacle detected while moving past! Turning again.')
            self.current_state = RobotState.TURNING_AWAY
            self.state_start_time = time.time()
            self.last_state_change_time = time.time()
            return
        
        # After moving forward for a set duration, turn back toward line
        if time.time() - self.state_start_time > self.moving_past_duration:
            self.get_logger().info('Moved past obstacle. Turning back toward line.')
            self.current_state = RobotState.TURNING_BACK
            self.state_start_time = time.time()
            self.last_state_change_time = time.time()
    
    def execute_turning_back(self):
        """Turn back toward the line"""
        self.twist.linear.x = -self.linear_speed * 0.5  # Slower forward while turning
        self.twist.angular.z = -self.angular_speed * 1.5 * self.avoidance_direction  # Strong turn in opposite direction
        self.publisher.publish(self.twist)
        self.last_command_time = time.time()
        
        # After turning for a set duration, start looking for the line
        if time.time() - self.state_start_time > self.turn_back_duration:
            self.get_logger().info('Turned back toward line. Now searching for line.')
            self.current_state = RobotState.FINDING_LINE
            self.state_start_time = time.time()
            self.last_state_change_time = time.time()
    
    def find_line_after_avoidance(self, line_detected):
        """Search for the line after completing obstacle avoidance"""
        if line_detected:
            # Line found, resume normal line following
            self.get_logger().info('Line found after obstacle avoidance! Resuming line following.')
            self.current_state = RobotState.FOLLOWING_LINE
            self.last_state_change_time = time.time()
            return
        
        # Keep moving forward while looking for the line
        self.twist.linear.x = -self.linear_speed * 0.7
        
        # Slight turn toward where the line should be
        self.twist.angular.z = -self.angular_speed * 0.5 * self.avoidance_direction
        
        # If we've been searching for too long, start doing more aggressive turns
        current_time = time.time()
        if current_time - self.state_start_time > 2.0:
            self.twist.angular.z = -self.angular_speed * 1.0 * self.avoidance_direction
        
        # Give up and stop if we've been searching too long
        if current_time - self.state_start_time > 10.0:
            self.get_logger().warning('Failed to find line after extended search. Stopping.')
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            
            # After stopping, try to restore line following mode anyway
            # This is a fallback in case the line is completely lost
            if current_time - self.state_start_time > 11.0:
                self.get_logger().warning('Resetting to line following mode anyway.')
                self.current_state = RobotState.FOLLOWING_LINE
                self.last_state_change_time = time.time()
        
        # Publish velocity command
        self.publisher.publish(self.twist)
        self.last_command_time = time.time()
    
    def detect_line(self, image):
        """
        Process image to detect line and calculate position error
        Returns: (processed_image, error, line_detected, line_position)
        """
        # Get image dimensions
        height, width = image.shape[:2]
        
        # Create a copy of the original image for visualization
        result_image = image.copy()
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply thresholding for black line on light background
        # Use a simple threshold that works for your specific environment
        ret, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)
        
        # Display thresholded image in bottom corner for debugging
        if self.debug:
            small_thresh = cv2.resize(thresh, (width//4, height//4))
            result_image[height-small_thresh.shape[0]:height, 0:small_thresh.shape[1]] = cv2.cvtColor(small_thresh, cv2.COLOR_GRAY2BGR)
        
        # Find contours
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Calculate center of the image
        center_x = width // 2
        
        # Draw the center line of the image
        cv2.line(result_image, (center_x, 0), (center_x, height), (0, 0, 255), 2)
        
        # Default values
        error = 0
        line_detected = False
        line_position = "none"  # Can be "center", "left", "right", or "none"
        
        # Define region of interest - bottom half of the image
        roi_height = height // 2
        roi_y = height - roi_height
        
        # Draw ROI on the result image
        cv2.rectangle(result_image, (0, roi_y), (width, height), (255, 255, 0), 2)
        
        if contours:
            # Find contours in the ROI
            valid_contours = []
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                # Check if contour is in the ROI and big enough
                if (y + h > roi_y) and cv2.contourArea(contour) > 100:
                    valid_contours.append(contour)
            
            if valid_contours:
                # Find the largest contour in the ROI
                largest_contour = max(valid_contours, key=cv2.contourArea)
                
                # Get the contour area
                contour_area = cv2.contourArea(largest_contour)
                
                # Draw the contour
                cv2.drawContours(result_image, [largest_contour], -1, (0, 255, 0), 2)
                
                # Get the moments of the contour
                M = cv2.moments(largest_contour)
                
                # Calculate the center of the contour
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Draw a circle at the center of the contour
                    cv2.circle(result_image, (cx, cy), 7, (255, 0, 0), -1)
                    
                    # Mark as line detected
                    line_detected = True
                    
                    # Calculate error from center
                    error = cx - center_x
                    
                    # Determine line position relative to robot
                    if abs(error) < width * 0.1:  # If line is within 10% of center
                        line_position = "center"
                    elif error < 0:
                        line_position = "left"
                    else:
                        line_position = "right"
                    
                    # Display the error value and position on the image
                    cv2.putText(result_image, f"Error: {error}", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    cv2.putText(result_image, f"Position: {line_position}", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    cv2.putText(result_image, f"Area: {contour_area:.1f}", (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        # Display line detection status on image
        status_text = "Line Detected" if line_detected else "No Line Detected"
        status_color = (0, 255, 0) if line_detected else (0, 0, 255)
        cv2.putText(result_image, status_text, (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        # Return the processed image, error, and detection information
        return result_image, error, line_detected, line_position
    
    def approach_and_follow_line(self, error, line_detected, line_position):
        """
        Approach a detected line and follow it, or search for line if not detected
        """
        if line_detected:
            # Reset counter when line is found
            self.frames_without_line = 0
            
            # Calculate PID control
            self.integral = max(min(self.integral + error, 100), -100)  # Limit integral windup
            derivative = error - self.prev_error
            angular_z = self.kp * error + self.ki * self.integral + self.kd * derivative
            
            # Set movement based on line position
            if line_position == "center":
                # Follow line with PID control
                self.twist.linear.x = -self.linear_speed  # Negative for forward in your setup
                self.twist.angular.z = angular_z
                self.get_logger().debug('Following line in center')
            
            elif line_position == "left":
                # Turn left to follow the line
                self.twist.linear.x = -self.linear_speed * 0.8
                self.twist.angular.z = -0.3
                self.get_logger().debug('Line on left, turning left')
            
            elif line_position == "right":
                # Turn right to follow the line
                self.twist.linear.x = -self.linear_speed * 0.8
                self.twist.angular.z = 0.3
                self.get_logger().debug('Line on right, turning right')
        else:
            # Increment counter when no line is detected
            self.frames_without_line += 1
            
            if self.frames_without_line <= self.max_frames_without_line:
                # Keep last direction for a few frames
                self.twist.linear.x = -self.linear_speed * 0.5
                # Angular velocity remains the same (keep turning in same direction)
                self.get_logger().debug(f'Line temporarily lost, continuing last direction ({self.frames_without_line}/{self.max_frames_without_line})')
            else:
                # Stop after max frames without line
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.get_logger().debug('Line lost completely, stopping robot')
        
        # Publish velocity command
        self.publisher.publish(self.twist)
        self.last_command_time = time.time()
        
        # Update previous error for derivative term
        self.prev_error = error


def main(args=None):
    rclpy.init(args=args)
    
    line_follower = LineFollowerNode()
    
    try:
        rclpy.spin(line_follower)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        stop_msg = Twist()
        line_follower.publisher.publish(stop_msg)
        
        # Clean up
        line_follower.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
