import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import time
import signal
import sys

class MarkerFollowerNode(Node):
    def __init__(self):
        super().__init__('marker_follower_node')
        
        self.subscription = self.create_subscription(
            Image, '/pose_estimation/output_image', self.image_callback, 10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        
        # ArUco detector setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.parameters = cv2.aruco.DetectorParameters()
        
        self.target_marker_size = 100
        self.target_x_position = 640  
        self.target_y_position = 360 
        
        # PID controller parameters
        self.x_pid = PIDController(kp=0.0005, ki=0.00005, kd=0.0001)  # For x-axis positioning
        self.y_pid = PIDController(kp=0.0005, ki=0.00005, kd=0.0001)  # For y-axis positioning
        self.z_pid = PIDController(kp=0.0005, ki=0.00005, kd=0.0001)  # For distance/size control
        
        # Robot movement parameters
        self.max_linear_speed = 0.2 
        self.max_angular_speed = 0.3 
        
        # Search mode parameters
        self.search_mode = False
        self.search_timer = None
        self.last_marker_time = self.get_clock().now()
        self.marker_timeout = 5.0 
        self.marker_found = False
        
        self.was_tracking = False
        
        self.get_logger().info("Marker follower node started with reduced PID values...")

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # Process the frame to find and approach the marker
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        corners, ids, rejected = detector.detectMarkers(gray)
        
        twist = Twist()
        
        if ids is not None and len(corners) > 0:
            if self.search_mode:
                self.get_logger().info("Marker found after searching")
                # Reset PID controllers to avoid accumulated error
                self.x_pid.reset()
                self.y_pid.reset()
                self.z_pid.reset()
                
            self.search_mode = False
            self.last_marker_time = self.get_clock().now()
            self.marker_found = True
            self.was_tracking = True
            
            # We'll use the first detected marker
            marker_corners = corners[0][0]
            
            # Calculate marker center
            marker_center_x = np.mean(marker_corners[:, 0])
            marker_center_y = np.mean(marker_corners[:, 1])
            
            # Calculate marker size
            width = np.linalg.norm(marker_corners[0] - marker_corners[1])
            height = np.linalg.norm(marker_corners[1] - marker_corners[2])
            marker_size = (width + height) / 2
            
            # Calculate errors
            x_error = self.target_x_position - marker_center_x  # Positive: marker is to the left
            y_error = self.target_y_position - marker_center_y  # Positive: marker is above
            z_error = self.target_marker_size - marker_size     # Positive: marker is too small (too far)
            
            # Update PID controllers
            x_correction = self.x_pid.update(x_error)
            y_correction = self.y_pid.update(y_error)
            z_correction = self.z_pid.update(z_error)

            angular_z = x_correction
            linear_x = z_correction
            
            # Apply limits
            twist.linear.x = max(min(linear_x, self.max_linear_speed), -self.max_linear_speed)
            twist.angular.z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)

            if abs(twist.linear.x) < 0.01:
                twist.linear.x = 0.0
            if abs(twist.angular.z) < 0.01:
                twist.angular.z = 0.0
            
            # Log information
            self.get_logger().info(f"Marker detected: Center({marker_center_x:.1f}, {marker_center_y:.1f}), Size: {marker_size:.1f}")
            self.get_logger().info(f"Errors: X:{x_error:.1f}, Y:{y_error:.1f}, Size:{z_error:.1f}")
            self.get_logger().info(f"Commands: LinearX:{twist.linear.x:.3f}, AngularZ:{twist.angular.z:.3f}")
            
            # Draw target indicators on the frame
            cv2.circle(frame, (int(marker_center_x), int(marker_center_y)), 5, (0, 255, 0), -1)
            cv2.circle(frame, (self.target_x_position, self.target_y_position), 5, (0, 0, 255), -1)
            cv2.line(frame, (int(marker_center_x), int(marker_center_y)), 
                     (self.target_x_position, self.target_y_position), (255, 0, 0), 2)
            
        else:
            # No marker detected
            if self.was_tracking:
                self.x_pid.reset()
                self.y_pid.reset()
                self.z_pid.reset()
                self.was_tracking = False
                
            current_time = self.get_clock().now()
            time_since_last_marker = (current_time - self.last_marker_time).nanoseconds / 1e9
            
            if self.marker_found and time_since_last_marker > self.marker_timeout:
                if not self.search_mode:
                    self.get_logger().warning(f"No marker detected for {self.marker_timeout} seconds - starting search")
                    self.search_mode = True
                twist.angular.z = 0.1  
                twist.linear.x = 0.0 
            else:
                # Just stop if marker recently disappeared or never detected
                search_status = "waiting" if self.marker_found else "no marker ever detected"
                self.get_logger().warning(f"No marker detected - {search_status} ({time_since_last_marker:.1f}/{self.marker_timeout} sec)")
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        status_text = "Searching" if self.search_mode else "Tracking" if ids is not None else "Waiting"
        cv2.putText(frame, f"Status: {status_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        self.cmd_vel_publisher.publish(twist)

        cv2.imshow("Marker Follower", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.get_logger().info("Shutting down marker follower node...")
        cv2.destroyAllWindows()
        stop_twist = Twist()
        for _ in range(10):
            self.cmd_vel_publisher.publish(stop_twist)
            time.sleep(0.5)
        self.get_logger().info("Node shutdown complete")
        super().destroy_node()


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        
        self.previous_error = 0
        self.integral = 0
        
    def update(self, error):
        # Calculate P term
        p_term = self.kp * error
        
        # Calculate I term
        self.integral += error
        # Limit integral windup
        self.integral = max(min(self.integral, 1000), -1000)
        i_term = self.ki * self.integral
        
        # Calculate D term
        d_term = self.kd * (error - self.previous_error)
        self.previous_error = error
        
        # Return total correction
        return p_term + i_term + d_term
    
    def reset(self):
        """Reset the controller state"""
        self.previous_error = 0
        self.integral = 0


def main(args=None):
    rclpy.init(args=args)
    node = MarkerFollowerNode()
    
    # Add signal handlers for proper termination
    def signal_handler(sig, frame):
        node.get_logger().info("Received termination signal")
        stop_twist = Twist()
        for _ in range(10):
            node.cmd_vel_publisher.publish(stop_twist)
            time.sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt detected")
    finally:
        # Send stop command before exiting
        stop_twist = Twist()
        for _ in range(10):
            node.cmd_vel_publisher.publish(stop_twist)
            time.sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
