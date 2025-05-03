# Robotics-project
QR parking UGV Rover PI ROS2
ROS2 ArUco Marker Follower
This repository contains a ROS2 package for detecting and following ArUco markers using a robot with a camera. The system captures video, estimates the pose of ArUco markers, and controls the robot to follow a detected marker using PID control.
Description
The package consists of three main nodes:

VideoPublisher: Captures video from a camera and publishes it to /camera/image_raw.
PoseEstimationNode: Detects ArUco markers in the video feed, estimates their pose, and publishes the processed image to /pose_estimation/output_image.
MarkerFollowerNode: Processes the pose estimation output, calculates control commands using PID controllers, and publishes velocity commands to /cmd_vel to move the robot toward the marker.

Prerequisites

Ubuntu 20.04 or 22.04
ROS2 (Humble or later)
OpenCV with ArUco module
Python 3.8+
A camera compatible with OpenCV
A robot supporting /cmd_vel (e.g., TurtleBot3)

Installation

Install ROS2:Follow the official ROS2 installation guide for your Ubuntu version.

Replace <distro> with your ROS2 distribution (e.g., humble).

Create a ROS2 Workspace:
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws


Clone the Repository:
cd ~/ros2_ws/src
git clone <repository_url>


Build the Workspace:
cd ~/ros2_ws
colcon build
source install/setup.bash



Usage

Start the ROS2 Environment:
source ~/ros2_ws/install/setup.bash


Launch the Nodes:Run each node in a separate terminal or create a launch file.

Video Publisher:ros2 run <package_name> video_cap


Pose Estimation:ros2 run <package_name> pose_est


Marker Follower:ros2 run <package_name> controller



Replace <package_name> with the name of your ROS2 package.

Prepare the ArUco Marker:

Use a 5x5 ArUco marker (DICT_5X5_100) with a size of 0.02m (2cm).
Ensure the marker is visible to the camera.


Monitor the Output:

The processed video with marker detection and pose axes is displayed in a window.
Check the terminal logs for marker detection and control information.



Notes

The camera intrinsic parameters in pose_est.py are hardcoded. Calibrate your camera and update intrinsic_camera and distortion if necessary.
Adjust PID parameters in controller.py for your robot's dynamics.
Ensure the robot's /cmd_vel topic is properly configured.
