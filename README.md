# Robot Calibration by LQR Control
I aim to adjust the robot's position based on the coordinates of an AprilTag. The target position is at a certain distance from the AprilTag, with only a minor angle variation between the camera and the AprilTag.
## Step1
Run calibration.py in Camera_Calibration folder to get the camera parameters for AprilTag detection
## On_PC
### RUN THIS SCRIPT ON PC!!
The script in this folder is the controller base on distance and angle variation topics from "On_Raspberry_Pi"
## On_Raspberry_Pi
### RUN THIS SCRIPT ON RASPBERRY PI!!
The script in this folder aim to publish two topisc（distance variation and angle variation)
## Reference
AprilTag detection: https://github.com/Tinker-Twins/AprilTag
<br> Turtlebot: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
<br> ROS2 Foxy: https://docs.ros.org/en/foxy/Installation.html
