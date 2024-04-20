# Robot Calibration by LQR Control
I aim to adjust the robot's position based on the coordinates of an AprilTag. The target position is at a certain distance from the AprilTag, with only a minor angle variation between the camera and the AprilTag.
## Step1
Run calibration.py in Camera_Calibration folder to get the camera parameters for AprilTag detection.
<br> Calibration board: [Checkerboard-A3-40mm-9x6.pdf](https://github.com/StanleyChueh/Turtlebot_Calibration_AprilTag_LQR/files/15047269/Checkerboard-A3-40mm-9x6.pdf)
<br> You will get the result like the following image:
![Screenshot from 2024-04-20 15-15-47](https://github.com/StanleyChueh/Turtlebot_Calibration_AprilTag_LQR/assets/153347369/96bc001b-4800-41f6-ba3a-d3a0d722a7e7)
## Step2
Follow the instruciton in：https://github.com/Tinker-Twins/AprilTag
1. $ git clone https://github.com/Tinker-Twins/AprilTag.git
2. place this script(On_Raspberry_Pi/AprilTag_Detection.py) in this directory~/Desktop/AprilTag/scripts$ 
3. ~/Desktop/AprilTag$ chmod +x ./install.sh
4. ~/Desktop/AprilTag$ ./install.sh
5. python3 AprilTag_Detection.py
6. you will see this result like the following images:
<br> ![Screenshot from 2024-04-20 15-37-19](https://github.com/StanleyChueh/Turtlebot_Calibration_AprilTag_LQR/assets/153347369/2742ac78-2d20-4bad-b255-0946c9d9ec5a)
<br> ![Screenshot from 2024-04-20 15-37-39](https://github.com/StanleyChueh/Turtlebot_Calibration_AprilTag_LQR/assets/153347369/3b70e621-38fc-4e41-bf34-60b20240dbfa)
## On_PC
### RUN THIS SCRIPT ON PC!!
The script in this folder is the controller base on distance and angle variation topics from "On_Raspberry_Pi"
## On_Raspberry_Pi
### RUN THIS SCRIPT ON RASPBERRY PI!!
The script in this folder aim to publish two topisc（distance variation and angle variation)
## Demo


## Reference
AprilTag detection: https://github.com/Tinker-Twins/AprilTag
<br> Turtlebot: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
<br> ROS2 Foxy: https://docs.ros.org/en/foxy/Installation.html
