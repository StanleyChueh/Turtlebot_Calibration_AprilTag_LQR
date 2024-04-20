# Robot calibration by LQR control
I aim to adjust the robot's position based on the coordinates of an AprilTag. The target position is at a certain distance from the AprilTag, with only a minor angle variation between the camera and the AprilTag.
## Step1 
1. Run calibration.py in Camera_Calibration folder to get the camera parameters for AprilTag detection.
``` 
cd Desktop/
git clone https://github.com/StanleyChueh/Turtlebot_Calibration_AprilTag_LQR.git
cd Turtlebot_Calibration_AprilTag_LQR/
cd Camera_Calibration/
python3 calibration.py 
```
### use the following board to calibrate:
<br> Calibration board: [Checkerboard-A3-40mm-9x6.pdf](https://github.com/StanleyChueh/Turtlebot_Calibration_AprilTag_LQR/files/15047269/Checkerboard-A3-40mm-9x6.pdf)
<br> You will get the result like the following image:
![Screenshot from 2024-04-20 15-15-47](https://github.com/StanleyChueh/Turtlebot_Calibration_AprilTag_LQR/assets/153347369/96bc001b-4800-41f6-ba3a-d3a0d722a7e7)
## Step2
1. git clone 
```
cd Desktop/
git clone https://github.com/Tinker-Twins/AprilTag.git
```
2. Place this script(On_Raspberry_Pi/AprilTag_Detection.py) in this directory
```
cd ~/Desktop/AprilTag/scripts
```
3. Make it executable
```
cd ~/Desktop/AprilTag
chmod +x ./install.sh
``` 
4. Run it
```
 cd ~/Desktop/AprilTag
 ./install.sh
```
5. Run AprilTag detection script
```
python3 AprilTag_Detection.py
```
6. You will see this result like the following images:
<br> ![Screenshot from 2024-04-20 15-37-19](https://github.com/StanleyChueh/Turtlebot_Calibration_AprilTag_LQR/assets/153347369/2742ac78-2d20-4bad-b255-0946c9d9ec5a)
<br> ![Screenshot from 2024-04-20 15-37-39](https://github.com/StanleyChueh/Turtlebot_Calibration_AprilTag_LQR/assets/153347369/3b70e621-38fc-4e41-bf34-60b20240dbfa)
### notice: Run this script on Raspberry pi for real world testing(ssh -X ubuntu@ip)
## Step3
```
cd 
```
Run the script in this directory:(On_PC/controller.py)
## Demo
![Screenshot from 2024-04-20 16-18-16](https://github.com/StanleyChueh/Turtlebot_Calibration_AprilTag_LQR/assets/153347369/38b5b7ea-aa35-4e2f-9b54-a8c131abcbc0)

Following are the videos of different angle. <br>
1.https://youtube.com/shorts/0O0KmvSdWzU?si=QNJmLeCMe7fS9b95 <br>
2.https://youtube.com/shorts/TNQPaHkxuaM?si=CKUKYXBqYaNOFerX <br>
3.https://youtube.com/shorts/TcjzHa8okXQ?si=dLZovFBaN9nTNmqJ
## Reference
AprilTag detection: https://github.com/Tinker-Twins/AprilTag
<br> Turtlebot: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
<br> ROS2 Foxy: https://docs.ros.org/en/foxy/Installation.html
