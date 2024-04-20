from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
'''
For Better result, please calibrate the camera.
'''
class LQRController(Node):  #negative angular velocity=>right turn,negative linear velocity=>forward
    def __init__(self):
        super().__init__('lqr_controller')
        self.declare_parameters(namespace='', parameters=[                         
        ('dt', 1.0),                    #timestamp
        ('linear_velocity_limit', 0.1), 
        ('angular_velocity_limit', 0.1),
        ('distance_tolerance', 0.1),    # Tolerance for distance error
        ('angle_tolerance', 0.04)       # Tolerance for angle error
    ])
        
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription_distance = self.create_subscription(Vector3, 'distance_variation', self.distance_callback, 10) #please refer to apriltag_video.py
        self.subscription_angles = self.create_subscription(Vector3, 'angle_variation', self.angle_callback, 10)         #please refer to apriltag_video.py

        self.actual_state_x = np.array([0.0, 0.0, 0.0])     # Target angle
        self.desired_state_xf = np.array([1.77, 0.0, 0.0])  # Target distance unit(m)

        self.A = np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])   
        self.R = np.array([[0.01, 0], [0, 0.01]])
        self.Q = np.array([[0.639, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])

        self.target_reached = False

    def distance_callback(self, msg):
        self.actual_state_x[0] = msg.x  # x contains distance
        self.run_lqr()

    def angle_callback(self, msg):
        self.actual_state_x[2] = msg.y  #  y contains angle
        self.run_lqr()

    def run_lqr(self):
        if not self.target_reached:
            distance_error = abs(self.actual_state_x[0] - self.desired_state_xf[0])
            angle_error = abs(self.actual_state_x[2] - self.desired_state_xf[2])
            '''
            stop the robot if the target is reached
            '''
            if distance_error <= self.get_parameter('distance_tolerance').get_parameter_value().double_value and angle_error <= self.get_parameter('angle_tolerance').get_parameter_value().double_value:
                self.target_reached = True
                self.stop_robot()
            else:
                B = self.getB(self.actual_state_x[2], self.get_parameter('dt').get_parameter_value().double_value)
                optimal_control_input = self.lqr(self.actual_state_x, self.desired_state_xf, self.Q, self.R, self.A, B)
                self.publish_cmd(optimal_control_input)

    def getB(self, yaw, dt):
        return np.array([[np.cos(yaw) * dt, 0], [np.sin(yaw) * dt, 0], [0, dt]])

    def lqr(self, actual_state_x, desired_state_xf, Q, R, A, B):
        x_error = actual_state_x - desired_state_xf
        N = 100
        P = [None] * (N + 1)
        P[N] = Q
        for i in range(N, 0, -1):
            P[i - 1] = Q + A.T @ P[i] @ A - (A.T @ P[i] @ B) @ np.linalg.pinv(R + B.T @ P[i] @ B) @ (B.T @ P[i] @ A)
        K = [None] * N
        for i in range(N):
            K[i] = -np.linalg.pinv(R + B.T @ P[i + 1] @ B) @ B.T @ P[i + 1] @ A
        return K[i] @ x_error

    def publish_cmd(self, control_input):
        msg = Twist()
        msg.linear.x = -np.clip(control_input[0], -self.get_parameter('linear_velocity_limit').get_parameter_value().double_value, self.get_parameter('linear_velocity_limit').get_parameter_value().double_value)  # Limit linear velocity
        
        angle_variation_sign = np.sign(self.actual_state_x[2])  # Get the sign of the y component
        angular_velocity_sign = np.sign(control_input[1])  # Get the sign of the angular velocity
        
        if angle_variation_sign != angular_velocity_sign: #angle_velocity should have the same polarity as angle_variation,if not, invert the polarity of angle_velocity
            control_input[1] *= -1  
        
        msg.angular.z = np.clip(control_input[1], -self.get_parameter('angular_velocity_limit').get_parameter_value().double_value, self.get_parameter('angular_velocity_limit').get_parameter_value().double_value)
        self.publisher_cmd_vel.publish(msg)

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_cmd_vel.publish(msg)
        self.get_logger().info('Target reached. Shutting down the node...')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    lqr_controller = LQRController()
    rclpy.spin(lqr_controller)
    lqr_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
