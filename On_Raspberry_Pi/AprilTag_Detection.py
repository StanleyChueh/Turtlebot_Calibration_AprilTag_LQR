#!/usr/bin/env python

from argparse import ArgumentParser
import cv2
import apriltag
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
'''
By using inverse matrix to get the matrix in the apriltag frame
'''
def inverse_matrix(pose_matrix):
    rotation_matrix = pose_matrix[:3, :3]
    translation_vector = pose_matrix[:3, 3]

    inv_rotation_matrix = np.linalg.inv(rotation_matrix)
    inv_translation_vector = -np.dot(inv_rotation_matrix, translation_vector)

    inverse_pose_matrix = np.zeros_like(pose_matrix)
    inverse_pose_matrix[:3, :3] = inv_rotation_matrix
    inverse_pose_matrix[:3, 3] = inv_translation_vector
    inverse_pose_matrix[3, 3] = 1

    return inverse_pose_matrix
'''
find the angle by extracting rotational matrix in homogeneous transformation matrix
'''
def calculate_rotation_angles(rotation_matrix):
    rx = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
    ry = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2))
    rz = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

    return rx, ry, rz
'''
publish the distance and angle variation topic between camera and apriltag, and also publish the coordinate of apriltag in camera frame.
notice:remember to adjust the camera parameter, tag_size if needed.
'''
class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('april_tag_detector')
        self.publisher_distance = self.create_publisher(Vector3, 'distance_variation', 10)
        self.publisher_angles = self.create_publisher(Vector3, 'angle_variation', 10)
        self.publisher_coordinates = self.create_publisher(Vector3, 'tag_coordinates', 10)  # Create publisher for coordinates

        # Initialize AprilTag detector
        parser = ArgumentParser(description='Detect AprilTags from video stream.')
        apriltag.add_arguments(parser)
        options = parser.parse_args()
        self.detector = apriltag.Detector(options, searchpath=apriltag._get_dll_path())
    
    def run_detection(self, frame):
        result, overlay = apriltag.detect_tags(frame, self.detector, camera_params=(1179.1320837196463, 1135.0261675982474, 305.294945188202, 325.4071232674449),
                                                tag_size=0.0762, vizualization=3, verbose=3, annotation=True)#0.762
        if result:
            pose_info = result[1]
            inverse_homography = inverse_matrix(pose_info)
            translation = inverse_homography[:3, 3]  # Camera's position relative to AprilTag

            # Publish camera frame coordinates
            coordinates_msg = Vector3()
            coordinates_msg.x, coordinates_msg.y, coordinates_msg.z = translation
            self.publisher_coordinates.publish(coordinates_msg)

            # Calculate distance variation (positional variation)
            distance_variation = np.linalg.norm(translation)

            # Publish distance variation
            distance_msg = Vector3()
            distance_msg.x = distance_variation
            self.publisher_distance.publish(distance_msg)

            # Publish angle variation
            angles_msg = Vector3()
            angles_msg.x, angles_msg.y, angles_msg.z = calculate_rotation_angles(inverse_homography[:3, :3])
            self.publisher_angles.publish(angles_msg)

            # Display the calculated information
            self.get_logger().info("Distance Variation: %f" % distance_variation)
            self.get_logger().info("Angle Variation: X:%f, Y:%f, Z:%f" % calculate_rotation_angles(inverse_homography[:3, :3]))

def main(args=None):
    rclpy.init(args=args)
    april_tag_detector = AprilTagDetector()

    # Set up video capture
    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

    try:
        while rclpy.ok() and video.isOpened():
            success, frame = video.read()
            if not success:
                break

            frame = cv2.resize(frame, (160, 120))

            april_tag_detector.run_detection(frame)

            if cv2.waitKey(1) & 0xFF == ord(' '):
                break

    finally:
        video.release()
        cv2.destroyAllWindows()

    april_tag_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


