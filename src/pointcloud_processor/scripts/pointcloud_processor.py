#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2, PointField
from nav_msgs.msg import Odometry
import numpy as np
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Header
import transforms3d

class PointCloudProcessor:
    def __init__(self):
        rospy.init_node('pointcloud_processor', anonymous=True)

        # Set up publishers and subscribers
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.odom_sub = rospy.Subscriber("/camera/odom/sample", Odometry, self.odom_callback)
        self.pointcloud_pub = rospy.Publisher("/transformed_pointcloud", PointCloud2, queue_size=1)

        self.bridge = CvBridge()
        self.gravity_vector = None

        rospy.loginfo("PointCloudProcessor node initialized")

    def image_callback(self, img_msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")

            # Generate a dummy point cloud (you can add actual point cloud logic here)
            height, width = cv_image.shape[:2]
            points = []

            for i in range(0, height, 10):  # Sample every 10 pixels for efficiency
                for j in range(0, width, 10):
                    rgb = cv_image[i, j]
                    # Dummy XYZ values, replace with depth or other data if available
                    x, y, z = float(i) / height, float(j) / width, 1.0
                    r, g, b = rgb[2], rgb[1], rgb[0]  # BGR to RGB conversion
                    points.append([x, y, z, (r << 16) | (g << 8) | b])

            # Convert list of points to PointCloud2 message
            pointcloud_msg = self.create_pointcloud2(points, img_msg.header)

            # Publish the point cloud
            self.pointcloud_pub.publish(pointcloud_msg)
            rospy.loginfo("Published point cloud")

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def create_pointcloud2(self, points, header):
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
        ]
        return pc2.create_cloud(header, fields, points)

    def odom_callback(self, odom_msg):
        quaternion = odom_msg.pose.pose.orientation
        # Convert quaternion to a gravity vector
        self.gravity_vector = self.quaternion_to_gravity(quaternion)

        rospy.loginfo(f"Gravity vector: {self.gravity_vector}")

    def quaternion_to_gravity(self, quaternion):
        quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        # Convert quaternion to Euler angles to compute the gravity vector
        euler = transforms3d.euler.quat2euler(quat)

        # Return gravity vector assuming pitch and roll only (gravity in Z direction)
        gravity_vector = np.array([0, 0, 1.0])
        rotation_matrix = transforms3d.euler.euler2mat(euler[0], euler[1], 0)  # Ignore yaw for gravity
        gravity_vector = np.dot(rotation_matrix, gravity_vector)
        return gravity_vector

if __name__ == '__main__':
    try:
        processor = PointCloudProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

