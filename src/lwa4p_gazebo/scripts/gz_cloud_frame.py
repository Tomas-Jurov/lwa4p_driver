#!/usr/bin/env python3
"""Gazebo Harmonic only: its rgbd_camera publishes the point cloud in the
camera body axes (x forward, y left, z up) but labels it with the sensor's
gz_frame_id (the optical frame, z forward). This relay republishes the cloud
unchanged with frame_id = camera_link, whose axes match the data.

    /camera/gz_points  ->  /camera/depth/color/points   (frame camera_link)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class CloudFrame(Node):
    def __init__(self):
        super().__init__("gz_cloud_frame")
        self.frame = self.declare_parameter("frame_id", "camera_link").value
        # reliable, like Gazebo Classic and the real RealSense (works for best-effort subscribers too)
        self.pub = self.create_publisher(PointCloud2, "/camera/depth/color/points", 5)
        self.create_subscription(PointCloud2, "/camera/gz_points", self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, msg):
        msg.header.frame_id = self.frame
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CloudFrame()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
