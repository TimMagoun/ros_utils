#!/usr/bin/env python3

import argparse
import os

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image


class TfListener:
    """
    This class takes in a node handle and creates a tf listener. It also subscribes to an image topic and
    publishes the transform between the camera and the world frames as a geometry_msgs/PoseStamped message
    when a new image is received.
    """

    def __init__(
        self,
        output_file_path,
        node_handle,
        image_topic,
        pose_topic,
        child_frame_id,
        parent_frame_id,
    ):
        self.node_handle = node_handle
        self.output_file_path = output_file_path
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.child_frame_id = child_frame_id
        self.parent_frame_id = parent_frame_id
        self.image_subscriber = self.node_handle.Subscriber(
            image_topic, Image, self.image_callback
        )
        self.pose_publisher = self.node_handle.Publisher(
            pose_topic, PoseStamped, queue_size=1
        )

    def image_callback(self, image: Image):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame_id,
                self.child_frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            pose = PoseStamped()
            pose.header = image.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation.x = transform.transform.rotation.x
            pose.pose.orientation.y = transform.transform.rotation.y
            pose.pose.orientation.z = transform.transform.rotation.z
            pose.pose.orientation.w = transform.transform.rotation.w
            self.pose_publisher.publish(pose)

            with open(self.output_file_path, "a") as f:
                f.write(
                    f"{pose.header.stamp.secs}.{pose.header.stamp.nsecs} "
                    f"{pose.pose.position.x} {pose.pose.position.y} {pose.pose.position.z} "
                    f"{pose.pose.orientation.x} {pose.pose.orientation.y} {pose.pose.orientation.z} "
                    f"{pose.pose.orientation.w}\n"
                )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn("Unable to get transform between camera and world frames.")
            rospy.logwarn(e)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("output_file_path", type=str)
    parser.add_argument("--image_topic", type=str, default="/camera/color/image_raw")
    parser.add_argument("--pose_topic", type=str, default="cam_pose")
    parser.add_argument(
        "--child_frame_id", type=str, default="camera_color_optical_frame"
    )
    parser.add_argument("--parent_frame_id", type=str, default="world")

    args = parser.parse_args()
    assert not os.path.exists(
        args.output_file_path
    ), f"File {args.output_file_path} already exists."
    rospy.init_node("tf_listener")
    tf_listener = TfListener(
        args.output_file_path,
        rospy,
        args.image_topic,
        args.pose_topic,
        args.child_frame_id,
        args.parent_frame_id,
    )
    rospy.spin()
