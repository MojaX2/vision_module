#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import unicode_literals, print_function
import math

import yaml

import rospy
from tf.transformations import (
        euler_from_matrix,
        euler_from_quaternion,
        quaternion_from_euler,
        quaternion_from_matrix,
        quaternion_matrix,
        translation_from_matrix,
        translation_matrix,
        )
import tf2_ros
from std_msgs.msg import (
        Header,
        )
from geometry_msgs.msg import (
        Point,
        Quaternion,
        Pose,
        Transform,
        TransformStamped,
        )


class KinectV2FramePublisher(object):
    def __init__(self, frame_id="head_camera", kinect_frame="kinect2_rgb_optical_frame", transform=None):
        """
        Parameters
        -----------
        frame_id: str, default: head_camera
            kinect_frame の親となるフレーム名

        kinect_frame: str, default: kinect2_rgb_optical_frame
            キネクトのtf frame名

        transform: geometry_msgs.Transform, default: None
            キネクトの設置誤差
        """
        # attributes
        default_offset = {
                "x": -0.06,
                "y": 0.06,
                "z": 0.06,
                }
        self.tilt = rospy.get_param("camera2kinect/degree")
        self.offset = rospy.get_param("offset", default_offset)
        # tf
        self._broadcaster = tf2_ros.TransformBroadcaster()

        # transform
        self._transform = TransformStamped()
        self._transform.header.frame_id = "head_camera"
        self._transform.child_frame_id = kinect_frame
        if isinstance(transform, Transform):
            self._transform.transform = transform
        elif self.tilt:
            self._set_transform()

    def _set_transform(self):
        """
        送信する tf frame を作成するメソッド
        """
        trans = [self.offset["x"], self.offset["y"], self.offset["z"]]
        # 傾きは x軸周りにの回転になっている。
        quat = quaternion_from_euler(math.radians(self.tilt-80), 0, math.radians(180))
        transform = Transform(
                translation=Point(*trans),
                rotation=Quaternion(*quat)
                )
        self._transform.transform = transform

    def load_setting_file(self, file_name):
        """
        設定ファイルからパラメータを読み込む関数

        Parameters
        ----------
        file_name: str
            設定ファイル(yaml)のパス
        """
        # fileの読み込み
        with open(file_name, "r") as f:
            setting = yaml.safe_load(f)

        # パラメータの解析
        # offsetの読み込み
        try:
            offset = setting["offset"]
            self.offset = Point(offset["x"], offset["y"], offset["z"])
        except (KeyError) as e:
            rospy.logwarn(e)
            rospy.logwarn("i use default offset: {self.offset}")
        self.tilt = setting["camera2kinect"]["degree"]
        self._set_transform()
        rospy.logdebug("successflly load kinect parameters")

    def broadcast(self):
        """
        tfのstaticな関係をpublishするメソッド
        """
        if self.tilt is None:
            rospy.logerr("tilt parameter")
            return
        rospy.loginfo("starting kinect tf frame.")
        rospy.loginfo("\n{self._transform}".format(**locals()))
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # header の更新
            self._transform.header.stamp = rospy.Time.now()
            # 送信
            self._broadcaster.sendTransform(self._transform)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("kinect_frame_publisher")

    pub = KinectV2FramePublisher()
    pub.broadcast()
