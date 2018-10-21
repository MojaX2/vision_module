#!/usr/bin/env python
# -*- coding: utf-8 -*-
#from __future__ import unicode_literals, print_function, division
from __future__ import print_function, division
import math
from collections import namedtuple

import numpy as np
import yaml

import ClientNetPython as cnp

import rospy
import tf2_ros
from tf import transformations
from geometry_msgs.msg import (
        Point,
        Quaternion,
        Pose,
        Transform,
        TransformStamped,
        )

from vision_order_sender import VisionOrderSender


Vector = namedtuple("Vector", "x y z")
Quat = namedtuple("Quat", "x y z w")

def to_tuple(point):
    try:
        return Quat(point.x, point.y, point.z, point.w)
    except (AttributeError):
        return Vector(point.x, point.y, point.z)


class CalibrateSlope(VisionOrderSender):
    def __init__(self, net):
        super(CalibrateSlope, self).__init__(net)

    def get_tf_frame(self, from_, to_):
        try:
            trans = self._tf_buffer.lookup_transform(from_, to_, rospy.Time(0), rospy.Duration(1))
        except:
            return None
        return trans.transform

    def calc_slope(self, ):
        """
        Vision が返してくる物体検出時の法線ベクトルの値からカメラの傾きを計算するメソッド
        
        | 計算した

        - base から見た時の camera の傾き
        - base から見た時の kinect の傾き
        - camera から見た時の kinect の傾き

        を計算して、それぞれの角度 (radian/degree) を yaml 形式で標準出力に流す
        """
        info = self.object_recognize()
        if isinstance(info, cnp.JoeInfo):
            # 設置位置を取得
            base2head_camera = self.get_tf_frame("base", "head_camera")
            # head_camera 自体の傾きを計算
            # head_camera は base を x: -90 y: -90 だけ回転した座標系 + tilt の傾き
            head_mat = transformations.quaternion_matrix(to_tuple(base2head_camera.rotation))
            rot_mat = transformations.quaternion_matrix(
                transformations.quaternion_from_euler(-math.pi/2, -math.pi/2, 0)
                )
            new_mat = np.dot(head_mat, rot_mat)
            new_quat = transformations.quaternion_from_matrix(new_mat)
            new_euler = transformations.euler_from_quaternion(new_quat)
            head_camera_quat = Quaternion(*new_quat)
            # base 座標系 に戻しているので, tilt の傾きは y 軸の回転量になる
            base2camera = math.degrees(new_euler[1])
            
            # x の値が小さいもののみ利用
            vectors = [to_tuple(data.normal) for data in info.data if abs(data.normal.x) < 0.1]
            # y, z の法線の傾きから、kinectの x 軸周りの設置角度を概算
            # カメラから見た法線の傾きを計算
            thetas = np.array([math.atan2(vec.y, vec.z) - math.pi / 2 for vec in vectors])
            # 平均値を求める
            base2kinect = math.degrees(thetas.mean())
            camera2kinect = base2kinect - base2camera

            # 情報を標準出力へ
            kinect_info = {
                    "base2camera": {"degree": base2camera, "radian": math.radians(base2camera)},
                    "base2kinect": {"degree": base2kinect, "radian": math.radians(base2kinect)},
                    "camera2kinect": {"degree": camera2kinect, "radian": math.radians(camera2kinect)},
                    }
            kinect_info_yaml = yaml.safe_dump(kinect_info, default_flow_style=False, allow_unicode=True)
            print(kinect_info_yaml)


if __name__ == '__main__':
    rospy.init_node("kinect_slope_calibration")

    net = cnp.ClientNet("kinect_slope_calibration")
    net = CalibrateSlope.set_required_info(net)
    #net.Connect("192.168.1.239", 2000)
    net.Connect("192.168.1.33", 2000)
    calib = CalibrateSlope(net)

    calib.calc_slope()
