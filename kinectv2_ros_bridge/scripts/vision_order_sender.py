#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import unicode_literals, print_function
import math
import threading

import numpy as np
import cv2

import ClientNetPython as cnp
import rospy
from tf import transformations
import tf2_ros
from std_srvs.srv import (
    Empty, EmptyRequest, EmptyResponse,
)
from geometry_msgs.msg import (
    Point, Quaternion,
    TransformStamped,
)
from sensor_msgs.msg import (
    Image,
)
from cv_bridge.core import CvBridge

from kinectv2_ros_bridge.msg import (
    JoeData,
    FaceData,
)
from kinectv2_ros_bridge.srv import (
    GetVisionData, GetVisionDataRequest, GetVisionDataResponse,
    GetFaceData, GetFaceDataRequest, GetFaceDataResponse,
)


def point2list(msg):
    return [msg.x, msg.y, msg.z]


def quat2list(msg):
    return [msg.x, msg.y, msg.z, msg.w]


def pose2list(msg):
    try:
        pos = msg.position
    except:
        pos = msg.transform

    try:
        rot = msg.orientation
    except:
        rot = msg.rotation

    return [pos.x, pos.y, pos.z], [rot.x, rot.y, rot.z, rot.w]


class VisionOrderSender(object):
    def __init__(self, net, frame_id="head_kinectV2"):
        """
        Parameters
        ----------
        net: cnp.ClientNet
            クライアントネットのインスタンス。
            コネクト済みを入れる

        frame_id: str
            ベースとなる座標名
        """
        # attributes
        self._net = net
        self.frame_id = frame_id
        self._object_transforms = list()
        self._objects_image = Image()
        self._face_transforms = list()

        # topics
        self._pub_objects_image = rospy.Publisher("object_image", Image, queue_size=1, latch=True)
        # service
        self._object_srv = rospy.Service("ObjectRecognize", GetVisionData, self.object_recognize_cb)
        self._object_stop_srv = rospy.Service("ObjectTfStop", Empty, self.object_tf_stop_cb)
        self._face_srv = rospy.Service("FaceRecognize", GetFaceData, self.face_recognize_cb)
        self._face_stop_srv = rospy.Service("FaceTfStop", Empty, self.face_tf_stop_cb)

        # tf
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._tf_buffer = tf2_ros.BufferClient("/tf2_buffer_server", )
        if not self._tf_buffer.wait_for_server(rospy.Duration(3.)):
            rospy.logwarn("wait for tf server ({self._tf_buffer.client.action_client.ns})".format(**locals()))
            self._tf_buffer.wait_for_server()

    @classmethod
    def set_required_info(cls, net):
        """
        このクラスが必要な info をセットするメソッド
        """
        net.SetSubsInfo(cnp.JoeInfo())
        net.SetSubsInfo(cnp.FaceInfo())
        return net

    def object_recognize_cb(self, req):
        """
        物体認識の service を実行するためのコールバック関数

        Parameters
        ----------
        req: GetVisionDataRequest
        """
        # 前に保存していた変換を削除
        self._object_transforms = list()

        response = GetVisionDataResponse()
        if isinstance(req, GetVisionDataRequest):
            object_info = self.object_recognize()
            object_images = list()
            if object_info:
                # 物体認識が成功
                now = rospy.Time.now()
                i = 0
                for i, data in enumerate(object_info.data, 1):
                    d = JoeData()
                    # 座標
                    d.camera.header.frame_id = self.frame_id
                    d.camera.header.stamp = now
                    d.camera.point = Point(data.camera.x, data.camera.y, data.camera.z)
                    # 画像
                    image = decode_jpeg(data.image)
                    d.image = CvBridge().cv2_to_imgmsg(image)
                    # 特徴量
                    d.feature = list(data.features)

                    # tf の発行のために保存
                    trans = self.calc_object_transform(data, "obj{i}".format(i=i), "base")
                    if trans:
                        self._object_transforms.append(trans)
                    # 画像の保存
                    object_images.append(image)

                    response.datas.append(d)
                rospy.loginfo("{i} Objects recognize.".format(i=i))
                # 画像を結合 (横に結合)
                if object_images:
                    try:
                        hight = max([img.shape[0] for img in object_images])
                        width = sum([img.shape[1] for img in object_images])
                    except (ValueError):
                        hight = img.shape[0]
                        width = img.shape[1]
                    start_point = [img.shape[1] for img in object_images]
                    img = np.zeros((hight, width, 3), dtype=img.dtype) + 255
                    w_ = 0
                    for w, img_ in zip(start_point, object_images):
                        img[0:img_.shape[0], w_:w_ + img_.shape[1]] = img_
                        w_ += w
                    self._objects_image = CvBridge().cv2_to_imgmsg(img)
                    self._pub_objects_image.publish(self._objects_image)
                return response
            else:
                # 物体認識が失敗していた場合は空のレスポンスを返す
                return response
        else:
            return response

    def object_tf_stop_cb(self, req):
        """
        物体の tf 発行を止めるサービスのコールバック関数

        Parameters
        -----------
        req: EmptyRequest
        """
        if isinstance(req, EmptyRequest):
            self._object_transforms = list()
            return EmptyResponse()

    def object_recognize(self, timeout=3):
        """
        物体検出/認識を行うメソッド

        Parameters
        ----------
        timeout: int
            認識までのタイムアウト

        Returns
        ---------
        info: cnp.JoeInfo
            vision から受け取った Joe Info を返す
        """
        # vision order を送信
        order = cnp.VisionOrder()
        order.kind = cnp.VO_OBJ_RECOG
        self._net.Send(order)

        info = cnp.JoeInfo()
        event = self._net.OpenEvent(info)
        # 情報が来れば情報をリターン
        if event.wait(timeout):
            self._net.Get(info)
            return info
        # timeout
        else:
            rospy.logwarn("Vision info could not get")
            return

    def calc_object_transform(self, point, child_frame_id, frame_id=None):
        """
        得られた camera 座標系の物体位置を baxter の base 基準座標に回転させた
        変換を計算するメソッド

        Parameters
        ----------
        point: cnp.JoeData
            物体座標の型

        child_frame_id: str
            発行する frame 名

        frame_id: str, default: None
            起点となる座標系

        Returns
        -------
        trans: TransformStamped
            計算した変換
        """
        if frame_id is None:
            frame_id = self.frame_id

        try:
            base2kinect = self._tf_buffer.lookup_transform("base", "head_kinectV2", rospy.Time(0), rospy.Duration(3.))
        except (Exception) as e:
            rospy.logerr(e)
            return

        # 発行する座標系の基本設定
        trans = TransformStamped()
        trans.header.frame_id = frame_id
        trans.child_frame_id = child_frame_id
        # 座標の計算
        # baseからkinectへの変換
        base_p = point2list(base2kinect.transform.translation)
        base_r = quat2list(base2kinect.transform.rotation)
        base_point = transformations.translation_matrix(base_p)
        base_rot = transformations.quaternion_matrix(base_r)
        base_mat = np.dot(base_point, base_rot)
        # camera_point = [point.camera.z, point.camera.x, point.camera.y]
        # kanectからobjectへの変換
        camera_p = point2list(point.camera)
        camera_r = base_r[:]
        camera_r[3] *= -1
        camera_point = transformations.translation_matrix(camera_p)
        camera_rot = transformations.quaternion_matrix(camera_r)
        camera_mat = np.dot(camera_point, camera_rot)

        # 変換行列
        mat = np.dot(base_mat, camera_mat)
        translation = transformations.translation_from_matrix(mat)
        rotation = transformations.quaternion_from_matrix(mat)

        yow = math.atan2(translation[1], translation[0])
        rotation = transformations.quaternion_from_euler(0, 0, yow)

        trans.transform.translation = Point(*translation)
        # 姿勢はそのまま
        trans.transform.rotation = Quaternion(*rotation)

        return trans

    def generate_object_broadcaster(self, point, child_frame_id, publish_time=10, frame_id=None):
        """
        一定秒数 tf を発行するスレッドオブジェクトを生成/実行するメソッド

        Parameters
        ----------
        point: cnp.JoeData
            物体座標の型

        child_frame_id: str
            発行する frame 名

        publish_time: int, default: 10
            tf を発行する時間
            負値が与えられると無限時間スレッドが動作

        frame_id: str, default: None
            起点となる座標系

        Returns
        -------
        thread: threading.Thread
            スレッド

        terminate: function
            スレッドを終了するための関数

        Usage
        -----
        >> # object recognize
        >> info = vision_order_sernder.object_recognize()
        >> data = info.data[0]
        >> thread, terminate = vision_order_sernder.generate_object_broadcaster(data, "obj1", -1)
        >> # if you want to stop broadcasting,
        >> terminate()

            の実行でスレッドが終了する
        """
        is_runnning = [True]

        def broadcast():
            trans = self.calc_object_transform(point, child_frame_id, frame_id)
            rate = rospy.Rate(10)
            if publish_time < 0:
                check = lambda: True
            else:
                stop_time = rospy.Time.now() + rospy.Duration(publish_time)
                check = lambda: rospy.Time.now() < stop_time

            while check() and is_runnning[0] and not rospy.is_shutdown():
                trans.header.stamp = rospy.Time.now()
                self._tf_broadcaster.sendTransform(trans)
                rate.sleep()

        def terminate():
            is_runnning[0] = False

        thread = threading.Thread(target=broadcast)
        thread.setDaemon(True)
        thread.start()
        return thread, terminate

    def face_recognize_cb(self, req):
        """
        顔認識用の service を実行するコールバック関数

        Parameters
        ------------
        req: EmptyRequest
        """
        # 今までの変換を削除
        self._face_transforms = list()

        response = GetFaceDataResponse()
        if isinstance(req, GetFaceDataRequest):
            face = self.face_recognize()
            # 認識できた顔の分だけ tf を発行
            for i, f in enumerate(face.face, 1):
                trans = self.calc_face_transform(f, "face{i}".format(i=i))
                self._face_transforms.append(trans)

                # TransformStamped to PoseStamped
                data = FaceData()
                data.data = trans
                response.faces.append(data)
        return response

    def face_tf_stop_cb(self, req):
        """
        顔の tf 発行を止める service
        """
        self._face_transforms = list()
        return EmptyResponse()

    def face_recognize(self, timeout=3.):
        """
        顔認識結果を受け取るメソッド

        Parameters
        ----------
        timeout: float
            認識までのタイムアウト [sec]

        Returns
        -------
        face: cnp.FaceInfo
            顔情報のクラス
        """
        # order の送信
        order = cnp.VisionOrder()
        order.kind = cnp.VO_FACE_RECOG
        self._net.Send(order)

        # 認識結果の待機
        info = cnp.FaceInfo()
        event = self._net.OpenEvent(info)
        # 情報が来れば情報をリターン
        if event.wait(timeout):
            self._net.Get(info)
            return info
        # timeout
        else:
            rospy.logwarn("Vision info could not get")
            return

    def calc_face_transform(self, face, child_frame_id, frame_id=None):
        """
        顔の座標を視線推定を交えて計算するメソッド

        Parameters
        ----------
        face: cnp.FaceData
            物体座標の型

        child_frame_id: str
            発行する frame 名

        frame_id: str, default: None
            起点となる座標系

        Returns
        -------
        trans: TransformStamped
            変換後の姿勢
        """
        if frame_id is None:
            frame_id = self.frame_id

        # 発行する座標系の基本設定
        trans = TransformStamped()
        trans.header.frame_id = frame_id
        trans.child_frame_id = child_frame_id
        # 座標の計算
        trans.transform.translation = Point(face.camera.x, face.camera.y, face.camera.z)
        # 顔の向きが 0, 0 のときにカメラ座標系の x, y 平面に垂直になるように回転
        base_mat = transformations.quaternion_matrix([0., 0., 0., 1.])
        rot_mat = transformations.quaternion_matrix(
            transformations.quaternion_from_euler(
                0, math.pi / 2, math.pi / 2)
        )
        base_mat = np.dot(base_mat, rot_mat)
        # 顔の向きを考慮してそれぞれ回転
        rot_mat = transformations.quaternion_matrix(
            transformations.quaternion_from_euler(
                0, -math.radians(face.gazeDirUD), math.radians(face.gazeDirLR)
            )
        )
        quat = transformations.quaternion_from_matrix(base_mat.dot(rot_mat))
        trans.transform.rotation = Quaternion(*quat)
        return trans

    def generate_face_broadcaster(self, face, child_frame_id, publish_time=10, frame_id=None):
        """
    print (host)
        一定秒数 tf を発行するスレッドオブジェクトを生成/実行するメソッド

        Parameters
        ----------
        face: cnp.FaceData
            物体座標の型

        child_frame_id: str
            発行する frame 名

        publish_time: int, default: 10
            tf を発行する時間.
            負値が与えられると無限時間スレッドが動作.
            止めるときは返り値の `terminate()` を実行する必要がある.

        frame_id: str, default: None
            起点となる座標系

        Returns
        -------
        thread: threading.Thread
            スレッド

        terminate: function
            スレッドを終了するための関数

        Usage
        -----
        >> # object recognize
        >> info = vision_order_sernder.object_recognize()
        >> data = info.data[0]
        >> thread, terminate = vision_order_sernder.generate_face_broadcaster(data, "obj1", -1)
        >> # if you want to stop broadcasting,
        >> terminate()

            の実行でスレッドが終了する
        """
        is_runnning = [True]

        def broadcast():
            trans = self.calc_face_transform(face, child_frame_id, frame_id)

            rate = rospy.Rate(10)
            if publish_time < 0:
                check = lambda: True
            else:
                stop_time = rospy.Time.now() + rospy.Duration(publish_time)
                check = lambda: rospy.Time.now() < stop_time

            while check() and is_runnning[0] and not rospy.is_shutdown():
                trans.header.stamp = rospy.Time.now()
                self._tf_broadcaster.sendTransform(trans)
                rate.sleep()

        def terminate():
            is_runnning[0] = False

        thread = threading.Thread(target=broadcast)
        thread.setDaemon(True)
        thread.start()
        return thread, terminate

    def spin(self, rate=20):
        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            # object tf frame
            for trans in self._object_transforms:
                trans.header.stamp = rospy.Time.now()
                self._tf_broadcaster.sendTransform(trans)

            # face tf frame
            for trans in self._face_transforms:
                trans.header.stamp = rospy.Time.now()
                self._tf_broadcaster.sendTransform(trans)

            rate.sleep()


def decode_jpeg(img):
    """
    クライアントネットから来たイメージを ndarrayに変換するためのメソッド

    Parameters
    ----------
    img: JoeData.image
        クライアントから来たイメージそのまま

    Returns
    -------
    np_img: numpy.ndarray
        array に変換した jpg イメージ
    """
    # イメージを uchar で arrayに変換
    np_img = np.fromstring(img, dtype="B")
    # jpg で decode
    return cv2.imdecode(np_img, cv2.CV_LOAD_IMAGE_COLOR)


def show_image(img):
    """
    イメージを描画する関数

    Parameters
    ----------
    img: numpy.ndarray
        描画したいデータ
    """
    print("Exit: input any key")
    key = -1
    while key == -1:
        cv2.imshow("image", img)
        key = cv2.waitKey(100)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import sys

    # ros
    rospy.init_node("vision_ros_bridge")
    # vision host/port
    try:
        host = rospy.get_param("host")
    except (KeyError) as e:
        rospy.logerr("parameter of vision pc's host is not found from rosparam.")
        rospy.signal_shutdown("parameter of vision pc' host from rosparam.")
        sys.exit(1)
    port = rospy.get_param("port", 2000)
    # クライアントネットとの接続準備
    net = cnp.ClientNet("vision_ros_bridge")
    net.SetSubsInfo(cnp.JoeInfo())
    net.SetSubsInfo(cnp.FaceData())
    net.SetSubsInfo(cnp.FaceInfo())
    net.Connect(host, port)
    rospy.loginfo("connected vision pc.")

    sender = VisionOrderSender(net)

    rospy.loginfo("initialized node")
    sender.spin()
