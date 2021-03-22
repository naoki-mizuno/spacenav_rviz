#!/usr/bin/env python

import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import numpy as np


class TwistToTf(object):
    def __init__(self):
        self._timeout = rospy.get_param("~timeout", 0.1)
        self._frame_id = rospy.get_param("~frame_id", "world")
        self._child_frame_id = rospy.get_param("~child_frame_id", "camera")
        self._initial_pose_frame_id = rospy.get_param("~initial_pose_frame_id", None)
        # Scale for the raw joy pad input values
        scale = rospy.get_param("~initial_speed", [0.5, 0.2])
        if len(scale) == 2:
            scale = [scale[0]] * 3 + [scale[1]] * 3
        elif len(scale) != 6:
            fmt = "initial_speed must be length 2 or 6 but got {}"
            raise RuntimeError(fmt.format(len(scale)))
        self._speed_scale = np.array(scale)
        # Multiplier when button is pressed
        multiplier = rospy.get_param("~speed_multiplier", [2, 1.1])
        if len(multiplier) == 2:
            multiplier = [multiplier[0]] * 3 + [multiplier[1]] * 3
        elif len(multiplier) != 6:
            fmt = "speed_multiplier must be length 2 or 6 but got {}"
            raise RuntimeError(fmt.format(len(multiplier)))
        self._speed_multiplier = np.array(multiplier)
        self._ignore_roll = rospy.get_param("~ignore_roll", False)

        self._pose = self._get_initial_pose_()
        self._latest_stamp = rospy.Time.now()
        # True = pressed, False = not pressed
        self._prev_button_stats = np.array([False, False])

        self._tf_b = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/spacenav/joy", Joy, self.cb_joy, queue_size=1)

    def cb_joy(self, msg):
        """
        :type msg: sensor_msgs.msg.Joy

        axes: [
            x (front)
            y (left)
            z (up)
            R (around x)
            P (around y)
            Y (around z)
        ]
        buttons: [
            left button,
            right button,
        ]
        """
        curr = np.array(msg.buttons)
        prev = self._prev_button_stats
        pressed_buttons = (prev ^ curr) & curr
        self._prev_button_stats = curr

        # Slow down: left button pressed
        if pressed_buttons[0]:
            self._speed_scale /= self._speed_multiplier
        # Speed up: right button pressed
        elif pressed_buttons[1]:
            self._speed_scale *= self._speed_multiplier

        twist = Twist()
        twist.linear.x = msg.axes[0] * self._speed_scale[0]
        twist.linear.y = msg.axes[1] * self._speed_scale[1]
        twist.linear.z = msg.axes[2] * self._speed_scale[2]
        twist.angular.x = msg.axes[3] * self._speed_scale[3]
        twist.angular.y = msg.axes[4] * self._speed_scale[4]
        twist.angular.z = msg.axes[5] * self._speed_scale[5]
        self.cb_twist(twist)

    def cb_twist(self, msg):
        twist = msg
        current_time = rospy.Time.now()

        new_pose, stamp, _ = self.apply_twist(twist, current_time, self._ignore_roll)
        tform = TwistToTf.pose_to_tform(
            new_pose, stamp, self._frame_id, self._child_frame_id
        )
        self._tf_b.sendTransform(tform)

    def apply_twist(self, twist, current_time=None, ignore_roll=False):
        """
        :type twist: Twist
        :type current_time: rospy.Time
        """
        if current_time is None:
            current_time = rospy.Time.now()

        dt = (current_time - self._latest_stamp).to_sec()
        # Do not apply twist if called after a long time
        if abs(dt) > self._timeout:
            self._latest_stamp = current_time
            return self._pose, self._latest_stamp, 0
        self._latest_stamp = current_time

        # Add translation and rotation delta
        delta_trn = [
            dt * twist.linear.x,
            dt * twist.linear.y,
            dt * twist.linear.z,
        ]
        delta_q = tft.quaternion_from_euler(
            dt * twist.angular.x,
            dt * twist.angular.y,
            dt * twist.angular.z,
        )
        delta_mat = self._matrix_(delta_trn, delta_q)

        # Current pose
        curr_trn = [
            self._pose.position.x,
            self._pose.position.y,
            self._pose.position.z,
        ]
        curr_q = [
            self._pose.orientation.x,
            self._pose.orientation.y,
            self._pose.orientation.z,
            self._pose.orientation.w,
        ]
        curr_mat = self._matrix_(curr_trn, curr_q)

        # Apply delta to the current pose
        new_mat = np.dot(curr_mat, delta_mat)
        new_trn = tft.translation_from_matrix(new_mat)
        new_q = tft.quaternion_from_matrix(new_mat)
        if ignore_roll:
            yaw, pitch, roll = tft.euler_from_quaternion(new_q, axes="rzyx")
            new_q = tft.quaternion_from_euler(yaw, pitch, 0, axes="rzyx")
        self._pose.position.x = new_trn[0]
        self._pose.position.y = new_trn[1]
        self._pose.position.z = new_trn[2]
        self._pose.orientation.x = new_q[0]
        self._pose.orientation.y = new_q[1]
        self._pose.orientation.z = new_q[2]
        self._pose.orientation.w = new_q[3]

        return self._pose, current_time, dt

    def _get_initial_pose_(self):
        initial_pose = Pose()
        pos = initial_pose.position
        ori = initial_pose.orientation

        if self._initial_pose_frame_id is None:
            pos.x = 0
            pos.y = 0
            pos.z = 0
            ori.x = 0
            ori.y = 0
            ori.z = 0
            ori.w = 1
        else:
            try:
                tf_b = tf2_ros.Buffer()
                tf_l = tf2_ros.TransformListener(tf_b)

                tform = tf_b.lookup_transform(
                    self._frame_id,
                    self._initial_pose_frame_id,
                    rospy.Time(0),
                    rospy.Duration(0.1),
                )

                pos.x = tform.transform.translation.x
                pos.y = tform.transform.translation.y
                pos.z = tform.transform.translation.z
                ori.x = tform.transform.rotation.x
                ori.y = tform.transform.rotation.y
                ori.z = tform.transform.rotation.z
                ori.w = tform.transform.rotation.w
            except tf2_ros.LookupException as e:
                rospy.logwarn(e)

        pos.x = rospy.get_param("~initial/position/x", pos.x)
        pos.y = rospy.get_param("~initial/position/y", pos.y)
        pos.z = rospy.get_param("~initial/position/z", pos.z)
        ori.x = rospy.get_param("~initial/orientation/x", ori.x)
        ori.y = rospy.get_param("~initial/orientation/y", ori.y)
        ori.z = rospy.get_param("~initial/orientation/z", ori.z)
        ori.w = rospy.get_param("~initial/orientation/w", ori.w)

        return initial_pose

    @staticmethod
    def pose_to_tform(pose, stamp, frame_id, child_frame_id):
        """
        :type pose: Pose
        :type stamp: rospy.Duration
        :type frame_id: str
        :type child_frame_id: str
        """
        tform = TransformStamped()
        tform.header.frame_id = frame_id
        tform.header.stamp = stamp
        tform.child_frame_id = child_frame_id
        tform.transform.translation.x = pose.position.x
        tform.transform.translation.y = pose.position.y
        tform.transform.translation.z = pose.position.z
        tform.transform.rotation.x = pose.orientation.x
        tform.transform.rotation.y = pose.orientation.y
        tform.transform.rotation.z = pose.orientation.z
        tform.transform.rotation.w = pose.orientation.w
        return tform

    @staticmethod
    def _matrix_(translation, quaternion):
        mat = tft.quaternion_matrix(quaternion)
        mat[:3, -1] = translation
        return mat


def main():
    rospy.init_node("twist_to_tf")

    TwistToTf()

    rospy.spin()


if __name__ == "__main__":
    main()
