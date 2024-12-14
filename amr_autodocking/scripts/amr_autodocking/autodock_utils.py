#!/usr/bin/env python3

# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import numpy as np
import tf

from typing import Tuple, List
from amr_autodocking.msg import AutoDockingFeedback
from tf import transformations as ts
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

Pose2D = Tuple[float, float, float]


class DockState:
    ERROR = AutoDockingFeedback.STATE_ERROR
    IDLE = AutoDockingFeedback.STATE_IDLE
    PREDOCK = AutoDockingFeedback.STATE_PREDOCK
    CORRECTION = AutoDockingFeedback.STATE_CORRECTION
    STEER_DOCK = AutoDockingFeedback.STATE_STEER_DOCK
    LAST_MILE = AutoDockingFeedback.STATE_LAST_MILE
    SLIDER_GO_OUT = AutoDockingFeedback.STATE_SLIDER_GO_OUT
    SLIDER_GO_IN = AutoDockingFeedback.STATE_SLIDER_GO_IN
    GO_OUT_DOCK = AutoDockingFeedback.STATE_GO_OUT_DOCK
    ACTIVATE_CHARGER = AutoDockingFeedback.STATE_ACTIVATE_CHARGER
    SUCCESS = AutoDockingFeedback.STATE_SUCCESS
    RETRY = AutoDockingFeedback.STATE_RETRY
    RETRY_HIGH_CURRENT = AutoDockingFeedback.STATE_RETRY_HIGH_CURRENT
    PAUSE = AutoDockingFeedback.STATE_PAUSE
    CANCEL = AutoDockingFeedback.STATE_CANCEL

    def to_string(input):
        _map = {
            DockState.ERROR: 'ERROR',
            DockState.IDLE: 'IDLE',
            DockState.PREDOCK: 'PREDOCK',
            DockState.CORRECTION: 'CORRECTION',
            DockState.STEER_DOCK: 'STEER_DOCK',
            DockState.LAST_MILE: 'LAST_MILE',
            DockState.SLIDER_GO_OUT: 'SLIDER_GO_OUT',
            DockState.SLIDER_GO_IN: 'SLIDER_GO_IN',
            DockState.GO_OUT_DOCK: 'GO_OUT_DOCK',
            DockState.ACTIVATE_CHARGER: 'ACTIVATE_CHARGER',
            DockState.SUCCESS: 'SUCCESS',
            DockState.RETRY: 'RETRY',
            DockState.RETRY_HIGH_CURRENT: 'RETRY_HIGH_CURRENT',
            DockState.PAUSE: 'PAUSE',
            DockState.CANCEL: 'CANCEL'
        }
        return _map[input]

    def to_percent(input):
        """
        Simple util to convert DockState to percent representation,
        use in publishing feedback in dock server
        """
        _map = {
            DockState.ERROR: 0.0,
            DockState.IDLE: 0.0,
            DockState.PREDOCK: 0.15,
            DockState.CORRECTION: 0.35,
            DockState.STEER_DOCK: 0.50,
            DockState.LAST_MILE: 0.8,
            DockState.SLIDER_GO_OUT: 0.9,
            DockState.SLIDER_GO_IN: 0.9,
            DockState.ACTIVATE_CHARGER: 0.9,
            DockState.GO_OUT_DOCK: 1.0,
            DockState.SUCCESS: 1.0,
            DockState.RETRY: 0.1,
            DockState.RETRY_HIGH_CURRENT: 0.5,
            DockState.PAUSE: 0.1,
            DockState.CANCEL: 0.0  # TODO
        }
        return _map[input]

class PID:
    def __init__(self, kp, ki, kd, out_min: float = 0.0, out_max: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.out_min = out_min
        self.out_max = out_max

    def update(self, setpoint, feedback_value, dt):
        dt_ms = dt*1000
        error = setpoint - feedback_value
        self.integral += error * dt_ms
        derivative = (error - self.prev_error) / dt_ms
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        # print('setpoint: ', setpoint)
        # print('feedback_value: ', feedback_value)
        # print('output: ', output)
        if (self.out_min != 0 or self.out_max != 0):
            return self.clamp(abs(output), self.out_min, self.out_max)
        return output

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def set_tunings(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def clamp(self, value, lower_limit, upper_limit):
        if value > upper_limit:
            return upper_limit
        elif value < lower_limit:
            return lower_limit
        return value

def get_mat_from_transfrom_msg(msg: TransformStamped) -> np.ndarray:
    """
    This will return a homogenous transformation of transform msg
    :param :    input transform msg
    :return :   homogenous transformation matrix
    """
    _rot = msg.transform.rotation
    _q = (_rot.x, _rot.y, _rot.z, _rot.w)

    _trans = msg.transform.translation
    _tr = (_trans.x, _trans.y, _trans.z)
    _tf_mat = ts.concatenate_matrices(
        ts.translation_matrix(_tr), ts.quaternion_matrix(_q))
    return _tf_mat


def get_mat_from_odom_msg(msg: Odometry) -> np.ndarray:
    """
    This will return a homogenous transformation of odom pose msg
    :param :    input odom msg
    :return :   homogenous transformation matrix
    """
    _rot = msg.pose.pose.orientation
    _q = (_rot.x, _rot.y, _rot.z, _rot.w)
    _trans = msg.pose.pose.position
    _tr = (_trans.x, _trans.y, _trans.z)
    _tf_mat = ts.concatenate_matrices(
        ts.translation_matrix(_tr), ts.quaternion_matrix(_q))
    return _tf_mat


def apply_2d_transform(mat: np.ndarray, transform: Pose2D) -> np.ndarray:
    """
    Apply a 2d transform to a homogenous matrix
    :param mat:         the input 4x4 homogenous matrix
    :param transform :  2d transform which to apply to the mat
    :return :           transformed homogenous transformation matrix
    """
    # req target transformation from base
    q = tf.transformations.quaternion_from_euler(0, 0, transform[2])
    tf_mat = ts.concatenate_matrices(
        ts.translation_matrix(
            (transform[0], transform[1], 0)), ts.quaternion_matrix(q))
    return np.matmul(mat, tf_mat)


def compute_tf_diff(current_tf: np.ndarray, ref_tf: np.ndarray) -> Pose2D:
    """
    Find the diff of two transformation matrix
    :param :  homogenous transformation of 2 matrices
    :return :  the 2d planer trans fo the 2 inputs; [x, y, yaw]
    """
    tf_diff = np.matmul(ts.inverse_matrix(current_tf), ref_tf)
    trans = ts.translation_from_matrix(tf_diff)
    euler = ts.euler_from_matrix(tf_diff)
    return trans[0], trans[1], euler[2]


def get_2d_pose(_tf: np.ndarray) -> Pose2D:
    """
    :param:   input homogenous matrix
    :return : 2dPose in x, y, yaw format
    """
    trans = ts.translation_from_matrix(_tf)
    euler = ts.euler_from_matrix(_tf)
    return trans[0], trans[1], euler[2]


def avg_2d_poses(poses: List[Pose2D]) -> Pose2D:
    """
    Provide the average of a list of 2D poses
    :param poses    : a list of 2d poses
    :return         : output avg Pose2D
    """
    _l = len(poses)
    if (_l == 0):
        return None
    _x = 0
    _y = 0
    _yaw = 0
    if poses[0][2] >= 0:
        sign = 1
    else:
        sign = -1

    for pose in poses:
        _x += pose[0]
        _y += pose[1]
        _yaw += sign*abs(pose[2])

    return _x/_l, _y/_l, _yaw/_l


def sat_proportional_filter(input: float, abs_min=0.0, abs_max=10.0, factor=1.0) -> float:
    """
    Simple saturated proportional filter
    :param input                : input value
    :param abs_min and abs_max  : upper and lower bound, abs value
    :param factor               : multiplier factor for the input value
    :return                     : output filtered value, within boundary
    """
    output = 0.0
    input *= factor
    if abs(input) < abs_min:
        if (input < 0):
            output = -abs_min
        else:
            output = abs_min
    elif abs(input) > abs_max:
        if (input > 0):
            output = abs_max
        else:
            output = -abs_max
    else:
        output = input
    return output


def bin_filter(input: float, abs_boundary: float) -> float:
    """
    Simple binary filter, will provide abs_ceiling as a binary output,
    according to the 'negativity' of the input value
    :param input        : input value
    :param abs_boundary : abs boundary value
    :return             : output binary value
    """
    output = abs(abs_boundary)
    if input < 0:
        output = -abs(abs_boundary)
    return output


def flip_yaw(yaw: float) -> float:
    """
    Flip yaw angle by 180 degree, input yaw range should be within
    [-pi, pi] radian. Else use set_angle() fn to fix the convention.
    Output will also be within the same range of [-pi, pi] radian.
    """
    if yaw >= 0:
        return yaw - math.pi
    else:
        return yaw + math.pi


def set_angle(angle: float) -> float:
    """
    Ensure the angle is within the range of [-pi, pi] radian convention
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def flip_base_frame(input: Pose2D):
    """
    Flip the current reference frame by 180 degree. As such, the negativity 
    of translation is flipped, and the yaw angle is located at the opposite 
    quadrant. Currently is used to flip from 'back dock' to 'front dock'
    """
    return -input[0], -input[1], flip_yaw(input[2])


def clamp(value, low, high):
    """
    Clamps a value within a specified range.
    * `value`: The value to be clamped.
    * `low`: The lower bound of the range.
    * `high`: The upper bound of the range.
    * @return: The clamped value within the specified range.
    """
    return max(low, min(value, high))