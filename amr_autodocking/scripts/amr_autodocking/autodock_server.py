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

import rospy
import tf2_ros
import math
import numpy as np
import actionlib
import amr_autodocking.autodock_utils as utils
import dynamic_reconfigure.client as client
from typing import List

from amr_autodocking.autodock_utils import DockState
from amr_autodocking.msg import AutoDockingFeedback
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int16, Empty
from amr_autodocking.msg import AutoDockingAction
from amr_autodocking.msg import AutoDockingGoal, AutoDockingResult
from amr_msgs.msg import CheckerSensorStateStamped, DockLimit, DockMode, DockParam
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


class AutodockConfig:
    # [General configure]
    tf_expiry = 1.0  # sec
    dock_timeout = 300  # sec
    time_for_waiting_dock_frame = 300  # sec
    controller_frequency = 20.0  # hz
    # control_duration = 50.0
    back_laser_offset = (
        0.52322  # Distance from base_link to back_laser_link when slider at original position
    )
    front_laser_offset = 0.14802 + 0.1025 / 2
    steer_distance_threshold = 0.97
    odom_topic = "odom"

    # [Frames]
    base_link = "base_link"
    charger_link = "charger_frame"
    first_frame = "first_frame"
    last_frame = "last_frame"
    parallel_frame = "parallel_frame"
    dropoff_frame = "dropoff_frame"

    # [Goal threshold]
    stop_yaw_diff = 0.05  # radian
    stop_trans_diff = 0.01  # meters
    y_tolerance_pid = 0.02
    yaw_predock_tolerance = 0.05
    max_parallel_offset = 0.03  # m, will move to parallel.c if exceeded
    predock_tf_samples = 10     # tf samples to avg, parallel.c validation

    # [Velocity Informations]
    linear_vel_range = [-0.2, 0.2]
    angular_vel_range = [-0.3, 0.3]
    max_angular_vel = 0.3  # rad/s
    max_angular_accel = 3.2
    max_angular_deccel = 0.3
    min_angular_vel = 0.05
    rotate_to_heading_angular_vel = 0.5
    angle_threshold = 0.35
    max_linear_vel = 0.2
    max_linear_vel_predock = 0.15
    min_linear_vel = 0.025
    max_x_pid_steer = 0.02
    min_x_pid_steer = 0.02
    max_x_pid_lastmile = 0.04
    min_x_pid_lastmile = 0.02
    max_x_lastmile = 0.05  # m/s, for lastmile

    k_p = 0.8
    k_i = 0.0
    k_d = 0.01
    k_p_steer = 2.5
    k_i_steer = 0.0
    k_d_steer = 0.01

    retry_count = 5
    debug_mode = True


class AutoDockServer:
    def __init__(self, config: AutodockConfig, run_server: bool):

        self.cfg = config
        self.run_server = run_server

        # param check
        assert (
            len(self.cfg.linear_vel_range) == 2 and len(self.cfg.linear_vel_range) == 2
        ), "linear and angular vel range should have size of 2"
        assert (
            self.cfg.linear_vel_range[0] < self.cfg.linear_vel_range[1]
        ), "linear vel min should be larger than max"
        assert (
            self.cfg.angular_vel_range[0] < self.cfg.angular_vel_range[1]
        ), "linear vel min should be larger than max"

        # create_subscriber to tf broadcaster
        self.__tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.__tf_listener = tf2_ros.TransformListener(self.__tfBuffer)

        # Initialize variables:
        self.left_range_ = 0.0
        self.right_range_ = 0.0
        self.is_pause_ = False
        self.high_motor_pickup_current_ = False
        self.high_motor_drop_current_ = False
        self.obstacle_detected_ = False
        self.pause_flag_ = False
        self.is_waiting_dock_ = False
        self.slider_sensor_state_ = [0, 0]
        self.cart_sensor_state_ = (0, 0)
        self.tag_frame_ = ""
        self.current_speed_ = Twist()
        self.dock_state_ = DockState.IDLE
        self.start_time_ = rospy.Time.now()
        self.dock_timeout_ = self.cfg.dock_timeout
        self.rate_ = rospy.Rate(self.cfg.controller_frequency)

        # PID
        self.kp = 2.5
        self.kd = 0.005
        self.last_error = 0.0

        # Dynamic reconfigure for line extraction
        self.line_extraction_client = client.Client("/back_line_extractor")
        self.default_le_params = {"max_range": 2.0, "max_line_gap": 0.01}
        self.pickup_le_params = {"max_range": 0.5, "max_line_gap": 0.02}
        self.dropoff_le_params = {"max_range": 1.0, "max_line_gap": 0.02}

        # Dynamic reconfigure for polygon scanner
        self.polygon_client = client.Client("/back_scanner_filter/back_scanner_filter")
        self.default_polygon_params = {
            "polygon": [[-0.05, -0.75], [-0.05, 0.75], [2.0, 0.75], [2.0, -0.75]]
        }
        self.pickup_polygon_params = {
            "polygon": [[-0.05, -0.75], [-0.05, 0.75], [2.0, 0.75], [2.0, -0.75]]
        }
        self.dropoff_polygon_params = {
            "polygon": [[-0.05, -0.75], [-0.05, 0.75], [2.0, 0.75], [2.0, -0.75]]
        }

        # debug timer for state machine marker
        if self.cfg.debug_mode:
            self.pub_marker = rospy.Publisher("/sm_maker", Marker, queue_size=1)
            self.__timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        # Apriltag continuous detection service
        self.back_apriltag_detector_cli_ = rospy.ServiceProxy(
            "/back_camera/apriltag_ros/enable_detector", SetBool
        )
        rospy.loginfo("Connecting to /back_camera/apriltag_ros/enable_detector service...")
        self.back_apriltag_detector_cli_.wait_for_service()
        rospy.loginfo("Connected to /back_camera/apriltag_ros/enable_detector service.")

        self.front_apriltag_detector_cli_ = rospy.ServiceProxy(
            "/front_camera/apriltag_ros/enable_detector", SetBool
        )
        rospy.loginfo("Connecting to /front_camera/apriltag_ros/enable_detector service...")
        self.front_apriltag_detector_cli_.wait_for_service()
        rospy.loginfo("Connected to /front_camera/apriltag_ros/enable_detector service.")

        # Create line extraction service
        self.front_line_extraction_client = rospy.ServiceProxy(
            "/front_line_extractor/enable_detector", SetBool
        )
        self.back_line_extraction_client = rospy.ServiceProxy(
            "/back_line_extractor/enable_detector", SetBool
        )
        rospy.loginfo("Connecting to front & back line extraction detector service...")
        self.front_line_extraction_client.wait_for_service()
        self.back_line_extraction_client.wait_for_service()
        rospy.loginfo("Connected to front & back line extraction detector service.")

        # Publishers
        self.cmd_vel_pub_ = rospy.Publisher(
            "/amr/mobile_base_controller/cmd_vel", Twist, queue_size=5
        )
        self.cmd_brake_pub_ = rospy.Publisher("cmd_brake", Bool, queue_size=5)
        self.cmd_slider_pub_ = rospy.Publisher("cmd_slider", Int16, queue_size=5)
        self.error_pub_ = rospy.Publisher("error_name", Int16, queue_size=5)
        self.turn_off_front_safety_pub_ = rospy.Publisher(
            "turn_off_front_safety", Bool, queue_size=5
        )
        self.turn_off_back_safety_pub_ = rospy.Publisher("turn_off_back_safety", Bool, queue_size=5)
        self.turn_off_ultrasonic_safety_pub_ = rospy.Publisher(
            "turn_off_ultrasonic_safety", Bool, queue_size=5
        )
        self.turn_off_front_depth_scan_safety_pub_ = rospy.Publisher(
            "turn_off_front_depth_autodock", Bool, queue_size=5
        )
        self.wait_dock_frame_pub_ = rospy.Publisher("wait_dock_frame", Bool, queue_size=5)

        # Subscribers
        rospy.Subscriber("/amr/odometry/filtered", Odometry, self.odom_callback)
        rospy.Subscriber("PAUSE_AMR", Bool, self.pause_callback)
        rospy.Subscriber("pickup_current_state", Bool, self.pickup_current_state_callback)
        rospy.Subscriber("drop_current_state", Bool, self.dropoff_current_state_callback)
        rospy.Subscriber("cart_sensor_state", CheckerSensorStateStamped, self.cart_sensor_state_callback)
        rospy.Subscriber("slider_sensor_state", CheckerSensorStateStamped, self.slider_sensor_state_callback)
        rospy.Subscriber("status_protected_field", Bool, self.protected_field_callback)
        rospy.Subscriber("back_scan_rep177", LaserScan, self.laser_scan_callback)

        # Autodock action
        if run_server:
            self.autodock_action = actionlib.SimpleActionServer(
                "autodock_action",
                AutoDockingAction,
                execute_cb=self.autodock_callback,
                auto_start=False,
            )
            self.autodock_action.start()
            self.feedback_msg = AutoDockingFeedback()

    def reset(self):
        self.publish_velocity()
        self.enable_line_detector("front", False)
        self.enable_line_detector("back", False)
        self.turn_off_back_scan_safety(False)
        self.turn_off_front_scan_safety(False)
        self.turn_off_ultrasonic_safety(False)
        self.turn_off_front_depth_safety(False)

    def waiting_dock_frame(self, rotate_to_dock):
        start_time = rospy.Time.now()
        isPaused = False
        remainTime = self.cfg.time_for_waiting_dock_frame

        while not rospy.is_shutdown():
            if self.check_cancel():
                return False

            elif self.do_pause():
                if not isPaused:
                    remainTime -= (rospy.Time.now() - start_time).to_sec()
                    start_time = rospy.Time.now()
                    isPaused = True
                pass

            # if self.do_pause():
            #     self.publish_velocity()
            elif (rospy.Time.now() - start_time).to_sec() <= remainTime:
                isPaused = False
                cart_tf = self.get_tf(self.cfg.first_frame, print_out=True)
                dock_tf = self.get_tf(self.cfg.parallel_frame, print_out=True)

                if (cart_tf is not None or dock_tf is None) and not self.is_waiting_dock_:
                    rospy.logwarn(
                        "DROPOFF: Detect cart frame don't move out, will rotate and wait for hand control"
                    )
                    self.rotate_with_odom(-rotate_to_dock * math.pi / 180)
                    self.wait_dock_frame_pub_.publish(True)
                    self.is_waiting_dock_ = True

                elif self.is_waiting_dock_:
                    try:
                        rospy.wait_for_message("/amr/hand_dock_trigger", Empty, timeout=1.0)
                        rospy.logwarn(
                            "DROPOFF: Receive trigger continue DROPOFF, will continue docking"
                        )
                        self.wait_dock_frame_pub_.publish(False)
                        self.rotate_with_odom(rotate_to_dock * math.pi / 180)
                        self.start_time_ = rospy.Time.now()
                        return True
                    except:
                        pass

                else:
                    return True
            else:
                rospy.logerr("DROPOFF: timeout waiting for move cart reaches!!!!!")
                self.wait_dock_frame_pub_.publish(False)
                self.is_waiting_dock_ = False
                return False

            self.rate_.sleep()

    def distance2D(self, x, y):
        return math.sqrt(pow(x, 2) + pow(y, 2))

    def update_line_extraction_param(self, signal):
        """
        `0`: Default params | `1`: Pickup params | `2`: Dropoff params
        """
        if signal == 0:
            self.line_extraction_client.update_configuration(self.default_le_params)

        elif signal == 1:
            self.line_extraction_client.update_configuration(self.pickup_le_params)

        elif signal == 2:
            self.line_extraction_client.update_configuration(self.dropoff_le_params)

    def update_polygon_param(self, signal):
        """
        `0`: Default params | `1`: Pickup params | `2`: Dropoff params
        """
        if signal == 0:
            self.polygon_client.update_configuration(self.default_polygon_params)

        elif signal == 1:
            self.polygon_client.update_configuration(self.pickup_polygon_params)

        elif signal == 2:
            self.polygon_client.update_configuration(self.dropoff_polygon_params)

    def check_dock_frame(self, laser_frame, tag_frame):
        laser_tf = self.get_tf(laser_frame)
        if laser_tf is None:
            return False
        
        tag_tf = self.get_tf(tag_frame)
        if tag_tf is not None:
            x, y, yaw    = utils.get_2d_pose(laser_tf)
            x1, y1, yaw1 = utils.get_2d_pose(tag_tf)

            return (abs(x-x1) <= 0.05 and abs(y-y1)<= 0.03)
        return True

    def pid_controller(self, dis_y):
        error = dis_y - self.last_error
        angle = self.kp * dis_y + self.kd * error
        self.last_error = dis_y

        return angle

    def correct_robot(
        self, offset, front_dock: bool = False, rotate_angle=30, rotate_orientation=0
    ) -> bool:
        """
        Correcting robot respective to dock frame
        `front_dock`: dock in frontoff robot or backward
        `rotate_angle` (degrees)

        `rotate_orientation = 1`: Counter clockwise
        `rotate_orientation = 2`: Clockwise
        """
        assert type(rotate_orientation) == int, "rotate_orientation is not int type!"

        dir = 1
        ori = 1
        if rotate_orientation == 1:
            if (front_dock and offset > 0) or (not front_dock and offset < 0):
                dir = -1
        elif rotate_orientation == 2:
            ori = -1
            if (front_dock and offset < 0) or (not front_dock and offset > 0):
                dir = -1
        elif (front_dock and offset > 0) or (not front_dock and offset < 0):
            ori = -1

        rot_rad = math.radians(rotate_angle)
        dis_move = abs(offset / math.sin(rot_rad))
        dis_back = abs(offset / math.tan(rot_rad))

        self.set_state(
            DockState.CORRECTION, f"Correcting robot {rotate_angle} degrees with {offset:.2f}m!"
        )

        return (
            self.rotate_with_odom(ori * rot_rad)
            and self.move_with_odom(
                self.cfg.min_linear_vel, self.cfg.max_linear_vel_predock, dir * dis_move
            )
            and self.rotate_with_odom(-ori * rot_rad)
            and self.move_with_odom(
                self.cfg.min_linear_vel, self.cfg.max_linear_vel_predock, -dir * dis_back
            )
        )

    def auto_correction(
        self, x_distance, y_distance, distance_threshold, laser_offset, is_dock_limit: Bool = False
    ):
        if is_dock_limit:
            return False
        safety_distance = abs(x_distance) - (distance_threshold + laser_offset)
        if laser_offset == self.cfg.front_laser_offset:
            dir = 1
        elif laser_offset == self.cfg.back_laser_offset:
            dir = -1

        rot_angle = 0.0
        if safety_distance < 0:
            return False
        else:
            rot_angle_min = -math.atan(y_distance / safety_distance)
            sign = 1 if (rot_angle_min > 0) else -1
            if (abs(rot_angle_min) + (5 * math.pi / 180)) >= math.pi / 2:
                if abs(rot_angle_min) < math.pi / 2:
                    rot_angle = rot_angle_min
                else:
                    rot_angle = sign * (math.pi / 2)
                    # return False
            else:
                rot_angle = rot_angle_min + sign * (5 * math.pi / 180)

            dis_move = dir * abs(y_distance / math.sin(rot_angle))

            if self.cfg.debug_mode:
                print(f"Rotate robot with {rot_angle}rad and move {dis_move}m.")

            self.set_state(
                DockState.CORRECTION,
                f"Correcting robot with angle flex - offset: {y_distance:.2f}m!",
            )

            return (
                self.rotate_with_odom(rot_angle)
                and self.move_with_odom(
                    self.cfg.min_linear_vel, self.cfg.max_linear_vel_predock, dis_move
                )
                and self.rotate_with_odom(-rot_angle)
            )
        

    def enable_apriltag_detector(self, signal):
        """
        `signal = False`: Disable | `signal = True`: Enable
        """
        try:
            self.back_apriltag_detector_cli_.call(signal)
            self.front_apriltag_detector_cli_.call(not signal)
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return False

    def enable_line_detector(self, laser_name: str, signal: bool):
        """
        `laser_name`: "front" or "back"
        """
        try:
            if laser_name == "front":
                result = self.front_line_extraction_client.call(signal)
            elif laser_name == "back":
                result = self.back_line_extraction_client.call(signal)

            if result.success:
                rospy.logwarn(result.message)
                rospy.sleep(1.0)
                return True
            else:
                return False

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def reset_all(self):
        """
        Reset before starting autodock
        """
        self.dock_state_ = DockState.IDLE
        self.is_pause_ = False
        self.pause_flag_ = False
        self.is_waiting_dock_ = False
        self.high_motor_pickup_current_ = False
        self.high_motor_drop_current_ = False
        self.update_polygon_param(0)
        self.brake(False)

    def reset_high_current(self):
        self.high_motor_pickup_current_ = False
        self.high_motor_drop_current_ = False

    def odom_callback(self, msg:Odometry):
        self.current_speed_ = msg.twist.twist

    def laser_scan_callback(self, msg: LaserScan):
        self.left_range_ = min(msg.ranges[80:100])
        self.right_range_ = min(msg.ranges[1171:1191])

    def slider_sensor_state_callback(self, msg: CheckerSensorStateStamped):
        self.slider_sensor_state_ = msg.sensor_state.data

    def protected_field_callback(self, msg: Bool):
        self.obstacle_detected_ = msg.data

    def pause_callback(self, msg: Bool):
        self.is_pause_ = msg.data

    def cart_sensor_state_callback(self, msg: CheckerSensorStateStamped):
        self.cart_sensor_state_ = msg.sensor_state.data

    def turn_off_back_scan_safety(self, signal):
        self.turn_off_back_safety_pub_.publish(signal)

    def turn_off_front_scan_safety(self, signal):
        self.turn_off_front_safety_pub_.publish(signal)

    def turn_off_ultrasonic_safety(self, signal):
        self.turn_off_ultrasonic_safety_pub_.publish(signal)

    def turn_off_front_depth_safety(self, signal):
        self.turn_off_front_depth_scan_safety_pub_.publish(signal)

    def brake(self, signal):
        self.cmd_brake_pub_.publish(signal)

    def cmd_slider(self, signal):
        """
        `signal = 1`: Slider go out
        `signal = 2`: Slider go in
        """
        msg = "out" if signal == 1 else "in"
        rospy.loginfo(f"Publishing slider motor go {msg}!")

        self.cmd_slider_pub_.publish(signal)

    def pickup_current_state_callback(self, msg: Bool):
        self.high_motor_pickup_current_ = msg.data

    def dropoff_current_state_callback(self, msg: Bool):
        self.high_motor_drop_current_ = msg.data

    def start(
        self,
        dock_name: str,
        dock_pose,
        mode: int,
        dock_limit: DockLimit,
        tag_names: List[str],
        go_in_dock: List[DockParam],
        go_out_dock: List[DockParam],
    ) -> bool:
        """
        Virtual function. This function will be triggered when autodock request
        is requested
        :return : if action succeeded
        """
        rospy.logwarn(
            "Server implementation has not been specified. " "Do overload the start() function"
        )
        return False

    def set_state(self, state: DockState, printout=""):
        """
        set state of the auto dock server
        :param state:       Current utils.DockState
        :param printout:    Verbose description of the state
        """
        state_str = DockState.to_string(state)
        self.dock_state_ = state

        if state == DockState.ERROR:
            rospy.logerr(f"State: [{state_str}] | {printout}!")

        else:
            rospy.loginfo(f"State: [{state_str}] | {printout}")

        if self.run_server:
            self.feedback_msg.state = state
            self.feedback_msg.progress = DockState.to_percent(state)
            self.feedback_msg.status = f"{state_str} | {printout}"
            self.autodock_action.publish_feedback(self.feedback_msg)

    def print_success(self, printout=""):
        rospy.loginfo(f"State: [{DockState.to_string(self.dock_state_)}] | {printout}")

    def print_error(self, printout=""):
        rospy.logerr(f"State: [{DockState.to_string(self.dock_state_)}] | {printout}")

    def retry(self, dock_tf_name) -> bool:
        """
        Check if not dectect the dock frame, will be retry
        """
        self.set_state(DockState.RETRY, "Retrying auto docking...!")

        counter = 1
        while not rospy.is_shutdown():
            if self.do_pause():
                pass
            else:
                dock_tf = self.get_tf(dock_tf_name)
                if dock_tf is None:
                    if counter > self.cfg.retry_count:
                        rospy.logerr("Not dectect the dock frame after execute retry!")
                        return False
                    rospy.logwarn(f"Retrying again: {counter}/{self.cfg.retry_count}!")
                    counter += 1
                else:
                    return True
            rospy.Rate(5).sleep()

    def check_cancel(self) -> bool:
        if self.run_server and self.autodock_action.is_preempt_requested():
            self.set_state(self.dock_state_, "Cancel Requested!")
            return True

        # check if dock_timeout reaches
        if not self.pause_flag_ and not self.is_waiting_dock_:
            if (rospy.Time.now() - self.start_time_).secs > self.dock_timeout_:
                rospy.logwarn("Timeout reaches!")
                self.set_state(self.dock_state_, "Reach Timeout")
                return True
        return False

    def do_pause(self):
        if self.is_pause_ or self.obstacle_detected_:
            if not self.pause_flag_:
                self.set_state(self.dock_state_, "Pause Requested!")
                self.pause_flag_ = True
                self.dock_timeout_ -= (rospy.Time.now() - self.start_time_).secs
                rospy.loginfo(f"Timeout docking remain {self.dock_timeout_}s ")

        else:
            if self.pause_flag_:
                self.pause_flag_ = False
                self.start_time_ = rospy.Time.now()
        return self.is_pause_ or self.obstacle_detected_
    

    def publish_velocity(self, linear_vel=0.0, angular_vel=0.0):
        """
        Command the robot to move, default param is STOP!
        """
        msg = Twist()
        msg.linear.x = utils.clamp(linear_vel,
                                  self.cfg.linear_vel_range[0],
                                  self.cfg.linear_vel_range[1])
        msg.angular.z = utils.clamp(angular_vel,
                                  self.cfg.angular_vel_range[0],
                                  self.cfg.angular_vel_range[1])

        self.cmd_vel_pub_.publish(msg)

    def get_tf(
        self, target_link=None, ref_link=None, target_time=None, print_out=True
    ) -> np.ndarray:
        """
        This will provide the transformation of the marker,
        if ref_link is not provided, we will use robot's base_link as ref
        :param now : this is a hack fix
        :return : 4x4 homogenous matrix, None if not avail
        """
        if ref_link is None:
            ref_link = self.cfg.base_link
        if target_link is None:
            target_link = self.cfg.last_frame
        if target_time is None:
            target_time = rospy.Time.now()

        try:
            return utils.get_mat_from_transfrom_msg(
                self.__tfBuffer.lookup_transform(
                    ref_link, target_link, target_time, rospy.Duration(self.cfg.tf_expiry)
                )
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            if print_out:
                rospy.logwarn(f"Failed lookup: {target_link}, from {ref_link}")
            return None

    def get_2D_pose(self, target_link=None, base_link=None):

        if target_link is None:
            target_link = self.cfg.last_frame
        if base_link is None:
            base_link = self.cfg.base_link

        try:
            trans = self.__tfBuffer.lookup_transform(base_link, target_link, rospy.Time(0))

            rotation = euler_from_quaternion(
                [
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ]
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logerr(f"Failed lookup: {target_link}, from {base_link}")
            return None

        x = trans.transform.translation.x
        y = trans.transform.translation.y
        theta = rotation[2]

        return x, y, theta

    def get_odom(self) -> np.ndarray:
        """
        Get the current odom of the robot
        :return : 4x4 homogenous matrix, None if not avail
        """
        try:
            return utils.get_mat_from_odom_msg(
                rospy.wait_for_message(
                    "/amr/odometry/filtered", Odometry, timeout=self.cfg.tf_expiry
                )
            )
        except rospy.exceptions.ROSException:
            rospy.logerr(f"Failed to get odom")
            return None

    def retry_with_high_current(self, forward, times=0, limit=None) -> bool:
        """
        Move robot forward when catch high motor current
        `forward`: How far for moving robot
        `times`: How many times for retry
        `limit`: If `times` > `limit` --> ERROR
        """
        if times == limit:
            rospy.logerr(f"The times of high current exceed {limit}!")
            self.error_pub_.publish(1)
            return False

        self.set_state(self.dock_state_, f"Move with odom {forward}m because high motor current!")
        return self.move_with_odom(self.cfg.min_linear_vel, self.cfg.max_linear_vel, forward)

    def move_with_odom(self, min_speed: float, max_speed: float, forward: float) -> bool:
        """
        Move robot in linear motion with Odom. Blocking function
        :return : success
        """
        self.set_state(self.dock_state_, f"move robot: {forward:.2f}m!")

        _initial_tf = self.get_odom()
        if _initial_tf is None:
            return False

        # get the tf mat from odom to goal frame
        _goal_tf = utils.apply_2d_transform(_initial_tf, (forward, 0, 0))
        _pid = utils.PID(self.cfg.k_p, self.cfg.k_i, self.cfg.k_d, min_speed, max_speed)
        # second mat
        prev_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.check_cancel():
                return False

            elif self.do_pause():
                pass

            else:
                if (
                    self.dock_state_ == DockState.STEER_DOCK
                    or self.dock_state_ == DockState.LAST_MILE
                ):
                    if self.high_motor_drop_current_ or self.high_motor_pickup_current_:
                        self.reset_high_current()
                        self.publish_velocity()
                        if not self.retry_with_high_current(0.4):
                            return False
                        return True

                _curr_tf = self.get_odom()
                if _curr_tf is None:
                    return False

                dx, dy, dyaw = utils.compute_tf_diff(_curr_tf, _goal_tf)

                if abs(dx) < self.cfg.stop_trans_diff:
                    self.publish_velocity()
                    rospy.logwarn("Done with move robot")
                    return True

                # This makes sure that the robot is actually moving linearly
                time_now = rospy.Time.now()
                dt = (time_now - prev_time).to_sec()
                l_vel_pid = _pid.update(forward, forward - dx, dt)
                ang_vel = utils.sat_proportional_filter(
                    dyaw, abs_max=self.cfg.min_angular_vel, factor=0.2
                )
                l_vel = utils.bin_filter(dx, l_vel_pid)

                self.publish_velocity(linear_vel=l_vel, angular_vel=ang_vel)
                prev_time = time_now
            self.rate_.sleep()
        exit(0)

    def rotate_with_odom(self, rotate: float) -> bool:
        """
        Spot Rotate the robot with odom. Blocking function
        * `rotate`: In degree
        * @return : success
        """
        self.set_state(self.dock_state_, f"Turn robot: {rotate:.2f} rad!")

        _initial_tf = self.get_odom()
        if _initial_tf is None:
            return False

        # get the tf mat from odom to goal frame
        _goal_tf = utils.apply_2d_transform(_initial_tf, (0, 0, rotate))

        while not rospy.is_shutdown():

            if self.check_cancel():
                return False

            elif self.do_pause():
                pass

            else:
                if (
                    self.dock_state_ == DockState.STEER_DOCK
                    or self.dock_state_ == DockState.LAST_MILE
                ):
                    if self.high_motor_drop_current_ or self.high_motor_pickup_current_:
                        self.reset_high_current()
                        self.publish_velocity()
                        return False

                _curr_tf = self.get_odom()
                if _curr_tf is None:
                    return False

                dx, dy, dyaw = utils.compute_tf_diff(_curr_tf, _goal_tf)

                if abs(dyaw) < self.cfg.stop_yaw_diff:
                    self.publish_velocity()
                    rospy.logwarn("Done with rotate robot")
                    return True
                
                sign = 1 if rotate > 0 else -1

                angular_vel = sign * self.cfg.rotate_to_heading_angular_vel
                dt = 1 / self.cfg.controller_frequency
                
                min_feasible_angular_speed = self.current_speed_.angular.z - self.cfg.max_angular_accel * dt
                max_feasible_angular_speed = self.current_speed_.angular.z + self.cfg.max_angular_accel * dt
                
                angular_vel = utils.clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed)

                if abs(dyaw) < self.cfg.angle_threshold:
                    angular_vel = \
                        sign * utils.clamp(self.cfg.rotate_to_heading_angular_vel * abs(dyaw) / self.cfg.max_angular_deccel,
                                    self.cfg.min_angular_vel, self.cfg.rotate_to_heading_angular_vel)

                self.publish_velocity(angular_vel=angular_vel)
            self.rate_.sleep()
        exit(0)

    def autodock_callback(self, goal: AutoDockingGoal):
        self.start_time_ = rospy.Time.now()
        self.dock_timeout_ = self.cfg.dock_timeout
        _result = AutoDockingResult()
        _result.is_success = self.start(
            goal.dock_name,
            goal.dock_pose,
            goal.dock_mode.mode,
            goal.dock_limit,
            goal.tag_names,
            goal.go_in_dock,
            goal.go_out_dock,
        )

        _prev_state = DockState.to_string(self.feedback_msg.state)

        if _result.is_success:
            _duration = rospy.Time.now() - self.start_time_
            _result.status = f"Succeeded! Took {_duration.secs}s"
            self.autodock_action.set_succeeded(_result)

        elif self.autodock_action.is_preempt_requested():
            _result.is_success = False
            _result.status = (
                f"Cancel during [{_prev_state}], " f"with status: {self.feedback_msg.status}"
            )
            self.autodock_action.set_preempted(_result)
            self.set_state(DockState.IDLE, "Dock Action is canceled")

        else:
            _result.is_success = False
            _result.status = (
                f"Failed during [{_prev_state}], " f"with status: {self.feedback_msg.status}"
            )
            self.autodock_action.set_aborted(_result)
            self.set_state(DockState.IDLE, "Failed execute Dock Action")

    def timer_callback(self, timer):
        # This is mainly for debuging
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.cfg.base_link
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.z = 1.1
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1
        marker.color.b = 1
        marker.color.a = 1
        marker.text = DockState.to_string(self.dock_state_)
        self.pub_marker.publish(marker)
