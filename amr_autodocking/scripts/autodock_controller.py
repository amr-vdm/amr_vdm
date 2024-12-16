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
import math
from typing import List
import amr_autodocking.autodock_utils as utils

from amr_autodocking.autodock_server import AutodockConfig, AutoDockServer
from amr_autodocking.autodock_utils import DockState
from amr_msgs.msg import DockLimit, DockMode, DockParam
from apriltag_ros.msg import AprilTagDetectionArray


class AutoDockStateMachine(AutoDockServer):
    def __init__(
        self, config: AutodockConfig, run_server=True, load_rosparam=False, fake_clock=False
    ):
        if fake_clock:
            rospy.logwarn("/autodock_controller: WARNING!!!! fake clock is in used, Temporary set use_sim_time to true")
            rospy.set_param("/use_sim_time", True)

        rospy.init_node("auto_dock_node")
        rospy.loginfo("/autodock_controller: Initialized auto_dock_node.")

        if fake_clock:
            rospy.logwarn("/autodock_controller: WARNING!!!! fake clock enabled! now disable use_sim_time")
            rospy.set_param("/use_sim_time", False)

        self.cfg = config
        super().__init__(self.cfg, run_server)

        if load_rosparam:
            self.init_params()

        self.dock_state_ = DockState.IDLE

    def init_params(self):
        print("/autodock_controller: Autodock params:")
        param_names = [
            attr
            for attr in dir(self.cfg)
            if not callable(getattr(self.cfg, attr)) and not attr.startswith("__")
        ]

        # get private rosparam, if none use default
        for param_name in param_names:
            param_val = rospy.get_param("~" + param_name, getattr(self.cfg, param_name))
            print(f"* /{param_name}: {param_val}")
            setattr(self.cfg, param_name, param_val)
            # brute check to ensure numerical config is positive
            if isinstance(param_val, (int, float)):
                assert param_val >= 0, f"[{param_name}] param should be +ve"

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
        Start Docking Sequence
        """
        rospy.loginfo(f"/autodock_controller: Start autodock! Will attempt with {self.cfg.retry_count} retry!")

        if self.cfg.debug_mode:
            go_in_dock_debug = ""
            for i in range(len(go_in_dock)):
                if go_in_dock[i].action_type == DockParam.TYPE_ROTATE:
                    go_in_dock_debug += f"rot:{go_in_dock[i].value}"
                else:
                    go_in_dock_debug += f"mov:{go_in_dock[i].value}"
                if i < len(go_in_dock) - 1:
                    go_in_dock_debug += " -> "

            go_out_dock_debug = ""
            for i in range(len(go_out_dock)):
                if go_out_dock[i].action_type == DockParam.TYPE_ROTATE:
                    go_out_dock_debug += f"rot:{go_out_dock[i].value}"
                else:
                    go_out_dock_debug += f"mov:{go_out_dock[i].value}"
                if i < len(go_out_dock) - 1:
                    go_out_dock_debug += " -> "
            print(f"[Auto dock node]: RECIEVE VALUE FROM SERVER")
            print(f"* dock_name: {dock_name}")
            print(f"* mode: {mode}")
            print(
                f"* dock_limit: [angle: {dock_limit.rotate_angle}, orientation: {dock_limit.rotate_orientation}]"
            )
            print(f"* tag_names: {tag_names}")
            print(f"* go_in_dock: {go_in_dock_debug}")
            print(f"* go_out_dock: {go_out_dock_debug}")

        self.brake(False)

        if mode == DockMode.MODE_UNDOCK:
            if self.goOutDock(mode, go_out_dock):
                self.reset()
                self.set_state(DockState.IDLE, "/autodock_controller: Undock completed!")
                return True
            else:
                self.reset()
                self.set_state(DockState.ERROR, "/autodock_controller: Undock failed!")
                return False

        # Custom for robot head to dock
        if not self.goInDock(go_in_dock):
            return False

        # Reset some needed values when start docking
        self.reset_all()

        is_dock_limit = True
        if dock_limit.rotate_angle == 0 and dock_limit.rotate_orientation == 0:
            # Reset to default value
            dock_limit.rotate_angle = 30
            dock_limit.rotate_orientation = 0
            is_dock_limit = False

        if mode == DockMode.MODE_CHARGE:
            self.enable_line_detector("front", True)

        else:
            self.enable_line_detector("back", True)
            if mode == DockMode.MODE_PICKUP:
                self.update_line_extraction_param(0)
                self.enable_apriltag_detector(True)

                try:
                    tag_detections = rospy.wait_for_message(
                        "/back_camera/tag_detections", AprilTagDetectionArray, timeout=1.0
                    )

                    if tag_detections is not None:
                        tags = tag_detections.detections

                        min_distance = 100
                        tag_name = ""

                        if len(tag_names) > 0:
                            for tag in tags:
                                if f"tag_frame_{tag.id[0]}" in tag_names:
                                    bot2dock = self.get_tf(f"tag_frame_{tag.id[0]}")
                                    x, y, yaw = utils.get_2d_pose(bot2dock)
                                    distance = self.distance2D(x, y)

                                    if distance < min_distance:
                                        tag_name = f"tag_frame_{tag.id[0]}"
                                        min_distance = distance
                        else:
                            for tag in tags:
                                bot2dock = self.get_tf(f"tag_frame_{tag.id[0]}")
                                x, y, yaw = utils.get_2d_pose(bot2dock)
                                distance = self.distance2D(x, y)

                                if distance < min_distance:
                                    tag_name = f"tag_frame_{tag.id[0]}"
                                    min_distance = distance

                    if tag_name != "":
                        self.tag_frame_ = tag_name
                    else:
                        self.enable_apriltag_detector(False)
                        self.tag_frame_ = None
                except Exception as e:
                    print(e)

            elif mode == DockMode.MODE_DROPOFF:
                self.update_line_extraction_param(2)
                self.update_polygon_param(2)

        while True:
            if (
                self.pre_dock(
                    mode,
                    is_dock_limit,
                    dock_limit.rotate_angle,
                    dock_limit.rotate_orientation,
                    None,
                )
                and self.steer_dock(mode)
                and self.lastmile_dock(mode)
                and self.cmd_slider_mortor(mode)
            ):
                if mode == DockMode.MODE_CHARGE:
                    self.reset()
                self.set_state(DockState.IDLE, "/autodock_controller: Autodock completed!")
                return True

            # If Dock failed
            self.publish_velocity()
            self.print_error("Error!")

            if (
                self.dock_state_ == DockState.SLIDER_GO_IN
                or self.dock_state_ == DockState.SLIDER_GO_OUT
                or self.dock_state_ == DockState.GO_OUT_DOCK
            ):
                break

            self.set_state(DockState.ERROR, "/autodock_controller: Autodock failed!")
            # check again if it failed because of canceled
            if self.check_cancel():
                break

            if mode == DockMode.MODE_PICKUP:
                if not self.retry(self.tag_frame_):
                    if not self.retry(self.cfg.first_frame):
                        break

            elif mode == DockMode.MODE_DROPOFF:
                if not self.retry(self.cfg.parallel_frame):
                    break

        self.reset()
        return False

    def check_slider_sensor_state(
        self, slider_cmd: int, sensor_order: int, sensor_check: int, timeout: float
    ) -> bool:
        start_time = rospy.Time.now()
        check_pause = False
        timeout_checksensor = 5.0

        self.cmd_slider(slider_cmd)
        while True:
            if self.check_cancel():
                return False

            elif self.do_pause():
                if not check_pause:
                    timeout_used = (rospy.Time.now() - start_time).to_sec()
                    timeout -= timeout_used
                    timeout_checksensor -= timeout_used
                    check_pause = True
            else:
                if check_pause:
                    self.cmd_slider(slider_cmd)
                    start_time = rospy.Time.now()
                    check_pause = False

                elif (rospy.Time.now() - start_time).to_sec() >= timeout:
                    rospy.logerr("/autodock_controller: Slider motor error: Exceed timeout 15s")
                    self.error_pub_.publish(2)
                    return False

                elif self.slider_sensor_state_[sensor_order] == 1:
                    msg = "max position" if sensor_order == 1 else "original position"
                    rospy.loginfo(f"/autodock_controller: Slider motor is at {msg}")
                    return True

                elif (rospy.Time.now() - start_time).to_sec() >= timeout_checksensor:
                    if self.slider_sensor_state_[sensor_check] == 1:
                        rospy.logerr("/autodock_controller: Slider motor error: Not working!")
                        self.error_pub_.publish(3)
                        return False
            self.rate_.sleep()

    def steer_to_charger(self, max_vel_x, min_vel_x):
        _pid = utils.PID(self.cfg.k_p_steer, self.cfg.k_i_steer, self.cfg.k_d_steer)
        prev_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.check_cancel():
                return False

            elif self.do_pause():
                pass

            elif self.high_motor_drop_current_ or self.high_motor_pickup_current_:
                self.reset_high_current()
                self.publish_velocity()

                if not self.move_with_odom(
                    self.cfg.min_linear_vel, self.cfg.max_linear_vel, -0.5
                ) and not self.pre_dock(DockMode.MODE_CHARGE):
                    rospy.logerr("/autodock_controller: Can't execute after recovery!")
                    return False
                continue

            else:
                dock_tf = self.get_tf(self.cfg.charger_link)
                if dock_tf is None:
                    rospy.logerr("Can not detect dock frame: %s", self.cfg.charger_link)
                    # dis_move = distance - 0.26
                    # if (dis_move > 0):
                    #     rospy.logwarn(f"[Steer to charger]: Move with odom {distance}m!")
                    #     return (self.move_with_odom(0.02, 0.035, dis_move + self.cfg.stop_trans_diff))
                    # else:
                    #     return True
                    return False
                dock_pose = utils.get_2d_pose(dock_tf)
                x, y, yaw = utils.flip_base_frame(dock_pose)

                if abs(x) - 0.4 < 0:
                    return True

                time_now = rospy.Time.now()
                dt = (time_now - prev_time).to_sec()
                angle = _pid.update(0, y, dt)

                v = max_vel_x

                if abs(x) < 0.5:
                    v = min_vel_x

                w = angle

                self.publish_velocity(v, w)
                prev_time = time_now
            self.rate_.sleep()

    def steer_with_pickup(self, max_vel_x, min_vel_x):
        flag = False
        _pid = utils.PID(self.cfg.k_p_steer, self.cfg.k_i_steer, self.cfg.k_d_steer)
        prev_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.check_cancel():
                return False

            elif self.do_pause():
                pass

            elif self.high_motor_pickup_current_:
                self.reset_high_current()
                self.publish_velocity()
                self.retry_with_high_current(0.35, 1, 2)
                return False

            else:
                dock_laser_tf = self.get_tf(self.cfg.first_frame)
                if self.tag_frame_:
                    dock_tag_tf = self.get_tf(self.tag_frame_)
                else:
                    dock_tag_tf = None

                if dock_tag_tf is not None:
                    dock_tf = dock_tag_tf
                elif dock_laser_tf is not None:
                    dock_tf = dock_laser_tf
                else:
                    dock_tf = None

                if dock_tf is None:
                    rospy.loginfo("/autodock_controller: Steerdock_pickup will return True because lost dock")
                    return True

                dock_pose = utils.get_2d_pose(dock_tf)
                x, y, yaw = dock_pose

                sign = 1 if x > 0 else -1

                if dock_laser_tf is not None:
                    x_dock_laser = utils.get_2d_pose(dock_laser_tf)[0]
                    if abs(x_dock_laser) < (self.cfg.back_laser_offset + 0.2) and not flag:
                        self.update_line_extraction_param(1)
                        flag = True

                    elif (abs(x_dock_laser) - self.cfg.steer_distance_threshold) < 0:
                        return True

                time_now = rospy.Time.now()
                dt = (time_now - prev_time).to_sec()
                angle = _pid.update(0, y, dt)

                v = sign * max_vel_x

                if abs(x) < 0.5:
                    v = sign * min_vel_x

                w = angle

                if abs(w) > 0.15:
                    w = 0.0

                self.publish_velocity(v, w)
                prev_time = time_now
            self.rate_.sleep()

    def steer_with_dropoff(self, max_vel_x, min_vel_x):
        flag = False
        _pid = utils.PID(self.cfg.k_p_steer, self.cfg.k_i_steer, self.cfg.k_d_steer)
        prev_time = rospy.Time.now()
        start = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.check_cancel():
                return False

            elif self.do_pause():
                pass

            elif self.high_motor_drop_current_:
                if (rospy.Time.now() - start).to_sec() <= 3.0:
                    self.reset_high_current()
                    continue
                self.reset_high_current()
                flag = False
                self.publish_velocity()
                self.update_line_extraction_param(2)

                self.retry_with_high_current(0.4, 1, 2)
                return False

            else:
                # Check whether back laser in dock (Depend on distance from center of laser to both side of the dock)
                if self.left_range_ < 0.3 and self.right_range_ < 0.3:
                    if not flag:
                        self.update_line_extraction_param(1)
                        start_time = rospy.Time.now()
                        flag = True
                        rospy.loginfo("/autodock_controller: BackLaser is in dropoff dock!")

                    if (rospy.Time.now() - start_time).to_sec() > 4.5:
                        if not self.high_motor_drop_current_:
                            return True

                elif flag:
                    rospy.logwarn("/autodock_controller: BackLaser is out dropoff dock, it's wrong. Please check!")
                    flag = False  # Reset flag for calculate total time

                dock_tf = self.get_tf(self.cfg.parallel_frame)
                if dock_tf is None:
                    return False

                dock_pose = utils.get_2d_pose(dock_tf)

                x, y, yaw = dock_pose

                sign = 1 if x > 0 else -1

                time_now = rospy.Time.now()
                dt = (time_now - prev_time).to_sec()
                angle = _pid.update(0, y, dt)

                v = sign * max_vel_x

                if abs(x) < 0.5:
                    v = sign * min_vel_x

                w = angle

                self.publish_velocity(v, w)
                prev_time = time_now
            self.rate_.sleep()

    def lastmile_with_dropoff(self) -> bool:
        while not rospy.is_shutdown():
            if self.check_cancel():
                return False
            elif self.do_pause():
                pass
            elif self.high_motor_drop_current_:
                self.reset_high_current()
                self.publish_velocity()
                break
            else:
                self.publish_velocity(-self.cfg.max_x_lastmile)

            self.rate_.sleep()
        return True

    def lastmile_with_pickup(self) -> bool:
        flag = False
        is_setLineExtra = False
        count_lost_dock = 0
        _pid = utils.PID(self.cfg.k_p_steer, self.cfg.k_i_steer, self.cfg.k_d_steer)
        prev_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.check_cancel():
                return False

            elif self.do_pause():
                pass

            elif self.high_motor_pickup_current_:
                self.reset_high_current()
                if not (
                    self.cart_sensor_state_ == (1, 1)
                    or self.cart_sensor_state_ == (1, 0)
                    or self.cart_sensor_state_ == (0, 1)
                ):
                    self.publish_velocity()
                    self.update_line_extraction_param(0)
                    flag = False
                    is_setLineExtra = False

                    self.retry_with_high_current(0.45, 1, 2)
                    return False

            else:
                if not is_setLineExtra:
                    self.update_line_extraction_param(1)
                    is_setLineExtra = True

                if not flag:
                    dock_tf = self.get_tf(self.cfg.last_frame)

                    v = -self.cfg.min_x_pid_lastmile
                    w = 0.0
                    # if dock_tf is None:
                    # if count_lost_dock <= 5:
                    #     rospy.logwarn(f"Can not get {self.cfg.last_frame}!, will retry find dock: {count_lost_dock} retry")
                    #     count_lost_dock += 1
                    #     self.publish_velocity(-self.cfg.min_x_pid_lastmile,0)
                    #     continue
                    # else:
                    #     rospy.logerr(f"Maximum retry find dock frame: {self.cfg.last_frame}")
                    #     self.error_pub_.publish(4)
                    #     return False
                    if dock_tf is not None:
                        # count_lost_dock = 0

                        dock_pose = utils.get_2d_pose(dock_tf)
                        x, y, yaw = dock_pose

                        sign = 1 if x > 0 else -1

                        time_now = rospy.Time.now()
                        dt = (time_now - prev_time).to_sec()
                        angle = _pid.update(0, y, dt)

                        w = angle

                        if abs(w) > 0.15:
                            w = 0.0

                        v = sign * self.cfg.max_x_pid_lastmile

                if (
                    self.cart_sensor_state_ == (1, 1)
                    or self.cart_sensor_state_ == (1, 0)
                    or self.cart_sensor_state_ == (0, 1)
                ):

                    flag = True
                    v = sign * self.cfg.min_x_pid_lastmile

                    if self.cart_sensor_state_ == (1, 1):
                        # self.brake(True)
                        return True
                    elif self.cart_sensor_state_ == (1, 0):
                        w = -0.02
                    elif self.cart_sensor_state_ == (0, 1):
                        w = 0.02

                self.publish_velocity(v, w)
                prev_time = time_now
            self.rate_.sleep()

    # ============> MAIN RUN <============#

    def pre_dock(
        self, mode, is_dock_limit=False, rotate_angle=0, rotate_orientation=0, dock_name=None
    ) -> bool:
        self.set_state(DockState.PREDOCK, "/autodock_controller: Running!")

        pose_list = []
        check_yaw_counter = 0
        check_y_counter = 0

        if mode == DockMode.MODE_CHARGE:
            self.turn_off_front_scan_safety(True)
            self.turn_off_front_depth_safety(True)
            self.turn_off_ultrasonic_safety(True)

        self.turn_off_back_scan_safety(False)

        while not rospy.is_shutdown():
            if self.check_cancel():
                return False
            elif self.do_pause():
                pass
            else:
                if mode == DockMode.MODE_CHARGE:
                    dock_tf = self.get_tf(self.cfg.charger_link)
                    if dock_tf is None:
                        rospy.logerr("/autodock_controller: Can not detect dock frame: %s", self.cfg.charger_link)

                    dock_pose = utils.get_2d_pose(dock_tf)
                    if len(pose_list) < self.cfg.predock_tf_samples:
                        pose_list.append(dock_pose)
                        self.rate_.sleep()
                        continue

                    avg_pose = utils.avg_2d_poses(pose_list)
                    pose_list = []

                    x, y, yaw = utils.flip_base_frame(avg_pose)

                    # Check yaw
                    if (check_yaw_counter < 2) and (abs(yaw) > self.cfg.yaw_predock_tolerance):
                        if not self.rotate_with_odom(
                            self.cfg.min_angular_vel, self.cfg.max_angular_vel, yaw
                        ):
                            return False
                        check_yaw_counter += 1
                        self.rate_.sleep()
                        continue

                    if (check_y_counter < 3) and (abs(y) > self.cfg.max_parallel_offset):
                        if not self.auto_correction(
                            x, y, 0.2, self.cfg.front_laser_offset, is_dock_limit
                        ) and not self.correct_robot(y, True, rotate_angle, rotate_orientation):
                            return False
                        check_y_counter += 1
                        check_yaw_counter = 0
                        self.set_state(DockState.PREDOCK, "")
                        self.rate_.sleep()
                        continue
                else:
                    if (mode == DockMode.MODE_DROPOFF):
                        dock_tf = self.get_tf(self.cfg.parallel_frame)
                        if dock_tf is None:
                            rospy.logerr(f"/autodock_controller: Can not detect all frame!")
                            self.error_pub_.publish(4)
                            return False
                        dock_pose = utils.get_2d_pose(dock_tf)
                    else:
                        try:
                            dock_pose = self.get_dock_pose(self.cfg.first_frame, self.tag_frame_)
                        except ValueError as e:
                            rospy.logerr(f"/autodock_controller: {e}")
                            self.error_pub_.publish(4)
                            return False

                    if len(pose_list) < self.cfg.predock_tf_samples:
                        pose_list.append(dock_pose)
                        self.rate_.sleep()
                        continue

                    avg_pose = utils.avg_2d_poses(pose_list)
                    pose_list = []

                    x, y, yaw = avg_pose

                    # Check yaw
                    if check_yaw_counter < 2:
                        if abs(yaw) > self.cfg.yaw_predock_tolerance:
                            if not self.rotate_with_odom(yaw):
                                return False
                            check_yaw_counter += 1
                            self.rate_.sleep()
                            continue

                    # Check y
                    if check_y_counter < 2:
                        # Add tolerance value when dropoff
                        # if (mode == DockMode.MODE_DROPOFF):
                        #     y += 0.025 if y < 0 else -0.025

                        if abs(y) > self.cfg.max_parallel_offset:
                            if mode == DockMode.MODE_PICKUP:
                                if not self.auto_correction(
                                    x, y, 0.28, self.cfg.back_laser_offset, is_dock_limit
                                ) and not self.correct_robot(
                                    y, False, rotate_angle, rotate_orientation
                                ):
                                    return False

                                check_y_counter += 1
                                check_yaw_counter = 0
                                self.update_polygon_param(1)
                                self.set_state(DockState.PREDOCK, "")
                                self.rate_.sleep()
                                continue

                            elif mode == DockMode.MODE_DROPOFF and not self.correct_robot(
                                y, False, rotate_angle, rotate_orientation
                            ):
                                return False

                            check_y_counter += 1
                            check_yaw_counter = 0
                            self.rate_.sleep()
                            continue

                        elif mode == DockMode.MODE_PICKUP:
                            self.update_polygon_param(1)
                            return True

                self.print_success("Completed!")
                return True

            self.rate_.sleep()
        exit(0)

    def steer_dock(self, mode: int) -> bool:
        self.set_state(DockState.STEER_DOCK, "Running!")

        if mode == DockMode.MODE_CHARGE:
            self.turn_off_front_scan_safety(True)
            if not self.steer_to_charger(self.cfg.max_x_pid_steer, self.cfg.min_x_pid_steer):
                return False
        else:
            self.turn_off_back_scan_safety(True)
            if mode == DockMode.MODE_PICKUP:
                if not self.steer_with_pickup(self.cfg.max_x_pid_steer, self.cfg.min_x_pid_steer):
                    return False
            elif mode == DockMode.MODE_DROPOFF:
                if not self.steer_with_dropoff(
                    self.cfg.max_x_pid_steer, self.cfg.min_x_pid_steer
                ):
                    return False

        self.print_success("Completed!")
        return True

    def lastmile_dock(self, mode: int) -> bool:
        self.set_state(DockState.LAST_MILE, "Running!")
        if mode == DockMode.MODE_PICKUP:
            if not self.lastmile_with_pickup():
                return False
        elif mode == DockMode.MODE_DROPOFF:
            if not self.lastmile_with_dropoff():
                return False
        elif mode == DockMode.MODE_CHARGE:
            if not self.move_with_odom(0.04, 0.055, 0.15):
                return False

        self.brake(True)
        self.print_success("Completed!")
        return True

    def cmd_slider_mortor(self, mode: int, timeout=20.0) -> bool:
        if mode == DockMode.MODE_PICKUP:
            self.set_state(DockState.SLIDER_GO_OUT, "Running!")
            cmd_slider = 1
            sensor_order = 1
            sensor_check = 0
            self.enable_apriltag_detector(False)
        elif mode == DockMode.MODE_DROPOFF:
            self.set_state(DockState.SLIDER_GO_IN, "Running!")
            cmd_slider = 2
            sensor_order = 0
            sensor_check = 1
        elif mode == DockMode.MODE_CHARGE:
            return True

        if not self.check_slider_sensor_state(cmd_slider, sensor_order, sensor_check, timeout):
            return False

        self.print_success("Completed!")
        return True

    def goInDock(self, go_in_dock: List[DockParam]):
        self.brake(False)
        for action in go_in_dock:
            if action.action_type == DockParam.TYPE_ROTATE:
                if not self.rotate_with_odom(action.value * math.pi / 180):
                    return False
            elif action.action_type == DockParam.TYPE_MOVE:
                if not self.move_with_odom(
                    self.cfg.min_linear_vel, self.cfg.max_linear_vel, action.value
                ):
                    return False

        self.print_success("GoInDock finished!")
        return True

    def goOutDock(self, mode: int, go_out_dock: List[DockParam]):
        if mode == DockMode.MODE_CHARGE:
            return True

        self.brake(False)
        if go_out_dock[0].value > 0:
            self.turn_off_back_scan_safety(True)
            self.turn_off_front_scan_safety(False)
            self.turn_off_ultrasonic_safety(False)
            self.turn_off_front_depth_safety(False)
        else:
            self.turn_off_front_scan_safety(True)
            self.turn_off_ultrasonic_safety(True)
            self.turn_off_front_depth_safety(True)
            self.turn_off_back_scan_safety(False)

        rospy.sleep(1.0)

        for action in go_out_dock:
            if action.action_type == DockParam.TYPE_ROTATE:
                self.set_state(DockState.GO_OUT_DOCK, f"Rotate robot {action.value} degrees!")
                if not self.rotate_with_odom(action.value * math.pi / 180):
                    return False
            elif action.action_type == DockParam.TYPE_MOVE:
                self.set_state(DockState.GO_OUT_DOCK, f"Move robot {action.value}m !")
                if not self.move_with_odom(
                    self.cfg.min_linear_vel, self.cfg.max_linear_vel, action.value
                ):
                    return False

        self.print_success("Completed!")
        return True


if __name__ == "__main__":
    config = AutodockConfig()

    node = AutoDockStateMachine(config, run_server=True, load_rosparam=True, fake_clock=False)
    rospy.spin()
