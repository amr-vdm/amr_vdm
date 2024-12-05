#!/usr/bin/env python3
import rospy
import time
import math
import yaml
import numpy as np
import actionlib
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from amr_autodocking.msg import AutoDockingAction, AutoDockingGoal
from nav_msgs.srv import LoadMap, LoadMapResponse
from std_srvs.srv import Empty, SetBool
from std_msgs.msg import Bool, Int16, Float64, Int16MultiArray
from geometry_msgs.msg import Twist
from tf import transformations as ts
from nav_msgs.msg import Odometry
from typing import Tuple

Pose2D = Tuple[float, float, float]

INFO  = rospy.loginfo
WARN  = rospy.logwarn
ERROR = rospy.logerr

amr_waypoint_file = "/home/amr/catkin_ws/src/amr/amr_waypoint_generator/config/amr_waypoints.yaml"
map_file = "/home/amr/catkin_ws/src/amr/amr_gazebo/maps/custom_map_5/custom_map_5.yaml"

class Parameter():
    map_frame_id = "map"
    odom_frame_id = "odom"
    robot_base_frame_id = "base_footprint"
    distance_tolerance = 1.5
    frequency = 100
    tf_expiry = 1.0
    max_x_move = 0.2
    max_w_rot = 0.2 
    min_x_move = 0.15
    min_w_rot = 0.15
    stop_yaw_diff = 0.05
    min_angular_vel = 0.05
    max_angular_vel = 0.25
    max_linear_vel: 0.1
    min_linear_vel: 0.035 
    max_x_out_dock = 0.2
    map_file = ""
    yaw_threshold = 0.35 # rad ~ 20 degrees
    waypoints_to_follow_topic = "/initialpose"
    waypoints_list_topic = "/waypoints"


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
    

class AMRNavigation():

    def __init__(self):
        rospy.init_node('amr_navigation')

        self.params = Parameter()
        self.initParams()

        # Listen to Transfromation
        self.__tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.__tf_listener = tf2_ros.TransformListener(self.__tfBuffer)

        # Params
        self.sleep_period = rospy.Duration(1/50.0)
        self.rate = rospy.Rate(self.params.frequency)

        # list of waypoints to follow
        self.waypoints = []

        # move_base Action Client
        self.move_base_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        WARN("Connecting to move base...")
        self.move_base_client.wait_for_server()
        INFO("Connected to move base.")

        # # Auto docking Client
        # self.autodock_client = actionlib.SimpleActionClient("autodock_action", AutoDockingAction)
        # WARN("Connecting to auto docking...")
        # self.autodock_client.wait_for_server()
        # INFO("Connected to auto docking.")

        # # Change map service
        # self.change_map_client = rospy.ServiceProxy("/change_map", LoadMap)
        # WARN("Connecting to change map service...")
        # self.change_map_client.wait_for_service()
        # INFO("Connected to change map service.")

        # # Clear costmap service
        # self.clear_costmap = rospy.ServiceProxy("/move_base_node/clear_costmaps", Empty)
        # WARN("Connecting to /move_base_node/clear_costmaps service...")
        # self.clear_costmap.wait_for_service()
        # INFO("Connected to /move_base_node/clear_costmaps service.")

        # # Apriltag continuous detection service
        # self.apriltag_client = rospy.ServiceProxy("/back_camera/apriltag_ros/enable_detector", SetBool)
        # WARN("Connecting to /back_camera/apriltag_ros/enable_detector service...")
        # self.apriltag_client.wait_for_service()
        # INFO("Connected to /back_camera/apriltag_ros/enable_detector service.")
        
        # Variables:
        self.battery_state = 0
        self.obstacle_status = False
        self.start_amr = 0
        self.is_cancel = False
        self.is_pause  = False
        self.error_name = 0
        self.dock_name = 0
        self.error = 2   #[0] - Loi cap hang, [1] - Loi lay hang, [2] - Loi vi tri
        self.position_error = 0
        self.mode_error = Int16MultiArray()

        # PID Controller:
        self.k_p = 0.8
        self.k_i = 0.0
        self.k_d = 0.01

        # Waypoints:
        # self.cap_hang_waypoints = self.yamlPose("lk1_caphang_waypoints")
        # self.cap_hang_goal      = self.yamlPose("lk1_caphang_goal")

        self.test_lift_tp3 = self.yamlPose("tp3_test_lift")
        self.test_lift_tp31 = self.yamlPose("tp3_test_lift1")

        self.MODE_CAPHANG = 2
        self.MODE_THAHANG = 3

        # Publishers:
        self.pub_pose_array  = rospy.Publisher(self.params.waypoints_list_topic, PoseArray, queue_size=1, latch=True)
        self.pub_cmd_brake   = rospy.Publisher("cmd_brake", Bool, queue_size=5)
        self.pub_cmd_vel     = rospy.Publisher("/amr/mobile_base_controller/cmd_vel", Twist, queue_size=5)
        self.pub_runonce     = rospy.Publisher("state_runonce_nav", Bool, queue_size=5)

        # Subscribers:
        rospy.Subscriber("START_AMR_Test_in_out_lift", Int16, self.mainRun)
        rospy.Subscriber("CANCEL_AMR", Bool, self.cancelCb)
        rospy.Subscriber("PAUSE_AMR", Bool, self.pauseCb)
        rospy.Subscriber("battery_state", Int16, self.baterryStateCb)
        rospy.Subscriber("error_name", Int16, self.errorNameCb)
        rospy.Subscriber("status_protected_field", Bool, self.protectedFieldCb)


    def initParams(self):
        rospy.loginfo("NODE: %s", rospy.get_name())

        param_names = [attr for attr in dir(self.params) if not callable(getattr(self.params, attr)) and not attr.startswith("__")]

        # get private rosparam, if none use default
        for param_name in param_names:
            param_val = rospy.get_param("~" + param_name, getattr(self.params, param_name))
            print(f"set param [{param_name}] to [{param_val}]")
            setattr(self.params, param_name, param_val)


    def pauseCb(self, msg:Bool):
        self.is_pause = msg.data
    

    def cancelCb(self, msg:Bool):
        self.is_cancel = msg.data

        if self.is_cancel:
            self.runOnce(False)


    def protectedFieldCb(self, msg:Bool):
        self.obstacle_status = msg.data
    

    def errorNameCb(self, msg:Int16):
        self.error_name = msg.data

    
    def turnOnBrake(self, signal):
        self.pub_cmd_brake.publish(signal)
    
    def runOnce(self, signal):
        self.pub_runonce.publish(signal)


    def baterryStateCb(self, msg: Int16):
        self.battery_state = msg.data 

    
    def getError(self, error_name:int):
        """
        `error_name = 0`: Loi cap hang
        `error_name = 1`: Loi lay hang
        `error_name = 2`: Loi vi tri 
        """
        self.error = error_name

    def getPositionError(self, position_error_name:int):
        """
        `position_error_name = 0`: Nothing (Default)
        `position_error_name = 1`: Error at the cleaning room
        `position_error_name = 2`: Error at the line (eg: Line 55 or Line 56)
        """
        self.position_error = position_error_name

    def onOffBrake(self):
        self.turnOnBrake(True)
        rospy.sleep(0.5)
        self.turnOnBrake(False)

    def resetAllVariables(self):
        self.battery_state = 0
        self.start_amr = 0
        self.error = 2
        self.error_name = 0
        self.position_error = 0
        self.is_cancel = False
        self.runOnce(False)
        self.turnOnBrake(True)

    
    def yamlPose(self, position_name: str):
        """
        Get position from yaml file
        """
        assert (type(position_name) == str), "position_name is not str type"

        with open(amr_waypoint_file, 'r') as file:
            data = yaml.safe_load(file)
            try:
                position = data[f'{position_name}']['pose']
                float_position_list = [[float(value) if isinstance(value, (int, float, str)) \
                                    else value for value in sublist] for sublist in position] 
                return float_position_list   
            except:
                return position    
    

    def getMatFromOdomMsg(self, msg: Odometry) -> np.ndarray:
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
    

    def getOdom(self) -> np.ndarray:
        """
        Get the current odom of the robot
        :return : 4x4 homogenous matrix, None if not avail
        """
        try:
            return self.getMatFromOdomMsg(
                rospy.wait_for_message(
                    "/amr/odometry/filtered", Odometry, timeout=1.0)
            )
        except rospy.exceptions.ROSException:
            rospy.logerr(f"Failed to get odom")
            return None
        
    
    def apply2DTransform(self, mat: np.ndarray, transform: Pose2D) -> np.ndarray:
        """
        Apply a 2d transform to a homogenous matrix
        :param mat:         the input 4x4 homogenous matrix
        :param transform :  2d transform which to apply to the mat
        :return :           transformed homogenous transformation matrix
        """
        # req target transformation from base
        q = quaternion_from_euler(0, 0, transform[2])
        tf_mat = ts.concatenate_matrices(
            ts.translation_matrix(
                (transform[0], transform[1], 0)), ts.quaternion_matrix(q))
        return np.matmul(mat, tf_mat)
    

    def computeTfDiff(self, current_tf: np.ndarray, ref_tf: np.ndarray) -> Pose2D:
        """
        Find the diff of two transformation matrix
        :param :  homogenous transformation of 2 matrices
        :return :  the 2d planer trans fo the 2 inputs; [x, y, yaw]
        """
        tf_diff = np.matmul(ts.inverse_matrix(current_tf), ref_tf)
        trans = ts.translation_from_matrix(tf_diff)
        euler = ts.euler_from_matrix(tf_diff)
        return trans[0], trans[1], euler[2]
    

    def satProportionalFilter(self,
            input: float, abs_min=0.0, abs_max=10.0, factor=1.0) -> float:
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
    

    def binFilter(self, input: float, abs_boundary: float) -> float:
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
    

    def clamp(self, value, lower_limit, upper_limit):
        if value > upper_limit:
            return upper_limit
        elif value < lower_limit:
            return lower_limit
        return value
    

    def pubSpeedCmd(self, linear_vel=0.0, angular_vel=0.0):
        """
        Command the robot to move, default param is STOP!
        """
        msg = Twist()
        msg.linear.x = self.clamp(linear_vel, -0.2, 0.2)
        msg.angular.z = self.clamp(angular_vel, -0.3, 0.3)
        self.pub_cmd_vel.publish(msg)
    

    def rotateWithOdom(self, v_w_min: float, v_w_max: float, rotate: float) -> bool:
        """
        Spot Rotate the robot with odom. Blocking function
        :return : success
        """
        rospy.logwarn(f"Turn robot: {rotate:.2f} rad")

        _initial_tf = self.getOdom()
        if _initial_tf is None:
            return False

        # get the tf mat from odom to goal frame
        _goal_tf = self.apply2DTransform(_initial_tf, (0, 0, rotate))
        _pid = PID(self.k_p, self.k_i, self.k_d, v_w_min, v_w_max)

        prev_time = rospy.Time.now()
        while not rospy.is_shutdown():

            if self.is_cancel:
                self.error = 2
                return False

            if self.is_pause or self.obstacle_status:
                self.rate.sleep()
                continue

            _curr_tf = self.getOdom()
            if _curr_tf is None:
                self.error = 2
                return False

            dx, dy, dyaw = self.computeTfDiff(_curr_tf, _goal_tf)

            if abs(dyaw) < self.params.stop_yaw_diff:
                self.pubSpeedCmd()
                rospy.logwarn("Done with rotate robot")
                return True
            
            time_now = rospy.Time.now()
            dt = (time_now - prev_time).to_sec()
            angular_vel_pid = _pid.update(rotate, rotate - dyaw, dt)             
            sign = 1 if rotate > 0 else -1
            angular_vel = sign*angular_vel_pid
            
            self.pubSpeedCmd(angular_vel=angular_vel)
            prev_time = time_now
            rospy.sleep(self.sleep_period)
        exit(0)
    
    def moveWithOdom(self, min_speed: float, max_speed: float ,forward: float) -> bool:
        """
        Move robot in linear motion with Odom. Blocking function
        :return : success
        """

        rospy.sleep(3.0)

        rospy.loginfo(f"Move robot: {forward:.2f} m")

        _initial_tf = self.getOdom()
        if _initial_tf is None:
            return False

        # get the tf mat from odom to goal frame
        _goal_tf = self.apply2DTransform(_initial_tf, (forward, 0, 0))
        _pid = PID(self.k_p, self.k_i, self.k_d, min_speed, max_speed)

        # second mat
        prev_time = rospy.Time.now()
        while not rospy.is_shutdown():
            
            if self.is_cancel:
                self.error = 2
                return False
            
            if self.is_pause or self.obstacle_status:
                self.rate.sleep()
                continue

            _curr_tf = self.getOdom()
            if _curr_tf is None:
                self.error = 2
                return False

            dx, dy, dyaw = self.computeTfDiff(_curr_tf, _goal_tf)

            if abs(dx) < 0.02:
                self.pubSpeedCmd()
                rospy.logwarn("Done with move robot")
                return True
            
            time_now = rospy.Time.now()
            dt = (time_now - prev_time).to_sec()
            l_vel_pid = _pid.update(forward, forward - dx, dt)
            ang_vel = self.satProportionalFilter(dyaw, abs_max=0.05, factor=0.2)
            l_vel   = self.binFilter(dx, l_vel_pid)

            self.pubSpeedCmd(linear_vel=l_vel, angular_vel=ang_vel)
            prev_time = time_now
            rospy.sleep(self.sleep_period)
        exit(0)


    def get_2D_pose(self):
        """
        Take 2D Pose
        """
        try:
            trans = self.__tfBuffer.lookup_transform(
                self.params.map_frame_id,
                self.params.robot_base_frame_id,
                rospy.Time.now(), timeout=rospy.Duration(1.0))
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
        
            rx = trans.transform.rotation.x
            ry = trans.transform.rotation.y
            rz = trans.transform.rotation.z
            rw = trans.transform.rotation.w

            orientation = euler_from_quaternion([rx, ry, rz, rw])

            return x, y, orientation[2], rw

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logwarn(f"Failed lookup: {self.params.robot_base_frame_id}, from {self.params.map_frame_id}")
            return None
        
    
    def pubWaypointList(self):
        """Helper method to publish the waypoints that should be followed."""
        try:
            self.pub_pose_array.publish(self.toPoseArray(self.waypoints))
            return True
        except:
            return False
        
    
    # helper methods
    def toPoseArray(self, waypoints):
        """Publish waypoints as a pose array so that you can see them in rviz."""
        poses = PoseArray()
        poses.header.frame_id = self.params.map_frame_id
        poses.poses = [pose for pose in waypoints]
        return poses
    

    def sendMoveBaseGoal(self, pose: Pose):
        """Assemble and send a new goal to move_base"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.params.map_frame_id
        goal.target_pose.pose.position = pose.position
        goal.target_pose.pose.orientation = pose.orientation

        self.move_base_client.send_goal(goal)

    
    def navigateToGoal(self, pose):
        """
        Navigating robot to a goal on map
        """
        rospy.sleep(3.0)
        poses = Pose()
        poses.position.x = float(pose[0])
        poses.position.y = float(pose[1])
        poses.position.z = 0.0
        poses.orientation.x = 0.0
        poses.orientation.y = 0.0
        poses.orientation.z = float(pose[2])
        poses.orientation.w = float(pose[3])

        WARN(f"Starting go to goal({pose[0]:.4f}, {pose[1]:.4f})")

        self.sendMoveBaseGoal(poses)

        self.move_base_client.wait_for_result()

        if self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
            INFO("Goal is SUCCESS!")
            return True 
        else:
            ERROR("Goal is FAILED!")
            self.error = 2
            return False


    def navigateThroughGoals(self, poses):
        """
        Navigating robot through goal list on map
        """
        self.waypoints.clear()

        pose_arr = np.array(poses)

        amr_tf = self.get_2D_pose()
        x, y, rz, rw = amr_tf

        amr_pose = np.array([x, y, rz, rw])

        # Calculate distance between two arrays
        distances = np.linalg.norm(pose_arr - amr_pose, axis=1)

        # Find closest array in pose_arr respective to amr_pose
        closest_index = np.argmin(distances)
        try:
            closest_poses = pose_arr[(closest_index+1):]
        except:
            closest_poses = pose_arr[[poses[-1]]]
     
        for i in closest_poses:
            pose = Pose()
            pose.position.x = float(i[0])
            pose.position.y = float(i[1])
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = float(i[2])
            pose.orientation.w = float(i[3])
            
            self.waypoints.append(pose)
        
        WARN(f"Start moving robot through {len(closest_poses)} waypoints!")

        while not rospy.is_shutdown():

            if not self.waypoints:
                ERROR("No more waypoints to follow!")
                return False
            
            goal = self.waypoints[0]

            self.sendMoveBaseGoal(goal)

            if len(self.waypoints) == 1:
                self.move_base_client.wait_for_result()

                if self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
                    INFO("Goal is SUCCESS!")
                    return True
                else:
                    self.error = 2
                    return False 
            else:
                distance = 10
                while distance > self.params.distance_tolerance:

                    curr_pose = self.get_2D_pose()
                    x, y, _, _ = curr_pose

                    distance = math.sqrt(pow(goal.position.x - x, 2) + pow(goal.position.y - y, 2))

                    if (self.move_base_client.get_state() == GoalStatus.ABORTED
                        or self.move_base_client.get_state() == GoalStatus.PREEMPTED):
                        self.error = 2
                        return False
                    
                    self.rate.sleep()
                
                self.waypoints.pop(0)

            self.pubWaypointList()
            self.rate.sleep()

    
    def change_map(self, map_file):
        """
        Change map service
        """
        try:
            WARN("Waiting result from change_map server...")
            resp = self.change_map_client(map_file)
            
            status = resp.result

            if status != LoadMapResponse.RESULT_SUCCESS:
                ERROR('Change map request failed!')
                return False
            else:
                INFO('Change map request was successful!')
                time.sleep(1.0)
                return True
        
        except rospy.ServiceException as e:
            ERROR("Service call failed: %s"%e)

    
    def clearCostmap(self):
        """
        Clear all costmaps
        """
        try:
            WARN("Requesting clear all costmaps...")
            self.clear_costmap.call()
        
        except rospy.ServiceException as e:
            ERROR("Service call failed: %s"%e)

    
    def autoDocking(self, mode:int):
        """
        Auto docking action
        """
        goal = AutoDockingGoal()
        goal.mode = mode
        goal.custom_docking = False
        goal.rotate_to_dock = 0

        self.autodock_client.send_goal(goal)
        self.autodock_client.wait_for_result()

        if self.autodock_client.get_state() == GoalStatus.SUCCEEDED:
            WARN("Auto docking is complete!")
            return True
        else:
            ERROR("Auto docking is failed!")
            return False
    

    #===============> CAP HANG <================#
    def capHang(self):
        if (not self.navigateThroughGoals(self.cap_hang_waypoints)
            or not self.rotateWithOdom(self.params.min_angular_vel,
                                       self.params.max_angular_vel,
                                       math.pi)
            or not self.navigateToGoal(self.cap_hang_goal)
            or not self.rotateWithOdom(self.params.min_angular_vel,
                                       self.params.max_angular_vel,
                                       -math.pi/2)):
            return False
        
        if not self.autoDocking(self.MODE_CAPHANG):
            return False
        
        return True

    
    def mainRun(self, signal:Int16):
        # self.runOnce(True)
        self.turnOnBrake(False)

        count_testing = 0
        while True:
            if (self.navigateToGoal(self.test_lift_tp3)
                and self.navigateToGoal(self.test_lift_tp31)):
                count_testing += 1
                print(f"Testing in-out lift {count_testing}")

        if signal.data == 1:
            if not self.capHang():
                ERROR("Cap hang is failed!")
                self.resetAllVariables()
                return False
        else:
            pass
            
        INFO("AMR Navigation is complete!")
        self.resetAllVariables()
        return True    
        

if __name__== '__main__':
    try:
        autonav = AMRNavigation()
        WARN("%s is running!", rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
