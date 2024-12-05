#!/usr/bin/env python3

import rospy
import tf2_ros

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class PoseEstimation():

    def __init__(self):
        # Listen to Transfromation
        self.__tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.__tf_listener = tf2_ros.TransformListener(self.__tfBuffer)

        self.frame_id = "map"
        self.base_frame_id = "base_footprint"

        self.pose = Pose()
        self.poseFromInitial = None

        # Publishers:
        self.pub_initial_pose = rospy.Publisher("/initialpose",
                                                PoseWithCovarianceStamped, queue_size=5)
        # Variables:
        self.state_flag = True
        self.map_is_ready = False
        self.timer = 0  
        self.set_position = 0
        self.is_manual_pose_ = False
    
        rospy.Subscriber("mobile_base_controller/odom", Odometry, self.odomCb)
        rospy.Subscriber("/initialposeAMR", PoseWithCovarianceStamped, self.initialPoseAMRCb)


    def get2DPose(self):
        """
        Take 2D Pose
        """
        try: 
            trans = self.__tfBuffer.lookup_transform(
                self.frame_id,
                self.base_frame_id,
                rospy.Time(0), timeout=rospy.Duration(0.1))
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            rz = trans.transform.rotation.z
            rw = trans.transform.rotation.w

            return x, y, rz, rw
        
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logwarn(f"Failed lookup: {self.base_frame_id}, from {self.frame_id}")
            return None

    
    def initialPoseAMRCb(self, msg: PoseWithCovarianceStamped):
        self.map_is_ready = True
        self.poseFromInitial = msg.pose.pose
        self.setPoseEstimation(self.poseFromInitial)

    def setPoseEstimation(self, pose: Pose):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = pose
        msg.header.frame_id = self.frame_id

        self.pub_initial_pose.publish(msg)    

    def odomCb(self, msg: Odometry):
        if not self.map_is_ready:
            return

        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z

        if (abs(linear_vel) <= 0.01 and abs(angular_vel) <= 0.01):
            if not self.state_flag:
                pose = self.get2DPose()
                self.pose.position.x = pose[0]
                self.pose.position.y = pose[1]
                self.pose.orientation.z = pose[2]
                self.pose.orientation.w = pose[3]

                rospy.loginfo("Take the robot's current position")
                self.setPoseEstimation(self.pose)
                self.state_flag = True
            
            if self.timer >= 100:
                if self.poseFromInitial is not None:
                    rospy.logwarn("Take initial position from topic /initialposeAMR!")
                    self.pose = self.poseFromInitial
                    self.poseFromInitial = None
                self.setPoseEstimation(self.pose)
                self.timer = 0
            
            else:
                self.timer += 1
        else:
            self.poseFromInitial = None
            self.state_flag = False
            self.timer = 0
    

if __name__ == "__main__":
    rospy.init_node("pose_estimation")
    try:
        pose_estimation = PoseEstimation()
        rospy.loginfo("%s node is running!", rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
