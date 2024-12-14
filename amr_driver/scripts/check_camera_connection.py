#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import SetBool, Empty
from std_msgs.msg import Bool
import dynamic_reconfigure.client as dyncli


class CheckCameraConnection:
    def __init__(self):
        self.back_camera_status_ = False
        self.front_camera_status_ = False

        self.camera_finished_pub_ = rospy.Publisher("/camera_finished",
                                                    Bool, queue_size=1)

        rospy.Subscriber("/back_camera/camera_startup_finished", Bool, self.back_camera_finished_cb)
        rospy.Subscriber("/front_camera/camera_startup_finished", Bool, self.front_camera_finished_cb)
        
        self.reset_back_camera_ = rospy.ServiceProxy("/back_camera/realsense2_camera/reset", Empty)
        rospy.loginfo("/check_camera_connection: Connecting to /back_camera/realsense2_camera/reset service...")
        self.reset_back_camera_.wait_for_service()
        rospy.loginfo("/check_camera_connection: Connected to /back_camera/realsense2_camera/reset service.")

        self.reset_front_camera_ = rospy.ServiceProxy("/front_camera/realsense2_camera/reset", Empty)
        rospy.loginfo("/check_camera_connection: Connecting to /front_camera/realsense2_camera/reset service...")
        self.reset_front_camera_.wait_for_service()
        rospy.loginfo("/check_camera_connection: Connected to /front_camera/realsense2_camera/reset service.")

        self.enable_emitter_dyncli_ = dyncli.Client("/back_camera/stereo_module")

    def disable_back_camera_emitter(self):
        config = {'emitter_enabled': 0}
        self.enable_emitter_dyncli_.update_configuration(config)

    def back_camera_finished_cb(self, msg: Bool):
        if msg.data:
            try:
                rospy.wait_for_message("/back_camera/color/camera_info",
                                       CameraInfo, timeout=0.5)
                self.back_camera_status_ = True
            except Exception as ex:
                rospy.logwarn(ex)
                if not self.back_camera_status_:
                    rospy.logwarn("/check_camera_connection: Reseting back camera again...!")
                    self.reset_back_camera_.call()

            if self.back_camera_status_ and self.front_camera_status_:
                self.camera_finished_pub_.publish(True)
                self.disable_back_camera_emitter()

    def front_camera_finished_cb(self, msg: Bool):
        if msg.data:                    
            try:
                rospy.wait_for_message("/front_camera/color/camera_info",
                                    CameraInfo, timeout=0.5)
                self.front_camera_status_ = True
            except:
                if not self.front_camera_status_:
                    rospy.logwarn("/check_camera_connection: Reseting front camera again...!")
                    self.reset_front_camera_.call()
            
            if self.back_camera_status_ and self.front_camera_status_:
                self.camera_finished_pub_.publish(True)
                self.disable_back_camera_emitter()


if __name__ == "__main__":
    rospy.init_node("check_camera_connection")
    try:
        check_camera_connection = CheckCameraConnection()
        rospy.loginfo("%s node is running!", rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass