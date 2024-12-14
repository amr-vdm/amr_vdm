#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int16, Bool
from amr_msgs.msg import SafetyStatusStamped, SafetyStatus

NO_OBSTACLE = 0
OBSTACLE = 1

class FrontDepthScannerSafety():

    def __init__(self):
        self.prev_obstacle_state_ = SafetyStatus.NORMAL
        self.is_turn_off_ = False
        self.is_running_  = False
        
        # Publishers:
        self.front_depth_scanner_status_pub_ = rospy.Publisher("front_depth_scanner_status",
                                                              SafetyStatusStamped, queue_size=5)
        # Subscribers:
        rospy.Subscriber("/front_camera/depth_scan/status", Int16, self.front_depth_scan_callback)
        rospy.Subscriber("turn_off_front_depth_safety", Bool, self.turn_off_front_depth_scanner)
        rospy.Subscriber("state_runonce_nav", Bool, self.runonce_callback)
    
    def runonce_callback(self,msg: Bool):
        self.is_running_ = msg.data

    def turn_off_front_depth_scanner(self, msg:Bool):
        self.is_turn_off_ = msg.data
        if msg.data:
            rospy.loginfo("/front_depth_scan_status: Turn off front depth scanner!")

    def front_depth_scan_callback(self, msg: Int16):

        if self.is_turn_off_ or not self.is_running_:
            return
        
        if not msg.data:
            obstacle_state = SafetyStatus.NORMAL
        else:
            obstacle_state = SafetyStatus.PROTECTED

        if obstacle_state != self.prev_obstacle_state_:

            if obstacle_state == SafetyStatus.PROTECTED:
                rospy.logwarn("/front_depth_scan_status: Detect obstacle!")            
            else:
                rospy.loginfo("/front_depth_scan_status: No obstacle in field.")

            safety = SafetyStatusStamped()
            safety.header.frame_id = "front_camera_depth_frame"
            safety.header.stamp = rospy.Time.now()
            safety.safety_status.status = obstacle_state
            self.front_depth_scanner_status_pub_.publish(safety)
            self.prev_obstacle_state_ = obstacle_state


if __name__== '__main__':
    rospy.init_node("front_depth_scanner_safety_status")
    try:
        front_depth_scanner_safety = FrontDepthScannerSafety()
        rospy.loginfo("%s node is running!", rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass