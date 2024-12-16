#!/usr/bin/env python3
import rospy

from sick_safetyscanners.msg import OutputPathsMsg
from std_msgs.msg import Int16, Bool
from amr_msgs.msg import SafetyStatusStamped, SafetyStatus


class FrontScannerSafety():

    def __init__(self):
        self.prev_obstacle_state_ = SafetyStatus.NORMAL
        self.is_turn_off_ = False
        self.is_running_  = False
        self.is_pause_    = False
        
        # Publishers:
        self.front_scanner_status_pub_ = rospy.Publisher("front_scanner_status", SafetyStatusStamped, queue_size=5)

        # Subscribers:
        rospy.Subscriber("/sick_safetyscanners_front/output_paths", OutputPathsMsg, self.front_safety_state_callback)
        rospy.Subscriber("turn_off_front_safety", Bool, self.turn_off_front_safety_callback)
        rospy.Subscriber("state_runonce_nav", Bool, self.runonce_callback)
        rospy.Subscriber('PAUSE_AMR', Bool, self.pause_callback)

    def pause_callback(self,msg: Bool):
        self.is_pause_ = msg.data

    def runonce_callback(self,msg: Bool):
        self.is_running_ = msg.data

    def turn_off_front_safety_callback(self, msg:Bool):
        self.is_turn_off_ = msg.data
        if msg.data:
            rospy.loginfo("/front_safety_status: Turn off front safety scanner!")

    def front_safety_state_callback(self, msg: OutputPathsMsg):

        if self.is_turn_off_ or not self.is_running_:
            return
        
        if not msg.status[0]:
            obstacle_state = SafetyStatus.PROTECTED
        elif not msg.status[1]:
            obstacle_state = SafetyStatus.WARNING_LV1
        elif not msg.status[2]:
            obstacle_state = SafetyStatus.WARNING_LV2
        else:
            obstacle_state = SafetyStatus.NORMAL

        if obstacle_state != self.prev_obstacle_state_:
            if not self.is_pause_:
                if obstacle_state == SafetyStatus.PROTECTED:
                    rospy.logwarn("/front_safety_status: Detect obstacle!")
                    
                elif obstacle_state == SafetyStatus.NORMAL:
                    rospy.loginfo("/front_safety_status: No obstacle in field.")

            safety = SafetyStatusStamped()
            safety.header.frame_id = "front_laser_link"
            safety.header.stamp = rospy.Time.now()
            safety.safety_status.status = obstacle_state
            self.front_scanner_status_pub_.publish(safety)
            self.prev_obstacle_state_ = obstacle_state


if __name__== '__main__':
    rospy.init_node("front_scanner_safety_status")
    try:
        front_scanner_safety = FrontScannerSafety()
        rospy.loginfo("%s node is running!", rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
