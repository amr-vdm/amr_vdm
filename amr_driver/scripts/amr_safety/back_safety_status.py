#!/usr/bin/env python3
import rospy

from sick_safetyscanners.msg import OutputPathsMsg
from std_msgs.msg import Bool
from amr_msgs.msg import SafetyStatusStamped, SafetyStatus

class BackScannerSafety():

    def __init__(self):
        self.is_running_  = False
        self.is_pause_    = False
        self.is_turn_off_ = False
        self.prev_obstacle_state_ = SafetyStatus.NORMAL

        # Publishers:
        self.back_scanner_status_pub_ = rospy.Publisher("back_scanner_status", SafetyStatusStamped, queue_size=5)

        # Subscribers:
        rospy.Subscriber("/sick_safetyscanners_back/output_paths", OutputPathsMsg, self.back_safety_state_callback)
        rospy.Subscriber("turn_off_back_safety", Bool, self.turn_off_back_safety_callback)
        rospy.Subscriber("state_runonce_nav", Bool, self.runonce_callback)
        rospy.Subscriber('PAUSE_AMR', Bool, self.pause_callback)

    def pause_callback(self,msg: Bool):
        self.is_pause_ = msg.data
    
    def runonce_callback(self,msg: Bool):
        self.is_running_ = msg.data

    def turn_off_back_safety_callback(self, msg:Bool):
        self.is_turn_off_ = msg.data
        if msg.data:
            rospy.loginfo("/back_safety_status: Turn off back safety scanner!")

    def back_safety_state_callback(self, msg: OutputPathsMsg):
        
        if self.is_turn_off_ or not self.is_running_:
            return
        
        if not msg.status[0]:
            obstacle_state = SafetyStatus.PROTECTED
        elif not msg.status[1]:
            obstacle_state = SafetyStatus.WARNING_LV1
        else:
            obstacle_state = SafetyStatus.NORMAL
        
        if obstacle_state != self.prev_obstacle_state_:
            if not self.is_pause_:
                if obstacle_state == SafetyStatus.PROTECTED:
                    rospy.logwarn("/back_safety_status: Detect obstacle!")            
                else:
                    rospy.loginfo("/back_safety_status: No obstacle in field.")

            safety = SafetyStatusStamped()
            safety.header.frame_id = "back_laser_link"
            safety.header.stamp = rospy.Time.now()
            safety.safety_status.status = obstacle_state
            self.back_scanner_status_pub_.publish(safety)
            self.prev_obstacle_state_ = obstacle_state


if __name__== '__main__':
    rospy.init_node("back_scanner_safety_status")
    try:
        back_scanner_safety = BackScannerSafety()
        rospy.loginfo("%s node is running!", rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
