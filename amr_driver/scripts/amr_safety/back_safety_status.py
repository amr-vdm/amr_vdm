#!/usr/bin/env python3
import rospy

from sick_safetyscanners.msg import OutputPathsMsg
from std_msgs.msg import Int16, Bool
from amr_msgs.msg import SafetyStatusStamped, SafetyStatus


class BackScannerSafety():

    def __init__(self):
        # Params:
        self.frame_id = rospy.get_param("~frame_id", "back_laser_link")

        # Publishers:
        self.pub_back_scanner_status  = rospy.Publisher("back_scanner_status", SafetyStatusStamped, queue_size=5)

        # Variables:
        self.is_turn_off_back_scanner = False
        self.prev_obstacle_state = SafetyStatus.NORMAL 

        # Subscribers:
        rospy.Subscriber("/sick_safetyscanners_back/output_paths", OutputPathsMsg, self.backSafetyStatusCb)
        rospy.Subscriber("turn_off_back_safety", Bool, self.turnOffBackSafetyScannerCb)
    

    def turnOffBackSafetyScannerCb(self, msg:Bool):
        self.is_turn_off_back_scanner = msg.data


    def backSafetyStatusCb(self, msg: OutputPathsMsg):
        
        if self.is_turn_off_back_scanner:
            return
        
        if not msg.status[0]:
            obstacle_state = SafetyStatus.PROTECTED
        elif not msg.status[1]:
            obstacle_state = SafetyStatus.WARNING_LV1
        else:
            obstacle_state = SafetyStatus.NORMAL
        
        if obstacle_state != self.prev_obstacle_state:
            safety = SafetyStatusStamped()
            safety.header.frame_id = self.frame_id
            safety.header.stamp = rospy.Time.now()
            safety.safety_status.status = obstacle_state
            self.pub_back_scanner_status.publish(safety)
            self.prev_obstacle_state = obstacle_state


if __name__== '__main__':
    rospy.init_node("back_scanner_safety_status")
    try:
        back_scanner_safety = BackScannerSafety()
        rospy.loginfo("%s node is running!", rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
