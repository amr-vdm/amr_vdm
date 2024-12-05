#!/usr/bin/env python3
import rospy

from sick_safetyscanners.msg import OutputPathsMsg
from std_msgs.msg import Int16, Bool
from amr_msgs.msg import SafetyStatusStamped, SafetyStatus


class FrontScannerSafety():

    def __init__(self):
        # Params:
        self.frame_id = rospy.get_param("~frame_id", "front_laser_link")

        # Publishers:
        self.pub_front_scanner_status = rospy.Publisher("front_scanner_status", SafetyStatusStamped, queue_size=5)

        # Variables:
        self.prev_obstacle_state = SafetyStatus.NORMAL
        self.is_turn_off_front_scanner = False

        # Subscribers:
        rospy.Subscriber("/sick_safetyscanners_front/output_paths", OutputPathsMsg, self.frontScannerStatusCb)
        rospy.Subscriber("turn_off_front_safety", Bool, self.turnOffFrontSafetyScannerCb)


    def turnOffFrontSafetyScannerCb(self, msg:Bool):
        self.is_turn_off_front_scanner = msg.data

    def frontScannerStatusCb(self, msg: OutputPathsMsg):

        if self.is_turn_off_front_scanner:
            return
        
        if not msg.status[0]:
            obstacle_state = SafetyStatus.PROTECTED
        elif not msg.status[1]:
            obstacle_state = SafetyStatus.WARNING_LV1
        elif not msg.status[2]:
            obstacle_state = SafetyStatus.WARNING_LV2
        else:
            obstacle_state = SafetyStatus.NORMAL

        if obstacle_state != self.prev_obstacle_state:
            safety = SafetyStatusStamped()
            safety.header.frame_id = self.frame_id
            safety.header.stamp = rospy.Time.now()
            safety.safety_status.status = obstacle_state
            self.pub_front_scanner_status.publish(safety)
            self.prev_obstacle_state = obstacle_state


if __name__== '__main__':
    rospy.init_node("front_scanner_safety_status")
    try:
        front_scanner_safety = FrontScannerSafety()
        rospy.loginfo("%s node is running!", rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
