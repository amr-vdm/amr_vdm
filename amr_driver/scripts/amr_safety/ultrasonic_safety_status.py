#!/usr/bin/env python3
import rospy

from sensor_msgs.msg import Range
from std_msgs.msg import Int16, Bool
from amr_msgs.msg import SafetyStatusStamped, SafetyStatus, SafetyZone


class UltrasonicSafety():

    def __init__(self):
        
        # Params
        self.big_distance   = 0.3
        self.small_distance = 0.075

        self.rate = rospy.Rate(50)

        # Publishers:
        self.pub_ultrasonic_safety_status  = rospy.Publisher("ultrasonic_safety_status",
                                                             SafetyStatusStamped, queue_size=5)

        # Variables:
        self.is_run_once = False
        self.is_turn_off_ultrasonic_safety = False
        self.left_obstacle_state = 0
        self.right_obstacle_state = 0
        self.prev_obstacle_state = SafetyStatus.NORMAL
        self.safety_zone_type = SafetyZone.BIG_ZONE            # [0: big zone, 1: small zone]

        # Subscribers:
        rospy.Subscriber("left_ultrasonic/range", Range, self.leftUltrasonicSensorCb)
        rospy.Subscriber("right_ultrasonic/range", Range, self.rightUltrasonicSensorCb)
        rospy.Subscriber("safety_zone_type", SafetyZone, self.safetyZoneTypeCb)
        rospy.Subscriber("state_runonce_nav", Bool, self.runOnceStateCb)
        rospy.Subscriber("turn_off_ultrasonic_safety", Bool, self.turnOffUltrasonicSafetyCb)

    
    def turnOffUltrasonicSafetyCb(self, msg:Bool):
        self.is_turn_off_ultrasonic_safety = msg.data


    def runOnceStateCb(self, msg:Bool):
        self.is_run_once = msg.data


    def safetyZoneTypeCb(self, msg: SafetyZone):
        self.safety_zone_type = msg.zone
        msg_des = "big" if self.safety_zone_type == SafetyZone.BIG_ZONE else "small"
        rospy.logwarn(f"Switched to ultrasonic {msg_des} safety zone")


    def leftUltrasonicSensorCb(self, msg:Range):
        self.left_obstacle_state = msg.range


    def rightUltrasonicSensorCb(self, msg:Range):
        self.right_obstacle_state = msg.range


    def run(self):
        while not rospy.is_shutdown():
            if (not self.is_run_once or
                self.is_turn_off_ultrasonic_safety):
                pass
            else:   
                if (self.safety_zone_type == SafetyZone.SMALL_ZONE):
                    if (self.left_obstacle_state <= self.small_distance 
                        or self.right_obstacle_state <= self.small_distance):
                        obstacle_state = SafetyStatus.PROTECTED
                    else:
                        obstacle_state = SafetyStatus.NORMAL
                elif (self.safety_zone_type == SafetyZone.BIG_ZONE):
                    if (self.left_obstacle_state <= self.big_distance
                        or self.right_obstacle_state <= self.big_distance):
                        obstacle_state = SafetyStatus.PROTECTED
                    else:
                        obstacle_state = SafetyStatus.NORMAL
                
                if obstacle_state != self.prev_obstacle_state:
                    safety = SafetyStatusStamped()
                    safety.header.stamp = rospy.Time.now()
                    safety.safety_status.status = obstacle_state
                    self.pub_ultrasonic_safety_status.publish(safety)
                    self.prev_obstacle_state = obstacle_state
            self.rate.sleep()    


if __name__== '__main__':
    rospy.init_node("ultrasonic_safety")
    try:
        ultrasonic_safety = UltrasonicSafety()
        rospy.loginfo("%s node is running!", rospy.get_name())
        ultrasonic_safety.run()

    except rospy.ROSInterruptException:
        pass
