#!/usr/bin/env python3
import rospy

from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from amr_msgs.msg import SafetyStatusStamped, SafetyStatus, SafetyZone


class UltrasonicSafety():

    def __init__(self):
        self.loop_freq_ = 5.0
        self.default_distance_   = 0.3
        self.small_distance_ = 0.075
        self.is_running_ = False
        self.turn_off_ultrasonic_safety_ = False
        self.left_ultrasonic_range_  = 0.0
        self.right_ultrasonic_range_ = 0.0
        self.obstacle_state_ = SafetyStatus.NORMAL
        self.prev_obstacle_state_ = SafetyStatus.NORMAL
        self.safety_zone_type_ = SafetyZone.BIG_ZONE

        # Publishers:
        self.ultrasonic_safety_status_pub_ = rospy.Publisher("ultrasonic_safety_status",
                                                             SafetyStatusStamped, queue_size=5)
        # Subscribers:
        rospy.Subscriber("left_ultrasonic/range", Range, self.left_range_callback)
        rospy.Subscriber("right_ultrasonic/range", Range, self.right_range_callback)
        rospy.Subscriber("safety_zone_type", SafetyZone, self.zone_type_callback)
        rospy.Subscriber("state_runonce_nav", Bool, self.runonce_callback)
        rospy.Subscriber("turn_off_ultrasonic_safety", Bool, self.turn_off_ultrasonic_callback)
    
    def turn_off_ultrasonic_callback(self, msg:Bool):
        self.turn_off_ultrasonic_safety_ = msg.data

    def runonce_callback(self, msg:Bool):
        self.is_running_ = msg.data

    def zone_type_callback(self, msg: SafetyZone):
        self.safety_zone_type_ = msg.zone
        msg_des = "big" if self.safety_zone_type_ == SafetyZone.BIG_ZONE else "small"
        rospy.logwarn(f"/ultrasonic_safety_status: Switched to ultrasonic {msg_des} safety zone")

    def left_range_callback(self, msg:Range):
        self.left_ultrasonic_range_ = msg.range

    def right_range_callback(self, msg:Range):
        self.right_ultrasonic_range_ = msg.range

    def run(self):
        while not rospy.is_shutdown():
            if (not self.is_running_ or self.turn_off_ultrasonic_safety_):
                self.obstacle_state_ = SafetyStatus.NORMAL
            else:
                if self.safety_zone_type_ == SafetyZone.SMALL_ZONE:
                    if (self.left_ultrasonic_range_ <= self.small_distance_ 
                        or self.right_ultrasonic_range_ <= self.small_distance_):
                        self.obstacle_state_ = SafetyStatus.PROTECTED

                    elif (self.left_ultrasonic_range_ > self.small_distance_ 
                          and self.right_ultrasonic_range_ > self.small_distance_):
                        self.obstacle_state_ = SafetyStatus.NORMAL       
                else:
                    if (self.left_ultrasonic_range_ <= self.default_distance_
                        or self.right_ultrasonic_range_ <= self.default_distance_):
                        self.obstacle_state_ = SafetyStatus.PROTECTED

                    elif (self.left_ultrasonic_range_ > self.default_distance_ 
                          and self.right_ultrasonic_range_ > self.default_distance_):
                        self.obstacle_state_ = SafetyStatus.NORMAL
                
                if self.obstacle_state_ != self.prev_obstacle_state_:
                    if self.obstacle_state_ == SafetyStatus.PROTECTED:
                        rospy.logwarn("/ultrasonic_safety_status: Detect obstacle!")
                    else:
                        rospy.loginfo("/ultrasonic_safety_status: No obstacle in range.")
                    
                    safety = SafetyStatusStamped()
                    safety.header.frame_id = "ultrasonic_link"
                    safety.header.stamp = rospy.Time.now()
                    safety.safety_status.status = self.obstacle_state_
                    self.ultrasonic_safety_status_pub_.publish(safety)
                
                self.prev_obstacle_state_ = self.obstacle_state_
            
            rospy.sleep(1/self.loop_freq_)    


if __name__== '__main__':
    rospy.init_node("ultrasonic_safety")
    try:
        ultrasonic_safety = UltrasonicSafety()
        rospy.loginfo("%s node is running!", rospy.get_name())
        ultrasonic_safety.run()

    except rospy.ROSInterruptException:
        pass
