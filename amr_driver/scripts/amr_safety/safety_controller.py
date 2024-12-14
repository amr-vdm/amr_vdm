#!/usr/bin/env python3
import rospy
import dynamic_reconfigure.client as dc

from std_msgs.msg import Bool, Int16, Float32
from amr_msgs.msg import SliderSensorStamped, SafetyStatusStamped, SafetyStatus

IN = 1
OUT = 1

class SafetyController():

    def __init__(self):
        self.max_speed_ = rospy.get_param("~max_speed", 0.7)
        self.speed_at_warninglv1 = rospy.get_param("~speed_at_warninglv1", 0.55)
        self.speed_at_warninglv2 = rospy.get_param("~speed_at_warninglv2", 0.7)

        self.client_movebase_          = dc.Client("/move_base_node")
        self.global_costmap_footprint_ = dc.Client("/move_base_node/global_costmap")
        self.local_costmap_footprint_  = dc.Client("/move_base_node/local_costmap")

        # Move base dynamic reconfigure:
        self.timeout_obstacles_ = {'oscillation_timeout': 0.0}
        self.timeout_normal_ = {'oscillation_timeout': 15.0}

        # Footprint dynamic reconfigure:
        self.default_footprint_ = {'footprint': [[0.22,0.22],[-0.6,0.22],[-0.6,-0.22],[0.22,-0.22]]}
        self.big_footprint_     = {'footprint': [[0.22,0.22],[-0.92,0.22],[-0.92,-0.22],[0.22,-0.22]]}

        # Inflation radius dynamic reconfigure
        self.default_inflation_radius_ = {'inflation_radius': 0.65}
        self.big_inflation_radius_     = {'inflation_radius': 0.955}

        # Avariables:
        self.loop_freq_ = 10.0
        self.is_running_ = False
        self.is_pause_ = False
        self.is_turn_off_back_ = False
        self.is_turn_off_front_ = False
        self.is_turn_off_ultrasonic_ = False
        self.is_turn_off_front_depth_scan_ = False
        self.back_scanner_state_  = SafetyStatus.NORMAL
        self.front_scanner_state_ = SafetyStatus.NORMAL
        self.front_depth_scan_state_  = SafetyStatus.NORMAL
        self.ultrasonic_safety_state_ = SafetyStatus.NORMAL
        self.field_state_      = SafetyStatus.NORMAL
        self.prev_field_state_ = SafetyStatus.NORMAL
        
        # Publishers:
        self.protected_field_status_pub_ = rospy.Publisher("status_protected_field", Bool, queue_size=5)
        self.speed_at_field_pub_         = rospy.Publisher("/speed_at_field", Float32, queue_size=5)

        # Subscribers:
        rospy.Subscriber("front_scanner_status", SafetyStatusStamped, self.front_scanner_state_callback)
        rospy.Subscriber("back_scanner_status", SafetyStatusStamped, self.back_scanner_state_callback)
        rospy.Subscriber("front_depth_scanner_status", SafetyStatusStamped, self.front_depth_scanner_state_callback)
        rospy.Subscriber("ultrasonic_safety_status", SafetyStatusStamped, self.ultrasonic_state_callback)
        rospy.Subscriber("state_runonce_nav", Bool, self.runonce_callback)
        rospy.Subscriber('PAUSE_AMR', Bool, self.pause_callback)
        rospy.Subscriber("turn_off_back_safety",Bool,self.turn_off_back_safety_callback)
        rospy.Subscriber("turn_off_front_safety", Bool, self.turn_off_front_safety_callback)
        rospy.Subscriber("turn_off_ultrasonic_safety", Bool, self.turn_off_ultrasonic_safety_callback)
        rospy.Subscriber("turn_off_front_depth_safety", Bool, self.turn_off_front_depth_scan_safety_callback)
        rospy.Subscriber("slider_sensor_state", SliderSensorStamped, self.switch_footprint_callback)

    
    def switch_footprint_callback(self, msg:SliderSensorStamped):
        if msg.sensor_state.data:
            if msg.sensor_state.data[0] == IN:
                self.global_costmap_footprint_.update_configuration(self.default_footprint_)
                self.local_costmap_footprint_.update_configuration(self.default_footprint_)

            elif msg.sensor_state.data[1] == OUT:
                self.global_costmap_footprint_.update_configuration(self.big_footprint_)
                self.local_costmap_footprint_.update_configuration(self.big_footprint_) 

    def runonce_callback(self,msg: Bool):
        self.is_running_ = msg.data
        if not self.is_running_:
            self.protected_field_status_pub_.publish(False)

    def pause_callback(self,msg: Bool):
        self.is_pause_ = msg.data
        if self.is_pause_:
            self.protected_field_status_pub_.publish(False)

    def turn_off_back_safety_callback(self,msg: Bool):
        self.is_turn_off_back_ = msg.data
        if msg.data:
            self.back_scanner_state_ = SafetyStatus.NORMAL

    def turn_off_front_safety_callback(self, msg:Bool):
        self.is_turn_off_front_ = msg.data
        if msg.data:
            self.front_scanner_state_ = SafetyStatus.NORMAL

    def turn_off_front_depth_scan_safety_callback(self, msg:Bool):
        self.is_turn_off_front_depth_scan_ = msg.data
        if msg.data:
            self.front_depth_scan_state_ = SafetyStatus.NORMAL

    def turn_off_ultrasonic_safety_callback(self, msg:Bool):
        self.is_turn_off_ultrasonic_ = msg.data
        if msg.data:
            self.ultrasonic_safety_state_ = SafetyStatus.NORMAL

    def configure_timeout(self,config):
        if self.is_pause_:
            return
        self.client_movebase_.update_configuration(config)

    def back_scanner_state_callback(self,msg: SafetyStatusStamped):
        if self.is_turn_off_back_:
            return
        self.back_scanner_state_ = msg.safety_status.status

    def front_scanner_state_callback(self,msg: SafetyStatusStamped):
        if self.is_turn_off_front_:
            return
        self.front_scanner_state_ = msg.safety_status.status

    def front_depth_scanner_state_callback(self, msg: SafetyStatusStamped):
        if self.is_turn_off_front_depth_scan_:
            return
        self.front_depth_scan_state_ = msg.safety_status.status
    
    def ultrasonic_state_callback(self, msg:SafetyStatusStamped):
        if self.is_turn_off_ultrasonic_:
            return
        self.ultrasonic_safety_state_ = msg.safety_status.status

    def update_velocity(self, field):
        if field == SafetyStatus.WARNING_LV1:
            speed_limit = self.speed_at_warninglv1  # m/s
        elif field == SafetyStatus.WARNING_LV2:
            speed_limit = self.speed_at_warninglv2  # m/s
        elif field == SafetyStatus.NORMAL:
            speed_limit = self.max_speed_           # m/s

        msg = Float32()
        msg.data = speed_limit
        self.speed_at_field_pub_.publish(msg)

    def run(self):
        delay_time = 0.0
        while not rospy.is_shutdown():
            if not self.is_running_ or self.is_pause_:
                self.field_state_ = SafetyStatus.NORMAL
            else:
                if (self.front_scanner_state_ == SafetyStatus.PROTECTED
                    or self.back_scanner_state_ == SafetyStatus.PROTECTED
                    or self.ultrasonic_safety_state_ == SafetyStatus.PROTECTED
                    or self.front_depth_scan_state_ == SafetyStatus.PROTECTED):
                    
                    self.field_state_ = SafetyStatus.PROTECTED
                    delay_time = 0.0

                    if self.field_state_ != self.prev_field_state_:
                        self.protected_field_status_pub_.publish(True)
                        self.configure_timeout(self.timeout_obstacles_)
                else:
                    if (self.field_state_ == SafetyStatus.PROTECTED):
                        if delay_time >= 2.0:
                            self.protected_field_status_pub_.publish(False)
                            self.field_state_ = SafetyStatus.NORMAL
                            delay_time = 0.0
                        else: 
                            delay_time += 1/self.loop_freq_
                    
                    if (self.front_scanner_state_ != SafetyStatus.PROTECTED
                        and self.field_state_ != SafetyStatus.PROTECTED):
                        self.field_state_ = self.front_scanner_state_
                    
                    if (self.field_state_ != self.prev_field_state_):
                        delay_time = 0.0
                        self.configure_timeout(self.timeout_normal_)
                        self.update_velocity(self.front_scanner_state_)
                    
                self.prev_field_state_ = self.field_state_
            
            rospy.sleep(1/self.loop_freq_)

if __name__== '__main__':
    rospy.init_node("safety_controller")
    try:
        safety_controller = SafetyController()
        rospy.loginfo("%s node is running!", rospy.get_name())
        safety_controller.run()
        
    except rospy.ROSInterruptException:
        pass