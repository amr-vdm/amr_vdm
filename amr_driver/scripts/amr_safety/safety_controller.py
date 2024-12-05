#!/usr/bin/env python3
import rospy
import dynamic_reconfigure.client as dc

from std_msgs.msg import Bool, Int16, Float32
from amr_msgs.msg import CheckerSensorStateStamped, SafetyStatusStamped, SafetyStatus

NO_OBSTACLE = 0
OBSTACLE = 1

class SafetyController():

    def __init__(self):

        # Get params from server
        self.pub_frequency = rospy.get_param("~pub_frequency", 10)
        self.max_speed = rospy.get_param("~max_speed", 0.7)
        self.speed_after_protected_stop = rospy.get_param("~speed_after_protected_stop", 0.2)
        self.speed_warning_zone_lv1 = rospy.get_param("~speed_warning_zone_lv1", 0.3)
        self.speed_warning_zone_lv2 = rospy.get_param("~speed_warning_zone_lv2", 0.4)

        self.rate = rospy.Rate(self.pub_frequency)

        # Move base dynamic reconfigure:
        self.param_oscillation_timeout_obstacles = {'oscillation_timeout': 0.0}
        self.param_oscillation_timeout_normal = {'oscillation_timeout': 15.0}

        # DWB local planner dynamic reconfigure
        # self.DWB_slow_speed_zone_vel_1 = {'max_speed_xy': 0.3,'max_vel_x': 0.3,'max_vel_theta': 0.3}

        # TEB local planner dynamic reconfigure
        # self.TEB_slow_speed_zone_vel_1 = {'max_vel_x': 0.3, 'max_vel_x_backwards': 0.15, 'max_vel_theta': 0.3}

        # Footprint dynamic reconfigure:
        self.default_footprint = {'footprint': [[0.22,0.22],[-0.6,0.22],[-0.6,-0.22],[0.22,-0.22]]}
        self.big_footprint = {'footprint': [[0.22,0.22],[-0.92,0.22],[-0.92,-0.22],[0.22,-0.22]]}

        # Inflation radius dynamic reconfigure
        self.default_inflation_radius = {'inflation_radius': 0.65}
        self.big_inflation_radius = {'inflation_radius': 0.955}

        # Avariables:
        self.state_runonceNAV = False
        self.Pause_AMR_state = False
        self.is_turn_off_back = False
        self.is_turn_off_front = False
        self.is_turn_off_ultrasonic = False
        self.is_turn_off_front_depth_scan = False
        self.status_field_back = SafetyStatus.NORMAL
        self.status_field_front = SafetyStatus.NORMAL
        self.front_depth_scan_status = SafetyStatus.NORMAL
        self.ultrasonic_safety_status = SafetyStatus.NORMAL
        self.status_field = SafetyStatus.NORMAL
        self.footprint = 0
        
        # Publishers:
        self.pub_status_protected_field = rospy.Publisher("status_protected_field", Bool, queue_size=5)
        self.pub_speed_limit_safety = rospy.Publisher("/speed_limit_safety", Float32, queue_size=5)

        # Subscribers:
        rospy.Subscriber("front_scanner_status", SafetyStatusStamped, self.frontFieldStatusCb)
        rospy.Subscriber("back_scanner_status", SafetyStatusStamped, self.backFieldStatusCb)
        rospy.Subscriber("front_depth_scanner_status", SafetyStatusStamped, self.frontDepthScanStatusCb)
        rospy.Subscriber("ultrasonic_safety_status", SafetyStatusStamped, self.ultrasonicSafetyCb)
        rospy.Subscriber("state_runonce_nav", Bool, self.runOnceStateCb)
        rospy.Subscriber('PAUSE_AMR', Bool, self.pauseAMRCb)
        rospy.Subscriber("turn_off_back_safety",Bool,self.turnOffBackSafetyScannerCb)
        rospy.Subscriber("turn_off_front_safety", Bool, self.turnOffFrontSafetyScannerCB)
        rospy.Subscriber("turn_off_ultrasonic_safety", Bool, self.turnOffUltrasonicSafetyCB)
        rospy.Subscriber("turn_off_front_depth_safety", Bool, self.turnOffFrontDepthScanCb)
        rospy.Subscriber("slider_sensor_state", CheckerSensorStateStamped, self.switchFootprintCb)

    
    def switchFootprintCb(self, msg:CheckerSensorStateStamped):

        global_footprint_client = dc.Client("/move_base_node/global_costmap")
        local_footprint_client = dc.Client("/move_base_node/local_costmap")
        global_ir_client = dc.Client("/move_base_node/global_costmap/inflation")
        local_ir_client = dc.Client("/move_base_node/local_costmap/inflation")

        if msg.sensor_state.data:
            if msg.sensor_state.data[0] == 1:
                self.footprint = 1
            elif msg.sensor_state.data[1] == 1:
                self.footprint = 2
            else:
                return

        if self.footprint == 1:
            global_footprint_client.update_configuration(self.default_footprint)
            # global_ir_client.update_configuration(self.default_inflation_radius)
            local_footprint_client.update_configuration(self.default_footprint)
            # local_ir_client.update_configuration(self.default_inflation_radius)
        else:
            global_footprint_client.update_configuration(self.big_footprint)
            # global_ir_client.update_configuration(self.big_inflation_radius)
            local_footprint_client.update_configuration(self.big_footprint)
            # local_ir_client.update_configuration(self.big_inflation_radius)
            

    def runOnceStateCb(self,msg: Bool):
        self.state_runonceNAV = msg.data
        if not self.state_runonceNAV:
            self.pub_status_protected_field.publish(False)


    def pauseAMRCb(self,msg: Bool):
        self.Pause_AMR_state = msg.data
        if self.Pause_AMR_state:
            self.pub_status_protected_field.publish(False)


    def turnOffBackSafetyScannerCb(self,msg: Bool):
        self.is_turn_off_back = msg.data
        if msg.data:
            self.status_field_back = SafetyStatus.NORMAL

    def turnOffFrontSafetyScannerCB(self, msg:Bool):
        self.is_turn_off_front = msg.data
        if msg.data:
            self.status_field_front = SafetyStatus.NORMAL

    def turnOffFrontDepthScanCb(self, msg:Bool):
        self.is_turn_off_front_depth_scan = msg.data
        if msg.data:
            self.front_depth_scan_status = SafetyStatus.NORMAL


    def turnOffUltrasonicSafetyCB(self, msg:Bool):
        self.is_turn_off_ultrasonic = msg.data
        if msg.data:
            self.ultrasonic_safety_status = SafetyStatus.NORMAL

    def configureOscillationTimeOut(self,config):
        if self.Pause_AMR_state:
            return
        self.client_movebase = dc.Client("/move_base_node")
        self.client_movebase.update_configuration(config)

    def backFieldStatusCb(self,msg: SafetyStatusStamped):
        if self.is_turn_off_back:
            return
        self.status_field_back = msg.safety_status.status

    def frontFieldStatusCb(self,msg: SafetyStatusStamped):
        if self.is_turn_off_front:
            return
        self.status_field_front = msg.safety_status.status

    def frontDepthScanStatusCb(self, msg: SafetyStatusStamped):
        if self.is_turn_off_front_depth_scan:
            return
        self.front_depth_scan_status = msg.safety_status.status
    
    def ultrasonicSafetyCb(self, msg:SafetyStatusStamped):
        if self.is_turn_off_ultrasonic:
            return
        self.ultrasonic_safety_status = msg.safety_status.status


    def AMRFieldStatus(self):
        pulse_2s = 0
        while not rospy.is_shutdown():
            if (not self.state_runonceNAV
                or self.Pause_AMR_state):
                if self.status_field != SafetyStatus.NORMAL:
                    self.status_field = SafetyStatus.NORMAL
                    self.pub_speed_limit_safety.publish(self.max_speed)
                    self.pub_status_protected_field.publish(False)
            # elif self.Pause_AMR_state:
            #     if self.status_field != SafetyStatus.NORMAL:
            #         self.status_field = SafetyStatus.NORMAL
            #         self.pub_speed_limit_safety.publish(self.max_speed)
            else:
                if (self.status_field_front == SafetyStatus.PROTECTED
                    or self.status_field_back == SafetyStatus.PROTECTED
                    or self.ultrasonic_safety_status == SafetyStatus.PROTECTED
                    or self.front_depth_scan_status == SafetyStatus.PROTECTED):
                    pulse_2s = 0
                    if self.status_field != SafetyStatus.PROTECTED:
                        rospy.logwarn(f"{rospy.get_name()}: have obstacle in protected zone, robot stop!")
                        self.pub_status_protected_field.publish(True)
                        self.configureOscillationTimeOut(self.param_oscillation_timeout_obstacles)
                        self.status_field = SafetyStatus.PROTECTED
                else:
                    if (self.status_field == SafetyStatus.PROTECTED):
                        if pulse_2s == 20 :
                            rospy.logwarn(f"{rospy.get_name()}: no obstacle in protected zone,robot will run after 2s!")
                            self.pub_status_protected_field.publish(False)
                            self.configureOscillationTimeOut(self.param_oscillation_timeout_normal)
                            self.pub_speed_limit_safety.publish(self.speed_after_protected_stop)
                            self.status_field = SafetyStatus.WARNING_LV1
                            pulse_2s = 0
                        else: 
                            pulse_2s += 1
                
                    elif (self.status_field_front == SafetyStatus.WARNING_LV1
                          or self.status_field_back == SafetyStatus.WARNING_LV1):
                        pulse_2s = 0
                        if self.status_field != SafetyStatus.WARNING_LV1:
                            rospy.logwarn(f"{rospy.get_name()}: obstacle in warning zone LV1, " 
                                          f"robot will reduce speed ({self.speed_warning_zone_lv1}m/s)!")
                            # self.pub_status_protected_field.publish(False)
                            self.configureOscillationTimeOut(self.param_oscillation_timeout_normal)
                            self.pub_speed_limit_safety.publish(self.speed_warning_zone_lv1)
                            # self.configureVelocity(self.TEB_slow_speed_zone_vel_1)
                            self.status_field = SafetyStatus.WARNING_LV1

                    elif (self.status_field_front == SafetyStatus.WARNING_LV2
                          or self.status_field_back == SafetyStatus.WARNING_LV2):
                        pulse_2s = 0
                        if self.status_field != SafetyStatus.WARNING_LV2:
                            rospy.logwarn(f"{rospy.get_name()}: obstacle in warning zone LV2 " 
                                          f"robot will reduce speed ({self.speed_warning_zone_lv2}m/s)!")
                            # self.pub_status_protected_field.publish(False)
                            self.configureOscillationTimeOut(self.param_oscillation_timeout_normal)
                            self.pub_speed_limit_safety.publish(self.speed_warning_zone_lv2)
                            # self.configureVelocity(self.TEB_slow_speed_zone_vel_2)
                            self.status_field = SafetyStatus.WARNING_LV2
                        
                    elif self.status_field != SafetyStatus.NORMAL:
                        if pulse_2s == 10:
                            rospy.logwarn(f"{rospy.get_name()}: no obstacle, robot will run with max speed!")
                            # self.pub_status_protected_field.publish(False)
                            self.configureOscillationTimeOut(self.param_oscillation_timeout_normal)
                            self.pub_speed_limit_safety.publish(self.max_speed)
                            self.status_field = SafetyStatus.NORMAL
                        else: pulse_2s += 1
            self.rate.sleep()    


if __name__== '__main__':
    rospy.init_node("safety_controller")
    try:
        safety_controller = SafetyController()
        rospy.loginfo("%s node is running!", rospy.get_name())
        safety_controller.AMRFieldStatus()
        
    except rospy.ROSInterruptException:
        pass