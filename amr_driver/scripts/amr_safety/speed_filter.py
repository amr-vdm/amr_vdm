#!/usr/bin/env python3
import rospy
import math
import dynamic_reconfigure.client as dc

from std_msgs.msg import Bool,Int16, Float32

class SpeedFilter():

    def __init__(self):

        # Get params from server
        self.pub_frequency = rospy.get_param("~pub_frequency", 5)
        self.max_speed = rospy.get_param("~max_speed", 0.7)

        self.rate = rospy.Rate(self.pub_frequency)

        # self.client_velocity = dc.Client("/move_base_node/RotationShimController/DWBController")
        # self.client_velocity = dc.Client("/move_base_node/RotationShimController/VectorPursuitController")
        self.client_velocity = dc.Client("/move_base_node/RotationShimController/TebLocalPlannerROS")
        
        # Move base dynamic reconfigure:

        # DWB local planner dynamic reconfigure
        # self.DWB_normal_vel = {'max_speed_xy': 0.7,'max_vel_x': 0.7}

        # TEB local planner dynamic reconfigure
        # self.TEB_normal_vel = {'max_vel_x': 0.7, 'max_vel_x_backwards': 0.25, 'max_vel_theta': 0.7}

        # Avariables:
        # self.state_runonceNAV = False
        # self.Pause_AMR_state = False
        self.speed_limit_lane = self.max_speed
        self.speed_limit_safety = self.max_speed
        self.current_speed = self.max_speed

        # Subscribers:
        rospy.Subscriber("speed_limit_lane", Float32, self.speed_limit_lane_cb)
        rospy.Subscriber("speed_limit_safety", Float32, self.speed_limit_safety_cb)
        # rospy.Subscriber("state_runonce_nav", Bool, self.runOnceStateCb)
        # rospy.Subscriber('PAUSE_AMR', Bool, self.pauseAMRCb)

    # def runOnceStateCb(self,msg: Bool):
    #     self.state_runonceNAV = msg.data

    # def pauseAMRCb(self,msg: Bool):
    #     self.Pause_AMR_state = msg.data



    def speed_limit_lane_cb(self, msg: Float32):
        self.speed_limit_lane = round(msg.data,2)
        # if (self.speed_limit_lane <= self.max_speed
        #     and self.speed_limit_lane < self.speed_limit_safety):
        #     self.configureVelocity(self.speed_limit_lane)
        
        # elif self.speed_limit_lane > self.max_speed:
        #     self.speed_limit_lane = self.max_speed

    def speed_limit_safety_cb(self, msg: Float32):
        self.speed_limit_safety = round(msg.data,2)
        # if (self.speed_limit_safety <= self.max_speed
        #     and self.speed_limit_safety < self.speed_limit_lane):
        #     self.configureVelocity(self.speed_limit_safety)
        
        # elif self.speed_limit_safety > self.max_speed:
        #     self.speed_limit_safety = self.max_speed

    def configureVelocity(self,speed: float):
        # if self.Pause_AMR_state:
        #     return
        acc_speed = speed * 2 + 0.1
        acc_theta = speed * 2 + 0.6
        DWB_vel = {'max_speed_xy': speed,'max_vel_x': speed,'max_vel_theta': 1.2*speed,
                   'acc_lim_x': acc_speed, 'decel_lim_x': -acc_speed,
                   'acc_lim_theta': acc_theta,'decel_lim_theta': -acc_theta}
        # DWB_vel = {'max_speed_xy': speed,'max_vel_x': speed}
        VPP_vel = {'desired_linear_vel': speed}
        TEB_vel = {'max_vel_x': speed}
        # self.client_velocity.update_configuration(DWB_vel)
        # self.client_velocity.update_configuration(VPP_vel)
        self.client_velocity.update_configuration(TEB_vel)

    def run(self):
        while not rospy.is_shutdown():
            speed = min(self.speed_limit_lane, self.speed_limit_safety, self.max_speed)
            if self.current_speed != speed:
                self.current_speed = speed
                self.configureVelocity(self.current_speed)
                print('New speed configure: ', self.current_speed)
            self.rate.sleep()

if __name__== '__main__':
    rospy.init_node("speed_filter")
    try:
        speed_filter = SpeedFilter()
        rospy.loginfo("%s node is running!", rospy.get_name())
        speed_filter.run()
        
    except rospy.ROSInterruptException:
        pass