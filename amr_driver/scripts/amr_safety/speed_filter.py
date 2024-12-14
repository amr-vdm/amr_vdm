#!/usr/bin/env python3
import rospy
import dynamic_reconfigure.client as dc

from std_msgs.msg import Float32

class SpeedFilter():

    def __init__(self):
        self.max_speed_ = rospy.get_param("~max_speed", 0.7)

        # self.client_velocity = dc.Client("/move_base_node/RotationShimController/DWBController")
        # self.client_velocity = dc.Client("/move_base_node/RotationShimController/VectorPursuitController")
        self.client_velocity = dc.Client("/move_base_node/RotationShimController/TebLocalPlannerROS")

        self.loop_freq_ = 5.0
        self.speed_limit_lane_ = self.max_speed_
        self.speed_at_field_ = self.max_speed_
        self.current_speed_ = self.max_speed_

        # Subscribers:
        rospy.Subscriber("speed_limit_lane", Float32, self.speed_limit_lane_callback)
        rospy.Subscriber("speed_at_field", Float32, self.speed_at_field_callback)

    def speed_limit_lane_callback(self, msg: Float32):
        self.speed_limit_lane_ = round(msg.data,2)

    def speed_at_field_callback(self, msg: Float32):
        self.speed_at_field_ = round(msg.data,2)

    def update_velocity(self, speed: float):
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
        rospy.loginfo((f"/speed_filter: Updated velocity to {round(speed, 2)}m/s "
                       f"from max velocity is {self.max_speed_}m/s."))

    def run(self):
        while not rospy.is_shutdown():
            speed = min(self.speed_limit_lane_, self.speed_at_field_, self.max_speed_)
            if self.current_speed_ != speed:
                self.current_speed_ = speed
                self.update_velocity(self.current_speed_)
            
            rospy.sleep(1/self.loop_freq_)

if __name__== '__main__':
    rospy.init_node("speed_filter")
    try:
        speed_filter = SpeedFilter()
        rospy.loginfo("%s node is running!", rospy.get_name())
        speed_filter.run()
        
    except rospy.ROSInterruptException:
        pass