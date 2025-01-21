#!/usr/bin/env python3
import rospy
import dynamic_reconfigure.client as dc

from std_msgs.msg import Float32
from amr_msgs.msg import SliderSensorStamped

IN = 1
OUT = 1

class SpeedFilter():

    def __init__(self):
        self.max_speed_ = rospy.get_param("~max_speed", 0.8)
        self.velocity_percentage_ = rospy.get_param("~velocity_per", 0.3)
        self.loop_freq_ = 5.0
        self.velocity_per_ = self.velocity_percentage_
        self.speed_limit_lane_ = self.max_speed_
        self.speed_at_field_ = self.max_speed_
        self.current_speed_ = self.max_speed_

        self.client_velocity = dc.Client("/move_base_node/RotationShimController/TebLocalPlannerROS")

        # Subscribers:
        rospy.Subscriber("speed_limit_lane", Float32, self.speed_limit_lane_callback)
        rospy.Subscriber("speed_at_field", Float32, self.speed_at_field_callback)
        rospy.Subscriber("/amr/slider_sensor_state", SliderSensorStamped, self.slider_sensor_state_callback)

    def slider_sensor_state_callback(self, msg: SliderSensorStamped):
        if msg.sensor_state.data:
            if msg.sensor_state.data[0] == IN:
                self.velocity_per_ = self.velocity_percentage_

            elif msg.sensor_state.data[1] == OUT:
                self.velocity_per_ = 0.0

    def speed_limit_lane_callback(self, msg: Float32):
        self.speed_limit_lane_ = round(msg.data,2)

    def speed_at_field_callback(self, msg: Float32):
        self.speed_at_field_ = round(msg.data,2)

    def update_velocity(self, speed: float):
        TEB_vel = {'max_vel_x': speed}
        self.client_velocity.update_configuration(TEB_vel)
        rospy.loginfo(f"/speed_filter: Current velocity is {speed}m/s.")

    def run(self):
        while not rospy.is_shutdown():
            speed = min(self.speed_limit_lane_, self.speed_at_field_, self.max_speed_) \
                    + (min(self.speed_limit_lane_, self.speed_at_field_, self.max_speed_) * self.velocity_per_)
            if self.current_speed_ != speed:
                self.current_speed_ = round(speed, 2)
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