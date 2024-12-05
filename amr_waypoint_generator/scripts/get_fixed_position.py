#!/usr/bin/env python3

from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

import rospy
import ruamel.yaml


class FixedPosition(object):

    def __init__(self, file_name):

        rospy.init_node("fixed_position")

        # Variables
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_x = 0.0
        self.orientation_y = 0.0
        self.orientation_z = 0.0
        self.orientation_w = 0.0

        self.frame_id = 'map'

        self.yaml = ruamel.yaml.YAML()
        self.yaml.default_flow_style = None

        self.file_name_ = file_name

        # Subscribers
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.AMCLPoseCB)


    def AMCLPoseCB(self, pose: PoseWithCovarianceStamped):
        
        self.frame_id = pose.header.frame_id
        self.position_x = pose.pose.pose.position.x
        self.position_y = pose.pose.pose.position.y
        self.orientation_x = pose.pose.pose.orientation.x
        self.orientation_y = pose.pose.pose.orientation.y
        self.orientation_z = pose.pose.pose.orientation.z
        self.orientation_w = pose.pose.pose.orientation.w


    def save_waypoints(self):

        position_name = input("Position name: ")
            
        pose_data = [float(f'{self.position_x:.6f}'), float(f'{self.position_y:.6f}'), 
                        float(f'{self.orientation_z:.6f}'), float(f'{self.orientation_w:.6f}')]
        pose2d = [float(f'{self.position_x:.6f}'),
                  float(f'{self.position_y:.6f}'),
                  round(euler_from_quaternion([self.orientation_x,
                                      self.orientation_y,
                                      self.orientation_z,
                                      self.orientation_w])[2], 6)]

        with open(self.file_name_, 'r') as file:
            data = self.yaml.load(file)

        data[f'{position_name}'] = {}
        data[f'{position_name}']['frame_id'] = self.frame_id
        data[f'{position_name}']['pose'] = pose_data
        data[f'{position_name}']['pose2d'] = pose2d

        print(data[f'{position_name}'])

        with open(self.file_name_, 'w') as f:
            self.yaml.dump(data, f)
            print("Written to file succesfully!")


if __name__ == "__main__":
    
    file_name = "/home/amr/catkin_ws/src/amr/amr_waypoint_generator/config/fixed_position.yaml"

    try:
        fixed_position = FixedPosition(file_name)
        fixed_position.save_waypoints()
    
    except rospy.ROSInterruptException:
        pass
