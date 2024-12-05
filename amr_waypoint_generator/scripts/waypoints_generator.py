#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import Bool

import rospy
import ruamel.yaml
import rosnode

class WaypointsGenerator(object):

    def __init__(self, file_name):
        
        # Kill move_base_node when launch this file
        rosnode.kill_nodes(['move_base_node'])

        self.position_name = str(input("Enter position name: "))

        assert (type(self.position_name) == str), "Position name must be str type!"

        # Variables
        self.frame_id = "map"

        self.yaml = ruamel.yaml.YAML()
        self.yaml.default_flow_style = None

        self.file_name_ = file_name

        self.waypoints = []
        self.pose_list = []
        self.pose_data = []
        self.is_get_waypoints = True
        self.waypoint_num = 0

        # Publishers:
        self.pub_amr_pose_array = rospy.Publisher("amr_goal_array", PoseArray, queue_size=1)

        # Subscribers:
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.rvizGoalCB)
        rospy.Subscriber("get_waypoint_list", Bool, self.waypointsListCB)     


    def rvizGoalCB(self, goal:PoseStamped):
        pose = Pose()

        pose.position.x = goal.pose.position.x
        pose.position.y = goal.pose.position.y
        pose.position.z = goal.pose.position.z
        pose.orientation.x = goal.pose.orientation.x
        pose.orientation.y = goal.pose.orientation.y
        pose.orientation.z = goal.pose.orientation.z
        pose.orientation.w = goal.pose.orientation.w

        self.waypoints.append(pose)

        self.pose_data = [float(f'{pose.position.x:.6f}'), float(f'{pose.position.y:.6f}'), 
                          float(f'{pose.orientation.z:.6f}'), float(f'{pose.orientation.w:.6f}')]
        
        self.pose_list.append(self.pose_data)

        self.pub_pose_array(self.waypoints)
        self.waypoint_num += 1
        rospy.loginfo(f"Get position {self.waypoint_num}!")

    
    def pub_pose_array(self, pose_array):
        """
        Publish pose array in rviz
        """
        poses = PoseArray()
        poses.header.frame_id = self.frame_id
        poses.poses = [poses for poses in pose_array]

        self.pub_amr_pose_array.publish(poses)

    
    def waypointsListCB(self, msg:Bool):
        self.is_get_waypoints = msg.data

        if not self.is_get_waypoints:
            self.save_waypoints()

    
    def save_waypoints(self):
        
        with open(self.file_name_, 'r') as file:
            data = self.yaml.load(file)

        data[f'{self.position_name}'] = {}
        data[f'{self.position_name}']['frame_id'] = self.frame_id
        data[f'{self.position_name}']['pose'] = self.pose_list

        with open(self.file_name_, 'w') as f:
            self.yaml.dump(data, f)
            rospy.loginfo(f"Written {self.waypoint_num} waypoints to file succesfully!")
        
        rospy.signal_shutdown("Get all waypoint!")


if __name__ == "__main__":
    
    file_name = "/home/amr/catkin_ws/src/amr/amr_waypoint_generator/config/amr_waypoints.yaml"
    
    rospy.init_node("waypoints_generator")

    try:
        waypoint_generator = WaypointsGenerator(file_name)
        rospy.logwarn("Waypoint Generator node is runnning!")
        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass

