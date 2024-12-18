#!/usr/bin/env python3

import math
import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import utils

from laser_line_extraction.msg import LineSegmentList
from geometry_msgs.msg import Pose2D

class FindTF():

    def __init__(self):

        # Parameters:
        self.pattern_length_vector_b_ = rospy.get_param("~pattern_length_vector_b", 0.15)
        self.pattern_length_vector_c_ = rospy.get_param("~pattern_length_vector_c", 0.15)
        self.pattern_angle_bc_  = rospy.get_param("~pattern_angle_bc", 0)
        self.angle_tolerance_   = rospy.get_param("~angle_tolerance", 0.15)
        self.angle_threshold_   = rospy.get_param("~angle_threshold", 0.5)
        self.pattern_dis_bc_    = rospy.get_param("~pattern_dis_bc", 0.18)
        self.laser_frame_id_    = rospy.get_param("~laser_frame_id", "back_laser_link")
        self.dock_frame_        = rospy.get_param("~dock_frame", "last_frame")

        # Subscribers:
        rospy.Subscriber("/back_line_segments", LineSegmentList, self.line_segment_callback)

    def line_segment_callback(self, msg: LineSegmentList):

        list_line_segments_raw = msg.line_segments
        line_num = len(list_line_segments_raw)

        flag = False

        # Check number of line
        # print(f"Number of line = {line_num}")

        if line_num < 2:
            # rospy.logwarn("There isn't enough line in the laser field!")
            return
        
        # Find vector b, c and angle
        for i in range(line_num-1):
            for j in range(i+1, line_num):
                if utils.check_angle_two_vectors(list_line_segments_raw[i].angle,
                                                 list_line_segments_raw[j].angle,
                                                 math.pi - self.pattern_angle_bc_,
                                                 self.angle_tolerance_):
                    
                    if utils.check_angle(list_line_segments_raw[i].angle,
                                       list_line_segments_raw[j].angle,
                                       self.angle_threshold_):
                        
                        length_b = utils.vector_length(list_line_segments_raw[i].start,
                                                        list_line_segments_raw[i].end)
                        length_c = utils.vector_length(list_line_segments_raw[j].start,
                                                        list_line_segments_raw[j].end)
                        
                        # print("length_b = ", length_b)
                        # print("length_c = ", length_c)

                        if (length_b >= self.pattern_length_vector_b_ and
                            length_c >= self.pattern_length_vector_c_):
                            
                            dis_bc = utils.vector_length(list_line_segments_raw[i].start,
                                                        list_line_segments_raw[j].end)
                            dis_cb = utils.vector_length(list_line_segments_raw[j].start,
                                                        list_line_segments_raw[i].end)

                            # print("dis_bc = ", dis_bc)
                            # print("dis_cb = ", dis_cb)

                            if (abs(dis_bc - self.pattern_dis_bc_) <= 0.05):
                                vector_b = list_line_segments_raw[i]
                                vector_c = list_line_segments_raw[j]
                                flag = True

                            elif (abs(dis_cb - self.pattern_dis_bc_) <= 0.05):
                                vector_b = list_line_segments_raw[j]
                                vector_c = list_line_segments_raw[i]
                                flag = True
                            
                            if (flag):
                                pose2d = Pose2D
                                pose2d.x = (vector_b.end[0] + vector_c.start[0])/2
                                pose2d.y = (vector_b.end[1] + vector_c.start[1])/2
                                pose2d.theta = math.pi + (vector_b.angle + vector_c.angle)/2

                                utils.publish_tf(pose2d, self.laser_frame_id_, self.dock_frame_)
                                break

if __name__== '__main__':
    rospy.init_node("find_last_tf")
    try:
        find_tf = FindTF()
        rospy.loginfo("%s node is running!", rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass