#!/usr/bin/env python3

import math
import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import utils

from laser_line_extraction.msg import LineSegmentList
from geometry_msgs.msg import Pose2D

class FindDockTF():

    def __init__(self):

        # Parameters:
        self.left_vector_length_         = rospy.get_param("~left_vector_length", 0.1)
        self.right_vector_length_        = rospy.get_param("~right_vector_length", 0.1)
        self.pattern_angle_between_lr_   = rospy.get_param("~pattern_angle_between_lr", 0)
        self.distance_tolerance_         = rospy.get_param("~distance_tolerance", 0.1)
        self.detect_angle_tolerance_     = rospy.get_param("~detect_angle_tolerance", 0.15)
        self.angle_threshold_            = rospy.get_param("~angle_threshold", 0.5)
        self.pattern_distance_lr_        = rospy.get_param("~pattern_distance_lr", 0.407)
        self.laser_frame_id_             = rospy.get_param("~laser_frame_id", "back_laser_link")
        self.dock_frame_                 = rospy.get_param("~dock_frame", "parallel_frame")

        # Publishes, subscribes:
        rospy.Subscriber("/back_line_segments", LineSegmentList, self.line_segment_callback)

    def line_segment_callback(self, msg: LineSegmentList):

        line_segments = msg.line_segments
        line_num = len(line_segments)

        flag = False

        # Check number of line
        # print(f"Number of line = {line_num}")

        if line_num < 2:
            # rospy.logwarn("There isn't enough line in the laser field!")
            return
        
        # Find vector c, d
        for i in range(line_num-1):
            for j in range(i+1, line_num):
                if (utils.check_angle_two_vectors(line_segments[i].angle,
                                                  line_segments[j].angle,
                                                  math.pi - self.pattern_angle_between_lr_,
                                                  self.detect_angle_tolerance_)):
                    
                    if utils.check_angle(line_segments[i].angle,
                                         line_segments[j].angle,
                                         self.angle_threshold_):

                        if utils.check_distance_two_vectors(line_segments[i].start,
                                                            line_segments[j].end,
                                                            self.pattern_distance_lr_,
                                                            self.distance_tolerance_):
                            left_vector = line_segments[i]
                            right_vector = line_segments[j]
                            flag = True
                        
                        elif utils.check_distance_two_vectors(line_segments[j].start,
                                                              line_segments[i].end,
                                                              self.pattern_distance_lr_,
                                                              self.distance_tolerance_):
                            left_vector = line_segments[j]
                            right_vector = line_segments[i]
                            flag = True
                        
                        if (flag):
                            if (utils.check_length_two_vectors(left_vector.start, left_vector.end, self.left_vector_length_) and
                                utils.check_length_two_vectors(right_vector.start, right_vector.end, self.right_vector_length_)):
                                
                                pose2d = Pose2D
                                pose2d.x = (left_vector.end[0] + right_vector.start[0])/2
                                pose2d.y = (left_vector.end[1] + right_vector.start[1])/2
                                pose2d.theta = math.pi + (left_vector.angle + right_vector.angle)/2

                                utils.publish_tf(pose2d, self.laser_frame_id_, self.dock_frame_)
                                break

if __name__== '__main__':
    rospy.init_node("find_parallel_tf")
    try:
        find_dock_tf = FindDockTF()
        rospy.loginfo("%s is running!", rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass