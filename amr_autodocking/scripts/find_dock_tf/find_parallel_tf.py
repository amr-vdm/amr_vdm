#!/usr/bin/env python3

import math
import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg

from laser_line_extraction.msg import LineSegmentList


class FindDockTF():

    def __init__(self):

        # Parameters:
        self.left_vector_length         = rospy.get_param("~left_vector_length", 0.1)
        self.right_vector_length        = rospy.get_param("~right_vector_length", 0.1)
        self.pattern_angle_between_lr   = rospy.get_param("~pattern_angle_between_lr", 0)
        self.distance_tolerance         = rospy.get_param("~distance_tolerance", 0.1)
        self.detect_angle_tolerance     = rospy.get_param("~detect_angle_tolerance", 0.15)
        self.angle_threshold            = rospy.get_param("~angle_threshold", 0.5)
        self.pattern_distance_lr        = rospy.get_param("~pattern_distance_lr", 0.407)
        self.laser_frame_id             = rospy.get_param("~laser_frame_id", "back_laser_link")
        self.dock_name                  = rospy.get_param("~dock_name", "parallel_frame")

        # Publishes, subscribes:
        rospy.Subscriber("/back_line_segments", LineSegmentList, self.lineSegmentCB)
        
        
    def distance2D(self, x, y):
        return (math.sqrt(pow(x, 2) + pow(y, 2)))
    

    def checkAngle(self, angle_1, angle_2, angle_threshold):
        return (abs(angle_1) - abs(angle_2) <= angle_threshold)
        
        
    def calculateLengthVector(self, start_point, end_point):
        length_vector = self.distance2D(start_point[0] - end_point[0],
                                        start_point[1] - end_point[1])
        return length_vector
    

    def checkAngleBetweenTwoVectors(self, angle_1, angle_2, angle_pattern_12, angle_tolerance):
        if (angle_1 * angle_2) > 0:
            angle = abs(angle_1 - angle_2)
            return (abs(angle_pattern_12 - angle) <= angle_tolerance)
        else:
            angle = 2*math.pi - abs(angle_1 - angle_2)
            return (abs(angle - (2*math.pi - angle_pattern_12)) <= angle_tolerance)
    

    def checkDistanceBetweenTwoVectors(self, start_point, end_point, pattern_distance, distance_tolerance):
        distance = self.calculateLengthVector(start_point, end_point)
        return (abs(distance - pattern_distance) <= distance_tolerance)
    

    def checkLengthBetweenTwoVectors(self, start, end, pattern_length):
        result = self.calculateLengthVector(start, end) - pattern_length
        return (result > 0)
    

    def sendTransform(self, vector_1, vector_2, dock_name):
        x = (vector_1.end[0] + vector_2.start[0])/2
        y = (vector_1.end[1] + vector_2.start[1])/2
        theta = math.pi + (vector_1.angle + vector_2.angle)/2

        self.transformTFDock(x, y, theta, dock_name)
            

    def transformTFDock(self, x, y, theta, frame_name):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.laser_frame_id
        t.child_frame_id = frame_name
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0,0,theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
    

    def lineSegmentCB(self, msg: LineSegmentList):

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
                if (self.checkAngleBetweenTwoVectors(line_segments[i].angle,
                                                    line_segments[j].angle,
                                                    math.pi - self.pattern_angle_between_lr,
                                                    self.detect_angle_tolerance)):
                    
                    if self.checkAngle(line_segments[i].angle,
                                       line_segments[j].angle,
                                       self.angle_threshold):

                        if self.checkDistanceBetweenTwoVectors(line_segments[i].start,
                                                            line_segments[j].end,
                                                            self.pattern_distance_lr,
                                                            self.distance_tolerance):
                            left_vector = line_segments[i]
                            right_vector = line_segments[j]
                            flag = True
                        
                        elif self.checkDistanceBetweenTwoVectors(line_segments[j].start,
                                                                line_segments[i].end,
                                                                self.pattern_distance_lr,
                                                                self.distance_tolerance):
                            left_vector = line_segments[j]
                            right_vector = line_segments[i]
                            flag = True
                        
                        if (flag):
                            if (self.checkLengthBetweenTwoVectors(left_vector.start, left_vector.end, self.left_vector_length) and
                                self.checkLengthBetweenTwoVectors(right_vector.start, right_vector.end, self.right_vector_length)):
                                self.sendTransform(left_vector, right_vector, self.dock_name)
                                break


if __name__== '__main__':
    rospy.init_node("find_parallel_tf")
    try:
        find_dock_tf = FindDockTF()
        rospy.loginfo("%s is running!", rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass