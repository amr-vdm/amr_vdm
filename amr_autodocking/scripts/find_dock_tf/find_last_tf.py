#!/usr/bin/env python3

import math
import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg

from laser_line_extraction.msg import LineSegmentList


class FindTF():

    def __init__(self):

        # Parameters:
        self.pattern_length_vector_b = rospy.get_param("~pattern_length_vector_b", 0.15)
        self.pattern_length_vector_c = rospy.get_param("~pattern_length_vector_c", 0.15)
        self.pattern_angle_bc  = rospy.get_param("~pattern_angle_bc", 0)
        self.angle_tolerance   = rospy.get_param("~angle_tolerance", 0.15)
        self.angle_threshold   = rospy.get_param("~angle_threshold", 0.5)
        self.pattern_dis_bc    = rospy.get_param("~pattern_dis_bc", 0.18)
        self.laser_frame_id    = rospy.get_param("~laser_frame_id", "back_laser_link")
        self.dock_name         = rospy.get_param("~dock_name", "last_frame")

        # Subscribers:
        rospy.Subscriber("/back_line_segments", LineSegmentList, self.lineSegmentCB)

    
    def distance2D(self, x, y):
        return (math.sqrt(pow(x, 2) + pow(y, 2)))
    

    def checkAngle(self, angle_1, angle_2, angle_threshold):
        return (abs(angle_1) - abs(angle_2) <= angle_threshold)


    def checkVectorLength(self, start_point , end_point, pattern_length, distance_tolerance):

        length_line = self.distance2D(start_point[0] - end_point[0],
                                      start_point[1] - end_point[1])
        return (abs(length_line - pattern_length) <= distance_tolerance)
        

    def calVectorLength(self, start_point, end_point):
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
            
            
    def transformTf(self, x, y, theta, frame_name):
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

    
    def sendTransfromTf(self, vector_1, vector_2, dock_name):

        x = (vector_1.end[0] + vector_2.start[0])/2
        y = (vector_1.end[1] + vector_2.start[1])/2
        theta = math.pi + (vector_1.angle + vector_2.angle)/2

        self.transformTf(x, y, theta, dock_name)


    def lineSegmentCB(self, msg: LineSegmentList):

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
                if self.checkAngleBetweenTwoVectors(list_line_segments_raw[i].angle,
                                                    list_line_segments_raw[j].angle,
                                                    math.pi - self.pattern_angle_bc,
                                                    self.angle_tolerance):
                    
                    if self.checkAngle(list_line_segments_raw[i].angle,
                                       list_line_segments_raw[j].angle,
                                       self.angle_threshold):
                        
                        length_b = self.calVectorLength(list_line_segments_raw[i].start,
                                                        list_line_segments_raw[i].end)
                        length_c = self.calVectorLength(list_line_segments_raw[j].start,
                                                        list_line_segments_raw[j].end)
                        
                        # print("length_b = ", length_b)
                        # print("length_c = ", length_c)

                        if (length_b >= self.pattern_length_vector_b and
                            length_c >= self.pattern_length_vector_c):
                            
                            dis_bc = self.calVectorLength(list_line_segments_raw[i].start,
                                                        list_line_segments_raw[j].end)
                            dis_cb = self.calVectorLength(list_line_segments_raw[j].start,
                                                        list_line_segments_raw[i].end)

                            # print("dis_bc = ", dis_bc)
                            # print("dis_cb = ", dis_cb)

                            if (abs(dis_bc - self.pattern_dis_bc) <= 0.05):
                                vector_b = list_line_segments_raw[i]
                                vector_c = list_line_segments_raw[j]
                                flag = True

                            elif (abs(dis_cb - self.pattern_dis_bc) <= 0.05):
                                vector_b = list_line_segments_raw[j]
                                vector_c = list_line_segments_raw[i]
                                flag = True
                            
                            if (flag):
                                self.sendTransfromTf(vector_b, vector_c, self.dock_name)
                                # print("Find vector b and c success!!!")
                                break


if __name__== '__main__':
    rospy.init_node("find_last_tf")
    try:
        find_tf = FindTF()
        rospy.loginfo("%s node is running!", rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass