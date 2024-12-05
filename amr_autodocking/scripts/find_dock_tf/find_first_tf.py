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
        self.length_vector_a    = rospy.get_param("~length_vector_a", 0.09)
        self.length_vector_b    = rospy.get_param("~length_vector_b", 0.15)
        self.pattern_angle_ab   = rospy.get_param("~pattern_angle_ab", 3*math.pi/2)
        self.pattern_angle_bc   = rospy.get_param("~pattern_angle_bc", 0)
        self.pattern_angle_ad   = rospy.get_param("~pattern_angle_ad", math.pi)
        self.angle_threshold    = rospy.get_param("~angle_threshold", 0.3)
        self.length_tolerance   = rospy.get_param("~length_tolerance", 0.08)
        self.detect_angle_tol   = rospy.get_param("~detect_angle_tol", 0.15)
        self.pattern_dis_ad     = rospy.get_param("~pattern_dis_ad", 0.359)
        self.pattern_dis_bc     = rospy.get_param("~pattern_dis_bc", 0.18)
        self.pattern_dis_ab     = rospy.get_param("~pattern_dis_ab", 0.04)
        self.laser_frame_id     = rospy.get_param("~laser_frame_id", "back_laser_link")
        self.dock_name          = rospy.get_param("~dock_name", "first_frame")

        # Subscribers:
        rospy.Subscriber("/back_line_segments", LineSegmentList, self.lineSegmentsCB)


    def distance2D(self, x, y):
        return (math.sqrt(pow(x, 2) + pow(y, 2)))
    

    def checkVectorLength(self, start_point , end_point, pattern_length, distance_tolerance):

        length_line = self.distance2D(start_point[0] - end_point[0],
                                      start_point[1] - end_point[1])
        return (abs(length_line - pattern_length) <= distance_tolerance)
        
        
    def calVectorLength(self, start_point, end_point):
        length_vector = self.distance2D(start_point[0] - end_point[0],
                                        start_point[1] - end_point[1])
        return length_vector
    

    def checkAngleBetweenTwoVectors(self, angle_1, angle_2,
                                    angle_pattern_12, angle_tolerance):
        
        if (angle_1 * angle_2) > 0:
            angle = abs(angle_1 - angle_2)
            return (abs(angle_pattern_12 - angle) <= angle_tolerance)
        else:
            angle = 2*math.pi - abs(angle_1 - angle_2)
            return (abs(angle - (2*math.pi - angle_pattern_12)) <= angle_tolerance)
            

    def checkAngle(self, angle_1, angle_2, angle_threshold):
        return (abs(angle_1) - abs(angle_2) <= angle_threshold)
    

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


    def lineSegmentsCB(self, msg: LineSegmentList):

        list_line_segments_raw = msg.line_segments
        line_num = len(list_line_segments_raw)

        list_line_segments_ad = []
        list_line_segments_bc = []
        check_angle_ab = False
        check_angle_bc = False
        check_angle_ad = False

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
                                                    math.pi - self.pattern_angle_ad,
                                                    self.detect_angle_tol):
                    
                    if self.checkAngle(list_line_segments_raw[i].angle,
                                       list_line_segments_raw[j].angle,
                                       self.angle_threshold):
                        
                        if (self.checkVectorLength(list_line_segments_raw[i].start,
                                                   list_line_segments_raw[i].end,
                                                   self.length_vector_a,
                                                   self.length_tolerance) and
                            self.checkVectorLength(list_line_segments_raw[j].start,
                                                   list_line_segments_raw[j].end,
                                                   self.length_vector_a,
                                                   self.length_tolerance)):
                            distance_ad = self.calVectorLength(list_line_segments_raw[i].start, list_line_segments_raw[j].end)
                            distance_da = self.calVectorLength(list_line_segments_raw[j].start, list_line_segments_raw[i].end)

                            if (abs(distance_ad - self.pattern_dis_bc) <= 0.08):
                                if (abs(distance_da - self.pattern_dis_ad) <= 0.08):
                                    vector_a = list_line_segments_raw[i]
                                    vector_d = list_line_segments_raw[j]
                                    print("Found vector a and d success!!!")
                                    print(f"a angle: {vector_a.angle}")
                                    print(f"d angle: {vector_d.angle}")
                                    list_line_segments_ad.append([vector_a,vector_d])
                                    check_angle_ad = True

                            elif (abs(distance_da - self.pattern_dis_bc) <= 0.08):
                                if (abs(distance_ad - self.pattern_dis_ad) <= 0.08):
                                    vector_a = list_line_segments_raw[j]
                                    vector_d = list_line_segments_raw[i]
                                    print("Found vector a and d success!!!")
                                    print(f"a angle: {vector_a.angle}")
                                    print(f"d angle: {vector_d.angle}")
                                    list_line_segments_ad.append([vector_a,vector_d])
                                    check_angle_ad = True
                        # else: print('khong co a,d')

                elif self.checkAngleBetweenTwoVectors(list_line_segments_raw[i].angle, list_line_segments_raw[j].angle, math.pi - self.pattern_angle_bc, self.detect_angle_tol):
                    if (self.checkVectorLength(list_line_segments_raw[i].start, list_line_segments_raw[i].end, self.length_vector_b, self.length_tolerance) and
                        self.checkVectorLength(list_line_segments_raw[j].start, list_line_segments_raw[j].end, self.length_vector_b, self.length_tolerance)):
                        dis_1 = self.calVectorLength(list_line_segments_raw[i].end, list_line_segments_raw[j].start)
                        dis_2 = self.calVectorLength(list_line_segments_raw[j].end, list_line_segments_raw[i].start)
                        # print("dis_1: ", dis_1)
                        # print("dis_2: ", dis_2)

                        if (abs(dis_1 - self.pattern_dis_bc) <= 0.08):
                            vector_b = list_line_segments_raw[i]
                            vector_c = list_line_segments_raw[j]
                            list_line_segments_bc.append([vector_b,vector_c])
                            # print("Find vector b and c success!!!")
                            check_angle_bc = True

                        elif (abs(dis_2 - self.pattern_dis_bc) <= 0.08):
                            vector_b = list_line_segments_raw[j]
                            vector_c = list_line_segments_raw[i]
                            list_line_segments_bc.append([vector_b,vector_c])
                            # print("Find vetor b and c success!!!")
                            check_angle_bc = True
                    # else: print('khong co b,c')

        # Find vector a
        if check_angle_ad and check_angle_bc:
            for i in range(len(list_line_segments_ad)):
                for j in range(len(list_line_segments_bc)):

                    dis_ab = self.calVectorLength(list_line_segments_ad[i][0].start, list_line_segments_bc[j][0].end)
                    # print(f"dis_ab = {dis_ab}")

                    if dis_ab <= self.pattern_dis_ab:

                        if self.checkAngleBetweenTwoVectors(list_line_segments_ad[i][0].angle,
                                                             list_line_segments_bc[j][0].angle,
                                                             self.pattern_angle_ab - math.pi,
                                                             self.detect_angle_tol):

                            vector_a = list_line_segments_ad[i][0]
                            vector_d = list_line_segments_ad[i][1]
                            # print("Find vector a,b,c,d success!!!")
                            check_angle_ab = True
                            x = (vector_a.start[0] + vector_d.end[0])/2
                            y = (vector_b.end[1] + vector_c.start[1])/2
                            theta = math.pi + (vector_a.angle + vector_d.angle)/2
            
                            self.transformTf(x, y, theta, self.dock_name)
                            return

            if not check_angle_ab:
                vector_a = list_line_segments_ad[0][0]
                vector_d = list_line_segments_ad[0][1]       
                x = (vector_a.start[0] + vector_d.end[0])/2
                y = (vector_a.start[1] + vector_d.end[1])/2
                theta = math.pi + (vector_a.angle + vector_d.angle)/2

                self.transformTf(x, y, theta, self.dock_name)
                # print("Find vector a,d not match with b,c success!!!")
                return
        
        elif check_angle_ad:
            vector_a = list_line_segments_ad[0][0]
            vector_d = list_line_segments_ad[0][1]       
            x = (vector_a.start[0] + vector_d.end[0])/2
            y = (vector_a.start[1] + vector_d.end[1])/2
            theta = math.pi + (vector_a.angle + vector_d.angle)/2

            self.transformTf(x, y, theta, self.dock_name)
            # print("Find vector a,d success!!!")
            return

if __name__== '__main__':
    rospy.init_node("find_first_tf")
    try:
        find_tf = FindTF()
        rospy.loginfo("%s node is running!", rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass