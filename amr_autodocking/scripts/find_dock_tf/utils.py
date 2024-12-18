#!/usr/bin/env python3

import rospy
import math
import tf2_ros
import tf_conversions
import geometry_msgs.msg

from geometry_msgs.msg import Pose2D

def publish_tf(pose: Pose2D, source_frame, target_frame):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = source_frame
    t.child_frame_id = target_frame
    t.transform.translation.x = pose.x
    t.transform.translation.y = pose.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0,0,pose.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    br.sendTransform(t)

def check_vector_length(start_point , end_point, pattern_length, distance_tolerance):
    """
    Params:
    * `start_point`: pose2d in x, y format.
    * `end_point`: pose2d in x, y format.
    """
    line_length = math.hypot(start_point[0] - end_point[0],
                                start_point[1] - end_point[1])
    return (abs(line_length - pattern_length) <= distance_tolerance)

def vector_length(start_point, end_point):
    """
    Calculate vector length 2D from start to end point.
    * `start_point`: pose2d in x, y format.
    * `end_point`: pose2d in x, y format.
    """
    vector_length = math.hypot(start_point[0] - end_point[0],
                                start_point[1] - end_point[1])
    return vector_length

def check_angle_two_vectors(angle_1, angle_2,
                            angle_pattern_12, angle_tolerance):
    if (angle_1 * angle_2) > 0:
        angle = abs(angle_1 - angle_2)
        return (abs(angle_pattern_12 - angle) <= angle_tolerance)
    else:
        angle = 2*math.pi - abs(angle_1 - angle_2)
        return (abs(angle - (2*math.pi - angle_pattern_12)) <= angle_tolerance)

def check_distance_two_vectors(start_point, end_point, pattern_distance, distance_tolerance):
    """
    Params:
    * `start_point`: pose2d in x, y format.
    * `end_point`: pose2d in x, y format.
    """
    distance = vector_length(start_point, end_point)
    return (abs(distance - pattern_distance) <= distance_tolerance)

def check_length_two_vectors(start_point, end_point, pattern_length):
    """
    Params:
    * `start_point`: pose2d in x, y format.
    * `end_point`: pose2d in x, y format.
    """
    result = vector_length(start_point, end_point) - pattern_length
    return (result > 0)

def check_angle(angle_1, angle_2, angle_threshold):
    return (abs(angle_1) - abs(angle_2) <= angle_threshold)