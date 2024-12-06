#!/usr/bin/env python3
import os
import ruamel.yaml
import ruamel.yaml.comments
from typing import List, Dict
import rospy

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.srv import LoadMap, LoadMapResponse
from std_srvs.srv import Empty
from std_msgs.msg import Bool, String
from amr_msgs.srv import ChangeFloor, ChangeFloorRequest, ChangeFloorResponse
from amr_msgs.msg import PoseInitial

class PoseInfo():
    def __init__(self, pose: Pose):
        self.pose = pose

class MapContext():
    _floor_id: int
    _floor_name: str
    _map_name: str
    _path: str
    _pose_list: List[PoseInfo]

    _map_path: str
    _virtual_walls_path: str
    _obstacle_filter_path: str
    _safety_filter_path: str

    def __init__(self) -> None:
        return 

    def initialize(self, config: dict) -> bool:
        self._floor_id = config.get("floor_id",None)
        self._floor_name = config.get("floor_name",None)
        self._map_name = config.get("map_name",None)
        self._path = config.get("path",None)
        pose_list_tmp = config.get("pose_list", None)

        if self._floor_id == None:
            print("Error: <floor_id> is not specified!")
            return False

        if self._floor_name == None:
            print("Error: <floor_name> is not specified!")
            return False
        
        if self._map_name == None:
            print("Error: <map_name> is not specified!")
            return False
        
        if self._path == None:
            print("Error: <path> is not specified!")
            return False

        if type(pose_list_tmp) is not ruamel.yaml.comments.CommentedSeq:
            print("Error: <pose_list> is not a list")
            return False

        self._pose_list = []
        for p in pose_list_tmp:
            pose = Pose()
            if (
                isinstance(p,list)
                and len(p) == 4
            ):
                pose.position.x = p[0]
                pose.position.y = p[1]
                pose.orientation.z = p[2]
                pose.orientation.w = p[3]
            self._pose_list.append(PoseInfo(pose))
        
        if len(self._pose_list) > 8:
            print("Error: Pose_initial_hand supports no more than 8 positions of floor")
            return False

        self._map_path = os.path.join(self._path, self._map_name, "edited", "map.yaml")
        self._virtual_walls_path = os.path.join(self._path, self._map_name, "virtual_walls", "map.yaml")
        self._obstacle_filter_path = os.path.join(self._path, self._map_name, "disable_obstacle", "map.yaml")
        self._safety_filter_path = os.path.join(self._path, self._map_name, "safety_filter", "map.yaml")
        return True


class ChangeMapServer():
    _floor_context_dict: Dict[str, MapContext]

    def __init__(self):
        # Parameters:
        self.map_frame          = rospy.get_param("~map_frame", "map")
        self.robot_frame        = rospy.get_param("~robot_frame", "base_footprint")
        self.debug              = rospy.get_param("~debug", False)
        # self.update_frequency   = rospy.get_param("~update_frequency", 10.0)
        self.map_config_path = rospy.get_param("~map_config_path", "")

        # Listen to Transfromation
        # self.rate = rospy.Rate(self.update_frequency)

        # Service:
        # Change map service
        self.change_map_client = rospy.ServiceProxy("/change_map", LoadMap)
        self.loginfo("Connecting to change map service...")
        self.change_map_client.wait_for_service()
        self.loginfo("Connected to change map service.")

        # Change virtual wall map
        self.change_virtual_wall_map_client = rospy.ServiceProxy("/virtual_walls/change_map", LoadMap)
        self.loginfo("Connecting to /virtual_walls/change_map service...")
        self.change_virtual_wall_map_client.wait_for_service()
        self.loginfo("Connected to /virtual_walls/change_map service.")

        # Change obstacle map
        self.change_obtascle_map_client = rospy.ServiceProxy("/obstacle_filter/change_map", LoadMap)
        self.loginfo("Connecting to /obstacle_filter/change_map service...")
        self.change_obtascle_map_client.wait_for_service()
        self.loginfo("Connected to /obstacle_filter/change_map service.")

        # Change safety filter map
        self.change_safety_filter_map_client = rospy.ServiceProxy("/safety_filter/change_map", LoadMap)
        self.loginfo("Connecting to /safety_filter/change_map service...")
        self.change_safety_filter_map_client.wait_for_service()
        self.loginfo("Connected to /safety_filter/change_map service.")

        # Clear costmap service
        self.clear_all_costmap = rospy.ServiceProxy("/move_base_node/clear_costmaps", Empty)
        self.loginfo("Connecting to /move_base_node/clear_costmaps service...")
        self.clear_all_costmap.wait_for_service()
        self.loginfo("Connected to /move_base_node/clear_costmaps service.")

        # Change floor service server:
        self.change_floor_server = rospy.Service("change_floor", ChangeFloor, self.change_floor_callback)
        
        # Publishers:
        self.pub_initialpose = rospy.Publisher("/initialposeAMR", PoseWithCovarianceStamped, queue_size=5)
        self.pub_is_initial_pose = rospy.Publisher("is_intialpose", Bool, queue_size=5)
        self.pub_floor_name = rospy.Publisher("floor_name", String, queue_size=5)


        # Subcribers:
        rospy.Subscriber("pose_initial_hand", PoseInitial, self.poseInitialCb)
        

        self._floor_context_dict = {}
        self._current_floor = "L1"

        yaml = ruamel.yaml.YAML()
        with open(self.map_config_path) as file:
            config = yaml.load(file.read())
        
        floors_config = config.get("floors", None)
        if type(floors_config) is ruamel.yaml.comments.CommentedSeq:
            for fc in floors_config:
                context = MapContext()
                if context.initialize(fc):
                    self._floor_context_dict.update({context._floor_name: context})
        
        # for i in self._floor_context_dict:
        #     self.logwarn(f"floor_name: {self._floor_context_dict.get(i)._floor_name}")
        #     self.logwarn(f"map_path: {self._floor_context_dict.get(i)._map_path}")
        #     self.logwarn(f"pose_list: {self._floor_context_dict.get(i)._pose_list}")

    def setInitialPose(self, pose: Pose):
        initialPose = PoseWithCovarianceStamped()
        initialPose.pose.pose = pose
        initialPose.header.frame_id = self.map_frame
        self.pub_initialpose.publish(initialPose)
        return

    def change_floor_callback(self, req: ChangeFloorRequest):
        floor_name = req.floor_name
        resp = ChangeFloorResponse()

        if floor_name != self._current_floor:
            # resp.success = True
            # return resp

            self.clearCostmap()

            fl_context = self._floor_context_dict.get(floor_name, None)
            if fl_context is None:
                rospy.logerr("Not found floor_name in config!")
                return resp
            
            if not self.change_map(map_file=fl_context._map_path,
                                virtual_wall_map_file=fl_context._virtual_walls_path,
                                obstacle_map_file= fl_context._obstacle_filter_path,
                                safety_filter_map_file=fl_context._safety_filter_path):
                return resp
            
            rospy.sleep(2.0)
            
        msg = String()
        self._current_floor = floor_name
        msg.data = self._current_floor
        self.pub_floor_name.publish(msg)
        
        self.setInitialPose(req.initial_pose)
        resp.success = True
        return resp

    def poseInitialCb(self, msg: PoseInitial):
        floor_id = msg.floor_id
        for fl in self._floor_context_dict:
            fl_context = self._floor_context_dict.get(fl)
            if fl_context._floor_id == floor_id:
                if msg.pose_id > len(fl_context._pose_list):
                    self.logwarn(f"Floor:{fl_context._floor_name} - Pose_ID: {msg.pose_id} is not configure, will not set initial position!")
                    self.pub_is_initial_pose.publish(False)
                    return
                
                pose = fl_context._pose_list[msg.pose_id - 1]
                self.loginfo(f"Set initial_position:\n"
                             f"   floor_name: {fl_context._floor_name}\n"
                             f"   pose_id: {msg.pose_id}")
                
                # if self._current_floor != fl_context._floor_name:
                #     cfReq = ChangeFloorRequest()
                #     cfReq.floor_name = fl_context._floor_name
                #     cfReq.initial_pose = pose.pose
                #     resp = self.change_floor_callback(req=cfReq)
                #     if not resp.success:
                #         self.pub_is_initial_pose.publish(False)
                #         return
                # else:
                #     self.setInitialPose(pose.pose)
                #     msgStr = String()
                #     msgStr.data = self._current_floor
                #     self.pub_floor_name.publish(msgStr)
                
                cfReq = ChangeFloorRequest()
                cfReq.floor_name = fl_context._floor_name
                cfReq.initial_pose = pose.pose
                resp = self.change_floor_callback(req=cfReq)
                if not resp.success:
                    self.pub_is_initial_pose.publish(False)
                    return
                    
                self.pub_is_initial_pose.publish(True)
                return
        self.logwarn(f"not found floor_id-{msg.floor_id} in config!")
        return

    def change_map(self, map_file, virtual_wall_map_file, obstacle_map_file, safety_filter_map_file):
        """
        Change map service
        """
        try:
            self.loginfo("Waiting result from change_map server...")
            resp = self.change_map_client.call(map_file)
            resp1 = self.change_virtual_wall_map_client.call(virtual_wall_map_file)
            resp2 = self.change_obtascle_map_client.call(obstacle_map_file)
            resp3 = self.change_safety_filter_map_client.call(safety_filter_map_file)
        
            status = resp.result
            status1 = resp1.result
            status2 = resp2.result
            status3 = resp3.result

            if (status != LoadMapResponse.RESULT_SUCCESS
                and status1 != LoadMapResponse.RESULT_SUCCESS
                and status2 != LoadMapResponse.RESULT_SUCCESS
                and status3 != LoadMapResponse.RESULT_SUCCESS):
                rospy.logerr('Change map failed!')
                return False
            else:
                self.loginfo('Change map successful!')
                return True
            

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            self.shutdownCostMap(enable=True)

    def clearCostmap(self):
        """
        Clear all costmaps
        """
        try:
            self.loginfo("Requesting clear all costmaps...")
            self.clear_all_costmap.call()
        
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def loginfo(self, msg: str):
        msg_out = rospy.get_name() + ': ' + msg
        rospy.loginfo(msg_out)

    def logwarn(self, msg: str):
        msg_out = rospy.get_name() + ': ' + msg
        rospy.logwarn(msg_out)


if __name__== '__main__':
    rospy.init_node('amr_change_map')
    try:
        change_map_server = ChangeMapServer()
        change_map_server.loginfo("ChangeMap server node is running!")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass