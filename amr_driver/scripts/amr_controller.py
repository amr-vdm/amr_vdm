#!/usr/bin/env python3

import rospy
import actionlib
import dynamic_reconfigure.client as dc
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction
from amr_autodocking.msg import AutoDockingAction
from std_srvs.srv import SetBool


class AMRController():
    
    def __init__(self):

        # Client:
        self.move_base_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.auto_dock_client = actionlib.SimpleActionClient("autodock_action", AutoDockingAction)
        self.auto_dock_client.wait_for_server()

        self.client_global_costmap = dc.Client("/move_base_node/global_costmap/obstacles")
        self.client_local_costmap  = dc.Client("/move_base_node/local_costmap/obstacles")

        self.front_apriltag_detector_cli_ = rospy.ServiceProxy("/front_camera/apriltag_ros/enable_detector", SetBool)
        rospy.logwarn("Connecting to /front_camera/apriltag_ros/enable_detector service...")
        self.front_apriltag_detector_cli_.wait_for_service()
        rospy.logwarn("Connected to /front_camera/apriltag_ros/enable_detector service.")

        # Variables:
        self.timeout_pause = 0.0
        self.timeout_normal = 15.0
        self.is_runonce_NAV = False
        
        # Subscribers:
        rospy.Subscriber("CANCEL_AMR", Bool, self.cancelAMRCb)
        rospy.Subscriber("PAUSE_AMR", Bool, self.pauseAMRCb)
        rospy.Subscriber("state_runonce_nav", Bool, self.runOnceStateCb)

        self.enableCostmap(False)
    
    def cancelAMRCb(self, msg: Bool):
        if self.is_runonce_NAV:
            if msg.data:
                self.move_base_client.cancel_all_goals()
                self.auto_dock_client.cancel_all_goals()
                return

    def enableCostmap(self, data):
        """
        `data = True`: Enable costmap
        `data = False`: Disable costmap
        """
        rospy.logwarn("Enable costmap!") if data else rospy.logwarn("Disable costmap!")

        self.client_global_costmap.update_configuration({'enabled': data})
        self.client_local_costmap.update_configuration({'enabled': data})

    def controllerConfiguration(self,timeout_mvb):
        client_movebase_timeout = dc.Client("/move_base_node")
        client_movebase_timeout.update_configuration({'oscillation_timeout': timeout_mvb})
        return

    def runOnceStateCb(self,msg: Bool):
        self.is_runonce_NAV = msg.data
        if self.is_runonce_NAV:
            self.enableCostmap(True)
            self.front_apriltag_detector_cli_.call(True)
        else:
            self.enableCostmap(False)
            self.front_apriltag_detector_cli_.call(False)

    def pauseAMRCb(self, is_pause: Bool):
        if not self.is_runonce_NAV:
            return
        if is_pause.data:
            self.enableCostmap(False)
            self.controllerConfiguration(self.timeout_pause)
        else:
            self.enableCostmap(True)
            self.controllerConfiguration(self.timeout_normal)
        

if __name__ == "__main__":
    rospy.init_node("amr_controller")
    try:
        amr_controller = AMRController()
        rospy.loginfo("%s node is running!", rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass