#!/usr/bin/env python3

import rospy
import actionlib
import dynamic_reconfigure.client as dc
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction
from amr_autodocking.msg import AutoDockingAction


class Controller():
    
    def __init__(self):
        self.loop_freq_ = 5.0
        self.timeout_pause_  = 0.0
        self.timeout_normal_ = 15.0
        self.is_runonce_        = False
        self.is_pause_          = False
        self.obstacle_detected_ = False
        self.prev_state_ = False

        self.move_base_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.auto_dock_client = actionlib.SimpleActionClient("autodock_action", AutoDockingAction)
        self.auto_dock_client.wait_for_server()

        self.client_global_costmap = dc.Client("/move_base_node/global_costmap/obstacles")
        self.client_local_costmap  = dc.Client("/move_base_node/local_costmap/obstacles")
        
        # Subscribers:
        rospy.Subscriber("CANCEL_AMR", Bool, self.cancel_callback)
        rospy.Subscriber("PAUSE_AMR", Bool, self.pause_callback)
        rospy.Subscriber("status_protected_field",Bool,self.protected_field_callback)
        rospy.Subscriber("state_runonce_nav", Bool, self.runonce_callback)

        self.enable_costmap_layer(False)

    def protected_field_callback(self, msg: Bool):
        if self.is_pause_:
            self.obstacle_detected_ = False
            return
        self.obstacle_detected_ = msg.data
    
    def cancel_callback(self, msg: Bool):
        if self.is_runonce_:
            if msg.data:
                self.move_base_client.cancel_all_goals()
                self.auto_dock_client.cancel_all_goals()
                return

    def enable_costmap_layer(self, data):
        rospy.loginfo("/amr_controller: Enable costmap layer.") if data else \
        rospy.loginfo("/amr_controller: Disable costmap layer.")

        self.client_global_costmap.update_configuration({'enabled': data})
        self.client_local_costmap.update_configuration({'enabled': data})

    def update_configuration(self, timeout_mvb, enable_costmap_layer:bool):
        client_movebase_timeout = dc.Client("/move_base_node")
        self.enable_costmap_layer(enable_costmap_layer)
        client_movebase_timeout.update_configuration({'oscillation_timeout': timeout_mvb})
        rospy.loginfo(f"/amr_controller: Set movebase timeout is {timeout_mvb}s.")
        return

    def runonce_callback(self, msg:Bool):
        self.is_runonce_ = msg.data
        if self.is_runonce_:
            self.enable_costmap_layer(True)
        else:
            self.enable_costmap_layer(False)

    def pause_callback(self, msg:Bool):
        self.is_pause_ = msg.data

    def run(self):
        while not rospy.is_shutdown():
            if not self.is_runonce_:
                self.prev_state_ = False
            else:
                if self.is_pause_ or self.obstacle_detected_:
                    if not self.prev_state_:
                        self.update_configuration(self.timeout_pause_, False)
                        self.prev_state_ = True
                else:
                    if self.prev_state_:
                        self.update_configuration(self.timeout_normal_, True)
                        self.prev_state_ = False

            rospy.sleep(1/self.loop_freq_)

if __name__ == "__main__":
    rospy.init_node("amr_controller")
    try:
        amr_controller = Controller()
        rospy.loginfo("%s node is running!", rospy.get_name())
        amr_controller.run()
    except rospy.ROSInterruptException:
        pass