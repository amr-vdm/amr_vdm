#!/usr/bin/env python3
import rospy

from std_msgs.msg import Bool, Int16, Empty
from amr_msgs.msg import ErrorStamped, SafetyZone, LightMode
from amr_driver.mcprotocol.type1e import Type1E
import os

class Parameter():
    PLC_port = 8002
    PLC_IP_address = '192.168.0.250'
    obstacle_detecting_bit = "M4"
    safety_zone_bit = "M28"
    initialpose_bit = "M406"
    initialposeError_bit= "M416"
    brake_bit = "M161"
    runonce_bit = "M8"
    start_bit = "M405"
    error_mode_bit = ["M70", "M71", "M72"]
    go_out_silder_bit = "M151"
    go_in_silder_bit = "M150"
    load_finished_bit = "M9"
    server_cancel_bit = "M40"
    reset_amr_bit = "M413"
    wait_dock_bit = "M24"
    pause_server_bit = "M54"


class PCWritePLC(Type1E):

    def __init__(self):
        
        super().__init__()
        rospy.init_node("PC_write_PLC")

        self.params = Parameter()
        self.initParams()

        # Connect to PLC
        self.connect_PLC(self.params.PLC_IP_address, self.params.PLC_port)
        
        if self._is_connected:
            rospy.loginfo("PC_controller(WRITE): is connected to PLC success")
        else:
            rospy.logerr("PC_controller(WRITE): can't connect to PLC")

        # Avariables:
        self.status_protected_field = False
        self.brake_cmd = False
        self.slider_cmd = 0
        self.mode_error = 0
        self.camera_startup_finished = 0

        # Only Subcriber:
        rospy.Subscriber("status_protected_field",Bool,self.protectedFieldCb)
        rospy.Subscriber("cmd_brake",Bool, self.brakeCb)
        rospy.Subscriber("state_runonce_nav", Bool, self.runOnceStateCb)
        rospy.Subscriber("is_intialpose", Bool, self.isInitialPoseCb)
        rospy.Subscriber("safety_zone_type", SafetyZone, self.safetyTurnZoneCb)
        rospy.Subscriber("light_status", LightMode, self.lightModeCb)
        rospy.Subscriber("error_mode", ErrorStamped, self.errorModeCb)
        rospy.Subscriber("cmd_slider", Int16, self.cmdSliderCb)
        rospy.Subscriber("/camera_finished", Bool, self.loadFinishedCb)
        rospy.Subscriber("RESET_AMR", Empty, self.resetAMRCb)
        rospy.Subscriber("PAUSE_AMR_FROM_SERVER", Bool, self.pauseAMRCb)
        rospy.Subscriber("wait_dock_frame", Bool, self.waitDockFrameCB)

        # Bat vung back laser sang vung nho
        self.batchwrite_bitunits(self.params.safety_zone_bit, [0,1])


    def initParams(self):
        print("PC_WRITE_PLC params:")

        param_names = [attr for attr in dir(self.params) if not callable(getattr(self.params, attr)) and not attr.startswith("__")]

        # get private rosparam, if none use default
        for param_name in param_names:
            param_val = rospy.get_param("~" + param_name, getattr(self.params, param_name))
            print(f"* /{param_name}: {param_val}")
            setattr(self.params, param_name, param_val)

    def waitDockFrameCB(self, msg:Bool):
        if msg.data:
            self.batchwrite_bitunits(self.params.wait_dock_bit, [1])
        #     rospy.logwarn("Can not detect the parallel_frame! Wait 3 minutes until it appears...")
        else:
            self.batchwrite_bitunits('M414', [0])

    def loadFinishedCb(self, msg):
        self.batchwrite_bitunits(self.params.load_finished_bit, [1])
        os.system('rosnode kill /check_camera_connection')
        rospy.loginfo("AMR is ready for running!")

    def cmdSliderCb(self, msg: Int16):
        self.slider_cmd = msg.data
        
        if self.slider_cmd == 1:
            self.batchwrite_bitunits(self.params.go_out_silder_bit, [1])   # M151 - Slider go out
            rospy.loginfo("Slider cmd go out: Recieved.")

        elif self.slider_cmd == 2:
            self.batchwrite_bitunits(self.params.go_in_silder_bit, [1])    # M150 - Slider go in
            rospy.loginfo("Slider cmd go in: Recieved.")

    def protectedFieldCb(self, msg: Bool):
        self.status_protected_field = msg.data

        if self.status_protected_field:
            self.batchwrite_bitunits(self.params.obstacle_detecting_bit, [1])  # M4
            rospy.logerr("Detect obtacles in protected filed!")
        else:
            self.batchwrite_bitunits(self.params.obstacle_detecting_bit, [0])

    def safetyTurnZoneCb(self, msg: SafetyZone):
        if (msg.zone == SafetyZone.SMALL_ZONE):
            self.batchwrite_bitunits(self.params.safety_zone_bit, [1,1])       # M28: Front - M29: Back
            rospy.loginfo("Switched back and front safety zone to small zone!")

        elif (msg.zone == SafetyZone.BIG_ZONE):
            self.batchwrite_bitunits(self.params.safety_zone_bit, [0,1])
            rospy.loginfo("Switched back and front safety zone to default zone!")

    def isInitialPoseCb(self, msg: Bool):
        if msg.data:
            self.batchwrite_bitunits(self.params.initialpose_bit, [0])         # M406
        else:
            self.batchwrite_bitunits(self.params.initialposeError_bit,[1]) # M416

    def brakeCb(self, msg: Bool):
        self.brake_cmd = msg.data

        if self.brake_cmd:
            self.batchwrite_bitunits(self.params.brake_bit, [0,0])   # Brake on [M1-M2]
            rospy.loginfo("Brake State: ON")
        else:
            self.batchwrite_bitunits(self.params.brake_bit, [1,1])   # Brake off
            rospy.loginfo("Brake State: OFF")

    def runOnceStateCb(self, msg: Bool):
        if msg.data:
            self.batchwrite_bitunits(self.params.runonce_bit, [1])    # M8
        else:
            self.batchwrite_bitunits(self.params.runonce_bit, [0])

    def lightModeCb(self, msg: LightMode):
        mode = msg.lightMode
        self.batchwrite_wordunits("D201", [mode])

    def errorModeCb(self, msg: ErrorStamped):
        error = msg.error
        error_msg = error.description
        if not error.nolog:
            rospy.logerr(f"AMR_ERROR: {error_msg}")
        self.batchwrite_wordunits("D200", [error.code])

    def resetAMRCb(self, msg: Empty):
        self.batchwrite_bitunits(self.params.reset_amr_bit, [0])
        rospy.loginfo("PC turned off bit reset")

    def pauseAMRCb(self, msg: Bool):
        self.batchwrite_bitunits(self.params.pause_server_bit, [1])
        rospy.loginfo(f"Write bit pause for PLC success (cmd from server-pause: {msg.data})")
        
if __name__== '__main__':
    try:
        PC_bridge_PLC = PCWritePLC()
        rospy.loginfo("%s node is running!", rospy.get_name())
        rospy.spin()
        PC_bridge_PLC.close()
    except rospy.ROSInterruptException:
        pass