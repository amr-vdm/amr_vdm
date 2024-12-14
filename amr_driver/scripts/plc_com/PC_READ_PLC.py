#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Bool, Int16, Int16MultiArray, Empty
from sensor_msgs.msg import Range, BatteryState
from amr_msgs.msg import SliderSensorStamped, PoseInitial
from amr_driver.mcprotocol.type1e import Type1E

class Parameter():
    PLC_port = 8001
    PLC_IP_address = '192.168.0.250'
    pub_frequency = 10
    left_ultrasonic_frame = 'left_ultrasonic_link'
    right_ultrasonic_frame = 'right_ultrasonic_link'
    min_range = 0.065
    max_range = 0.35
    field_of_view = 0.05236 

class PCReadPLC(Type1E):

    def __init__(self):
        
        super().__init__()
        rospy.init_node("PC_read_PLC")

        self.params = Parameter()
        self.initParams()

        self.rate = rospy.Rate(self.params.pub_frequency)
        
        # Connect to PLC
        self.connect_PLC(self.params.PLC_IP_address, self.params.PLC_port)
        if self._is_connected:
            rospy.loginfo("/PC_READ_PLC: Connected to PLC.")
        else:
            rospy.logerr("/PC_READ_PLC: Can't connect to PLC! Please check IP address or Port again!")

        # Only Publishers:
        self.pub_left_ultrasonic      = rospy.Publisher("left_ultrasonic/range", Range, queue_size=5)
        self.pub_right_ultrasonic     = rospy.Publisher("right_ultrasonic/range", Range, queue_size=5)
        self.pub_cmd_cancel_AMR       = rospy.Publisher("CANCEL_AMR", Bool,queue_size=5)
        self.pub_cmd_pause_AMR        = rospy.Publisher("PAUSE_AMR", Bool, queue_size=5)
        self.pub_cmd_reset_AMR        = rospy.Publisher("RESET_AMR", Empty, queue_size=1)
        self.pub_stop_amr             = rospy.Publisher("STOP_AMR", Bool, queue_size=1)
        self.pub_hand_control         = rospy.Publisher("HAND_CONTROL_AMR", Bool, queue_size=5)
        self.pub_EMS                  = rospy.Publisher("emergency_stop", Bool, queue_size=5)
        self.pub_initialpose          = rospy.Publisher("pose_initial_hand", PoseInitial, queue_size=1)
        self.pub_cart_sensor          = rospy.Publisher("cart_sensor_state", SliderSensorStamped, queue_size=5)
        self.pub_max_slider           = rospy.Publisher("slider_sensor_state", SliderSensorStamped, queue_size=5)
        self.pub_pickup_current_state = rospy.Publisher("pickup_current_state", Bool, queue_size=1)
        self.pub_drop_current_state   = rospy.Publisher("drop_current_state", Bool, queue_size=1)
        self.pub_battery_state        = rospy.Publisher("/battery_state", BatteryState, queue_size=5)
        self.pub_hand_dock_trigger    = rospy.Publisher("hand_dock_trigger", Empty, queue_size=1)

        # Subcribers:
        rospy.Subscriber("wait_dock_frame", Bool, self.waitDockFrameCB)
        rospy.Subscriber("state_runonce_nav", Bool, self.runOnceStateCB)
        
        # Avariables:
        self.is_runonce_NAV = False
        self.Emergency_STOP_state = 0
        self.START_state = 0
        self.Pause_AMR_state = 0
        self.RESET_AMR_state = 0
        self.hand_control_state = 0
        self.pause_timer = 0
        self.pause_off_delay = 2
        self.initialpose_state = 0
        self.pickup_current_state = 0
        self.drop_current_state = 0
        self.wait_dock_state = 0
        self.cart_sensor_state = [0,0]
        self.slider_sensor_state = [0,0]
        self.sensor_state_array = SliderSensorStamped()

        self.isPubWaitDockBit = False

    def initParams(self):
        print("/PC_READ_PLC: Parameters:")

        param_names = [attr for attr in dir(self.params) if not callable(getattr(self.params, attr)) and not attr.startswith("__")]

        # get private rosparam, if none use default
        for param_name in param_names:
            param_val = rospy.get_param(rospy.get_name() + "/" + param_name, getattr(self.params, param_name))
            print(f"* /{param_name}: {param_val}")
            setattr(self.params, param_name, param_val)

    def handleUltrasonicSensor(self, data):

        for i in range(0,len(data)):
            data_msg = Range()
            data_msg.header.stamp = rospy.Time.now()
            data_msg.radiation_type = 0
            data_msg.field_of_view = self.params.field_of_view
            data_msg.min_range = self.params.min_range
            data_msg.max_range = self.params.max_range
            # if data[i] >= 4000:
            #     data_msg.range = float("Inf")
            # elif data[i] <= 5:
            #     data_msg.range = float("-Inf")
            # else:
            #     data_msg.range = (65 + 0.07125 * data[i]) / 1000 

            # if i == 0:
            #     data_msg.header.frame_id = self.params.left_ultrasonic_frame
            #     self.pub_left_ultrasonic.publish(data_msg)
            # else:
            #     data_msg.header.frame_id = self.params.right_ultrasonic_frame
            #     self.pub_right_ultrasonic.publish(data_msg)
            data_msg.range = 10.0
            data_msg.header.frame_id = self.params.left_ultrasonic_frame
            self.pub_left_ultrasonic.publish(data_msg)
            data_msg.header.frame_id = self.params.right_ultrasonic_frame
            self.pub_right_ultrasonic.publish(data_msg)
   
    def handle_battery_publisher(self, data: list, charging: int):
        # data[0]: voltage cell 1     (mV)
        # data[1]: voltage cell 2     (mV)
        # data[2]: voltage cell 3     (mV)
        # data[3]: voltage cell 4     (mV)
        # data[4]: voltage cell 5     (mV)
        # data[5]: voltage cell 6     (mV)
        # data[6]: voltage cell 7     (mV)
        # data[7]: voltage cell 8     (mV)
        # data[8]: voltage            (1 unit = 10mV)
        # data[9]: remain_capacity    (1 unit = 0.1AH)
        # data[10]: design_capacity   (1 unit = 0.1AH)
        # data[11]: percentage
        # data[12]: current           (1 unit = 0.1A)
        # data[13]: temperature 1     (C)
        # data[14]: temperature 2     (C)
        msg = BatteryState()
        msg.voltage = data[8] / 100
        msg.temperature = (data[13] + data[14]) / 2
        msg.current = data[12] / 10
        msg.capacity = data[9] / 10
        msg.design_capacity = data[10] / 10
        msg.percentage = msg.capacity / msg.design_capacity
        msg.power_supply_status = charging
        msg.present = True
        cell_voltage = []
        for cell in data[:8]:
            cell_voltage.append(cell / 1000)
        msg.cell_voltage = cell_voltage
        msg.cell_temperature = data[13:]
        msg.header.stamp = rospy.Time.now()
        self.pub_battery_state.publish(msg)


    def waitDockFrameCB(self, msg:Bool):
        self.isPubWaitDockBit = msg.data

    def runOnceStateCB(self, msg: Bool):
        self.is_runonce_NAV = msg.data
        if (not msg.data and self.Pause_AMR_state):
            self.Pause_AMR_state = 0
            self.pause_timer = 0
            self.pub_cmd_pause_AMR.publish(False)
            self.pub_stop_amr.publish(False)

    def run(self):
        while not rospy.is_shutdown():
            # bit_array[0]  - M400: EMS bit
            # bit_array[1]  - M401: plc_control bit
            # bit_array[2]  - M402: pause bit
            # bit_array[3]  - M403: cancel bit
            # bit_array[4]  - M404: high current pickup bit
            # bit_array[5]  - M405: start bit
            # bit_array[6]  - M406: initialpose bit
            # bit_array[7]  - M407: left_checker_sensor
            # bit_array[8]  - M408: right_checker_sensor
            # bit_array[9]  - M409: original_slider_sensor
            # bit_array[10] - M410: max_slider_sensor
            # bit_array[11] - M411: manual hand control
            # bit_array[12] - M412: high current dropoff bit
            # bit_array[13] - M413: reset bit
            # bit_array[14] - M414: wait_dock bit
            # bit_array[15] - M415: battery_is_charging bit

            bit_array = self.batchread_bitunits("M400", 16)

            # reg_array[0]  - D600: ultrasonic sensor left
            # reg_array[1]  - D601: ultrasonic sensor right
            # reg_array[2]  - D602: voltage cell 1
            # reg_array[3]  - D603: voltage cell 2
            # reg_array[4]  - D604: voltage cell 3
            # reg_array[5]  - D605: voltage cell 4
            # reg_array[6]  - D606: voltage cell 5
            # reg_array[7]  - D607: voltage cell 6
            # reg_array[8]  - D608: voltage cell 7
            # reg_array[9]  - D609: voltage cell 8
            # reg_array[10]  - D610: voltage
            # reg_array[11]  - D611: capacity remain
            # reg_array[12]  - D612: design_capacity
            # reg_array[13]  - D613: percentage
            # reg_array[14]  - D614: current
            # reg_array[15]  - D615: temperature 1
            # reg_array[16]  - D616: temperature 2
            reg_array = self.batchread_wordunits("D600", 17)
            self.handleUltrasonicSensor(reg_array[:2])
            self.handle_battery_publisher(reg_array[2:17], bit_array[15])

            # bit_array[0] - M400: EMS bit
            if (bit_array[0] != self.Emergency_STOP_state):
                if bit_array[0]:
                    rospy.logerr("/PC_READ_PLC: EMS is ON!")
                    self.pub_stop_amr.publish(True)
                    self.pub_EMS.publish(True)
                    self.Emergency_STOP_state = 1

                else:
                    rospy.loginfo("/PC_READ_PLC: EMS is OFF.")
                    self.pub_stop_amr.publish(False)
                    self.pub_EMS.publish(False)
                    self.Emergency_STOP_state = 0
            
            if self.is_runonce_NAV:
                if not bit_array[11]:
                    # bit_array[2] - M402: pause bit
                    if bit_array[2] != self.Pause_AMR_state:
                        if bit_array[2]:
                            rospy.loginfo("/PC_READ_PLC: Pause is ON.")
                            self.pub_cmd_pause_AMR.publish(True)
                            self.pub_stop_amr.publish(True)
                            self.Pause_AMR_state = 1

                        elif self.pause_timer <= self.pause_off_delay:
                            self.pause_timer += (1 / self.params.pub_frequency)
                            
                        else:
                            rospy.loginfo("/PC_READ_PLC: Pause is OFF.")
                            self.pub_cmd_pause_AMR.publish(False)
                            self.pub_stop_amr.publish(False)
                            self.Pause_AMR_state = 0
                    else:
                        self.pause_timer = 0

                    # bit_array[3] - M403: cancel bit
                    if bit_array[3]:
                        rospy.loginfo("/PC_READ_PLC: Pressed cancel.")
                        self.pub_cmd_cancel_AMR.publish(True)

                    # bit_array[4] - M404: high pickup current bit
                    if bit_array[4] != self.pickup_current_state:
                        if bit_array[4]:
                            self.pub_pickup_current_state.publish(True)
                            self.pickup_current_state = bit_array[4]
                        else:
                            self.pickup_current_state = 0
                    
                    # bit_array[12] - M412: high dropoff current bit
                    elif (bit_array[12] != self.drop_current_state):
                        if (bit_array[12]):
                            self.pub_drop_current_state.publish(True)
                            self.drop_current_state = bit_array[12]
                        else:
                            self.drop_current_state = 0
            # else:
            #     if self.Pause_AMR_state:
            #         self.Pause_AMR_state = 0
            #         self.pause_timer = 0
            #         self.pub_cmd_pause_AMR.publish(False)

            # bit_array[6] - M406: initialpose bit
            if bit_array[6] != self.initialpose_state:
                if bit_array[6]:
                    pose_id, floor_id = self.batchread_wordunits("D501",2)
                    poseInitial = PoseInitial()
                    poseInitial.floor_id = floor_id
                    poseInitial.pose_id = pose_id
                    self.pub_initialpose.publish(poseInitial)
                    self.initialpose_state = 1
                else:
                    self.initialpose_state = 0
            
            # bit_array[7] - M407: left_cart_sensor
            # bit_array[8] - M408: right_cart_sensor   
            if bit_array[7:9] != self.cart_sensor_state:
                self.sensor_state_array.header.stamp = rospy.Time.now()
                self.sensor_state_array.sensor_state.sensor_name = ["left_cart, right_cart"]
                self.sensor_state_array.sensor_state.data = bit_array[7:9]
                self.pub_cart_sensor.publish(self.sensor_state_array)
                self.cart_sensor_state = bit_array[7:9]

            # bit_array[9]  - M409: original slider bit
            # bit_array[10] - M410: max slider bit
            if bit_array[9:11] != self.slider_sensor_state:
                self.sensor_state_array.header.stamp = rospy.Time.now()
                self.sensor_state_array.sensor_state.sensor_name = ["origin_slider, max_slider"]
                self.sensor_state_array.sensor_state.data = bit_array[9:11]
                self.pub_max_slider.publish(self.sensor_state_array)
                self.slider_sensor_state = bit_array[9:11]

            # bit_array[11] - M411: hand_control_bit
            if bit_array[11] != self.hand_control_state:
                self.pub_hand_control.publish(bool(bit_array[11]))
                self.hand_control_state = bit_array[11]

            # bit_array[13] - M413: reset_bit
            if (bit_array[13] != self.RESET_AMR_state):
                if bit_array[13]:
                    msg = Empty()
                    self.pub_cmd_reset_AMR.publish(msg)
                    rospy.loginfo("/PC_READ_PLC: Reset AMR.")
                self.RESET_AMR_state = bit_array[13]

            # bit_array[14] - M414: wait_dock bit
            if self.isPubWaitDockBit:
                if bit_array[14]:
                    msg = Empty()
                    self.pub_hand_dock_trigger.publish(msg)
                    # INFO("PLC hand trigger continues docking with mode DROP-OFF!")

            self.rate.sleep()

        self.close()
        
if __name__== '__main__':
    try:
        PC_bridge_PLC = PCReadPLC()
        rospy.loginfo("%s node is running!", rospy.get_name())
        PC_bridge_PLC.run()
    except rospy.ROSInterruptException:
        pass