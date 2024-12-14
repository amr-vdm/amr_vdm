#include <amr_base/amr_hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

//#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

#include <iomanip>
 
namespace amr_base
{
    AmrHWInterface::AmrHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model) : name_("hardware_interface"), nh_(nh)
    { 
        // Initialization of the robot's resources (joints, sensors, actuators) and
        // interfaces can be done here or inside init().
        // E.g. parse the URDF for joint names & interfaces, then initialize them
        // Check if the URDF model needs to be loaded
        if (urdf_model == NULL)
            loadURDF(nh, "robot_description");
        else
            urdf_model_ = urdf_model;

        // Load rosparams
        ros::NodeHandle rpnh(nh_, name_);
        std::size_t error = 0;
        // Code API of rosparam_shortcuts:
        // http://docs.ros.org/en/noetic/api/rosparam_shortcuts/html/namespacerosparam__shortcuts.html#aa6536fe0130903960b1de4872df68d5d
        error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
        error += !rosparam_shortcuts::get(name_, nh_, "mobile_base_controller/wheel_radius", wheel_radius_);
        error += !rosparam_shortcuts::get(name_, nh_, "mobile_base_controller/linear/x/max_velocity", max_velocity_);
        // error += !rosparam_shortcuts::get(name_, nh_, "joint_limits/slider_joint/min_position", min_position_);
        // error += !rosparam_shortcuts::get(name_, nh_, "joint_limits/slider_joint/max_position", max_position_);
        // Get additional parameters from the amr_base/config/base.yaml which is stored on the parameter server
        error += !rosparam_shortcuts::get(name_, nh_, "MR_encoder_resolution", MR_encoder_resolution_);
        error += !rosparam_shortcuts::get(name_, nh_, "MS_enc_resolution", MS_encoder_resolution_);
        error += !rosparam_shortcuts::get(name_, nh_, "gear_ratio", gear_ratio_);
        error += !rosparam_shortcuts::get(name_, nh_, "gear_ratio_pinion", gear_ratio_pinion);
        error += !rosparam_shortcuts::get(name_, nh_, "pinion_gear_radius", pinion_gear_radius);
        error += !rosparam_shortcuts::get(name_, nh_, "rack_pinion_gear_efficiency", rack_pinion_gear_efficiency);
        error += !rosparam_shortcuts::get(name_, nh_, "gain", gain_);
        error += !rosparam_shortcuts::get(name_, nh_, "trim", trim_);
        error += !rosparam_shortcuts::get(name_, nh_, "motor_constant", motor_constant_);
        error += !rosparam_shortcuts::get(name_, nh_, "debug/hardware_interface", debug_);
        error += !rosparam_shortcuts::get(name_, nh_, "acc_lim_motor", acc_lim_motor);
        error += !rosparam_shortcuts::get(name_, nh_, "acc_lim_motor_break", acc_lim_motor_break);
        error += !rosparam_shortcuts::get(name_, nh_, "current_lim_motor", current_lim_motor);
        error += !rosparam_shortcuts::get(name_, nh_, "current_lim_motor_break", current_lim_motor_break);
        error += !rosparam_shortcuts::get(name_, nh_, "slider_motor_origin_spd", slider_motor_origin_spd);
        error += !rosparam_shortcuts::get(name_, nh_, "slider_motor_spd", slider_motor_spd);
        error += !rosparam_shortcuts::get(name_, nh_, "operating_mode", operating_mode);
        error += !rosparam_shortcuts::get(name_, nh_, "bit_name", bit_name);
        error += !rosparam_shortcuts::get(name_, nh_, "word_name", word_name);
        // error += !rosparam_shortcuts::get(name_, nh_, "reg_data_left_motor", reg_data_left_motor);
        error += !rosparam_shortcuts::get(name_, nh_, "reg_data_motor", reg_data_motor);
        error += !rosparam_shortcuts::get(name_, nh_, "reg_data_motor_length", reg_data_motor_length);
        // error += !rosparam_shortcuts::get(name_, nh_, "reg_spd_cmd_left_motor", reg_spd_cmd_left_motor);
        error += !rosparam_shortcuts::get(name_, nh_, "reg_spd_cmd_motor", reg_spd_cmd_motor);
        error += !rosparam_shortcuts::get(name_, nh_, "reg_spd_cmd_length", reg_spd_cmd_length);
        error += !rosparam_shortcuts::get(name_, nh_, "bit_reset_motor", bit_reset_motor);
        error += !rosparam_shortcuts::get(name_, nh_, "bit_break_motor", bit_break_motor);
        // error += !rosparam_shortcuts::get(name_, nh_, "bit_reset_left_enc", bit_reset_left_enc);
        // error += !rosparam_shortcuts::get(name_, nh_, "bit_reset_right_enc", bit_reset_right_enc);
        // error += !rosparam_shortcuts::get(name_, nh_, "bit_reset_slider_enc", bit_reset_slider_enc);
        // error += !rosparam_shortcuts::get(name_, nh_, "bit_sensor_origin", bit_sensor_origin);
        error += !rosparam_shortcuts::get(name_, nh_, "bit_slider_origin", bit_slider_origin);
        error += !rosparam_shortcuts::get(name_, nh_, "ip_address", ip_address);
        error += !rosparam_shortcuts::get(name_, nh_, "ip_port", ip_port);

        rosparam_shortcuts::shutdownIfError(name_, error);

        wheel_diameter_ = 2.0 * wheel_radius_;
        // ros_control RobotHW needs velocity in rad/s but in the config its given in m/s
        max_velocity_ = linearToAngular(max_velocity_);

        ROS_INFO_STREAM("mobile_base_controller/wheel_radius: " << wheel_radius_);
        ROS_INFO_STREAM("mobile_base_controller/linear/x/max_velocity: " << max_velocity_);
        ROS_INFO_STREAM("MR_encoder_resolution: " << MR_encoder_resolution_);
        ROS_INFO_STREAM("MS_encoder_resolution: " << MS_encoder_resolution_);
        ROS_INFO_STREAM("gain: " << gain_);
        ROS_INFO_STREAM("trim: " << trim_);
        ROS_INFO_STREAM("motor_constant: " << motor_constant_);
        ROS_INFO_STREAM("PLC's IP Address: " << ip_address);
        ROS_INFO_STREAM("PLC's IP Port: " << ip_port);

        //Setup publisher for angular wheel joint velocity commands
        pub_wheel_cmd_velocities_ = nh_.advertise<amr_msgs::WheelsCmdStamped>("wheel_cmd_velocities", 10);

        // Publish Encoder ticks
        pub_encoder_ticks_ = nh_.advertise<amr_msgs::EncodersStamped> ("encoder_ticks", 10);

        // Publish Encoder target
        pub_encoder_target = nh_.advertise<std_msgs::Int32> ("encoder_target",10);

        // Publisj Error between Encoder ticks and Encoder target
        // pub_encoder_error = nh_.advertise<std_msgs::Int32> ("encoder_error",10);

        // Publish Measured Joint
        pub_measured_joint_states = nh_.advertise<sensor_msgs::JointState> ("measured_joint_states",10);

        // Setup subscriber for the button emergency stop
        //sub_emergency_stop_ = nh_.subscribe("emergency_stop",10, &AmrHWInterface::emergencyStopCallback, this);

        // Sub stop amr
        sub_stop_amr = nh_.subscribe("STOP_AMR",10, &AmrHWInterface::stop_amr_cb, this);

        // Sub hand control amr
        sub_hand_control_amr = nh_.subscribe("HAND_CONTROL_AMR",10, &AmrHWInterface::hand_control_cb, this);

        // Subscribe cmd_brake
        sub_cmd_brake = nh_.subscribe("cmd_brake",10, &AmrHWInterface::cmd_brake_cb, this);

        // Mode operating:
        MOTOR_MODE.WHEEL = "wheel";
        MOTOR_MODE.JOINT = "joint";
        MOTOR_MODE.SLIDER = "slider";

        // Initialize the hardware interface
        init(nh_, nh_);

        // Wait for socket PC connect success with PLC
        isSocketPcAndPlcOK(ros::Duration(10));

        // Reset Encoder (used during first launch of the hardware interface)
        set_origin_slider_motor(ros::Duration(25));
        reset_motor();
    }

 
    bool AmrHWInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        ROS_INFO("Initializing AMR Hardware Interface ...");
        num_joints_ = joint_names_.size();
        ROS_INFO("Number of joints: %d", (int)num_joints_);
        std::array<std::string, NUM_JOINTS> motor_names = {"left_motor", "right_motor", "slider_motor"};

        // ROS_INFO("REGISTERING... %s", operating_mode);

        for (unsigned int i = 0; i < num_joints_; i++)
        {
            // Create a JointStateHandle for each joint and register them with the 
            // JointStateInterface.
            hardware_interface::JointStateHandle joint_state_handle(joint_names_[i],
                                                                    &joint_positions_[i], 
                                                                    &joint_velocities_[i],
                                                                    &joint_efforts_[i]);
            joint_state_interface_.registerHandle(joint_state_handle);

            hardware_interface::JointHandle joint_handle(joint_state_handle, &joint_commands_[i]);

            if (operating_mode[i] == MOTOR_MODE.WHEEL) {
                // Create a JointHandle (read and write) for each controllable joint
                // using the read-only joint handles within the JointStateInterface and 
                // register them with the JointVelocityInterface.
                velocity_joint_interface_.registerHandle(joint_handle);

            } else if (operating_mode[i] == MOTOR_MODE.SLIDER) {
                // Create a JointHandle (read and write) for each controllable joint
                // using the read-only joint handles within the  JointStateInterface and 
                // register them with the JointPositionInterface.
                position_joint_interface_.registerHandle(joint_handle);

            } else {
                continue;
            }
            
            // Initialize joint states with zero values
            joint_positions_[i] = 0.0;
            joint_velocities_[i] = 0.0;
            joint_efforts_[i] = 0.0; // unused with diff_drive_controller

            joint_commands_[i] = 0.0;

            // Initialize encoder_ticks_ to zero because receiving meaningful
            // tick values from the microcontroller might take some time
            encoder_ticks_[i] = 0;
            encoder_ticks_prev[i] = 0;
            breaker_state[i] = false;

            // Initialize the pid controllers for the motors using the robot namespace
            std::string pid_namespace = "pid/" + motor_names[i];
            ROS_INFO_STREAM("pid namespace: " << pid_namespace);
            ros::NodeHandle nh(root_nh, pid_namespace);
            // TODO implement builder pattern to initialize values otherwise it is hard to see which parameter is what.
            pids_[i].init(nh, 0.8, 0.045, 0.0, 0.0001, 3.5, -3.5, false, slider_motor_spd, -slider_motor_spd);
            pids_[i].setOutputLimits(slider_motor_spd, -slider_motor_spd);
        }

        // Register the JointStateInterface containing the read only joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&joint_state_interface_);

        // Register the JointVelocityInterface containing the read/write joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&velocity_joint_interface_);
        
        // Register the JointPositionInterface containing the read/write joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&position_joint_interface_);

        stop_amr = false;
        hand_control = false;
        cmd_brake = false;
        uint8_t* bytes = sktcpclient_fx3u_.convert_uint16_to_2uint8((uint16_t)acc_lim_motor);
        accel_bytes[0] = bytes[0];
        accel_bytes[1] = bytes[1];
        bytes = sktcpclient_fx3u_.convert_uint16_to_2uint8((uint16_t)acc_lim_motor_break);
        accel_break_bytes[0] = bytes[0];
        accel_break_bytes[1] = bytes[1];
        bytes = sktcpclient_fx3u_.convert_uint16_to_2uint8((uint16_t)current_lim_motor);
        current_bytes[0] = bytes[0];
        current_bytes[1] = bytes[1];
        bytes = sktcpclient_fx3u_.convert_uint16_to_2uint8((uint16_t)current_lim_motor_break);
        current_break_bytes[0] = bytes[0];
        current_break_bytes[1] = bytes[1];

        ROS_INFO("Initialized hardware interface.");
        return true;
    }

    void AmrHWInterface::read(const ros::Time& time, const ros::Duration& period)
    {
        ros::Duration elapsed_time = period;
        // Doc thanh ghi PLC lay gia tri van toc va encoder
        // Sau khi lay duoc gia tri thanh ghi thuc hien giai ma roi gan gia tri cho joint state
        // std::vector<int32_t> encoder_ticks(num_joints_, 0);
        std::vector<int16_t> speeds(num_joints_, 0);

        uint8_t* register_data;

        register_data = sktcpclient_fx3u_.read_word_FX3U('D', (uint32_t)reg_data_motor[0], reg_data_motor_length * num_joints_);

        if (register_data[23] == 11){
            ROS_ERROR("Get data registers for position and velocity: ERROR");
        } else {
            
            for (std::size_t i = 0; i < num_joints_; ++i)
            {
                uint8_t speed_register[2] = {register_data[3 + i*6], register_data[2 + i*6]};
                uint8_t encoder_register[4] = {register_data[7 + i*6], register_data[6 + i*6],  register_data[5 + i*6], register_data[4 + i*6]};

                if ( i == 1) {
                    speeds[i] = -(*((int16_t*) speed_register));
                    encoder_ticks_[i] = -(*((int32_t*) encoder_register));
                    // encoder_ticks_[i] = *((int32_t*) encoder_register);
                } else {
                    speeds[i] = (*((int16_t*) speed_register));
                    encoder_ticks_[i] = *((int32_t*) encoder_register);
                }
                // ROS_WARN("SPEED: %d", speeds[i]);
                // ROS_WARN("ENCODER TICK[%d]: %d", i, encoder_ticks_[i]);

                if (operating_mode[i] == MOTOR_MODE.WHEEL ) {
                    joint_positions_[i] += sktcpclient_fx3u_.convertValue2Radian(encoder_ticks_prev[i], encoder_ticks_[i], MR_encoder_resolution_ , gear_ratio_);
                    joint_velocities_[i] = sktcpclient_fx3u_.convertValue2Velocity(speeds[i], gear_ratio_);
                } else if (operating_mode[i] == MOTOR_MODE.SLIDER) {
                    joint_positions_[i] = sktcpclient_fx3u_.convertPulse2Meter(encoder_ticks_[i], MR_encoder_resolution_ , gear_ratio_ * gear_ratio_pinion, pinion_gear_radius, rack_pinion_gear_efficiency);
                    joint_velocities_[i] = sktcpclient_fx3u_.convertValue2Velocity(speeds[i], gear_ratio_ * gear_ratio_pinion) * pinion_gear_radius;
                } else {
                    ROS_ERROR("Operating mode is Unknown, please check!");
                    return;
                }

                joint_efforts_[i] = 0.0; // unused

                encoder_ticks_prev[i] = encoder_ticks_[i];
            }
            // ROS_WARN("DELTA TICKS: %d", encoder_ticks_[0] - encoder_ticks_[1]);
            // ROS_WARN("Delta joint position: %f", joint_positions_[0] - joint_positions_[1]);

        }
        pub_encoder_ticks(encoder_ticks_);
        pub_measured_jointstate(joint_names_, joint_positions_, joint_velocities_);

        if (debug_)
        {
            const int width = 10;
            const char sep = ' ';
            std::stringstream ss;
            ss << std::left << std::setw(width) << std::setfill(sep) << "Read" << std::left << std::setw(width) << std::setfill(sep) << "ticks" << std::left << std::setw(width) << std::setfill(sep) << "angle" << std::left << std::setw(width) << std::setfill(sep) << "velocity" << std::endl;
            ss << std::left << std::setw(width) << std::setfill(sep) << "j1:" << std::left << std::setw(width) << std::setfill(sep) << encoder_ticks_[0] << std::left << std::setw(width) << std::setfill(sep) << joint_positions_[0] << std::left << std::setw(width) << std::setfill(sep) << joint_velocities_[0] << std::endl;
            ss << std::left << std::setw(width) << std::setfill(sep) << "j2:" << std::left << std::setw(width) << std::setfill(sep) << encoder_ticks_[1] << std::left << std::setw(width) << std::setfill(sep) << joint_positions_[1] << std::left << std::setw(width) << std::setfill(sep) << joint_velocities_[1];
            // ss << std::left << std::setw(width) << std::setfill(sep) << "j3:" << std::left << std::setw(width) << std::setfill(sep) << encoder_ticks_[2] << std::left << std::setw(width) << std::setfill(sep) << joint_positions_[2] << std::left << std::setw(width) << std::setfill(sep) << joint_velocities_[2];
            ROS_INFO_STREAM(std::endl << ss.str());
            //printState();
        }
    }

    void AmrHWInterface::write(const ros::Time& time, const ros::Duration& period)
    {
        ros::Duration elapsed_time = period;

        amr_msgs::WheelsCmdStamped wheel_cmd_msg;
        wheel_cmd_msg.header.stamp = ros::Time::now();

        std::vector<int16_t> motor_speed(num_joints_, 0);
        double pid_outputs[num_joints_];
        uint8_t data_transmit[reg_spd_cmd_length * 2 * num_joints_];
        uint8_t bit_breaker[num_joints_];


        // Write to robot hw
        // joint velocity commands from ros_control's RobotHW are in rad/s
        // adjusting k by gain and trim
        double motor_constant_right_inv = (gain_ + trim_) / motor_constant_;
        double motor_constant_left_inv = (gain_ - trim_) / motor_constant_;

        // Doi dau dong co cua banh trai de phu voi dau trong tf
        wheel_cmd_msg.wheels_cmd.angular_velocities.joint.push_back(joint_commands_[0]);
        wheel_cmd_msg.wheels_cmd.angular_velocities.joint.push_back(joint_commands_[1]);

        joint_commands_[0] = joint_commands_[0] * motor_constant_left_inv;
        joint_commands_[1] = -joint_commands_[1] * motor_constant_right_inv;
        
        // Decode commands for PLC: 
        for (int i = 0; i < 2; ++i)
        {
            // Motor driver cmd: [0]: 0-stop, 1-CCW, 3-CW
            //                   [1]: no use
            //                   [2],[3]: speed(RPM) - unsigned  
            //                   [4],[5]: acceleration(RPM/s) - unsigned
            //                   [6],[7]: current limit(0.1 Apeak) - unsigned
            if (operating_mode[i] == MOTOR_MODE.WHEEL) {
                motor_speed[i] =  sktcpclient_fx3u_.convertVelocity2Value(joint_commands_[i], gear_ratio_);
            } else if (operating_mode[i] == MOTOR_MODE.SLIDER) {
                motor_speed[i] = floor(pids_[i](encoder_ticks_[i],sktcpclient_fx3u_.convertMeter2Pulse(joint_commands_[i],MR_encoder_resolution_,
                gear_ratio_*gear_ratio_pinion,pinion_gear_radius), period));
            } else {
                ROS_ERROR("Operating mode is Unknown, please check!");
                return;
            }

            // Data transmit add bytes:
            if (stop_amr || cmd_brake ) {
                data_transmit[0 + i*8] = 0;
                data_transmit[1 + i*8] = 0;
                data_transmit[2 + i*8] = 0;
                data_transmit[3 + i*8] = 0;
                data_transmit[4 + i*8] = accel_break_bytes[0];
                data_transmit[5 + i*8] = accel_break_bytes[1];
                data_transmit[6 + i*8] = current_break_bytes[0];
                data_transmit[7 + i*8] = current_break_bytes[1];
                bit_breaker[i] = 0;                    
                // if (!breaker_state[i] && (i != 2)) {
                if (!breaker_state[i]) {
                    // ROS_WARN("BREAKER: ON");
                    // sktcpclient_fx3u_.write_bit_FX3U('M',(uint32_t)bit_break_motor[i],0);
                    breaker_state[i] = true;
                }
            } else if (motor_speed[i] == 0 || hand_control) {
                data_transmit[0 + i*8] = 1;
                data_transmit[1 + i*8] = 0;
                data_transmit[2 + i*8] = 0;
                data_transmit[3 + i*8] = 0;
                data_transmit[4 + i*8] = accel_break_bytes[0];
                data_transmit[5 + i*8] = accel_break_bytes[1];
                data_transmit[6 + i*8] = current_break_bytes[0];
                data_transmit[7 + i*8] = current_break_bytes[1];
                bit_breaker[i] = 1;
                if (breaker_state[i]) {
                    // ROS_WARN("BREAKER: OFF");
                    sktcpclient_fx3u_.write_bit_FX3U('M',(uint32_t)bit_break_motor[i],1);
                    breaker_state[i] = false;
                }  
            } else {
                uint8_t* speed_bytes_array = sktcpclient_fx3u_.convert_uint16_to_2uint8(abs(motor_speed[i]));
                uint8_t speed_bytes[2] = {speed_bytes_array[0], speed_bytes_array[1]};
                !!sktcpclient_fx3u_.sign(motor_speed[i]) ? data_transmit[0 + i*8] = 1 : data_transmit[0 + i*8] = 3;
                data_transmit[1 + i*8] = 0;
                data_transmit[2 + i*8] = speed_bytes[0];
                data_transmit[3 + i*8] = speed_bytes[1];
                data_transmit[4 + i*8] = accel_bytes[0];
                data_transmit[5 + i*8] = accel_bytes[1];
                data_transmit[6 + i*8] = current_bytes[0];
                data_transmit[7 + i*8] = current_bytes[1];
                bit_breaker[i] = 1;
                // if (breaker_state[i] && (i != 2)) {
                if (breaker_state[i]) {
                    // ROS_WARN("BREAKER: OFF");
                    sktcpclient_fx3u_.write_bit_FX3U('M',(uint32_t)bit_break_motor[i],1);
                    breaker_state[i] = false;
                }
                // sktcpclient_fx3u_.write_bit_FX3U('M',(uint32_t)bit_break_motor[i],1);
            }        
        }

        // for (int i=0; i< reg_spd_cmd_length * 2 * num_joints_; i++) {
        //     ROS_WARN("data_transmit[%d]: %d", i, data_transmit[i]);
        // }

        // ROS_WARN("MOTOR SPEED[%d]: %d",2,motor_speed[2]);
        // ROS_WARN("ENCODER target: %d", sktcpclient_fx3u_.convertMeter2Pulse(joint_commands_[2],MR_encoder_resolution_,
        //         gear_ratio_*gear_ratio_pinion,pinion_gear_radius));
        // ROS_WARN("ENCODER response: %d", encoder_ticks_prev[2]);

        // _pub_encoder_target(sktcpclient_fx3u_.convertMeter2Pulse(joint_commands_[2],MR_encoder_resolution_,
        //         gear_ratio_*gear_ratio_pinion,pinion_gear_radius));
        
        // _pub_encoder_error((abs(encoder_ticks_prev[2]) - abs(sktcpclient_fx3u_.convertMeter2Pulse(joint_commands_[2],MR_encoder_resolution_,
        //         gear_ratio_*gear_ratio_pinion,pinion_gear_radius))));

        // Send commands to PLC
        // sktcpclient_fx3u_.write_bit_FX3U('M',(uint32_t)bit_break_motor[0],num_joints_,bit_breaker);
        sktcpclient_fx3u_.write_word_FX3U('D',(uint32_t)reg_spd_cmd_motor[0],reg_spd_cmd_length * 2, data_transmit);
        pub_wheel_cmd_velocities_.publish(wheel_cmd_msg);
    }

    bool AmrHWInterface::isSocketPcAndPlcOK(const ros::Duration &timeout)
    {
        ROS_INFO("Intialize socket connect between PC and PLC!");
        ros::Time start = ros::Time::now();
        bool isConected = false;
        while ((!isConected) &&  (ros::Time::now() < start + timeout))
        {
            ROS_INFO("Waiting for socket (PC and PLC) connected...");
            isConected = sktcpclient_fx3u_.setup(ip_address, ip_port);

            ros::Duration(0.1).sleep();
        }
        if (isConected){
            ROS_INFO("PC connected to PLC successfully!");
            return true;
        } else {
            ROS_ERROR("PC can not connect to PLC (some error)!");
            return false;
        }
    }

    void AmrHWInterface::loadURDF(const ros::NodeHandle &nh, std::string param_name)
    {
        std::string urdf_string;
        urdf_model_ = new urdf::Model();

        // search and wait for robot_description on param server
        while (urdf_string.empty() && ros::ok())
        {
            std::string search_param_name;
            if (nh.searchParam(param_name, search_param_name))
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " << nh.getNamespace() << search_param_name);
                nh.getParam(search_param_name, urdf_string);
            }
            else
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " << nh.getNamespace() << param_name);
                nh.getParam(param_name, urdf_string);
            }

            usleep(100000);
        }

        if (!urdf_model_->initString(urdf_string))
            ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
        else
            ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
    }

    void AmrHWInterface::printState()
    {
        // WARNING: THIS IS NOT REALTIME SAFE
        // FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
        ROS_INFO_STREAM_THROTTLE(1, std::endl << printStateHelper());
    }

    std::string AmrHWInterface::printStateHelper()
    {
        std::stringstream ss;
        std::cout.precision(15);

        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            ss << "j" << i+1 << ": " << std::fixed << joint_positions_[i] << "\t ";
            ss << std::fixed << joint_velocities_[i] << "\t ";
            ss << std::fixed << joint_efforts_[i] << std::endl;
        }
        return ss.str();
    }

    std::string AmrHWInterface::printCommandHelper()
    {
        std::stringstream ss;
        std::cout.precision(15);
        ss << "    position     velocity         effort  \n";
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            ss << std::fixed << joint_commands_[i] << "\t ";
        }
        return ss.str();
    }

    double AmrHWInterface::linearToAngular(const double &distance) const
    {
        return distance / (wheel_diameter_ * 2.0);
    }

    void AmrHWInterface::pub_measured_jointstate(std::vector<std::string> name, double pos[NUM_JOINTS], double vel[NUM_JOINTS])
    {

        auto message = sensor_msgs::JointState();
        message.name.resize(num_joints_);
        message.position.resize(num_joints_);
        message.velocity.resize(num_joints_);

        message.header.stamp = ros::Time::now();
        for ( int i=0; i<num_joints_; ++i) {
            message.name[i] = name[i];
            message.position[i] = pos[i];
            message.velocity[i] = vel[i];
        }
        pub_measured_joint_states.publish(message);
    }

    void AmrHWInterface::pub_encoder_ticks(int32_t _encoder_ticks_[])
    {
        auto message = amr_msgs::EncodersStamped();
        
        for (int i=0; i<num_joints_; ++i){
            message.header.stamp = ros::Time::now();
            message.encoders.ticks[i] = _encoder_ticks_[i];
        }
        pub_encoder_ticks_.publish(message);
    }

    void AmrHWInterface::_pub_encoder_target(int32_t encoder_target){
        
        auto message = std_msgs::Int32();
        message.data = encoder_target;
        pub_encoder_target.publish(message);
    }

    // void AmrHWInterface::_pub_encoder_error(uint16_t encoder_error){

    //     auto message = std_msgs::Int32();
    //     message.data = encoder_error;
    //     pub_encoder_error.publish(message);
    // }

    void AmrHWInterface::stop_amr_cb(const std_msgs::Bool::ConstPtr& msg_stop_amr)
    {
        stop_amr = msg_stop_amr->data;
        breaker_state[0] = true;
        breaker_state[1] = true;
    }

    void AmrHWInterface::hand_control_cb(const std_msgs::Bool::ConstPtr& msg_hand_control)
    {
        hand_control = msg_hand_control->data;
    }

    void AmrHWInterface::cmd_brake_cb(const std_msgs::Bool::ConstPtr& msg_cmd_brake)
    {
        cmd_brake = msg_cmd_brake->data;
    }


    bool AmrHWInterface::set_origin_slider_motor(const ros::Duration &timeout)
    {
        ROS_INFO("Checking original state of slider motor!");
        ros::Time start = ros::Time::now();

        bool isOriginOK = sktcpclient_fx3u_.read_bit_FX3U('M',(uint32_t)bit_slider_origin[1]);

        if (!isOriginOK){
            sktcpclient_fx3u_.write_bit_FX3U('M',(uint32_t)bit_slider_origin[0],1);
        }

        while (!isOriginOK &&  (ros::Time::now() < start + timeout))
        {
            ROS_INFO("Setting slider motor to original postion...");
            isOriginOK = sktcpclient_fx3u_.read_bit_FX3U('M',(uint32_t)bit_slider_origin[1]);
            ros::Duration(0.5).sleep();
        }

        // ROS_WARN("IS_ORIGRIN = %d", isOriginOK);

        if (isOriginOK){
            ROS_INFO("Set slider motor's original position successfully!");
            return true;
        } else {
            ROS_ERROR("Can't set slider motor to origin position! Timeout!");
            return false;
        }
    }

    bool AmrHWInterface::reset_motor()
    {
        ROS_INFO("Reseting all motor...");
        uint8_t bits[num_joints_];
        for ( int i=0; i<num_joints_; ++i) {
            bits[i] = 1;
        }
        sktcpclient_fx3u_.write_bit_FX3U('M',(uint32_t)bit_reset_motor[0],num_joints_,bits);
        // sktcpclient_fx3u_.write_bit_FX3U('M',(uint32_t)bit_break_motor[0],num_joints_,bits);
        ROS_INFO("Reset all motor successfully!");

        return true;
    }
};
