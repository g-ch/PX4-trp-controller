/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "offb_ctrl.h"

OffboardControl::OffboardControl(): ModuleParams(nullptr)
{
    parameters_updated();
    _msg_sum_chk = 0x00;
    memset(&_vision_position, 0 , sizeof(_vision_position));
}

int
OffboardControl::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int
OffboardControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

OffboardControl *OffboardControl::instantiate(int argc, char *argv[])
{
    OffboardControl *instance = new OffboardControl();

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
    }

    return instance;
}

int
OffboardControl::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("offb_ctrl",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  2048,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);


    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

bool
OffboardControl::serial_init() {
    switch (_ser_com_num){
        case 0:
            _serial_fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY);
            break;
        case 1:
            _serial_fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY);
            break;
        case 2:
            _serial_fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY);
            break;
        case 3:
            _serial_fd = open("/dev/ttyS3", O_RDWR | O_NOCTTY);
            break;
        case 4:
            _serial_fd = open("/dev/ttyS4", O_RDWR | O_NOCTTY);
            break;
        case 5:
            _serial_fd = open("/dev/ttyS5", O_RDWR | O_NOCTTY);
            break;
    }
    if (_serial_fd < 0) {
        DPX4_INFO( "failed to open port: /dev/ttyS%d",_ser_com_num);
        return false;
    }
    int speed;
    switch (_ser_buadrate) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            PX4_INFO("ERR: baudrate: %d\n", _ser_buadrate);
            return -EINVAL;
    }
    struct termios uart_config;
    int termios_state;
    tcgetattr(_serial_fd, &uart_config);
    uart_config.c_oflag &= ~ONLCR;
    uart_config.c_cflag &= ~(CSTOPB|PARENB);
    uart_config.c_cflag &= ~CRTSCTS;
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        PX4_INFO("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }
    if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
        PX4_INFO("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }
    PX4_INFO("offb_ctrl ser init");
    return true;
}

bool
OffboardControl::parse_frame_head(uint8_t limit){
    _msg_sum_chk = 0x00;
    uint8_t count = 0;
    while(count++<limit){
        get_data();
        if(_cdata_buffer == FRAME_HEAD1)
        {
            get_data();
            if(_cdata_buffer==FRAME_HEAD2) return true;
        }
    }
    return false;
}

bool
OffboardControl::msg_checked(){
    get_data();
    bool res = (_cdata_buffer==_msg_sum_chk);
    _msg_sum_chk = 0x00;
    return res;
}

void
OffboardControl::run()
{
    if(!serial_init()){
        mavlink_debug_info("offb_ctrl /dev/ttyS%d  init failed",_ser_com_num);
        DPX4_INFO( "failed to open port: /dev/ttyS%d",_ser_com_num);

    }
    mavlink_debug_info("offb_ctrl /dev/ttyS%d serial init",_ser_com_num);
	parameters_update_poll();
	DPX4_INFO("UAV ID:%d",_drone_id);
    mavlink_debug_info("UAV ID:%d",_drone_id);



	while (!should_exit()) {
        //解析帧头数据
		if(!parse_frame_head(30)){
            DPX4_INFO( "failed to parse offb_ctrl uart check com and baudrate!!");
            mavlink_debug_info("failed to parse frame head 30 times");
            continue;
		}else{
            // DPX4_INFO("HEAD OK");
		}
		//得到消息类型
		_current_msg_type = parse_msg_type<MESSAGE_TYPE>();
		switch (_current_msg_type){
            case COMMAND:
                _current_mode = parse_msg_type<MODE>();
                _current_offboard_command = parse_msg_type<OFFBOARD_COMMAND>();
                _current_arm_command = parse_msg_type<ARM_COMMAND>();
                _current_takeoff_command = parse_msg_type<TAKEOFF_COMMAND >();
                _current_back_info = parse_msg_type<BACK_INFO >();

                _msg_sum_chk = _current_msg_type+_current_mode+_current_offboard_command+_current_arm_command+
                                _current_takeoff_command+_current_back_info;
                if(msg_checked()){
                    // DPX4_INFO("COMMAND CHECKED OK");
                    process_command();
                }else{
                    DPX4_INFO("ERROR!! COMMAND MSG CHECK FAILED!!!");
                }
                get_data(); //读取包尾
                break;
            case CURRENT_STATE :
                _current_send_state = parse_msg_type<SEND_CURRENT_STATE >();
                process_recv_state_data();
                get_data(); //读取包尾
                send_back_msg();
                break;
		    case SETPOINT:
		        _current_sp_type = parse_msg_type<SETPOINT_TYPE >();
		        process_recv_sp_data();
                get_data(); //读取包尾
		        send_back_msg();
                break;
		    case MESSAGE_BACK:
                DPX4_INFO("ERROR! Message back type should be uav send to gcs");
		        break;
            default:
                DPX4_INFO( "failed to parse offb_ctrl msg type!!");
                break;
		}
        parameters_update_poll(); //更新参数
	}
}

void OffboardControl::get_data(){
    _cdata_buffer = '0';

    auto _a =  read(_serial_fd,&_cdata_buffer,1);
    _a = _a;
}

void
OffboardControl::parse_income_i3data(bool clear_sum){
    if(clear_sum)_msg_sum_chk = 0x00;
    memset(&_char_12buffer, 0,sizeof(_char_12buffer));
    for (int i = 0; i < 12; ++i) {
        _cdata_buffer = '0';
        get_data();
        _char_12buffer[i] = _cdata_buffer;
        _msg_sum_chk+=_char_12buffer[i];
    }
    for (int j = 0; j < 3; ++j) {
        _income_3_idata[j] = (int)(_char_12buffer[4*j+3]<<24|_char_12buffer[4*j+2]<<16|
                                    _char_12buffer[4*j+1]<<8|_char_12buffer[4*j]);
        _income_3_fdata[j] = _income_3_idata[j]/SCALE;
    }
}

void
OffboardControl::parse_income_i1data(bool clear_sum){
    if(clear_sum)_msg_sum_chk = 0x00;
    memset(&_char_4buffer, 0,sizeof(_char_4buffer));
    for (int i = 0; i < 4; ++i) {
        _cdata_buffer = '0';
        get_data();
        _char_4buffer[i] = _cdata_buffer;
        _msg_sum_chk+=_char_4buffer[i];
    }
    _income_1_idata = (int)(_char_4buffer[3]<<24|_char_4buffer[2]<<16|_char_4buffer[1]<<8|_char_4buffer[0]);
}

void
OffboardControl::parameters_updated() {
    _ser_com_num = _param_ofc_ser_com.get();
    _ser_buadrate = _param_ofc_ser_baud.get();
    _print_debug_msg = _param_ofc_deb_prt.get();
    _drone_id = _param_ofc_cur_id.get();
}

void
OffboardControl::parameters_update_poll()
{
	parameter_update_s param_upd;
	if (_params_sub.update(&param_upd)) {
		updateParams();
		parameters_updated();
	}
}

int
OffboardControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void
OffboardControl::process_command() {
    switch (_current_mode){
        case GPS:
            break;
        case VICON:
            break;
        case CAMERA:
            break;
    }
    process_offboard_enable_cmd();
    process_arm_command();
}

void
OffboardControl::process_arm_command(){
    // mavlink_debug_info("ready process arm cmd");
    switch (_current_arm_command){
        case ARM_DO_NOTHING:
            break;
        case ARMED:
            _vehicle_status_sub.copy(&_vehicle_status);
            _vcmd.timestamp = hrt_absolute_time();
            _vcmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
            _vcmd.target_component = _vehicle_status.component_id;
            _vcmd.target_system = _vehicle_status.system_id;
            _vcmd.source_system = _vehicle_status.system_id;
            _vcmd.source_component = _vehicle_status.component_id;
            _vcmd.confirmation = 1;
            _vcmd.param1 = 1.0f;//解锁
            if(_vehicle_status.arming_state!= vehicle_status_s:: ARMING_STATE_ARMED){
                mavlink_debug_info("Not armed publish arm cmd...arming..");
                orb_publish_auto(ORB_ID(vehicle_command),
                                 &_cmd_pub,
                                 &_vcmd, nullptr, ORB_PRIO_DEFAULT);
            }
            break;
        case DISARMED:
            _vehicle_status_sub.copy(&_vehicle_status);
            _vcmd.timestamp = hrt_absolute_time();
            _vcmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
            _vcmd.target_component = _vehicle_status.component_id;
            _vcmd.target_system = _vehicle_status.system_id;
            _vcmd.source_system = _vehicle_status.system_id;
            _vcmd.source_component = _vehicle_status.component_id;
            _vcmd.confirmation = 1;
            _vcmd.param2 = 0.0f;//上锁
            if(_vehicle_status.arming_state==vehicle_status_s::ARMING_STATE_ARMED){
                mavlink_debug_info("Armed publish Disarm cmd... Disarming ");
                orb_publish_auto(ORB_ID(vehicle_command),
                                 &_cmd_pub,
                                 &_vcmd, nullptr, ORB_PRIO_DEFAULT);
            }


            break;
    }
}

void
OffboardControl::send_back_msg(){
    _current_msg_type = MESSAGE_BACK;
    if(_current_back_info==ONLY_POSITION){
        _local_position_sub.copy(&_local_position);
        send_frame_head();
        _msg_sum_chk = 0x00;
        send_msg_type(_current_msg_type);

        send_msg_type(_current_back_info);
        _income_3_idata[0] = (int)(_local_position.x*SCALE);
        _income_3_idata[1] = (int)(_local_position.y*SCALE);
        _income_3_idata[2] = (int)(_local_position.z*SCALE);
        send_int_data(_income_3_idata[0],1);
        send_int_data(_income_3_idata[1],0);
        send_int_data(_income_3_idata[2],0);
        if(write(_serial_fd,&_msg_sum_chk,1)>0){}
        _msg_sum_chk = 0x00;
        send_frame_tail();
    }else if(_current_back_info==ONLY_IMU){
        _local_position_sub.copy(&_local_position);
        send_frame_head();
        _msg_sum_chk = 0x00;
        send_msg_type(_current_msg_type);
        send_msg_type(_current_back_info);
        _income_3_idata[0] = (int)(_local_position.ax*SCALE);
        _income_3_idata[1] = (int)(_local_position.ay*SCALE);
        _income_3_idata[2] = (int)(_local_position.az*SCALE);
        send_int_data(_income_3_idata[0],1);
        send_int_data(_income_3_idata[1],0);
        send_int_data(_income_3_idata[2],0);
        if(write(_serial_fd,&_msg_sum_chk,1)>0){}
        _msg_sum_chk = 0x00;
        send_frame_tail();

    }else if(_current_back_info==POSITION_AND_IMU){
        _local_position_sub.copy(&_local_position);
        send_frame_head();
        _msg_sum_chk = 0x00;
        send_msg_type(_current_msg_type);
        send_msg_type(_current_back_info);
        _income_3_idata[0] = (int)(_local_position.x*SCALE);
        _income_3_idata[1] = (int)(_local_position.y*SCALE);
        _income_3_idata[2] = (int)(_local_position.z*SCALE);
        send_int_data(_income_3_idata[0],1);
        send_int_data(_income_3_idata[1],0);
        send_int_data(_income_3_idata[2],0);
        _income_3_idata[0] = (int)(_local_position.ax*SCALE);
        _income_3_idata[1] = (int)(_local_position.ay*SCALE);
        _income_3_idata[2] = (int)(_local_position.az*SCALE);
        send_int_data(_income_3_idata[0],0);
        send_int_data(_income_3_idata[1],0);
        send_int_data(_income_3_idata[2],0);
        if(write(_serial_fd,&_msg_sum_chk,1)>0){}
        _msg_sum_chk = 0x00;
        send_frame_tail();
    }else if(_current_back_info==DO_NOTHING){
    }
}

void
OffboardControl::process_recv_state_data() {
    //发布position
    if(_current_send_state==SEND_NED_POSITION_RPY){
        _vision_position.timestamp = hrt_absolute_time();
        parse_income_i3data(true);
        _vision_position.x = (float)_income_3_idata[0]/SCALE;
        _vision_position.y = (float)_income_3_idata[1]/SCALE;
        _vision_position.z = (float)_income_3_idata[2]/SCALE;
        parse_income_i3data(false);
        float rpy[3];
        rpy[0] = (float)_income_3_idata[0]/SCALE;
        rpy[1] = (float)_income_3_idata[1]/SCALE;
        rpy[2] = (float)_income_3_idata[2]/SCALE;
        matrix::Quatf q(matrix::Eulerf(rpy[0], rpy[1], rpy[2]));
        q.copyTo(_vision_position.q);
        if(msg_checked()){
            // DPX4_INFO("XYZRPY: %2.4f, %2.4f, %2.4f, %2.4f, %2.4f, %2.4f",
            //         (double)_vision_position.x,(double)_vision_position.y,(double)_vision_position.z,
            //          (double)rpy[0],(double)rpy[1],(double)rpy[2]
            // );
            orb_publish_auto(ORB_ID(vehicle_visual_odometry),
                    &_vision_position_pub,
                    &_vision_position, nullptr, ORB_PRIO_DEFAULT);
        }else{
            _msg_sum_chk = 0x00;
            memset(&_vision_position,0, sizeof(_vision_position));
        }
    }
    else if(_current_send_state==SEND_NED_POSITION){
        _vision_position.timestamp = hrt_absolute_time();
        parse_income_i3data(true);
        _vision_position.x = (float)_income_3_idata[0]/SCALE;
        _vision_position.y = (float)_income_3_idata[1]/SCALE;
        _vision_position.z = (float)_income_3_idata[2]/SCALE;
        if(msg_checked()){
            orb_publish_auto(ORB_ID(vehicle_visual_odometry),
                             &_vision_position_pub,
                             &_vision_position, nullptr, ORB_PRIO_DEFAULT);
        }else{
            _msg_sum_chk = 0x00;
            memset(&_vision_position,0, sizeof(_vision_position));
        }
    }
}

void
OffboardControl::process_recv_sp_data() {
    _offboard_control_mode.timestamp = hrt_absolute_time();
    if(_current_sp_type==ATTITUDE_SP){
        parse_income_i3data(true);
        parse_income_i1data(false);
        if(msg_checked()){
            _offboard_control_mode.ignore_thrust = 0;
            _offboard_control_mode.ignore_attitude = 0;
            _offboard_control_mode.ignore_position = 1;
            _offboard_control_mode.ignore_velocity = 1;
            _offboard_control_mode.ignore_acceleration_force = 1;
            _offboard_control_mode.ignore_bodyrate_x = 1;
            _offboard_control_mode.ignore_bodyrate_y = 1;
            _offboard_control_mode.ignore_bodyrate_z = 1;
            _offboard_control_mode.timestamp = hrt_absolute_time();
            orb_publish_auto(ORB_ID(offboard_control_mode), &_offboard_control_mode_pub,
                    &_offboard_control_mode, nullptr,ORB_PRIO_DEFAULT);
            _control_mod_sub.copy(&_control_mode);
            if(_control_mode.flag_control_offboard_enabled){
                // mavlink_debug_info("offboard enable publish att sp");
                if(!_offboard_control_mode.ignore_attitude){
                    _att_sp.timestamp = hrt_absolute_time();
                    matrix::Eulerf sp_euler(_income_3_fdata[0],_income_3_fdata[1],_income_3_fdata[2]);
                    matrix::Quatf sp_q(sp_euler);
                    sp_q.copyTo(_att_sp.q_d);
                    _att_sp.q_d_valid = true;
                    //发布姿态控制量
                    _att_sp.roll_body = _income_3_fdata[0];
                    _att_sp.pitch_body = _income_3_fdata[1];
                    _att_sp.yaw_body = _income_3_fdata[2];
                    _att_sp.yaw_sp_move_rate = 0.0f;
                    if (!_offboard_control_mode.ignore_thrust) {
                        _att_sp.thrust_body[2] = -(float)_income_1_idata/SCALE;
                    }
                }
                orb_publish_auto(ORB_ID(vehicle_attitude_setpoint),
                                              &_att_sp_pub,
                                              &_att_sp, nullptr, ORB_PRIO_DEFAULT);
            }

        }else{
            DPX4_INFO("ERROR!! ATTSP MSG CHECK FAILED!!!");
        }
    }else if(_current_sp_type == VELOCITY_SP){
        position_setpoint_triplet_s  _pos_sp_triplet{};
        parse_income_i3data(true);
        if(msg_checked()){
            _offboard_control_mode.ignore_velocity = 0;
            _offboard_control_mode.ignore_thrust = 1;
            _offboard_control_mode.ignore_position = 1;
            _offboard_control_mode.ignore_acceleration_force = 1;
            _offboard_control_mode.ignore_bodyrate_x = 1;
            _offboard_control_mode.ignore_bodyrate_y = 1;
            _offboard_control_mode.ignore_bodyrate_z = 1;
            _offboard_control_mode.ignore_attitude = 1;
            _offboard_control_mode.timestamp = hrt_absolute_time();
            orb_publish_auto(ORB_ID(offboard_control_mode), &_offboard_control_mode_pub,
                             &_offboard_control_mode, nullptr,ORB_PRIO_DEFAULT);
            _control_mod_sub.copy(&_control_mode);
            if (_current_takeoff_command == LAND || _current_takeoff_command == EMER_LAN) {
                send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);
                _pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;
            } else if (_current_takeoff_command == TAKEOFF) {
                _pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;
                send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF);

            } else {
                _pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
            }
            _pos_sp_triplet.current.valid = true;
            _pos_sp_triplet.current.velocity_valid = true;
            _pos_sp_triplet.current.acceleration_valid = false;
            _pos_sp_triplet.current.alt_valid = false;
            _pos_sp_triplet.current.yaw_valid = false;
            _pos_sp_triplet.current.yawspeed_valid = false;
            _pos_sp_triplet.previous.valid = false;
            _pos_sp_triplet.next.valid = false;
            _pos_sp_triplet.next.position_valid = false;
            _pos_sp_triplet.current.timestamp = hrt_absolute_time();
            if (!_offboard_control_mode.ignore_velocity) {
                _pos_sp_triplet.current.velocity_valid = true;
                _pos_sp_triplet.current.vx = _income_3_fdata[0];
                _pos_sp_triplet.current.vy = _income_3_fdata[1];
                _pos_sp_triplet.current.vz = _income_3_fdata[2];
            }
            _pos_sp_triplet_pub.publish(_pos_sp_triplet);



        }else{
            DPX4_INFO("ERROR!! POSSP MSG CHECK FAILED!!!");
        }
    }else if(_current_sp_type == LOCAL_POSITION_SP){
        position_setpoint_triplet_s  _pos_sp_triplet{};
        //Bitmask toindicate which dimensions should be ignored(1 means ignore,0 means not ignore; Bit 10 must set to 0)
        //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp,
        // bit 11:yaw, bit 12:yaw_rate Bit 10 should set to 0, means is not force sp
        uint16_t type_mask = 0b110111111000;
        parse_income_i3data(true);
        if(msg_checked()){
            _control_mod_sub.copy(&_control_mode);
            _offboard_control_mode.timestamp = hrt_absolute_time();
            _offboard_control_mode.ignore_position = (bool)(type_mask & 0x7);
            _offboard_control_mode.ignore_alt_hold = (bool)(type_mask & 0x4);
            _offboard_control_mode.ignore_velocity = (bool)(type_mask & 0x38);
            _offboard_control_mode.ignore_acceleration_force = (bool)(type_mask & 0x1C0);
            _offboard_control_mode.ignore_acceleration_force = true;
            _offboard_control_mode.ignore_attitude = (bool)(type_mask & 0x400);
            _offboard_control_mode.ignore_bodyrate_x = (bool)(type_mask & 0x800);
            _offboard_control_mode.ignore_bodyrate_y = (bool)(type_mask & 0x800);
            _offboard_control_mode.ignore_bodyrate_z = (bool)(type_mask & 0x800);
            orb_publish_auto(ORB_ID(offboard_control_mode), &_offboard_control_mode_pub,
                             &_offboard_control_mode, nullptr,ORB_PRIO_DEFAULT);
            if(_control_mode.flag_control_offboard_enabled){
                /* yaw ignore flag mapps to ignore_attitude */
                bool is_takeoff_sp = (bool)(type_mask & 0x1000);
                bool is_land_sp = (bool)(type_mask & 0x2000);
                bool is_loiter_sp = (bool)(type_mask & 0x3000);
                bool is_idle_sp = (bool)(type_mask & 0x4000);
                _pos_sp_triplet.timestamp = hrt_absolute_time();
                _pos_sp_triplet.previous.valid = false;
                _pos_sp_triplet.next.valid = false;
                _pos_sp_triplet.current.valid = true;

                if (is_loiter_sp) {
                    _pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;

                } else if (is_takeoff_sp) {
                    _pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

                } else if (is_land_sp) {
                    _pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;

                } else if (is_idle_sp) {
                    _pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;

                } else {
                    _pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
                }

                /* set the local pos values */
                if (!_offboard_control_mode.ignore_position) {

                    _pos_sp_triplet.current.position_valid = true;
                    _pos_sp_triplet.current.x = _income_3_fdata[0];
                    _pos_sp_triplet.current.y = _income_3_fdata[1];
                    _pos_sp_triplet.current.z = _income_3_fdata[2];
                    _pos_sp_triplet.current.vx = 0.0;
                    _pos_sp_triplet.current.vy = 0.0;
                    _pos_sp_triplet.current.vz = 0.0;

                } else {
                    _pos_sp_triplet.current.position_valid = false;
                }

                /* set the local vel values */
                if (!_offboard_control_mode.ignore_velocity) {

                    _pos_sp_triplet.current.velocity_valid = true;
                    _pos_sp_triplet.current.vx = _income_3_fdata[0];
                    _pos_sp_triplet.current.vy = _income_3_fdata[1];
                    _pos_sp_triplet.current.vz = _income_3_fdata[2];
                    _pos_sp_triplet.current.velocity_frame = 1;

                } else {
                    _pos_sp_triplet.current.velocity_valid = false;
                }
                _pos_sp_triplet.timestamp = hrt_absolute_time();
                _pos_sp_triplet.current.valid = true;
                _pos_sp_triplet.current.position_valid = true;
                _pos_sp_triplet_pub.publish(_pos_sp_triplet);
                position_setpoint_triplet_s  _pos_sp_triplet_buffer{};
                _pos_sp_triplet_sub.copy(&_pos_sp_triplet_buffer);
                // if(_pos_sp_triplet_buffer.current.valid&&_pos_sp_triplet_buffer.current.position_valid){
                //     mavlink_debug_info("published sub then all valid");
                // }
                // DPX4_INFO("RECV POSSP MSG publish!!!%2.4f ,%2.4f ,%2.4f ",
                //         (double)_income_3_fdata[0],
                //         (double)_income_3_fdata[1],
                //         (double)_income_3_fdata[2]
                //         );
            }

        }else{
            DPX4_INFO("ERROR!! SP MSG CHECK FAILED!!!");
        }
    }else if(_current_sp_type == GLOBAL_POSITION_SP){
        parse_income_i3data(true);
        if(msg_checked()){
            mavlink_debug_info("error recv global position sp");
        }else{
            mavlink_debug_info("error parse  global position sp");
        }
    }

}

void
OffboardControl::send_vehicle_command(uint16_t cmd, float param1, float param2)
{
    _vcmd.timestamp = hrt_absolute_time();
    _vcmd.param1 = param1;
    _vcmd.param2 = param2;
    _vcmd.param3 = NAN;
    _vcmd.param4 = NAN;
    _vcmd.param5 = (double)NAN;
    _vcmd.param6 = (double)NAN;
    _vcmd.param7 = NAN;
    _vcmd.command = cmd;
    _vcmd.from_external = 1;
    _vcmd.confirmation = 1;
    _vcmd.target_system = _vehicle_status.system_id;
    _vcmd.target_component = _vehicle_status.component_id;
    _vcmd.source_system = _vehicle_status.system_id;
    _vcmd.source_component = _vehicle_status.component_id;
    orb_publish_auto(ORB_ID(vehicle_command),
                     &_cmd_pub,
                     &_vcmd, nullptr, ORB_PRIO_DEFAULT);
}


void
OffboardControl::process_offboard_enable_cmd() {
    // mavlink_debug_info("ready process offboard cmd");
    _vehicle_status_sub.copy(&_vehicle_status);
    _vcmd.timestamp = hrt_absolute_time();
    _vcmd.target_component = _vehicle_status.component_id;
    _vcmd.target_system = _vehicle_status.system_id;
    _vcmd.source_system = _vehicle_status.system_id;
    _vcmd.source_component = _vehicle_status.component_id;
    switch(_current_offboard_command){

        case POS_IN:
            _internal_state.main_state = commander_state_s::MAIN_STATE_POSCTL;
            _internal_state.timestamp = hrt_absolute_time();
            orb_publish_auto(ORB_ID(commander_state), &_internal_state_pub,
                             &_internal_state,
                             nullptr,ORB_PRIO_DEFAULT);
            break;
        case ALT_IN:
            _internal_state.main_state = commander_state_s::MAIN_STATE_ALTCTL;
            _internal_state.timestamp = hrt_absolute_time();
            orb_publish_auto(ORB_ID(commander_state), &_internal_state_pub,
                             &_internal_state,
                             nullptr,ORB_PRIO_DEFAULT);
            break;

        case TRY_OUT:
            if(!_already_try_out){
                orb_publish_auto(ORB_ID(offboard_control_mode), &_offboard_control_mode_pub,
                                                                   &_offboard_control_mode,
                                                                   nullptr,ORB_PRIO_DEFAULT);
                send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_GUIDED_ENABLE,0);
                mavlink_debug_info("try out offboard cmd published");
                _already_try_out = _vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
            }
            break;
        case TRY_IN:
            if(!_already_try_in){
                orb_publish_auto(ORB_ID(offboard_control_mode), &_offboard_control_mode_pub,
                                 &_offboard_control_mode, nullptr,ORB_PRIO_DEFAULT);

                send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_GUIDED_ENABLE,1);

                mavlink_debug_info("try in offboard cmd published");
                _already_try_in = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
            }else{
                mavlink_debug_info("already in offboard ");
            }
            break;
        case STAY_IN:

            orb_publish_auto(ORB_ID(offboard_control_mode), &_offboard_control_mode_pub,
                             &_offboard_control_mode, nullptr,ORB_PRIO_DEFAULT);
            if(_vehicle_status.nav_state!=vehicle_status_s::NAVIGATION_STATE_OFFBOARD){
                send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_GUIDED_ENABLE,1);
                _cmd_ack_sub.copy(&_vcmd_ack);
                mavlink_debug_info("not in offboard stay in offboard cmd published");
                if(_vcmd_ack.result==0){
                    mavlink_debug_info("Successful Stay in Offboard");
                }
            }

            break;
        case STAY_OUT:
            send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_GUIDED_ENABLE,0);
            _cmd_ack_sub.copy(&_vcmd_ack);
            mavlink_debug_info("stay out offboard cmd published");
            break;
        case OFF_DO_NOTHING:
            break;
    }
}

void
OffboardControl::send_int_data(int a,bool clear){
    if(clear) _msg_sum_chk = 0x00;
    _char_4buffer[0] = BYTE3(a);_msg_sum_chk+=_char_4buffer[0];
    _char_4buffer[1] = BYTE2(a);_msg_sum_chk+=_char_4buffer[1];
    _char_4buffer[2] = BYTE1(a);_msg_sum_chk+=_char_4buffer[2];
    _char_4buffer[3] = BYTE0(a);_msg_sum_chk+=_char_4buffer[3];
    if(write(_serial_fd,_char_4buffer,4)>0){}
}

void
OffboardControl::send_frame_head() {
    _cdata_buffer = 0xfe;
    if(write(_serial_fd,&_cdata_buffer,1)>0){}
    _cdata_buffer = 0x22;
    if(write(_serial_fd,&_cdata_buffer,1)>0){}
}

void
OffboardControl::send_frame_tail() {
    _cdata_buffer = 0xee;
    if(write(_serial_fd,&_cdata_buffer,1)>0){}
}

int
offb_ctrl_main(int argc, char *argv[])
{
	return OffboardControl::main(argc, argv);
}
