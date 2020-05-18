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
    memset(&vision_position, 0 , sizeof(vision_position));
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
        err(1, "failed to open port: /dev/ttyS%d",_ser_com_num);
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
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        PX4_INFO("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }
    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        PX4_INFO("ERR: %d (cfsetospeed)\n", termios_state);
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
        err(1, "failed to open port: /dev/ttyS%d",_ser_com_num);
    }
	parameters_update_poll();
	while (!should_exit()) {
	    //解析帧头数据
		if(!parse_frame_head(30)){
            err(1, "failed to parse offb_ctrl uart check com and baudrate!!");
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
                _msg_sum_chk = _current_mode+_current_offboard_command+_current_arm_command+
                                _current_takeoff_command+_current_back_info;
                if(msg_checked()){
                    process_command();
                }else{
                    err(1,"ERROR!! COMMAND MSG CHECK FAILED!!!");
                }
                break;
            case CURRENT_STATE :
                _current_send_state = parse_msg_type<SEND_CURRENT_STATE >();
                process_recv_state_data();
                send_back_msg();
                break;
		    case SETPOINT:
		        _current_sp_type = parse_msg_type<SETPOINT_TYPE >();
		        process_recv_sp_data();

		        send_back_msg();
                break;
		    case MESSAGE_BACK:
                err(1,"ERROR! Message back type should be uav send to gcs");
		        break;
            default:
                err(1, "failed to parse offb_ctrl msg type!!");
                break;
		}
		get_data(); //读取包尾
        parameters_update_poll(); //更新参数
	}
}

void
OffboardControl::parse_income_i3data(bool clear_sum){
    if(clear_sum)_msg_sum_chk = 0x00;
    memset(&_char_12buffer, 0,sizeof(_char_12buffer));
    for (int i = 0; i < 12; ++i) {
        _cdata_buffer = '0';
        read(_serial_fd,&_cdata_buffer,1);
        _char_12buffer[i] = _cdata_buffer;
        _msg_sum_chk+=_char_12buffer[i];
    }
    for (int j = 0; j < 3; ++j) {
        _income_3_idata[j] = (int)(_char_12buffer[4*j]<<24|_char_12buffer[4*j+1]<<16|
                                    _char_12buffer[4*j+2]<<8|_char_12buffer[4*j+3]);
    }
}

void
OffboardControl::parse_income_i1data(bool clear_sum){
    if(clear_sum)_msg_sum_chk = 0x00;
    memset(&_char_4buffer, 0,sizeof(_char_4buffer));
    for (int i = 0; i < 4; ++i) {
        _cdata_buffer = '0';
        read(_serial_fd,&_cdata_buffer,1);
        _char_4buffer[i] = _cdata_buffer;
        _msg_sum_chk+=_char_4buffer[i];
    }
    _income_1_idata = (int)(_char_4buffer[0]<<24|_char_4buffer[1]<<16|_char_4buffer[2]<<8|_char_4buffer[3]);
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
}

void
OffboardControl::send_back_msg(){
    if(_current_back_info==ONLY_POSITION){

    }else if(_current_back_info==ONLY_IMU){

    }else if(_current_back_info==POSITION_AND_IMU){

    }
}

void OffboardControl::process_recv_state_data() {
    if(_current_send_state==SEND_NED_POSITION_RPY){
        vision_position.timestamp = hrt_absolute_time();
        parse_income_i3data(true);
        vision_position.x = (float)_income_3_idata[0]/SCALE;
        vision_position.y = (float)_income_3_idata[1]/SCALE;
        vision_position.z = (float)_income_3_idata[2]/SCALE;
        parse_income_i3data(false);
        float rpy[3];
        rpy[0] = (float)_income_3_idata[0]/SCALE;
        rpy[1] = (float)_income_3_idata[1]/SCALE;
        rpy[2] = (float)_income_3_idata[2]/SCALE;
        matrix::Quatf q(matrix::Eulerf(rpy[0], rpy[1], rpy[2]));
        q.copyTo(vision_position.q);
        if(msg_checked()){
            orb_publish_auto(ORB_ID(vehicle_visual_odometry),
                    &_vision_position_pub,
                    &vision_position, nullptr, ORB_PRIO_DEFAULT);
        }else{
            _msg_sum_chk = 0x00;
            memset(&vision_position,0, sizeof(vision_position));
        }
    }else if(_current_send_state==SEND_NED_POSITION){
        vision_position.timestamp = hrt_absolute_time();
        parse_income_i3data(true);
        vision_position.x = (float)_income_3_idata[0]/SCALE;
        vision_position.y = (float)_income_3_idata[1]/SCALE;
        vision_position.z = (float)_income_3_idata[2]/SCALE;
        if(msg_checked()){
            orb_publish_auto(ORB_ID(vehicle_visual_odometry),
                             &_vision_position_pub,
                             &vision_position, nullptr, ORB_PRIO_DEFAULT);
        }else{
            _msg_sum_chk = 0x00;
            memset(&vision_position,0, sizeof(vision_position));
        }
    }
}

void OffboardControl::process_recv_sp_data() {
    if(_current_sp_type==ATTITUDE_SP){
        parse_income_i3data(true);
        parse_income_i1data(false);
        if(msg_checked()){
            //TODO: publish data
        }else{
            err(1,"ERROR!! STATE MSG CHECK FAILED!!!");
        }
    }else if(_current_sp_type == VELOCITY_SP){
        parse_income_i3data(true);
        if(msg_checked()){
            //TODO: publish data
        }else{
            err(1,"ERROR!! STATE MSG CHECK FAILED!!!");
        }
    }else if(_current_sp_type == LOCAL_POSITION_SP){
        parse_income_i3data(true);
        if(msg_checked()){
            //TODO: publish data
        }else{
            err(1,"ERROR!! STATE MSG CHECK FAILED!!!");
        }
    }else if(_current_sp_type == GLOBAL_POSITION_SP){
        parse_income_i3data(true);
        if(msg_checked()){
            //TODO: publish data
        }else{
            err(1,"ERROR!! STATE MSG CHECK FAILED!!!");
        }
    }

}


int
offb_ctrl_main(int argc, char *argv[])
{
	return OffboardControl::main(argc, argv);
}
