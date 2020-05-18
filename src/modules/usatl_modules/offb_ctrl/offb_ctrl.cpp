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



OffboardControl::OffboardControl():
    ModuleParams(nullptr){
    parameters_updated();
}

int OffboardControl::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int OffboardControl::custom_command(int argc, char *argv[])
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

int OffboardControl::task_spawn(int argc, char *argv[])
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

bool OffboardControl::serial_init() {
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

void OffboardControl::run()
{
    serial_init();
	// Example: run the loop synchronized to the sensor_combined topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = sensor_combined_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	parameters_update_poll();

	while (!should_exit()) {
		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			struct sensor_combined_s sensor_combined;
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
			// TODO: do something with the data...
		}
        PX4_INFO("running OK!");
        parameters_update_poll();
	}

}

void OffboardControl::parameters_updated() {
    _ser_com_num = _param_ofc_ser_com.get();
    _ser_buadrate = _param_ofc_ser_baud.get();
    _print_debug_msg = _param_ofc_deb_prt.get();
    _drone_id = _param_ofc_cur_id.get();
}

void OffboardControl::parameters_update_poll()
{
	parameter_update_s param_upd;
	if (_params_sub.update(&param_upd)) {
		updateParams();
		parameters_updated();
	}
}

int OffboardControl::print_usage(const char *reason)
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

int offb_ctrl_main(int argc, char *argv[])
{
	return OffboardControl::main(argc, argv);
}
