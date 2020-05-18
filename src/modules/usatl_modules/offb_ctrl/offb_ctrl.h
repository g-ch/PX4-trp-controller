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

#pragma once
#include <cstdio>
#include <termios.h>
#include <unistd.h>
#include <px4_module.h>
#include <cstring>
#include <cerrno>
#include <cmath>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <mathlib/math/filter/LowPassFilter2pVector3f.hpp>
#include <matrix/matrix/math.hpp>
#include <matrix/matrix/math.hpp>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/distance_sensor.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <matrix/math.hpp>
#include <px4_module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/landing_gear.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include "message_type.h"
extern "C" __EXPORT int offb_ctrl_main(int argc, char *argv[]);

class OffboardControl : public ModuleBase<OffboardControl>, public ModuleParams
{
public:
    OffboardControl();

	virtual ~OffboardControl() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	static OffboardControl *instantiate(int argc, char *argv[]);
	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool serial_init();

private:

	uORB::Subscription    _params_sub{ORB_ID(parameter_update)};			/**< parameter updates subscription */
	uORB::Subscription    _imu_sub{ORB_ID(parameter_update)};			/**< imu updates subscription */
	uORB::Subscription    _local_position_sub{ORB_ID(parameter_update)};			/**< local_position updates subscription */
	uORB::Subscription    _global_position_sub{ORB_ID(parameter_update)};			/**< global_position updates subscription */


	orb_advert_t           _vision_position_pub{nullptr};

    struct                 vehicle_odometry_s vision_position{};
    struct                 vehicle_local_position_s local_position{};


    hrt_abstime           _task_start{hrt_absolute_time()};
    hrt_abstime           _time_now{0};
    hrt_abstime           _time_last{0};
    int                   _ser_buadrate{57600};
    int                   _ser_com_num{0};
    int                   _drone_id{0};
    int                   _serial_fd{-1};
    u_char                FRAME_HEAD1 {0xfe};
    u_char                FRAME_HEAD2 {0x22};
    u_char                _cdata_buffer{'0'};
    char                  _msg_sum_chk{};
    bool                  _print_debug_msg{true};
    int                   _income_3_idata[3];  //外界输入
    int                   _income_1_idata;  //外界输入
    matrix::Vector3f      _body_local_position;    //无人机内部
    matrix::Vector3f      _local_position_sp;
    matrix::Vector3f      _attitude_sp;
    float                 _throttle_sp;
    matrix::Vector3f      _current_imu;
    u_char                _char_12buffer[12]; //for income 3int data
    u_char                _char_4buffer[4]; //for income 1int data


    MESSAGE_TYPE          _current_msg_type{};
    MODE                  _current_mode{};
    OFFBOARD_COMMAND      _current_offboard_command{};
    ARM_COMMAND           _current_arm_command{};
    TAKEOFF_COMMAND       _current_takeoff_command{};
    BACK_INFO             _current_back_info{};
    SEND_CURRENT_STATE    _current_send_state{};
    SETPOINT_TYPE         _current_sp_type{};

    /**
    * initialize some vectors/matrices from parameters
    */
	void parameters_updated();
    /**
	 * Check for parameter update and handle it.
	 */
	void parameters_update_poll();

	void get_data(){
        _cdata_buffer = '0';
        read(_serial_fd,&_cdata_buffer,1);
    }

    template <typename T>
    T parse_msg_type(){
        get_data();
        return (T)_cdata_buffer;
    }


    bool parse_frame_head(uint8_t limit=30);
	bool msg_checked();
	void process_command();
	void send_back_msg();
	void process_recv_state_data();
    void parse_income_i3data(bool clear_sum);
    void parse_income_i1data(bool clear_sum);


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::OFC_SER_COM>) _param_ofc_ser_com,
		(ParamInt<px4::params::OFC_SER_BAUD>) _param_ofc_ser_baud,
		(ParamBool<px4::params::OFC_DEB_PRT>) _param_ofc_deb_prt,
		(ParamInt<px4::params::OFC_CUR_ID>) _param_ofc_cur_id
	)
};

