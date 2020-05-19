/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file offb_ctrl_params.c
 * Parameters for offboard control.
 *
 * @author Tonser
 */

 /**
 * offboard control com
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @value 0 Dev-ttys0 for fum-v2-v3-v4 uart 1 io debug except v4 here v4 wifi. for fmuv5 here gps
 * @value 1 Dev-ttys1 for fmu-v2-v3-v4-v5 telem1
 * @value 2 Dev-ttys2 for fum-v2-v3-v4-v5 telem2
 * @value 3 Dev-ttys3 for fum-v2-v3-v4 gps. for fmuv5 telem4
 * @value 4 Dev-ttys4 for fum-v2-v3-v4 serial4 for fmuv5 telem3
 * @group EKF2
 */
PARAM_DEFINE_INT32(OFC_SER_COM, 1);

/**
 * offboard control print debug
 *
 * ether print debug msg
 *
 * @boolean
 * @group EKF2
 */
PARAM_DEFINE_INT32(OFC_DEB_PRT, 1);

/**
 * offboard control serial buadrate
 *
 * buadrate rate.
 *
 * @value 9600 Baudrate 9600
 * @value 19200 Baudrate 19200
 * @value 57600 Baudrate 57600
 * @value 38400 Baudrate 38400
 * @value 115200 Baudrate 115200
 * @group EKF2
 */
PARAM_DEFINE_INT32(OFC_SER_BAUD, 57600);

/**
 * offboard control current drone id
 *
 * drone id.
 *
 * @value 0 Drone id 0
 * @value 1 Drone id 1
 * @value 2 Drone id 2
 * @value 3 Drone id 3
 * @value 4 Drone id 4
 * @value 5 Drone id 5
 * @group EKF2
 */
PARAM_DEFINE_INT32(OFC_CUR_ID, 0);


