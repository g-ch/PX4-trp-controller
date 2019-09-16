/****************************************************************************
 *
 *   Copyright (c) 2015-2016 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ekf2_params.c
 * Parameter definition for ekf2.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

/**
 * xbee serial com in v3/v2 serial always 3 in v5 serial 6
 *
 * @group EKF2
 * @value 0 ttyS0
 * @value 1 ttyS1
 * @value 2 ttyS2(in v2/v3)
 * @value 3 ttyS3
 * @value 4 ttyS4
 * @value 6 ttyS6(in v5)
 */
PARAM_DEFINE_INT32(XBEE_SERIAL_COM, 3);

/**
 * xbee baudrate
 *
 * @group EKF2
 * @value 0 9600
 * @value 1 19200
 * @value 2 38400
 * @value 3 57600
 * @value 4 115200
 */
PARAM_DEFINE_INT32(XBEE_BAUD_RATE, 1);

/**
 * XBEE_POS_SCALE same to the vicon send code
 *
 * @group EKF2
 * @min 100
 * @max 10000
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(XBEE_POS_SCALE, 1000.0f);

/**
 * true to print publish pose msg
 *
 * @group EKF2
 * @boolean
 */
PARAM_DEFINE_INT32(XBEE_PUB_PRINT, 0);

/**
 * true to print debug XBEE_SERIAL_COM msg
 *
 * @group EKF2
 * @boolean
 */
PARAM_DEFINE_INT32(XBEE_DEBUG_PRINT, 0);

/**
 * true to print debug error msg num XBEE_SERIAL_COM msg
 *
 * @group EKF2
 * @boolean
 */
PARAM_DEFINE_INT32(XBEE_DBNUM_PRINT, 0);

/**
 * true to send quaternion false to send eular angle
 *
 * @group EKF2
 * @boolean
 */
PARAM_DEFINE_INT32(XBEE_SEND_QUAT, 1);