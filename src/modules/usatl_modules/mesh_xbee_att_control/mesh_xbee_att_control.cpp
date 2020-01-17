#include <cstdio>
#include <termios.h>
#include <unistd.h>
#include <cstdlib>
#include <stdlib.h>
#include <cstring>
#include <cerrno>
#include <cmath>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <matrix/math.hpp>


/**
 * @file mesh_xbee_att_control.cpp
 * @author Tonser <sundxfansky@sjtu.edu.cn>
 * v1.0 2020.01.17
 *
 * 组网通过串口接受姿态指令,发布,并回传当前姿态,速度
 */

#define  BYTE0(dwTemp)       ( *( (uint8_t *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 3) )

#define SCALE 10000.0f
// #define SERIAL_COM "/dev/ttyS3" //fmuv5 ttys3 fmuv2,v3 ttys6 UART口
#define SERIAL_COM "/dev/ttyS3" //ttys2 telem2
#define BAUDRATE 115200
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;


extern "C" __EXPORT int mesh_xbee_att_control_main(int argc, char *argv[]);
int mesh_xbee_att_control_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud); //static
static void usage(const char *reason);            //static
orb_advert_t mavlink_log_pub_mesh_xbee_att_control = NULL;

int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            PX4_INFO("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;


    tcgetattr(fd, &uart_config);

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

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        PX4_INFO("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}

int uart_init(const char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}

static void usage(const char *reason)
{
    if (reason) {
        printf( "%s\n", reason);
    }

    printf( "WARN: lose para,use {start|stop|status} [param]\n\n");
    // exit(1);
}

int mesh_xbee_att_control_main(int argc, char *argv[])
{

    mavlink_log_info(&mavlink_log_pub_mesh_xbee_att_control,"[inav] mesh_xbee_att_control on init");

    if (argc < 2)
    {
        usage("[YCM]missinfg command");
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            PX4_INFO("[YCM]already running\n");
            // exit(0);
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("mesh_xbee_att_control",
                                         SCHED_DEFAULT,
                                         SCHED_PRIORITY_DEFAULT,
                                         2500,
                                         mesh_xbee_att_control_thread_main,
                                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        thread_running = false;
        return 0;
    }
    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("[YCM]running");

        } else {
            warnx("[YCM]stopped");
        }

        return 0;
    }

    usage("unrecognized command");
    return 1;
}

int mesh_xbee_att_control_thread_main(int argc, char *argv[])
{
    //输出消息显示程序启动成功************
    mavlink_log_info(&mavlink_log_pub_mesh_xbee_att_control,"  run ");

    //启动串口***************************
    int uart_read = uart_init(SERIAL_COM);//fmuv5 ttys3 fmuv2,v3 ttys6
    if(uart_read==0)
    {
        mavlink_log_critical(&mavlink_log_pub_mesh_xbee_att_control,"[YCM]mesh_xbee_att_control uart init is failed\n");
        return -1;
    }
    if(set_uart_baudrate(uart_read,BAUDRATE)==0){
        mavlink_log_critical(&mavlink_log_pub_mesh_xbee_att_control,"[YCM]set_mesh_xbee_att_control_baudrate is failed\n");
        return -1;
    }
    mavlink_log_info(&mavlink_log_pub_mesh_xbee_att_control,"[YCM]mesh_xbee_att_control uart init is successful\n");
    thread_running = true;

    // 定义话题结构*****************
    vehicle_attitude_setpoint_s att_sp{};
    offboard_control_mode_s _offboard_control_mode{};

    // 接收信息设置*****************
    u_char data = '0';
    float recv_fatt_sp[5];
    recv_fatt_sp[0]=recv_fatt_sp[0];
    int recv_iatt_sp[5];
    u_char _data_char_buffer[37];
    u_char _raw_data_char_buffer[20];
    bool debug = 0;
    bool ignore_thrust = false;
    bool ignore_bodyrate_msg = true;
    bool ignore_attitude_msg = false;
    bool is_enable_offboard = false;
    bool check_sum;
    bool ready_parse = false;

    //发送消息设置******************

    //offboard模式检测订阅设置**************
    int _control_mode_sub{orb_subscribe(ORB_ID(vehicle_control_mode))};
    bool control_mode_updated;
    vehicle_control_mode_s _control_mode {};
    vehicle_command_s vcmd{};

    // 姿态设定
    orb_advert_t _att_sp_pub{nullptr};
    _att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

    // offboard模式发布
    orb_advert_t _cmd_pub{nullptr};
    _cmd_pub = orb_advertise(ORB_ID(vehicle_command), &vcmd);

    // offboard控制模式设定
    orb_advert_t _offboard_control_mode_pub{nullptr};
    _offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &_offboard_control_mode);

    // //订阅姿态信息******
    // int attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
    // vehicle_attitude_s att;
    // bool att_updated = false;

    //订阅odom信息******
    while(thread_running) {

        if (debug) {
            memset(_data_char_buffer, 0, sizeof(_data_char_buffer));
            // read(uart_read,_data_char_buffer,33);
            for (int i = 0; i < 33; ++i) {
                data = '0';
                read(uart_read, &data, 1);
                _data_char_buffer[i] = data;
                // PX4_INFO("%02X",data);
            }
            PX4_INFO("%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,"
                     "%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,"
                     "%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,"
                     "%02X,%02X,%02X",
                     _data_char_buffer[0], _data_char_buffer[1], _data_char_buffer[2], _data_char_buffer[3],
                     _data_char_buffer[4], _data_char_buffer[5],
                     _data_char_buffer[6], _data_char_buffer[7], _data_char_buffer[8], _data_char_buffer[9],
                     _data_char_buffer[10], _data_char_buffer[11],
                     _data_char_buffer[12], _data_char_buffer[13], _data_char_buffer[14], _data_char_buffer[15],
                     _data_char_buffer[16], _data_char_buffer[17],
                     _data_char_buffer[18], _data_char_buffer[19], _data_char_buffer[20], _data_char_buffer[21],
                     _data_char_buffer[22], _data_char_buffer[23],
                     _data_char_buffer[24], _data_char_buffer[25], _data_char_buffer[26], _data_char_buffer[27],
                     _data_char_buffer[28], _data_char_buffer[29],
                     _data_char_buffer[30], _data_char_buffer[31], _data_char_buffer[32]);
        } else {
            data = '0';
            ready_parse = false;
            read(uart_read, &data, 1);
            if (data == 0xff) {
                data = '0';
                read(uart_read, &data, 1);
                if (data == 0xfe) {
                    ready_parse = true;
                    data = '0';
                    read(uart_read, &data, 1);
                }
                if (ready_parse) {
                    memset(_data_char_buffer, 0, sizeof(_data_char_buffer));
                    for (int i = 0; i < 20; ++i) {
                        data = '0';
                        read(uart_read, &data, 1);
                        _raw_data_char_buffer[i] = data;
                        // PX4_INFO("%02X",data);
                    }
                    // 数据头 15个
                    for (int j = 0; j < 5; ++j) {
                        recv_iatt_sp[j] = (int) (_raw_data_char_buffer[4 * j + 3] << 24 |
                                                 _raw_data_char_buffer[4 * j + 2] << 16 |
                                                 _raw_data_char_buffer[4 * j + 1] << 8 | _raw_data_char_buffer[4 * j]);
                        recv_fatt_sp[j] = recv_iatt_sp[j] / SCALE;
                    }
                }
                check_sum =
                        abs(recv_iatt_sp[4] - (recv_iatt_sp[0] + recv_iatt_sp[1] + recv_iatt_sp[2] + recv_iatt_sp[3])) <
                        100;

                //isoffboard
                data = '0';
                read(uart_read, &data, 1);
                is_enable_offboard = data == 0x01;
                //校验位
                data = '0';
                read(uart_read, &data, 1);
                // chk 1 start 1 length 2 type 1 addr 8
                // for (int i = 0; i < 12; ++i) {
                //     data = '0';
                //     read(uart_read,&data,1);
                // }

                PX4_INFO("roll: %4d, pitch: %4d, yaw: %4d, thrust: %4d, sum: %4d, check: %4d",
                         recv_iatt_sp[0], recv_iatt_sp[1], recv_iatt_sp[2], recv_iatt_sp[3], recv_iatt_sp[4],
                         (int) check_sum
                );
                // if(check_sum){
                //     PX4_INFO("roll: %4.4f, pitch: %4.4f, yaw: %4.4f, thrust: %4.4f, sum: %4.4f",
                //              (double)recv_fatt_sp[0],(double)recv_fatt_sp[1],(double)recv_fatt_sp[2],(double)recv_fatt_sp[3], (double)recv_fatt_sp[4]
                //     );
                // }
                // }
            }
        }

        vcmd.timestamp = hrt_absolute_time();
        /* copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
        vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_GUIDED_ENABLE;
        vcmd.confirmation = true;
        vcmd.from_external = true;
        if(is_enable_offboard){
            orb_publish(ORB_ID(vehicle_command), _cmd_pub, &vcmd);
        }

        //开启串口解析数据***********************
        //解析数据格式 FE 22 (int)roll (int)pitch (int)yaw (int)thrust (int)sum  15回传数据/14不需要回传数据
        // offborad control mode 设置************
        _offboard_control_mode.ignore_thrust = ignore_thrust;
        _offboard_control_mode.ignore_position = true;
        _offboard_control_mode.ignore_velocity = true;
        _offboard_control_mode.ignore_acceleration_force = true;
        _offboard_control_mode.ignore_bodyrate_x = ignore_bodyrate_msg;
        _offboard_control_mode.ignore_bodyrate_y = ignore_bodyrate_msg;
        _offboard_control_mode.ignore_bodyrate_z = ignore_bodyrate_msg;
        _offboard_control_mode.ignore_attitude = ignore_attitude_msg;
        _offboard_control_mode.timestamp = hrt_absolute_time();
        orb_publish(ORB_ID(offboard_control_mode), _offboard_control_mode_pub, &_offboard_control_mode);

        //检测offboard模式**********
        orb_check(_control_mode_sub, &control_mode_updated);
        if (control_mode_updated) {
            orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
        }

        //开启offboard模式后才发布**********
        if (_control_mode.flag_control_offboard_enabled) {
            if (!(_offboard_control_mode.ignore_attitude)) {
                att_sp.timestamp = hrt_absolute_time();
                matrix::Eulerf sp_euler(recv_fatt_sp[0], recv_fatt_sp[1], recv_fatt_sp[2]);
                matrix::Quatf sp_q(sp_euler);
                sp_q.copyTo(att_sp.q_d);
                att_sp.q_d_valid = true;
                att_sp.roll_body = recv_fatt_sp[0];
                att_sp.pitch_body = recv_fatt_sp[1];
                att_sp.yaw_body = recv_fatt_sp[2];
                att_sp.yaw_sp_move_rate = 0.0f;

                if (!_offboard_control_mode.ignore_thrust) {
                    // att_sp.thrust_body[0]用于 固定翼
                    att_sp.thrust_body[2] = recv_fatt_sp[3]; //
                }
            }
            orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &att_sp);
        }
    }
    thread_running = false;
    //取消订阅
    close(uart_read);
    return 0;
}
