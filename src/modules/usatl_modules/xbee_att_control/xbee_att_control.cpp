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
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <matrix/math.hpp>


/**
 * @file xbee_att_control.cpp
 * @author Tonser <sundxfansky@sjtu.edu.cn>
 * v1.0 2019.12.25
 *
 * 通过串口接受姿态指令,发布,并回传当前姿态,速度
 * 通过串口接受姿态指令,发布,并回传当前姿态,速度
 */

#define  BYTE0(dwTemp)       ( *( (uint8_t *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 3) )


#define SERIAL_COM "/dev/ttyS3" //fmuv5ttys3 fmuv2,v3 ttys6
#define BAUDRATE 19200
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;
#define SCALE 10000.0f


extern "C" __EXPORT int xbee_att_control_main(int argc, char *argv[]);
int xbee_att_control_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud); //static
static void usage(const char *reason);            //static
orb_advert_t mavlink_log_pub_xbee_att_control = NULL;

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

int xbee_att_control_main(int argc, char *argv[])
{

    mavlink_log_info(&mavlink_log_pub_xbee_att_control,"[inav] t265_sar_main on init");

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
        daemon_task = px4_task_spawn_cmd("xbee_att_control",
                                         SCHED_DEFAULT,
                                         SCHED_PRIORITY_DEFAULT,
                                         2500,
                                         xbee_att_control_thread_main,
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

int xbee_att_control_thread_main(int argc, char *argv[])
{
    //输出消息显示程序启动成功************
    mavlink_log_info(&mavlink_log_pub_xbee_att_control,"xbee_att_control run ");

    //启动串口***************************
    int uart_read = uart_init(SERIAL_COM);//fmuv5 ttys3 fmuv2,v3 ttys6
    if(uart_read==0)
    {
        mavlink_log_critical(&mavlink_log_pub_xbee_att_control,"[YCM]xbee_att_control uart init is failed\n");
        return -1;
    }
    if(set_uart_baudrate(uart_read,BAUDRATE)==0){
        mavlink_log_critical(&mavlink_log_pub_xbee_att_control,"[YCM]set_xbee_att_control_baudrate is failed\n");
        return -1;
    }
    mavlink_log_info(&mavlink_log_pub_xbee_att_control,"[YCM]xbee_att_control uart init is successful\n");
    thread_running = true;

    // 定义话题结构*****************
    vehicle_attitude_setpoint_s att_sp{};
    offboard_control_mode_s _offboard_control_mode{};

    // 接收信息设置*****************
    u_char data = '0';
    u_char data_buffer[21];
    int recv_iatt_sp[5];
    float recv_fatt_sp[5];
    bool ignore_thrust = false;
    bool recv_data_check = false;
    bool ignore_bodyrate_msg = true;
    bool ignore_attitude_msg = false;

    //发送消息设置******************
    u_char send_data = '0';
    u_char send_data_buffer[31];
    matrix::Eulerf current_euler;
    float send_fatt[3];
    float send_fvel[3];
    int send_iatt[3];
    int send_ivel[3];
    int send_sum;

    //offboard模式检测订阅设置**************
    int _control_mode_sub{orb_subscribe(ORB_ID(vehicle_control_mode))};
    bool control_mode_updated;
    vehicle_control_mode_s _control_mode {};



    // 姿态设定
    orb_advert_t _att_sp_pub{nullptr};
    _att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
    // offboard控制模式设定
    orb_advert_t _offboard_control_mode_pub{nullptr};
    _offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &_offboard_control_mode);

    // //订阅姿态信息******
    // int attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
    // vehicle_attitude_s att;
    // bool att_updated = false;

    //订阅odom信息******
    int odom_sub_fd = orb_subscribe(ORB_ID(vehicle_odometry));
    vehicle_odometry_s odom;
    bool odom_updated = false;

    while(thread_running)
    {
        //开启串口解析数据***********************

        // debug使用****
        // static char debug_buffer[20];
        // read(uart_read,&debug_buffer,20);
        // PX4_INFO("%X,%X,%X,%X,%X,",
        //          debug_buffer[0],debug_buffer[1],debug_buffer[2],debug_buffer[3],debug_buffer[4],debug_buffer[5]);


        //解析数据格式 FE 22 (int)roll (int)pitch (int)yaw (int)thrust (int)sum  21回传数据/19不需要回传数据
        data = '0';
        read(uart_read,&data,1);
        if(data == 0xFE){
            data = '0';
            read(uart_read,&data,1);
            if (data == 0x22){
                for (int index = 0;index<21;index++){
                    data = '0';
                    read(uart_read,&data,1);
                    data_buffer[index] = data;
                }
            }
            if((int)(data_buffer[20])==21||(int)(data_buffer[20])==19){
                for (int i = 0; i < 5; ++i) {
                    recv_iatt_sp[i] = (int)(data_buffer[4*i]<<24|data_buffer[4*i+1]<<16|data_buffer[4*i+2]<<8|data_buffer[4*i+3]);
                    recv_fatt_sp[i] = (float)recv_fatt_sp[i]/SCALE;
                }
                recv_data_check = (recv_iatt_sp[4]==(recv_iatt_sp[0]+recv_iatt_sp[1]+recv_iatt_sp[2]+recv_iatt_sp[3]));

                //判断是不是要回传****************
                if(data_buffer[20]==21){
                    // //更新姿态数据
                    // orb_check(attitude_sub_fd, &att_updated);
                    // if(att_updated){
                    //     orb_copy(ORB_ID(vehicle_attitude), attitude_sub_fd, &att);
                    // }

                    //更新里程计数据
                    orb_check(odom_sub_fd, &odom_updated);
                    if(odom_updated){
                        orb_copy(ORB_ID(vehicle_odometry), odom_sub_fd, &odom);
                    }
                    //赋值
                    current_euler = matrix::Quatf(odom.q);
                    send_fatt[0] = (float)current_euler.phi(); //roll
                    send_fatt[1] = (float)current_euler.theta(); //pitch
                    send_fatt[2] = (float)current_euler.psi(); //yaw

                    // Local NED to body-NED Dcm matrix
                    matrix::Dcmf Rlb(matrix::Quatf(odom.q));
                    matrix::Vector3f linvel_body(Rlb * matrix::Vector3f(odom.vx, odom.vy, odom.vz));

                    send_fvel[0] = (float)linvel_body(0);
                    send_fvel[1] = (float)linvel_body(1);
                    send_fvel[2] = (float)linvel_body(2);

                    for (int i = 0; i < 3; ++i) {
                        send_iatt[i] = (int)(send_fatt[i]*SCALE);
                        send_ivel[i] = (int)(send_fvel[i]*SCALE);
                    }

                    send_sum = send_iatt[0]+send_iatt[1]+send_iatt[2]+send_ivel[0]+send_ivel[1]+send_ivel[2];
                    send_data_buffer[0] = 0xFE;
                    send_data_buffer[1] = 0x23;

                    for (int j = 0; j < 3; ++j) {
                        send_data_buffer[2+j*4] = BYTE3(send_iatt[j]);
                        send_data_buffer[3+j*4] = BYTE2(send_iatt[j]);
                        send_data_buffer[4+j*4] = BYTE1(send_iatt[j]);
                        send_data_buffer[5+j*4] = BYTE0(send_iatt[j]);
                        send_data_buffer[14+j*4] = BYTE3(send_iatt[j]);
                        send_data_buffer[15+j*4] = BYTE2(send_iatt[j]);
                        send_data_buffer[16+j*4] = BYTE1(send_iatt[j]);
                        send_data_buffer[17+j*4] = BYTE0(send_iatt[j]);
                    }

                    send_data_buffer[26] = BYTE3(send_sum);
                    send_data_buffer[27] = BYTE2(send_sum);
                    send_data_buffer[28] = BYTE1(send_sum);
                    send_data_buffer[29] = BYTE0(send_sum);
                    send_data_buffer[30] = 0x19;

                    for (int k = 0; k < 31; ++k) {
                        send_data = '0';
                        send_data = send_data_buffer[k];
                        write(uart_read,&send_data,1);
                    }
                }
            }
        }

        //offborad control mode 设置************
        _offboard_control_mode.ignore_thrust = ignore_thrust;
        _offboard_control_mode.ignore_position = true;
        _offboard_control_mode.ignore_velocity = true;
        _offboard_control_mode.ignore_acceleration_force = true;
        _offboard_control_mode.ignore_bodyrate = ignore_bodyrate_msg;
        _offboard_control_mode.ignore_attitude = ignore_attitude_msg;
        _offboard_control_mode.timestamp = hrt_absolute_time();
        orb_publish(ORB_ID(offboard_control_mode), _offboard_control_mode_pub, &_offboard_control_mode);

        //检测offboard模式**********
        orb_check(_control_mode_sub, &control_mode_updated);
        if (control_mode_updated) {
            orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
        }

        //开启offboard模式后才发布**********
        if(_control_mode.flag_control_offboard_enabled&&recv_data_check){
            if (!(_offboard_control_mode.ignore_attitude)) {
                att_sp.timestamp = hrt_absolute_time();

                att_sp.q_d_valid = true;
                att_sp.roll_body = recv_fatt_sp[0];
                att_sp.pitch_body = recv_fatt_sp[1];
                att_sp.yaw_body = recv_fatt_sp[2];
                att_sp.yaw_sp_move_rate = 0.0f;

                if (!_offboard_control_mode.ignore_thrust) {
                    // att_sp.thrust_body[0]用于 固定翼
                    att_sp.thrust_body[2] = recv_fatt_sp[3];
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
