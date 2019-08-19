#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <math.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/sensor_accel.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <poll.h>
#include <px4_shutdown.h>
#include <px4_tasks.h>
#include <px4_time.h>

#include "float.h"
#include <math.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>


/**
 * @file intel_t265_sar.cpp
 * @author Tonser <sundxfansky@sjtu.edu.cn>
 * v1.0 2019.7.9
 *
 * 通过串口接受来自upcore board的T265位置
 * 发布ORB_ID(vehicle_vision_position)
 *    ORB_ID(vehicle_vision_attitude)
 *
 * This driver parse t265 data,publish ORB_ID(vehicle_vision_position)
 * and ORB_ID(vehicle_vision_attitude).
 *
 * 设置内容
 * 1 开启本模块，并编译
 * 2 确保机载电脑启动，串口通信正常，可以通过print开关设置
 * 3 注意坐标系，参考http://dev.px4.io/master/en/ros/external_position_estimation.html，无人机机体坐标系是FRD（前右地）对应xyz，
 *   机头正北方向，对应全局坐标系 NED（北东地）对应XYZ
 * 4 设置参数EV_POS 位置，设置无人机视觉姿态延迟EV——delay 0.025（观看log得25ms，或许对应40fps？）
 * 5 设置好激光定高后再尝试视觉定位，方便及时切换回来
 */

#define  BYTE0(dwTemp)       ( *( (uint8_t *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 3) )

#define T265_POS_X 0.12f
#define T265_POS_Y 0.0f
#define T265_POS_Z -0.02f


#define SERIAL_COM_T265 "/dev/ttyS6" //fmuv5ttys3 fmuv2,v3 ttys6
#define BAUDRATE 19200
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;
#define SCALE 10000.0f


extern "C" __EXPORT int intel_t265_sar_main(int argc, char *argv[]);
int intel_t265_sar_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud); //static
static void usage(const char *reason);            //static
orb_advert_t mavlink_log_pub_t265_sar = NULL;

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
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "WARN: lose para,use {start|stop|status} [param]\n\n");
    exit(1);
}

int intel_t265_sar_main(int argc, char *argv[])
{

    mavlink_log_info(&mavlink_log_pub_t265_sar,"[inav] t265_sar_main on init");

    if (argc < 2)
    {
        usage("[YCM]missinfg command");
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            PX4_INFO("[YCM]already running\n");
            exit(0);
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("intel_t265_sar",
                                         SCHED_DEFAULT,
                                         SCHED_PRIORITY_DEFAULT,
                                         2500,
                                         intel_t265_sar_thread_main,
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

int intel_t265_sar_thread_main(int argc, char *argv[])
{
    mavlink_log_info(&mavlink_log_pub_t265_sar,"t265 run ");
    char data = '0';
    char send_data = '0';
    int uart_read = uart_init(SERIAL_COM_T265);//fmuv5 ttys3 fmuv2,v3 ttys6
    if(uart_read==0)
    {
        mavlink_log_critical(&mavlink_log_pub_t265_sar,"[YCM]t265 uart init is failed\n");
        return -1;
    }
    if(set_uart_baudrate(uart_read,BAUDRATE)==0){
        mavlink_log_critical(&mavlink_log_pub_t265_sar,"[YCM]set_t265_uart_baudrate is failed\n");
        return -1;
    }
    mavlink_log_info(&mavlink_log_pub_t265_sar,"[YCM]t265 uart init is successful\n");
    thread_running = true;

    // 定义话题结构
    struct vehicle_local_position_s vision_position;
    struct vehicle_attitude_s vision_attitude;
    // 初始化数据
    memset(&vision_position, 0 , sizeof(vision_position));
    memset(&vision_attitude, 0 , sizeof(vision_attitude));
    u_char data_buffer[29];
    u_char send_data_buffer[23];
    // 输出开关
    bool t265_debug_enable = 0;
    bool print_received_data = 0;
    bool publish_vision_data = 1;
    // 程序出现未知问题，关闭位置修正，加入yaw角度修正
    bool _correct_pos = 0;
    int pos[3];//pos[0] = x;pos[1] = y;pos[2] = z;
    int angle[3];// angle[0]=roll,angle[1]=pitch,angle[2]=yaw
    int check;
    int sum;
    int _send_sum;
    bool _is_t265_data_check_true;
    float float_angle[3];
    int t265_uart_error_num = 0;
    float t265_R[3][3];
    float t265_position[3] = {T265_POS_X,T265_POS_Y,T265_POS_Z};
    float trans_pos_t265[3] = {0.0f,0.0f,0.0f};
    int _int_distance;
    int _int_accel[3];

    orb_advert_t _vision_position_pub = nullptr;
    orb_advert_t _vision_attitude_pub = nullptr;
    int distance_sub_fd = orb_subscribe(ORB_ID(distance_sensor));
    orb_set_interval(distance_sub_fd, 10);
    struct distance_sensor_s current_distance;
    memset(&current_distance,0, sizeof(current_distance));
    int accel_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
    orb_set_interval(accel_sub_fd, 10);
    struct vehicle_local_position_s current_accel;
    memset(&current_accel,0, sizeof(current_accel));
    // px4_pollfd_struct_t fds[] = {
    //         { .fd = distance_sub_fd,   .events = POLLIN },
    //         { .fd = accel_sub_fd,   .events = POLLIN },
    //         /* there could be more file descriptors here, in the form like:
    //          * { .fd = other_sub_fd,   .events = POLLIN },
    //          */
    // };
    // int poll_ret = px4_poll(fds, 1, 1000);

    while(thread_running)
    {
        if(t265_debug_enable){
            static char t265_buffer[31];
            read(uart_read,&t265_buffer,31);
            // for(int k = 0;k < 31;++k){
            //     data = '0';
            //     read(uart_read,&data,1);
            //     t265_buffer[k] = data;
            // }
            PX4_INFO("%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,"
                     "%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,"
                     "%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,"
                     "%X",
                     t265_buffer[0],t265_buffer[1],t265_buffer[2],t265_buffer[3],t265_buffer[4],t265_buffer[5],
                     t265_buffer[6],t265_buffer[7],t265_buffer[8],t265_buffer[9],t265_buffer[10],t265_buffer[11],
                     t265_buffer[12],t265_buffer[13],t265_buffer[14],t265_buffer[15],t265_buffer[16],t265_buffer[17],
                     t265_buffer[18],t265_buffer[19],t265_buffer[20],t265_buffer[21],t265_buffer[22],t265_buffer[23],
                     t265_buffer[24],t265_buffer[25],t265_buffer[26],t265_buffer[27],t265_buffer[28],t265_buffer[29],
                     t265_buffer[30]);
        }
        else{
            data = '0';
            read(uart_read,&data,1);
            if(data == 0xFE){
                data = '0';
                read(uart_read,&data,1);
                if (data == 0x22){
                    // read(uart_read,&data_buffer,28);
                    for (int index = 0;index<29;index++){
                        data = '0';
                        read(uart_read,&data,1);
                        data_buffer[index] = data;
                        // send_data = '0';
                        // write(uart_read,&send_data,1);
                    }
                    if((int)(data_buffer[28])==21){
                        // if((poll_ret>0)&&(fds[0].revents & POLLIN)){
                        // if((poll_ret>0)){
                        orb_copy(ORB_ID(distance_sensor),distance_sub_fd,&current_distance);
                        orb_copy(ORB_ID(vehicle_local_position),accel_sub_fd,&current_accel);
                        // }
                        _int_distance = (int)SCALE*(current_distance.current_distance);
                        _int_accel[0] = (int)SCALE*(current_accel.ax);
                        _int_accel[1] = (int)SCALE*(current_accel.ay);
                        _int_accel[2] = (int)SCALE*(current_accel.yaw);

                        send_data_buffer[0] = 0xFE;
                        send_data_buffer[1] = 0x22;

                        send_data_buffer[2] = BYTE3(_int_distance);
                        send_data_buffer[3] = BYTE2(_int_distance);
                        send_data_buffer[4] = BYTE1(_int_distance);
                        send_data_buffer[5] = BYTE0(_int_distance);

                        for (int send_buffer_index = 0;send_buffer_index<3;send_buffer_index++){
                            send_data_buffer[6+send_buffer_index*4] =BYTE3(_int_accel[send_buffer_index]);
                            send_data_buffer[7+send_buffer_index*4] =BYTE2(_int_accel[send_buffer_index]);
                            send_data_buffer[8+send_buffer_index*4] =BYTE1(_int_accel[send_buffer_index]);
                            send_data_buffer[9+send_buffer_index*4] =BYTE0(_int_accel[send_buffer_index]);
                        }//9+8=17,下一个18
                        _send_sum = _int_accel[0]+_int_accel[1]+_int_accel[2]+_int_distance;

                        send_data_buffer[18] = BYTE3(_send_sum);
                        send_data_buffer[19] = BYTE2(_send_sum);
                        send_data_buffer[20] = BYTE1(_send_sum);
                        send_data_buffer[21] = BYTE0(_send_sum);

                        send_data_buffer[22] = 0x14;



                        for (int index = 0;index<23;index++){
                            send_data = '0';
                            send_data = send_data_buffer[index];
                            write(uart_read,&send_data,1);
                        }
                    }
                    for (int i = 0;i < 3;i++){
                        pos[i] = (int)(data_buffer[4*i]<<24|data_buffer[4*i+1]<<16|data_buffer[4*i+2]<<8|data_buffer[4*i+3]);
                        angle[i] = (int)(data_buffer[4*i+12]<<24|data_buffer[4*i+13]<<16|data_buffer[4*i+14]<<8|data_buffer[4*i+15]);
                    }
                    check = (int)(data_buffer[24]<<24|data_buffer[25]<<16|data_buffer[26]<<8|data_buffer[27]);
                    sum = (int)(pos[0]+pos[1]+pos[2]+angle[0]+angle[1]+angle[2]);
                    _is_t265_data_check_true = (check == sum);
                    vision_position.timestamp = hrt_absolute_time();

                    vision_position.x = (float)pos[0]/SCALE;
                    //
                    //T265向前的方向安装正确时，与飞控的y，z方向相反。
                    vision_position.y = ((float)pos[1]/SCALE);
                    vision_position.z = ((float)pos[2]/SCALE);

                    vision_position.xy_valid = true;
                    vision_position.z_valid = true;
                    vision_position.v_xy_valid = true;
                    vision_position.v_z_valid = true;

                    vision_attitude.timestamp = hrt_absolute_time();

                    float_angle[0] = (float)angle[0]/SCALE;
                    // 需要验证是不是这样，同时此处取飞机数据更好，后续优化
                    float_angle[1] = ((float)angle[1]/SCALE);
                    float_angle[2] = ((float)angle[2]/SCALE);

                    // 修正位置
                    if(_correct_pos){

                        t265_R[0][0] = cosf(float_angle[1])*cosf(float_angle[2]);
                        t265_R[0][1] = -cosf(float_angle[1])*sinf(float_angle[2]);
                        t265_R[0][2] = sinf(float_angle[1]);
                        t265_R[1][0] = cosf(float_angle[0])*sinf(float_angle[2])+sinf(float_angle[0])*sinf(float_angle[1])*cosf(float_angle[2]);
                        t265_R[1][1] = cosf(float_angle[0])*cosf(float_angle[2])-sinf(float_angle[0])*sinf(float_angle[1])*sinf(float_angle[2]);
                        t265_R[1][2] = -sinf(float_angle[0])*cosf(float_angle[1]);
                        t265_R[2][0] = sinf(float_angle[0])*sinf(float_angle[2])-cosf(float_angle[0])*sinf(float_angle[1])*cosf(float_angle[2]);
                        t265_R[2][1] = sinf(float_angle[0])*cosf(float_angle[2])+cosf(float_angle[0])*sinf(float_angle[1])*sinf(float_angle[2]);
                        t265_R[2][2] = cosf(float_angle[0])*cosf(float_angle[1]);

                        //初始化旋转结果
                        trans_pos_t265[0] = 0.0f;
                        trans_pos_t265[1] = 0.0f;
                        trans_pos_t265[2] = 0.0f;
                        //旋转矩阵计算
                        for(int m = 0;m<3;m++){
                            for(int n=0;n<3;n++){
                                trans_pos_t265[m] += t265_R[m][n]*t265_position[n];
                            }
                        }
                        // PX4_INFO(" x: %2.4f,y: %2.4f,z: %2.4f,roll: %2.4f,pitch :%2.4f yaw:%2.4f",
                        //          (double)(t265_position[0]-trans_pos_t265[0]),
                        //          (double)(t265_position[1]-trans_pos_t265[1]),
                        //          (double)(t265_position[2]-trans_pos_t265[2]),
                        //          (double)float_angle[0],
                        //          (double)float_angle[1],
                        //          (double)float_angle[2]
                        // );
                        vision_position.x = vision_position.x+t265_position[0]-trans_pos_t265[0];
                        vision_position.y = vision_position.y+t265_position[1]-trans_pos_t265[1];
                        vision_position.z = vision_position.z+t265_position[2]-trans_pos_t265[2];
                    }

                    matrix::Quatf q(matrix::Eulerf(float_angle[0], float_angle[1], float_angle[2]));

                    // vision_attitude.q[2] = -vision_attitude.q[2];
                    // vision_attitude.q[3] = -vision_attitude.q[3];
                    q.copyTo(vision_attitude.q);
                    if(print_received_data){
                        PX4_INFO(" x: %2.4f,y: %2.4f,z: %2.4f,roll: %2.4f,pitch :%2.4f yaw:%2.4f,w:%2.4f,x:%2.4f,y:%2.4f,z:%2.4f _is_t265_data_check_true:%1d,flag %1d",
                                 (double)vision_position.x,(double)vision_position.y,(double)vision_position.z,
                                 (double)float_angle[0],(double)float_angle[1],(double)float_angle[2],
                                 (double)vision_attitude.q[0],(double)vision_attitude.q[1],(double)vision_attitude.q[2],(double)vision_attitude.q[3],
                                 (int)_is_t265_data_check_true,(int)data_buffer[28]
                        );
                    }
                    int inst = 0;
                    if(_is_t265_data_check_true){
                        // PX4_WARN("published");
                        orb_publish_auto(ORB_ID(vehicle_vision_position), &_vision_position_pub, &vision_position, &inst, ORB_PRIO_DEFAULT);
                        orb_publish_auto(ORB_ID(vehicle_vision_attitude), &_vision_attitude_pub, &vision_attitude, &inst, ORB_PRIO_DEFAULT);
                    } else{
                        // PX4_WARN("NOT published");
                    }
                }
                else{
                    // PX4_WARN("DATA WRONG 22222 %X,error_num:%\t4d",data,++t265_uart_error_num);
                }

                // data = '0';
                // read(uart_read,&data,1);
            }
            else{
                // PX4_WARN("DATA WRONG 11111 %X,error_num:%\t4d",data,++t265_uart_error_num);
            }
        }
    }
    thread_running = false;
    //取消订阅
    close(uart_read);
    fflush(stdout);
    return 0;
}
