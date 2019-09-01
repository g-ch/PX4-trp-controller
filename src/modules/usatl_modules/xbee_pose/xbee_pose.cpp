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
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_attitude.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_shutdown.h>
#include <px4_tasks.h>
#include <px4_time.h>

#include "float.h"
#include <math.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>


/**
 * @file xbee_pose.cpp
 * @author Tonser <sundxfansky@sjtu.edu.cn>
 * v1.0 2019.8.31
 *
 * 通过串口接受来自vicon的位置
 * 发布ORB_ID(vehicle_vision_position)
 *
 * This driver parse vicon pose data,publish ORB_ID(vehicle_vision_position)
 * and ORB_ID(vehicle_visual_odometry).
 *
 * 设置内容
 * 1 开启本模块，并编译
 * 2 确保机载电脑启动，串口通信正常，可以通过print开关设置
 * 3 注意坐标系，参考http://dev.px4.io/master/en/ros/external_position_estimation.html，无人机机体坐标系是FRD（前右地）对应xyz，
 *   机头正北方向，对应全局坐标系 NED（北东地）对应XYZ
 * 4 设置参数EV_POS 位置，设置无人机视觉姿态延迟EV——delay 0.025（观看log得25ms，或许对应40fps？）
 * 5 设置好激光定高后再尝试视觉定位，方便及时切换回来
 */

// #define  BYTE0(dwTemp)       ( *( (uint8_t *)(&dwTemp)	)  )
// #define  BYTE1(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 1) )
// #define  BYTE2(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 2) )
// #define  BYTE3(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 3) )

#define SERIAL_COM_XBEE "/dev/ttyS3" //fmuv5ttys3 fmuv2,v3 ttys6
#define BAUDRATE 57600
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;
#define SCALE 10000.0f


extern "C" __EXPORT int xbee_pose_main(int argc, char *argv[]);
int xbee_pose_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud); //static
static void usage(const char *reason);            //static
orb_advert_t mavlink_log_pub_xbee_pose = NULL;

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
        // fprintf(stderr, "%s\n", reason);
        printf("%s\n", reason);
    }

    printf("WARN: lose para,use {start|stop|status} [param]\n\n");
    // fprintf(stderr, "WARN: lose para,use {start|stop|status} [param]\n\n");
    // exit(1);
}

int xbee_pose_main(int argc, char *argv[])
{

mavlink_log_info(&mavlink_log_pub_xbee_pose,"[inav] t265_main on init");

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
        daemon_task = px4_task_spawn_cmd("xbee_pose",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         2500,
                         xbee_pose_thread_main,
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

int xbee_pose_thread_main(int argc, char *argv[])
{
    mavlink_log_info(&mavlink_log_pub_xbee_pose,"xbee run ");
    unsigned char data = '0';
    int uart_read = uart_init(SERIAL_COM_XBEE);//fmuv5 ttys3 fmuv2,v3 ttys6
    if(false == uart_read)
    {
         mavlink_log_critical(&mavlink_log_pub_xbee_pose,"[YCM]xbee pose uart init is failed\n");
         return -1;
    }
    if(false == set_uart_baudrate(uart_read,BAUDRATE)){
        mavlink_log_critical(&mavlink_log_pub_xbee_pose,"[YCM]set_xbee_uart_baudrate is failed\n");
        return -1;
    }
    mavlink_log_info(&mavlink_log_pub_xbee_pose,"[YCM]xbee uart init is successful\n");
    thread_running = true;

    // 定义话题结构
    struct vehicle_odometry_s vision_position;
    // 初始化数据
    memset(&vision_position, 0 , sizeof(vision_position));
    char data_buffer[28];
    // 输出开关
    bool xbee_debug_enable = 0;
    bool print_received_data = 1;
    // bool publish_vision_data = 1;
    // 程序出现未知问题，关闭位置修正，加入yaw角度修正
    int pos[3];//pos[0] = x;pos[1] = y;pos[2] = z;
    int angle[3];// angle[0]=roll,angle[1]=pitch,angle[2]=yaw
    int check;
    int sum;
    bool _is_xbee_data_check_true;
    float float_angle[3];
    orb_advert_t _vision_position_pub = nullptr;


    while(thread_running)
   {
        if(xbee_debug_enable){
            static char t265_buffer[31];
            for(int k = 0;k < 31;++k){
                data = '0';
                read(uart_read,&data,1);
                t265_buffer[k] = data;
            }
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
                    for (int index = 0;index<28;index++){
                        data = '0';
                        read(uart_read,&data,1);
                        data_buffer[index] = data;
                    }
                    for (int i = 0;i < 3;i++){
                        pos[i] = (int)(data_buffer[4*i]<<24|data_buffer[4*i+1]<<16|data_buffer[4*i+2]<<8|data_buffer[4*i+3]);
                        angle[i] = (int)(data_buffer[4*i+12]<<24|data_buffer[4*i+13]<<16|data_buffer[4*i+14]<<8|data_buffer[4*i+15]);
                    }
                    check = (int)(data_buffer[24]<<24|data_buffer[25]<<16|data_buffer[26]<<8|data_buffer[27]);
                    sum = (int)(pos[0]+pos[1]+pos[2]+angle[0]+angle[1]+angle[2]);
                    _is_xbee_data_check_true = (check == sum);
                    vision_position.timestamp = hrt_absolute_time();

                    vision_position.x = (float)pos[0]/SCALE;
                    //
                    //T265向前的方向安装正确时，与飞控的y，z方向相反。
                    vision_position.y = ((float)pos[1]/SCALE);
                    vision_position.z = ((float)pos[2]/SCALE);

                    // vision_position.xy_valid = true;
                    // vision_position.z_valid = true;
                    // vision_position.v_xy_valid = true;
                    // vision_position.v_z_valid = true;

                    // vision_position.timestamp = hrt_absolute_time();

                    float_angle[0] = (float)angle[0]/SCALE;
                    // 需要验证是不是这样，同时此处取飞机数据更好，后续优化
                    float_angle[1] = ((float)angle[1]/SCALE);
                    float_angle[2] = ((float)angle[2]/SCALE);

                    // 修正位置
                    matrix::Quatf q(matrix::Eulerf(float_angle[0], float_angle[1], float_angle[2]));

                    // vision_position.q[2] = -vision_position.q[2];
                    // vision_position.q[3] = -vision_position.q[3];
                    q.copyTo(vision_position.q);
                    vision_position.
                    if(print_received_data){
                        PX4_INFO(" x: %2.4f,y: %2.4f,z: %2.4f,roll: %2.4f,pitch :%2.4f yaw:%2.4f,w:%2.4f,x:%2.4f,y:%2.4f,z:%2.4f _is_xbee_data_check_true:%1d",
                                 (double)vision_position.x,(double)vision_position.y,(double)vision_position.z,
                                 (double)float_angle[0],(double)float_angle[1],(double)float_angle[2],
                                 (double)vision_position.q[0],(double)vision_position.q[1],(double)vision_position.q[2],(double)vision_position.q[3],
                                 (int)_is_xbee_data_check_true
                        );
                    }
                    int inst = 0;
                    if(_is_xbee_data_check_true){
                        // PX4_WARN("published");
                        orb_publish_auto(ORB_ID(vehicle_visual_odometry), &_vision_position_pub, &vision_position, &inst, ORB_PRIO_HIGH);
                    } else{
                        // PX4_WARN("NOT published");
                    }
                    }
                else{
                    // PX4_WARN("DATA WRONG 22222 %X,error_num:%\t4d",data,++t265_uart_error_num);
                }
                //校验
                data = '0';
                read(uart_read,&data,1);
            }
            else{
                // PX4_WARN("DATA WRONG 11111 %X,error_num:%\t4d",data,++t265_uart_error_num);
            }
        }
   }
    thread_running = false;
    //取消订阅
    close(uart_read);
    // fflush(stdout);
    return 0;
}
