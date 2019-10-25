#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_odometry.h>
#include <px4_defines.h>
#include <uORB/topics/parameter_update.h>
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

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;

extern "C" __EXPORT int xbee_pose_main(int argc, char *argv[]);
int xbee_pose_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud); //static
static void usage(const char *reason);            //static
orb_advert_t mavlink_log_pub_xbee_pose = NULL;

struct {
    param_t	xbee_baudrate;
    param_t	xbee_com;
    param_t	pose_scale;
    param_t	xbee_debug_print;
    param_t	xbee_pub_print;
    param_t	xbee_send_quaternion;
    param_t	xbee_print_debug_num;
} _params_handles{};		/**< handles for interesting parameters */

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
    bool xbee_debug_enable = 0;
    bool print_received_data = 0;
    bool print_debug_num = 0;
    int serial_com = 3;
    int baudrate_select = 1;
    unsigned int BAUDRATE=19200;
    int uart_read = -1;
    bool send_quaternion = 1;

    float SCALE=10000.0f;
    int _params_sub = orb_subscribe(ORB_ID(parameter_update));
    bool params_update = false;

    _params_handles.pose_scale = param_find("XBEE_POS_SCALE");
    _params_handles.xbee_baudrate = param_find("XBEE_BAUD_RATE");
    _params_handles.xbee_debug_print = param_find("XBEE_DEBUG_PRINT");
    _params_handles.xbee_pub_print = param_find("XBEE_PUB_PRINT");
    _params_handles.xbee_com = param_find("XBEE_SERIAL_COM");
    _params_handles.xbee_send_quaternion = param_find("XBEE_SEND_QUAT");
    _params_handles.xbee_print_debug_num = param_find("XBEE_DBNUM_PRINT");

    orb_check(_params_sub,&params_update);
    parameter_update_s param_update;
    orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);

    param_get(_params_handles.pose_scale,&SCALE);
    param_get(_params_handles.xbee_com,&serial_com);
    param_get(_params_handles.xbee_pub_print,&print_received_data);
    param_get(_params_handles.xbee_debug_print,&xbee_debug_enable);
    param_get(_params_handles.xbee_baudrate,&baudrate_select);
    param_get(_params_handles.xbee_send_quaternion,&send_quaternion);
    param_get(_params_handles.xbee_print_debug_num,&print_debug_num);

    switch(baudrate_select){
        case 0:
            BAUDRATE=9600;
            break;
        case 1:
            BAUDRATE=19200;
            break;
        case 2:
            BAUDRATE=38400;
            break;
        case 3:
            BAUDRATE=57600;
            break;
        case 4:
            BAUDRATE=115200;
            break;
        default:
            BAUDRATE=19200;
            break;
    }

    switch(serial_com){
        case 0:
            uart_read = uart_init("/dev/ttyS0"); //fmuv5ttys3 fmuv2,v3 ttys6
            break;
        case 1:
            uart_read = uart_init("/dev/ttyS1"); //fmuv5ttys3 fmuv2,v3 ttys6
            break;
        case 2:
            uart_read = uart_init("/dev/ttyS2"); //fmuv5ttys3 fmuv2,v3 ttys6
            break;
        case 3:
            uart_read = uart_init("/dev/ttyS3"); //fmuv5ttys3 fmuv2,v3 ttys6
            break;
        case 4:
            uart_read = uart_init("/dev/ttyS4"); //fmuv5ttys3 fmuv2,v3 ttys6
            break;
        case 6:
            uart_read = uart_init("/dev/ttyS6"); //fmuv5ttys3 fmuv2,v3 ttys6
            break;
        default:
            uart_read = uart_init("/dev/ttyS3");
            break;
    }

    mavlink_log_info(&mavlink_log_pub_xbee_pose,"xbee run ");
    unsigned char data = '0';
    if(uart_read==0)
    {
         mavlink_log_critical(&mavlink_log_pub_xbee_pose,"[YCM]xbee pose uart init is failed\n");
         return -1;
    }
    if(set_uart_baudrate(uart_read,BAUDRATE)==0){
        mavlink_log_critical(&mavlink_log_pub_xbee_pose,"[YCM]set_xbee_uart_baudrate is failed\n");
        return -1;
    }
    mavlink_log_info(&mavlink_log_pub_xbee_pose,"[YCM]xbee uart init is successful\n");
    thread_running = true;
    // 定义话题结构
    struct vehicle_odometry_s vision_position;
    // 初始化数据
    memset(&vision_position, 0 , sizeof(vision_position));
    int pos[3];//pos[0] = x;pos[1] = y;pos[2] = z;
    int check;
    int sum;
    bool _is_xbee_data_check_true;
    int error_count = 0;
    orb_advert_t _vision_position_pub = nullptr;

    PX4_INFO("start get pose!");

    while(thread_running){
        if(send_quaternion){
            static char data_buffer[32];
            static int angle_q[4];/*w,x,y,z*/
            static float float_angle_q[4];

            if(xbee_debug_enable){
                static char t265_buffer[35];
                for(int k = 0;k < 35;++k){
                    data = '0';
                    read(uart_read,&data,1);
                    t265_buffer[k] = data;
                }
                PX4_INFO("%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,"
                         "%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,"
                         "%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,"
                         "%X,%X,%X,%X,%X",
                         t265_buffer[0],t265_buffer[1],t265_buffer[2],t265_buffer[3],t265_buffer[4],t265_buffer[5],
                         t265_buffer[6],t265_buffer[7],t265_buffer[8],t265_buffer[9],t265_buffer[10],t265_buffer[11],
                         t265_buffer[12],t265_buffer[13],t265_buffer[14],t265_buffer[15],t265_buffer[16],t265_buffer[17],
                         t265_buffer[18],t265_buffer[19],t265_buffer[20],t265_buffer[21],t265_buffer[22],t265_buffer[23],
                         t265_buffer[24],t265_buffer[25],t265_buffer[26],t265_buffer[27],t265_buffer[28],t265_buffer[29],
                         t265_buffer[30],t265_buffer[31],t265_buffer[32],t265_buffer[33],t265_buffer[34]);
            }else{
                data = '0';
                read(uart_read,&data,1);
                if(data == 0xFE){
                    data = '0';
                    read(uart_read,&data,1);
                    if (data == 0x22){
                            for (int index = 0;index<32;index++){
                                data = '0';
                                read(uart_read,&data,1);
                                data_buffer[index] = data;
                            }
                            for (int i = 0;i < 3;i++){
                                pos[i] = (int)(data_buffer[4*i]<<24|data_buffer[4*i+1]<<16|data_buffer[4*i+2]<<8|data_buffer[4*i+3]);
                                angle_q[i] = (int)(data_buffer[4*i+12]<<24|data_buffer[4*i+13]<<16|data_buffer[4*i+14]<<8|data_buffer[4*i+15]);
                            }
                            angle_q[3] = (int)(data_buffer[24]<<24|data_buffer[25]<<16|data_buffer[26]<<8|data_buffer[27]);
                            check = (int)(data_buffer[28]<<24|data_buffer[29]<<16|data_buffer[30]<<8|data_buffer[31]);
                            sum = (int)(pos[0]+pos[1]+pos[2]+angle_q[0]+angle_q[1]+angle_q[2]+angle_q[3]);
                            _is_xbee_data_check_true = (check == sum);

                            vision_position.timestamp = hrt_absolute_time();
                            vision_position.x = (float)pos[0]/SCALE;
                            vision_position.y = ((float)pos[1]/SCALE);
                            vision_position.z = ((float)pos[2]/SCALE);
                            float_angle_q[0] = (float)angle_q[0]/SCALE;
                            float_angle_q[1] = ((float)angle_q[1]/SCALE);
                            float_angle_q[2] = ((float)angle_q[2]/SCALE);
                            float_angle_q[3] = ((float)angle_q[3]/SCALE);
                            matrix::Quatf q(float_angle_q);
                            q.copyTo(vision_position.q);



                            vision_position.vx = NAN;
                            vision_position.vy = NAN;
                            vision_position.vz = NAN;
                            vision_position.velocity_covariance[0] = NAN;
                            vision_position.pose_covariance[0] = NAN;
                            vision_position.rollspeed = NAN;
                            vision_position.pitchspeed = NAN;
                            vision_position.yawspeed = NAN;
                            vision_position.local_frame = 0;

                            if(print_received_data){
                                PX4_INFO(" x: %2.4f,y: %2.4f,z: %2.4f, _is_xbee_data_check_true:%1d",
                                         (double)vision_position.x,(double)vision_position.y,(double)vision_position.z,(int)_is_xbee_data_check_true);
                            }
                            int inst = 0;
                            if(_is_xbee_data_check_true){
                                orb_publish_auto(ORB_ID(vehicle_visual_odometry), &_vision_position_pub, &vision_position, &inst, ORB_PRIO_HIGH);
                            } else{
                                if(print_debug_num){
                                    PX4_WARN("Wrong pose data check !!! error num %d",error_count);
                                    error_count++;
                                    if(error_count>100000) error_count = 0;
                                }
                            }
                        }
                        else{
                            if(print_debug_num){
                                PX4_WARN("Wrong pose data 0x22 !!! error num %d",error_count);
                                error_count++;
                                if(error_count>100000) error_count = 0;
                            }
                        }
                        //包尾
                        data = '0';
                        read(uart_read,&data,1);
                    }
                    else{
                        if(print_debug_num){
                            PX4_WARN("Wrong pose data 0xfe !!! error num %d",error_count);
                            error_count++;
                            if(error_count>100000) error_count = 0;
                        }
                    }
                }
            }
        else {
            static char data_buffer[28];
            static int angle[3];/*roll,pitch,yaw*/
            static float float_angle[3];

            if (xbee_debug_enable) {
                static char t265_buffer[31];
                for (int k = 0; k < 31; ++k) {
                    data = '0';
                    read(uart_read, &data, 1);
                    t265_buffer[k] = data;
                }
                PX4_INFO("%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,"
                         "%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,"
                         "%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,"
                         "%X",
                         t265_buffer[0], t265_buffer[1], t265_buffer[2], t265_buffer[3], t265_buffer[4], t265_buffer[5],
                         t265_buffer[6], t265_buffer[7], t265_buffer[8], t265_buffer[9], t265_buffer[10],
                         t265_buffer[11],
                         t265_buffer[12], t265_buffer[13], t265_buffer[14], t265_buffer[15], t265_buffer[16],
                         t265_buffer[17],
                         t265_buffer[18], t265_buffer[19], t265_buffer[20], t265_buffer[21], t265_buffer[22],
                         t265_buffer[23],
                         t265_buffer[24], t265_buffer[25], t265_buffer[26], t265_buffer[27], t265_buffer[28],
                         t265_buffer[29],
                         t265_buffer[30]);
            } else {
                data = '0';
                read(uart_read, &data, 1);
                if (data == 0xFE) {
                    data = '0';
                    read(uart_read, &data, 1);
                    if (data == 0x22) {
                        for (int index = 0; index < 28; index++) {
                            data = '0';
                            read(uart_read, &data, 1);
                            data_buffer[index] = data;
                        }
                        for (int i = 0; i < 3; i++) {
                            pos[i] = (int) (data_buffer[4 * i] << 24 | data_buffer[4 * i + 1] << 16 |
                                            data_buffer[4 * i + 2] << 8 | data_buffer[4 * i + 3]);
                            angle[i] = (int) (data_buffer[4 * i + 12] << 24 | data_buffer[4 * i + 13] << 16 |
                                              data_buffer[4 * i + 14] << 8 | data_buffer[4 * i + 15]);
                        }
                        check = (int) (data_buffer[24] << 24 | data_buffer[25] << 16 | data_buffer[26] << 8 |
                                       data_buffer[27]);
                        sum = (int) (pos[0] + pos[1] + pos[2] + angle[0] + angle[1] + angle[2]);
                        _is_xbee_data_check_true = (check == sum);

                        vision_position.timestamp = hrt_absolute_time();
                        vision_position.x = (float) pos[0] / SCALE;
                        vision_position.y = ((float) pos[1] / SCALE);
                        vision_position.z = ((float) pos[2] / SCALE);
                        float_angle[0] = (float) angle[0] / SCALE;
                        float_angle[1] = ((float) angle[1] / SCALE);
                        float_angle[2] = ((float) angle[2] / SCALE);

                        matrix::Quatf q(matrix::Eulerf(float_angle[0], float_angle[1], float_angle[2]));
                        q.copyTo(vision_position.q);
                        vision_position.vx = NAN;
                        vision_position.vy = NAN;
                        vision_position.vz = NAN;
                        vision_position.velocity_covariance[0] = NAN;
                        vision_position.pose_covariance[0] = NAN;
                        vision_position.rollspeed = NAN;
                        vision_position.pitchspeed = NAN;
                        vision_position.yawspeed = NAN;
                        vision_position.local_frame = 0;

                        if(print_received_data){
                            PX4_INFO(" x: %2.4f,y: %2.4f,z: %2.4f, _is_xbee_data_check_true:%1d",
                                     (double)vision_position.x,(double)vision_position.y,(double)vision_position.z,(int)_is_xbee_data_check_true);
                        }
                        int inst = 0;
                        if(_is_xbee_data_check_true){
                            orb_publish_auto(ORB_ID(vehicle_visual_odometry), &_vision_position_pub, &vision_position, &inst, ORB_PRIO_HIGH);
                        } else{
                            if(print_debug_num){
                                PX4_WARN("Wrong pose data check !!! error num %d",error_count);
                                error_count++;
                                if(error_count>100000) error_count = 0;
                            }
                        }
                    }
                    else{
                        if(print_debug_num){
                            PX4_WARN("Wrong pose data 0x22 !!! error num %d",error_count);
                            error_count++;
                            if(error_count>100000) error_count = 0;
                        }
                    }
                    //包尾
                    data = '0';
                    read(uart_read,&data,1);
                }
                else{
                    if(print_debug_num){
                        PX4_WARN("Wrong pose data 0xfe !!! error num %d",error_count);
                        error_count++;
                        if(error_count>100000) error_count = 0;
                    }
                }
            }
        }
   }
    thread_running = false;
    //取消订阅
    close(uart_read);
    // fflush(stdout);
    return 0;
}
