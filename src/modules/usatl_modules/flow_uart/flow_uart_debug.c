#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
// #include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/upixels_flow.h>
#include <uORB/topics/debug_vect.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_shutdown.h>
#include <px4_tasks.h>
#include <px4_time.h>

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;


__EXPORT int flow_uart_debug_main(int argc, char *argv[]);
int flow_uart_thread_main(int argc, char *argv[]);
float average_buffer(float *buffer,int size);

static int uart_init(const char * uart_name);    //
static int set_uart_baudrate(const int fd, unsigned int baud); //static
static void usage(const char *reason);            //static
// orb_advert_t mavlink_log_pub = NULL;

//buffer 是个float数组，size 是数组的大小
float average_buffer(float *buffer,int size){
    float sum = 0;
    for(int i = 0; i < size; i++)
    {
        sum += *buffer++;
    }
    return (sum/size);
}


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

int flow_uart_debug_main(int argc, char *argv[])
{

// mavlink_log_critical(&mavlink_log_pub,"[inav] upixels_flow_main on init");
// mavlink_log_critical(mavlink_log_pub, "test>>>>>>> %8.4f",9.9898);
			
    if (argc < 2) 
    {
        usage("[YCM]missing command");
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            PX4_INFO("[YCM]already running\n");
            exit(0);
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("flow_uart",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         2500,
                         flow_uart_thread_main,
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

int flow_uart_thread_main(int argc, char *argv[])
{
    // mavlink_log_critical(&mavlink_log_pub,"upixels_flow run ");
    char data = '0';
    //const char *uart_name = argv[1];
    char buffer[7] = "";
    float upixels_flow_x = -1;
    float upixels_flow_y = -1;
    float integration_timespan = -1;
    // long checksum = 0; 
    // uint8_t valid = 0;
    int uart_read = uart_init("/dev/ttyS6");
    
    

    if(false == uart_read)
    {
        //  mavlink_log_critical(&mavlink_log_pub,"[YCM]uart init is failed\n");
         return -1;
    }
    if(false == set_uart_baudrate(uart_read,19200)){
        // mavlink_log_critical(&mavlink_log_pub,"[YCM]set_uart_baudrate is failed\n");
        return -1;
    }
    // mavlink_log_critical(&mavlink_log_pub,"[YCM]uart init is successful\n");

    thread_running = true;

    // 定义话题结构
    struct debug_vect_s flow_data;
    // 初始化数据
    memset(&flow_data, 0 , sizeof(flow_data));
   
    //公告消息
    orb_advert_t flow_data_handle = orb_advertise(ORB_ID(debug_vect), &flow_data);//公告这个主题

    // 测试订阅程序
    // int test_sub_handle = orb_subscribe(ORB_ID(upixels_flow));
    // struct upixels_flow_s test_sub;

    // int counter = 0;
    int buffer_size= 5;
    float fliter_buffer_x[buffer_size];
    float fliter_buffer_y[buffer_size];
    memset(&fliter_buffer_x, 0 , sizeof(fliter_buffer_x));
    memset(&fliter_buffer_x, 0 , sizeof(fliter_buffer_x));

    int count = 0;

    while(thread_running)
   {
        // PX4_WARN("warnxxxxxxxxxxxx!!!!");
        //解码 串口信息
    	read(uart_read,&data,1);
        if((data == 0xFE))
        {
           data = '0';
           read(uart_read,&data,1);
           if((data == 0x0A))
           {
               for(int k = 0;k < 6;++k)
                {
                    data = '0';
                    read(uart_read,&data,1);
                    buffer[k] = data;
                }
                for(int k = 0;k < 3;++k)
                {
                    data = '0';
                    read(uart_read,&data,1);
                }
                buffer[6] = data;
                // debug 输出数据 十六进制
                // PX4_INFO("%X,%X,%X,%X,%X,%X,%X,",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6]);

                //X 像素点累计时间内的累加位移,(radians*10000) [除以 10000 乘以高度(mm)后为实际位移(mm)]
                integration_timespan = (float)((uint16_t)(buffer[5]<<8|buffer[4])/1000);

                upixels_flow_x = (float)((int16_t)(buffer[1]<<8|buffer[0])/(10*integration_timespan));
                upixels_flow_y = (float)((int16_t)(buffer[3]<<8|buffer[2])/(10*integration_timespan ));
                // integration_timespan = (buffer[5]*256+buffer[4])*1000;//单位 ms


                

                if (buffer[6] == 0xF5) {
                    fliter_buffer_x[count] = upixels_flow_x;
                    fliter_buffer_y[count] = upixels_flow_y;
                    if (count==buffer_size-1) {
                        count = 0;
                    }
                    else
                    {
                        ++count;
                    }
                    
                    upixels_flow_x = average_buffer(fliter_buffer_x,buffer_size);
                    upixels_flow_y = average_buffer(fliter_buffer_y,buffer_size);
                    
                    //数据可用
                    PX4_INFO("upixels_flow_dara : flow_x: %8.4f\t flow_y: %8.4f\t time: %8.4f\t\n",
                    (double)upixels_flow_x,
                    (double)upixels_flow_y,
                    (double)integration_timespan);
                    //发布数据
                    flow_data.x = (float)upixels_flow_x;
                    flow_data.y = (float)upixels_flow_y;
                    flow_data.z = (float)integration_timespan;
                    strcpy(flow_data.name,"up_flow");
                    orb_publish(ORB_ID(debug_vect),flow_data_handle,&flow_data);
                    //初始化数据 
                    // memset(&buffer, 0 , sizeof(buffer));
                    // memset(&flow_data,0,sizeof(flow_data));

                    //测试订阅数据 并打印
                    // orb_copy(ORB_ID(upixels_flow),test_sub_handle,&test_sub);
                    // PX4_INFO("upixels_flow_dara : flow_x: %8.4f\t flow_y: %8.4f\t time: %8.4f\t\n",
                    // (double)test_sub.upixels_flow_x,
                    // (double)test_sub.upixels_flow_y,
                    // (double)test_sub.integration_timespan);
                }
                else{
                    continue;
                }

            }
        }
   }
    thread_running = false;
    //取消订阅
    // orb_unsubscribe(test_sub_handle);
    close(uart_read);
    fflush(stdout);
    return 0;

}
