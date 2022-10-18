#include "string"
#include "ros/ros.h"
#include "serial/serial.h"
#include "serial_try/serial_head.h"
#include "serial_try/msgs_stm32.h"

Set_Serial init;
ros:: Timer cnt_timer;
ros:: Subscriber usart_listener;
ros:: Publisher pub;

void timer_callback(const ros::TimerEvent&)
{
    std:: string some_msgs ;
    some_msgs = "120001010\r\n";
    init.SendMsgs(some_msgs);

    std:: string data_got,data_got1;
    data_got = init.SerialRead(); 
    ROS_INFO("%s",data_got.c_str());

    serial_try::msgs_stm32 data;
    data.number=1;
    pub.publish(data);
    return;
}

int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"serial_try");
    ros::NodeHandle nh;
    
    init.SerialInit("/dev/ttyUSB0");
    cnt_timer = nh.createTimer(ros::Duration(2),timer_callback);//2 seconds
    //nh.createtimer() is just a function whose return_type is a Timer
    pub = nh.advertise<serial_try::msgs_stm32>("msgs_stm32",10);
    cnt_timer.start();
    ros::spin();
    //init.run();


    return 0;
}