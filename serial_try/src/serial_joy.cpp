#include "string"
#include "ros/ros.h"
#include "serial/serial.h"
#include "serial_try/serial_head.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

Set_Serial init;
ros:: Timer cnt_timer;
ros:: Subscriber sub;
int buf_i[10];
float buf_f[8];
//对应按键:   左摇杆     右摇杆    上下左右键    
const int lx=0,ly=1,rx=2,ry=3,l_r=4,u_d=5,triangle=0,circle=1,cross=2,square=3,L2=4,R2=5,L1=6,R1=7;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_s)
{
    for(int i=0;i<=5;i++) {buf_f[i]=joy_s->axes[i];}
    for(int i=0;i<=7;i++) {buf_i[i]=joy_s->buttons[i];}
    ROS_INFO("Got it!");
    ROS_INFO("lx%.3f ly%.3f",buf_f[lx],buf_f[ly]);

    /**/
    std:: string some_msgs ;
    some_msgs = "120001010\r\n";
    init.SendMsgs(some_msgs);

    std:: string data_got,data_got1;
    data_got = init.SerialRead(); 

    ROS_INFO("%s",data_got.c_str());
    return;
    
}

void timer_callback(const ros::TimerEvent&)
{
    std:: string some_msgs ;
    some_msgs = "120001010\r\n";
    init.SendMsgs(some_msgs);

    std:: string data_got,data_got1;
    data_got = init.SerialRead(); 

    ROS_INFO("%s",data_got.c_str());
    return;
}

int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"serial_joy");
    ros::NodeHandle nh;
    
    init.SerialInit("/dev/ttyUSB0");
    //cnt_timer = nh.createTimer(ros::Duration(2),timer_callback);//2 seconds
    sub = nh.subscribe<sensor_msgs::Joy>("joy", 10,joyCallback);
    //nh.createtimer() is just a function whose return_type is a Timer
    //cnt_timer.start();
    ros::spin();
    //init.run();


    return 0;
}