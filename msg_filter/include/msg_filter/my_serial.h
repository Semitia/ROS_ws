#ifndef _SERIAL_HEAD_H
#define _SERIAL_HEAD_H

#include "ros/ros.h"
#include "serial/serial.h"
#include "string"

class Set_Serial
{
    private:
        serial:: Serial a_ser;
    public:
        void run();
        //void StopFans();
        void SerialInit(std::string port);
        void SendMsgs(std::string msgs);
        //void SetFan(float pre1, float pre2, float pre3);
        //void CloseSerial();
        std::string SerialRead();
        //void FanLimit(float& input_value, float max = 0.0, float min = -10.0);

};

void Set_Serial:: run()
{
    ROS_INFO("heyhey...My Keli...heyhey...");
}

void Set_Serial:: SendMsgs(std::string msgs)
{
    //char haha[7]="123456";
    //a_ser.write(haha);
    a_ser.write(msgs);
    //ROS_INFO("I have written %s !\r\n",msgs.c_str());
    //a_ser.flushOutput();
    return;
}

void Set_Serial:: SerialInit(std::string port)
{
    try
    {
        a_ser.setPort("/dev/ttyUSB0");
        a_ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(10);
        a_ser.setTimeout(to);
        a_ser.setStopbits(serial::stopbits_one);                     //一个停止位
            
        a_ser.open(); 
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
		return ;
    }
    
    if(a_ser.isOpen()) 
	{
         ROS_INFO_STREAM("Serial Port opened");
    } 
	else 
	{
        return;
    }
}

std::string Set_Serial::SerialRead()
{
    std::string data_str;
    size_t p = a_ser.available();//return the size of the string

    //串口数据读取
    if (p != 0)
    {
        data_str = a_ser.read(p);//tell ros the  number of the char you need to read;
        a_ser.flushInput();
    }

    //ROS_INFO("%s",data_ser.c_str());
    return data_str;
}

#endif