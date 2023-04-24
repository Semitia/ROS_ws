#include "string"
#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"
#include "../include/msg_filter/my_serial.h"

serial::Serial imu_serial;
ros:: Timer cnt_timer;
ros:: Publisher imu_pub;

void serial_init(std::string port)
{
    // 设置串口属性，并打开串口
    imu_serial.setPort(port);
    imu_serial.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    imu_serial.setTimeout(to);
    imu_serial.open();
}

//CRC校验
bool check_crc(std::vector<uint8_t> buff)
{
    uint8_t crc = 0;
    for (int i = 0; i < 30; i++)
    {
        crc ^= buff[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc <<= 1;
            }
            crc &= 0xff;
        }
    }
    return crc == buff[30];
}

// 将字节数组转换为浮点数,floats[0]~floats[5]为加速度和角速度，floats[6]为时间戳
std::vector<float> bytes_to_floats(std::vector<uint8_t> buff)
{
    std::vector<float> floats;
    // 加速度和角速度
    for (int i = 1; i < 29; i += 4)
    {
        uint8_t b[4];
        b[0] = buff[i];
        b[1] = buff[i+1];
        b[2] = buff[i+2];
        b[3] = buff[i+3];
        float f;
        memcpy(&f, b, 4);
        floats.push_back(f);
    }
    // 时间戳
    uint8_t t[4];
    t[0] = buff[25];
    t[1] = buff[26];
    t[2] = buff[27];
    t[3] = buff[28];
    uint32_t u;
    memcpy(&u, t, 4);
    floats.push_back(u);
    return floats;
}

void timer_callback(const ros::TimerEvent&)
{
    //创建imu消息
    sensor_msgs::Imu imu_msg;
    // 从串口读取32个字节的数据，返回一个字节数组
    std::vector<uint8_t> buff;
    imu_serial.read(buff, 32);
    // 检查数据的头尾和校验位是否正确，如果不正确，跳过这一帧数据
    if (buff[0] != 0xaa || buff[29] != 0xbb || !check_crc(buff))
    {
        return;
    }
    // 将字节数组转换为浮点数
    std::vector<float> floats = bytes_to_floats(buff);
    // 将浮点数赋值给imu消息
    imu_msg.linear_acceleration.x = floats[0]; 
    imu_msg.linear_acceleration.y = floats[1];
    imu_msg.linear_acceleration.z = floats[2];
    imu_msg.angular_velocity.x = floats[3];
    imu_msg.angular_velocity.y = floats[4];
    imu_msg.angular_velocity.z = floats[5];
    imu_msg.header.stamp = ros::Time(floats[6]);
    // 发布imu消息
    imu_pub.publish(imu_msg);
    return;
}

int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"serial_translator");
    ros::NodeHandle nh;
    
    serial_init("/dev/ttyUSB0");
    cnt_timer = nh.createTimer(ros::Duration(2),timer_callback);//2 seconds
    cnt_timer.start();
    
    ros::spin();
    return 0;
}