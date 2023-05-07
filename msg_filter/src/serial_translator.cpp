#include "string"
#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"
#include "../include/msg_filter/my_serial.h"
#define ACCEL_SCALE 1000
#define GYRO_SCALE 10000


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

// 将字节数组转换为浮点数,floats[0]~floats[2]加速度；floats[3]时间戳；
//                    floats[4]子弹速度；floats[5]~floats[7]角速度
std::vector<float> bytes_to_floats(std::vector<uint8_t> buff)
{
    std::vector<float> floats;
    // 加速度
    for (int i = 2; i < 7; i += 2)
    {
        uint8_t b[2];
        b[0] = buff[i];
        b[1] = buff[i+1];
        float f = (short)(b[0]<<8) + (short)b[1];
        f/=ACCEL_SCALE;
        floats.push_back(f);
    }
    // 时间戳
    uint8_t t[4];
    t[0] = buff[8];
    t[1] = buff[9];
    t[2] = buff[10];
    t[3] = buff[11];
    uint32_t u = (uint32_t)(t[0]<<24) + (uint32_t)(t[1]<<16) + (uint32_t)(t[2]<<8) + (uint32_t)t[3];
    floats.push_back(u);
    //bullet
    floats.push_back((short)buff[12]);
    // 角速度
    for (int i = 13; i < 18; i += 2)
    {
        uint8_t b[2];
        b[0] = buff[i];
        b[1] = buff[i+1];
        float f = (short)(b[0]<<8) + (short)b[1];
        f/=GYRO_SCALE;
        floats.push_back(f);
    }
    return floats;
}

void translate(std::vector<uint8_t> buff)
{
    //print buff
    size_t num = buff.size();
    for(int i=0;i<num;i++)
    {
        printf("%x ",buff[i]);
    }
    printf("\n");

    //创建imu消息
    sensor_msgs::Imu imu_msg;
    // 从串口读取32个字节的数据，返回一个字节数组

    // 检查数据的头尾和校验位是否正确，如果不正确，跳过这一帧数据
    // if (buff[0] != 0xaa || buff[29] != 0xbb || !check_crc(buff))
    // {
    //     return;
    // }

    // 将字节数组转换为浮点数
    std::vector<float> floats = bytes_to_floats(buff);
    // 将浮点数赋值给imu消息
    ROS_INFO("加速度：x:%f y:%f z:%f",floats[0],floats[1],floats[2]);
    imu_msg.linear_acceleration.x = floats[0];
    imu_msg.linear_acceleration.y = floats[1];
    imu_msg.linear_acceleration.z = floats[2];
    ROS_INFO("角速度：x:%f y:%f z:%f",floats[5],floats[6],floats[7]);
    imu_msg.angular_velocity.x = floats[5];
    imu_msg.angular_velocity.y = floats[6];
    imu_msg.angular_velocity.z = floats[7];
    ROS_INFO("时间戳：%f",floats[3]);
    imu_msg.header.stamp = ros::Time(floats[3]);
    // 发布imu消息
    imu_pub.publish(imu_msg);    
}

std::string string_to_hex(const std::string& input)
{
    static const char hex_digits[] = "0123456789ABCDEF";

    std::string output;
    output.reserve(input.length() * 3);
    for (unsigned char c : input)
    {
        output.push_back(hex_digits[c >> 4]);
        output.push_back(hex_digits[c & 15]);
        output.push_back(' '); // add a space after each hex pair
    }
    return output;
}

void timer_callback(const ros::TimerEvent&)
{
    static int end_flag = 0;
    static std::vector<uint8_t> buff;
    //receive data from serial port
    size_t p = imu_serial.available();
    if(p)
    {
        //DEBUG
        //ROS_INFO("receive %ld bytes",p);
        std::string s = imu_serial.read(p);
        //printf("%s\n",string_to_hex(s).c_str());

        //一次会接受到一个数据包的一小部分，需要将这些数据包拼接起来
        //结束标志是0xcc0xff
        
        //string_to_uint8_t
        std::vector<uint8_t> son_buff(p);
        for(int i=0;i<p;i++)
        {
            son_buff[i] = s[i];
        //    printf("%d ",son_buff[i]);
        }
        //printf("\n");

        for(int i=0;i<p;i++)
        {
            buff.push_back(son_buff[i]);
            if(end_flag == 1)
            {
                if(son_buff[i]==0xff)
                {
                    ROS_INFO("received a package");
                    translate(buff);
                    end_flag=0;
                    buff.clear();
                }
            }
            else 
            {
                end_flag=0;
                if(son_buff[i]==0xcc) end_flag=1;
            }

        }
    }
    return;
}

int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"serial_translator");
    ros::NodeHandle nh;
    
    serial_init("/dev/ttyUSB0");
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu",1000);
    cnt_timer = nh.createTimer(ros::Duration(0.001),timer_callback);//2 seconds
    cnt_timer.start();
    
    ros::spin();
    return 0;
}