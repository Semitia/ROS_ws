#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <string>
#include <sstream>
#include <termio.h>
#include <stdio.h>
#include "../csv-parser/single_include/csv.hpp"

using namespace csv;
using namespace std;
CSVWriter<ofstream>* writer;

void wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
    // 将消息中的数据写入到CSV文件中
    ostringstream oss;
    oss << msg->header.stamp << " " << msg->wrench.force.x << " " << msg->wrench.force.y << " " << msg->wrench.force.z << " " << msg->wrench.torque.x << " " << msg->wrench.torque.y << " " << msg->wrench.torque.z<<" "<< "\n";
        // 创建一个vector<string>对象
    vector<string> data;
    // 创建一个istringstream对象，用来读取oss中的字符串
    istringstream iss(oss.str());
    // 创建一个string对象，用来存储每个子字符串
    string word;
    // 使用空格作为分隔符，循环读取每个子字符串
    while (iss >> word) {
    // 将子字符串放入向量中
    data.push_back(word);
    }
    //*writer << data ;
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CSV_saver");
    ros::NodeHandle nh;

    message_filters::Subscriber<geometry_msgs::WrenchStamped> wrench_sub(nh, "wrench", 1);
    wrench_sub.registerCallback(boost::bind(&wrench_callback, _1));

    writer = new CSVWriter<ofstream>("/home/stonewu/Data/test.csv");
    // 向文件中添加一行表头
    auto header = std::vector<std::string>{"time", "fx", "fy", "fz", "tx","ty","tz","\n"};
    //*writer << header;
    ros::spin();
    delete writer;
    return 0;
}