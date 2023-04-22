/**
 * @file recoder.cpp
 * @author semitia.top
 * @brief 根据按键将/image保存为png；
 * 按下q保存当前最新的图片；按下w保存缓存中的所有图片
 * 将/wrench保存为csv
 * csv文件只用一个
 * @version 0.1
 * @date 2023-04-22
 * @copyright Copyright (c) 2023
 */

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

// Global variables
std::string image_topic; // The name of the image topic to subscribe to
std::string save_path; // The path to save the image files
std::string filename_format; // The format of the filename, should contain %s for timestamp
int cache_size; // The size of the cache buffer
double save_interval; // The interval of saving images in seconds
message_filters::Cache<sensor_msgs::Image> cache; // The cache buffer for image messages
message_filters::Cache<geometry_msgs::WrenchStamped> wrench_cache; // The cache buffer for wrench messages

void save_image(const sensor_msgs::ImageConstPtr& msg)
{
    // Convert the ROS image message to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Get the timestamp of the image message
    std::stringstream ss;
    ss << msg->header.stamp;
    std::string timestamp = ss.str();

    // Construct the filename with timestamp
    char filename[256];
    sprintf(filename, filename_format.c_str(), timestamp.c_str());

    // Save the image as a png file
    cv::imwrite(save_path + filename, cv_ptr->image);

    ROS_INFO("Saved image %s", filename);
}

void save_all_imgs()
{
    // Get the current time
    ros::Time now = ros::Time::now();
    // Get all the images in the cache buffer
    std::vector<sensor_msgs::ImageConstPtr> images = cache.getInterval(cache.getOldestTime(), now);

    // Loop through the images and save them as png files with timestamp in the filename
    for (size_t i = 0; i < images.size(); ++i)
    {
        save_image(images[i]);
    }
    return;
}

void save_latest_img()
{
    ros::Time latest_time = cache.getLatestTime();
    sensor_msgs::ImageConstPtr latest_image = cache.getElemBeforeTime(latest_time);
    save_image(latest_image);
    return;
}

bool save_flag=0;
void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    if(save_flag)
    {
        save_image(msg);
    }
    else
    {
        return;
    }
    return;
}

void wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg)
{

}

void keyboard_init()
{
    struct termios old_settings, new_settings;

    //保存原来的终端属性
    tcgetattr(STDIN_FILENO, &old_settings);
    //设置新的终端属性
    new_settings = old_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO); //关闭规范模式和回显
    new_settings.c_cc[VTIME] = 0; //设置超时时间为0
    new_settings.c_cc[VMIN] = 0; //设置最小字符数为0

    //应用新的终端属性
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
}

void keyboard_restore()
{
    struct termios old_settings;
    //恢复原来的终端属性
    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
}

//key scan timer callback function
void keytimer_callback(const ros::TimerEvent&)
{
    ROS_INFO("check keyboard input");
    char ch;
    int nread;
    //尝试读取一个字符
    nread = read(STDIN_FILENO, &ch, 1);

    if(nread == 1) //如果读取成功
    {
        printf("You pressed %c\n", ch); //打印字符
        switch(ch)
        {
            case 'q':
            {
                ROS_INFO("save single image");
                save_latest_img();
                break;
            }
            case 'w':
            {
                ROS_INFO("save all images");
                save_all_imgs();
                break;
            }
            case 't':
            {
                //get number of images in cache
                ros::Time now = ros::Time::now();
                std::vector<sensor_msgs::ImageConstPtr> check_images = cache.getInterval(cache.getOldestTime(), now);
                ROS_INFO("number of images in cache: %ld", check_images.size());
            }
            case 'b':
            {
                //begin to save image
                ROS_INFO("save image continuously");
                save_flag=1;
                break;
            }
            case 'e':
            {
                //end to save image
                ROS_INFO("stop saving image");
                save_flag=0;
                break;
            }
            default :
            {
                ROS_INFO("unknown input");
                break;
            }
        }
    }
    else //如果读取失败
    {
        printf("No input\n"); //打印提示信息
    }

    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "saver_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    keyboard_init();
    
    // Get the parameters from parameter server or command line arguments
    pnh.param<std::string>("image_topic", image_topic, "image");
    pnh.param<std::string>("save_path", save_path, "/home/stonewu/图片/");
    pnh.param<std::string>("filename_format", filename_format, "image_%s.png");
    pnh.param<int>("cache_size", cache_size, 100);
    pnh.param<double>("save_interval", save_interval, 2.0);
    
    //set up subscriber & cache
    message_filters::Subscriber<sensor_msgs::Image> sub(nh, image_topic, 1);
    sub.registerCallback(boost::bind(&image_callback, _1));
    cache.connectInput(sub);
    cache.setCacheSize(cache_size);
    message_filters::Subscriber<geometry_msgs::WrenchStamped> wrench_sub(nh, "wrench", 1);
    wrench_sub.registerCallback(boost::bind(&wrench_callback, _1));
    wrench_cache.connectInput(wrench_sub);
    wrench_cache.setCacheSize(cache_size);

    //key scan timer：check if keyboard input occured
    ros::Timer KeyScan_timer = nh.createTimer(ros::Duration(1),keytimer_callback);//0.5 second
    ros::spin();
    return 0;
}