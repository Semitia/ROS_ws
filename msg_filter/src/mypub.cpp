/**
 * @file mypub.cpp
 * @author Semitia.top
 * @brief 发布/imu, /image, /wrench三个话题，数据内容为随机数，但时间戳为发布时间
 * @version 0.1
 * @date 2023-04-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#define Periodicity_image 1
#define Periodicity_wrench 0.31
#define Periodicity_imu 0.5

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publisher_node");
  ros::NodeHandle nh;

  // create publishers for image and wrench topics
  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("image", 1);
  ros::Publisher wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("wrench", 1);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1);

  // create timers for different publishing frequencies
  ros::Timer image_timer = nh.createTimer(ros::Duration(Periodicity_image), [&](const ros::TimerEvent& event){
    // create a dummy image with random pixels
    cv::Mat image(48, 64, CV_8UC3);
    cv::randu(image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

    // convert the image to a sensor_msgs::Image message
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    //给这个消息加上时间戳
    msg->header.stamp = ros::Time::now();
    //再加上坐标系
    msg->header.frame_id = "base_link";
    // publish the image message
    image_pub.publish(msg);
    });

  ros::Timer wrench_timer = nh.createTimer(ros::Duration(Periodicity_wrench), [&](const ros::TimerEvent& event){
    // create a dummy wrench with random values
    geometry_msgs::WrenchStamped wrench;
    wrench.header.stamp = ros::Time::now();
    wrench.header.frame_id = "base_link";
    wrench.wrench.force.x = (double)rand() / RAND_MAX;
    wrench.wrench.force.y = (double)rand() / RAND_MAX;
    wrench.wrench.force.z = (double)rand() / RAND_MAX;
    wrench.wrench.torque.x = (double)rand() / RAND_MAX;
    wrench.wrench.torque.y = (double)rand() / RAND_MAX;
    wrench.wrench.torque.z = (double)rand() / RAND_MAX;
    // publish the wrench message
    wrench_pub.publish(wrench);
  });

  ros::Timer IMU_timer = nh.createTimer(ros::Duration(Periodicity_imu), [&](const ros::TimerEvent& event){
    //create a dummy IMU with random values
    sensor_msgs::Imu imu_data;
    imu_data.header.stamp = ros::Time::now();
    imu_data.header.frame_id = "base_link";
    imu_data.orientation.x = (double)rand() / RAND_MAX;
    imu_data.orientation.y = (double)rand() / RAND_MAX;
    imu_data.orientation.z = (double)rand() / RAND_MAX;
    imu_data.orientation.w = (double)rand() / RAND_MAX;
    imu_data.angular_velocity.x = (double)rand() / RAND_MAX;
    imu_data.angular_velocity.y = (double)rand() / RAND_MAX;
    imu_data.angular_velocity.z = (double)rand() / RAND_MAX;
    imu_data.linear_acceleration.x = (double)rand() / RAND_MAX;
    imu_data.linear_acceleration.y = (double)rand() / RAND_MAX;
    imu_data.linear_acceleration.z = (double)rand() / RAND_MAX;
    // publish the IMU message
    imu_pub.publish(imu_data);
  });
  // spin the node
  ros::spin();

  return 0;
}