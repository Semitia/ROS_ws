#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/WrenchStamped.h>

using namespace message_filters;
//给模糊策略用的回调函数
void callback(const sensor_msgs::ImageConstPtr& image, const geometry_msgs::WrenchStampedConstPtr& wrench)
{
  ROS_INFO("ApproximateTime");
  //计算两个消息的时间差，打印
  ros::Duration diff = image->header.stamp - wrench->header.stamp;
  ROS_INFO("diff: %f", diff.toSec());
  return;
}

//创建新的回调函数
void callback2(const sensor_msgs::ImageConstPtr& image, const geometry_msgs::WrenchStampedConstPtr& wrench)
{
  ROS_INFO("TimeSynchronizer");
  ros::Duration diff = image->header.stamp - wrench->header.stamp;
  ROS_INFO("diff: %f", diff.toSec());
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "approximate_time_sync");

  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "image", 1);
  message_filters::Subscriber<geometry_msgs::WrenchStamped> wrench_sub(nh, "wrench", 1);

  //创建一个ApproximateTime同步策略，队列大小为10
  typedef sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::WrenchStamped> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image_sub, wrench_sub);
  //注册的回调函数使用bind绑定callback，_1和_2分别表示回调函数两个参数
  sync.registerCallback(boost::bind(&callback, _1, _2));

  //TimeSynchronizer同步器
  //TimeSynchronizer<sensor_msgs::Image, geometry_msgs::WrenchStamped> sync2(image_sub, wrench_sub, 10);
  //sync2.registerCallback(boost::bind(&callback2, _1, _2));

  //创建一个定时器，每隔1秒调用一次回调函数
  //ros::Timer timer = nh.createTimer(ros::Duration(1), [&](const ros::TimerEvent& event){
  //    ROS_INFO("waiting for messages...");
  //   });

  ros::spin();

  return 0;
}