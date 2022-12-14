#ifndef CURB_AGGREGATE_H_
#define CURB_AGGREGATE_H_

#include <curb/helper.h>
#include <message_filters/subscriber.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

class Aggregate
{
private:
  std::string frame_id_;
  CurbAggregateParam param_;
  Cloud::Ptr aggregated_cloud_;

  ros::Publisher point_pub_;
  ros::Publisher curb_pub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  tf::TransformListener tf_;
  tf::MessageFilter<sensor_msgs::PointCloud2>* tf_filter_;

  Cloud::Ptr findCurbWithNormal(Cloud::Ptr cloud);
  void pointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

public:
  Aggregate(ros::NodeHandle nh);
  ~Aggregate() {}
};

#endif
