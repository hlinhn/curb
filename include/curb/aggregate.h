#ifndef AGGREGATE_H_
#define AGGREGATE_H_

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

class Aggregate
{
private:
  ros::Publisher point_pub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub_;
  std::string frame_id_;
  double factor_;
  void pointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  CloudT::Ptr aggregated_cloud_;
  tf::TransformListener tf_;
  tf::MessageFilter<sensor_msgs::PointCloud2> *tf_filter_;

public:
  Aggregate(ros::NodeHandle nh);
  ~Aggregate() {}
};

#endif
