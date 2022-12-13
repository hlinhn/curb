#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <curb/aggregate.h>

Aggregate::Aggregate(ros::NodeHandle nh)
{
  ros::NodeHandle nh_params("~");

  nh_params.param("frame_id", frame_id_, std::string("odom"));
  nh_params.param("factor", factor_, 7.0);
  cloud2_sub_.subscribe(nh, "input", 10);
  tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud2_sub_, tf_, frame_id_, 10);
  tf_filter_->registerCallback(boost::bind(&Aggregate::pointCallback, this, _1));
  point_pub = nh.advertise<sensor_msgs::PointCloud2>("output", 10);
  CloudT::Ptr full_cloud(new CloudT());
  aggregated_cloud_ = full_cloud;
}

void Aggregate::pointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  CloudT::Ptr cloud(new CloudT());
  pcl::fromROSMsg(*msg, *cloud);
  CloudT::Ptr transformed(new CloudT());

  pcl_ros::transformPointCloud<PointT>(frame_id_, *cloud, *transformed, tf_);
  *aggregated_cloud_ += *transformed;
  if (aggregated_cloud_->points.size() > transformed->points.size() * factor_) {
    aggregated_cloud_->erase(aggregated_cloud_->points.begin(),
                             aggregated_cloud_->points.begin() + transformed->points.size());
  }

  sensor_msgs::PointCloud2 out;
  pcl::toROSMsg(*aggregated_cloud_, out);
  out.header = msg->header;
  out.header.frame_id = frame_id_;
  point_pub.publish(out);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "aggregate");
  ros::NodeHandle nh;
  Aggregate r(nh);
  ros::spin();
  return 0;
}
