#include <curb/aggregate.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/transforms.h>

Aggregate::Aggregate(ros::NodeHandle nh)
{
  ros::NodeHandle nh_params("~");

  nh_params.param("frame_id", frame_id_, std::string("odom"));
  cloud_sub_.subscribe(nh, "input", 1);
  tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud_sub_, tf_, frame_id_, 10);
  tf_filter_->registerCallback(boost::bind(&Aggregate::pointCallback, this, _1));
  point_pub_ = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
  curb_pub_ = nh.advertise<sensor_msgs::PointCloud2>("curb_output", 1);
  aggregated_cloud_.reset(new Cloud);
}

Cloud::Ptr
Aggregate::findCurbWithNormal(Cloud::Ptr cloud)
{
  auto subsampled = subsample(cloud, param_.normal_sample);

  pcl::NormalEstimation<Point, pcl::PointXYZINormal> normal_estimator;
  normal_estimator.setKSearch(param_.normal_ksearch);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals(new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::copyPointCloud<Point, pcl::PointXYZINormal>(*subsampled, *normals);

  normal_estimator.setInputCloud(subsampled);
  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
  normal_estimator.setSearchMethod(tree);
  normal_estimator.compute(*normals);

  std::vector<int> indices;
  pcl::removeNaNNormalsFromPointCloud(*normals, *normals, indices);

  pcl::PassThrough<pcl::PointXYZINormal> normal_z_filter;
  normal_z_filter.setFilterFieldName("normal_z");
  normal_z_filter.setFilterLimits(param_.normal_z_lower, param_.normal_z_upper);
  normal_z_filter.setInputCloud(normals);
  normal_z_filter.filter(*normals);

  pcl::PassThrough<pcl::PointXYZINormal> curvature_filter;
  normal_z_filter.setFilterFieldName("curvature");
  normal_z_filter.setFilterLimits(param_.normal_cur_lower, param_.normal_cur_upper);
  normal_z_filter.setInputCloud(normals);
  normal_z_filter.filter(*normals);

  Cloud::Ptr curb(new Cloud);
  pcl::copyPointCloud<pcl::PointXYZINormal, Point>(*normals, *curb);
  return curb;
}

void
Aggregate::pointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  Cloud::Ptr cloud(new Cloud());
  pcl::fromROSMsg(*msg, *cloud);
  Cloud::Ptr transformed(new Cloud());

  pcl_ros::transformPointCloud<Point>(frame_id_, *cloud, *transformed, tf_);
  *aggregated_cloud_ += *transformed;
  if (aggregated_cloud_->points.size() > transformed->points.size() * param_.factor)
  {
    aggregated_cloud_->erase(aggregated_cloud_->points.begin(),
                             aggregated_cloud_->points.begin() + transformed->points.size());
  }

  sensor_msgs::PointCloud2 out;
  pcl::toROSMsg(*aggregated_cloud_, out);
  out.header = msg->header;
  out.header.frame_id = frame_id_;
  point_pub_.publish(out);

  if (param_.method.compare(std::string("normal")) == 0)
  {
    auto curb_cloud = findCurbWithNormal(aggregated_cloud_);
    sensor_msgs::PointCloud2 curb_msg;
    curb_msg.header = msg->header;
    curb_msg.header.frame_id = frame_id_;
    curb_pub_.publish(curb_msg);
  }
}

int
main(int argc, char* argv[])
{
  ros::init(argc, argv, "aggregate");
  ros::NodeHandle nh;
  Aggregate r(nh);
  ros::spin();
  return 0;
}
