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
  nh_params.param("robot_frame", robot_frame_, std::string("base_link"));
  std::string param_file_name;
  nh_params.param("param_file", param_file_name, std::string(""));
  param_ = readParamFile(param_file_name);

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

Cloud::Ptr
Aggregate::findCurbWithImage(Cloud::Ptr cloud, cv::Point2d pose)
{
  cv::Point2d min_point(pose.x - param_.image_half_size, pose.y - param_.image_half_size);

  unsigned int size = std::ceil(param_.image_half_size * 2 / param_.image_resolution);

  cv::Mat min_image(size, size, CV_64FC1, cv::Scalar(0));
  cv::Mat max_image(size, size, CV_64FC1, cv::Scalar(0));
  cv::Mat count_image(size, size, CV_64FC1, cv::Scalar(0));

  for (const auto p : *cloud)
  {
    int x = std::ceil((p.x - min_point.x) / param_.image_resolution);
    int y = std::ceil((p.y - min_point.y) / param_.image_resolution);
    if (x < 0 || x >= size || y < 0 || y >= size)
    {
      continue;
    }
    if (count_image.at<double>(x, y) < 1)
    {
      min_image.at<double>(x, y) = p.z;
      max_image.at<double>(x, y) = p.z;
      count_image.at<double>(x, y) = 1;
    }
    else
    {
      if (p.z < min_image.at<double>(x, y))
      {
        min_image.at<double>(x, y) = p.z;
      }
      if (p.z > max_image.at<double>(x, y))
      {
        max_image.at<double>(x, y) = p.z;
      }
      count_image.at<double>(x, y) += 1;
    }
  }

  cv::Mat bindiff(size, size, CV_8UC1, cv::Scalar(255));
  for (int i = 0; i < size; i++)
  {
    for (int j = 0; j < size; j++)
    {
      if (count_image.at<double>(i, j) < param_.image_min_num)
      {
        continue;
      }
      auto diff = max_image.at<double>(i, j) - min_image.at<double>(i, j);
      if (diff < param_.image_max && diff > param_.image_min)
      {
        bindiff.at<unsigned char>(i, j) = 255 - (diff - param_.image_min) / (param_.image_max - param_.image_min) * 255;
      }
    }
  }

  Cloud::Ptr curb_cloud(new Cloud);
  for (const auto p : *cloud)
  {
    int x = std::ceil((p.x - min_point.x) / param_.image_resolution);
    int y = std::ceil((p.y - min_point.y) / param_.image_resolution);
    if (x < 0 || x >= size || y < 0 || y >= size)
    {
      continue;
    }
    if (bindiff.at<unsigned char>(x, y) < param_.image_threshold)
    {
      curb_cloud->points.push_back(p);
    }
  }
  return curb_cloud;
}

void
Aggregate::pointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  Cloud::Ptr cloud(new Cloud);
  pcl::fromROSMsg(*msg, *cloud);
  Cloud::Ptr transformed_to_robot(new Cloud);

  pcl_ros::transformPointCloud<Point>(robot_frame_, *cloud, *transformed_to_robot, tf_);
  auto filtered_by_height = removeHighZ(transformed_to_robot, param_.min_z, param_.max_z);

  Cloud::Ptr transformed(new Cloud);
  pcl_ros::transformPointCloud<Point>(frame_id_, *filtered_by_height, *transformed, tf_);
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
    pcl::toROSMsg(*curb_cloud, curb_msg);
    curb_msg.header = msg->header;
    curb_msg.header.frame_id = frame_id_;
    curb_pub_.publish(curb_msg);
  }

  if (param_.method.compare(std::string("image")) == 0)
  {
    tf::StampedTransform base_pose;
    try
    {
      tf_.lookupTransform(frame_id_, robot_frame_, msg->header.stamp, base_pose);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
    cv::Point2d current_position(base_pose.getOrigin().x(), base_pose.getOrigin().y());
    auto curb_cloud = findCurbWithImage(aggregated_cloud_, current_position);
    sensor_msgs::PointCloud2 curb_msg;
    pcl::toROSMsg(*curb_cloud, curb_msg);
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
