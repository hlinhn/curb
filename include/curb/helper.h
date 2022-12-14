#ifndef CURB_HELPER_H_
#define CURB_HELPER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> Cloud;

struct CurbAggregateParam
{
  double factor;
  std::string method;
  double normal_sample;
  int normal_ksearch;
  double normal_z_upper;
  double normal_z_lower;
  double normal_cur_upper;
  double normal_cur_lower;
};

Cloud::Ptr subsample(Cloud::Ptr cloud, double ratio);
CurbAggregateParam readParamFile(std::string file_name);

#endif