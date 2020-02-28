#ifndef GENERATE_CURB_
#define GENERATE_CURB_

#include <grid_map_core/GridMap.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <grid_map_core/GridMapMath.hpp>

typedef pcl::PointCloud<pcl::PointXYZI> CloudT;
typedef pcl::PointXYZI PointT;

struct HeightInd {
  float height;
  int ind;
HeightInd(float h, int i): height(h), ind(i) {}
};

#endif
