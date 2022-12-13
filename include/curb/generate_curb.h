#ifndef GENERATE_CURB_
#define GENERATE_CURB_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZI> CloudT;
typedef pcl::PointXYZI PointT;

struct HeightInd {
    float height;
    int ind;
    HeightInd(float h, int i): height(h), ind(i) {}
};

#endif
