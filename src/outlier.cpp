#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

typedef pcl::PointCloud<pcl::PointXYZI> CloudT;
typedef pcl::PointXYZI PointT;

void radiusRemoval(CloudT::Ptr points, double radius, int neighbors) {
  pcl::RadiusOutlierRemoval<PointT> extract;
  extract.setInputCloud(points);
  extract.setRadiusSearch(radius);
  extract.setMinNeighborsInRadius(neighbors);
  CloudT::Ptr out(new CloudT());
  extract.filter(*out);
  pcl::io::savePCDFile("radius.pcd", *out);
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "USAGE: downsample filename\n";
    return 1;
  }

  CloudT::Ptr points(new CloudT());
  const bool read_success = pcl::io::loadPCDFile(std::string(argv[1]), *points) == 0;
  if(!read_success)
  {
    return 1;
  }

  double radius = 0.1;
  if (argc > 2) radius = std::stod(argv[2]);
  radiusRemoval(points, radius, 1);

  return 0;
}
