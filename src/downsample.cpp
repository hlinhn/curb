#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h>

typedef pcl::PointCloud<pcl::PointXYZI> CloudT;
typedef pcl::PointXYZI PointT;

void downsample(CloudT::Ptr points, double ratio) {
  pcl::RandomSample<PointT> extract;
  extract.setInputCloud(points);
  extract.setSample(int(points->size() * ratio));
  CloudT::Ptr out(new CloudT());
  extract.filter(*out);
  pcl::io::savePCDFile("downsampled.pcd", *out);
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "USAGE: downsample filename\n";
    return 1;
  }

  CloudT::Ptr points(new CloudT());
  const bool read_success = pcl::io::loadPLYFile(std::string(argv[1]), *points) == 0;
  if(!read_success)
  {
    std::cerr << "PCL failed to load .ply file: " << argv[1] << std::endl;
    return 1;
  }

  double ratio = 0.1;
  if (argc > 2) ratio = std::stod(argv[2]);
  downsample(points, ratio);

  return 0;
}
