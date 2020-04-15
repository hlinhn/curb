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
#include <grid_map_core/iterators/SlidingWindowIterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZI> CloudT;
typedef pcl::PointXYZI PointT;

std::pair<PointT, PointT> cloudMinmax(CloudT::Ptr points) {
  if (points->size() == 0) {
    std::cerr << "Empty cloud\n";
    throw std::invalid_argument("No point in cloud");
  }
  
  PointT min, max;
  std::pair<PointT, PointT> res;
  min.x = points->at(0).x;
  min.y = points->at(0).y;
  max = min;
  
  for (const auto p: *points) {
    if (p.x < min.x) min.x = p.x;
    if (p.y < min.y) min.y = p.y;
    if (p.x > max.x) max.x = p.x;
    if (p.y > max.y) max.y = p.y;
  }

  res.first = min;
  res.second = max;
  return res;
}


void createGround(CloudT::Ptr ground) {
  std::pair<PointT, PointT> minmax = cloudMinmax(ground);
  PointT cmin, cmax;
  cmin = minmax.first;
  cmax = minmax.second;
  
  grid_map::GridMap map;
  map.setGeometry(grid_map::Length(cmax.x - cmin.x, cmax.y - cmin.y), 0.05);
  map.add("orig");
  map.add("median");

  PointT adjust;
  adjust.x = (cmax.x + cmin.x) / 2;
  adjust.y = (cmax.y + cmin.y) / 2;

  std::vector<std::vector<float> > cell_points;
  cell_points.resize(map.getSize().x() * map.getSize().y());

  for (const auto& p : *ground) {
    grid_map::Position pos(p.x - adjust.x, p.y - adjust.y);
    if (!map.isInside(pos)) continue;
    grid_map::Index index;
    map.getIndex(pos, index);
    size_t linear_index = grid_map::getLinearIndexFromIndex(index, map.getSize());
    cell_points.at(linear_index).push_back(p.z);
  }

  grid_map::Matrix& data = map["orig"];
  for (size_t i = 0; i < cell_points.size(); i++) {
    auto cell_points_ordered = cell_points.at(i);
    if (cell_points_ordered.empty()) continue;
    if (cell_points_ordered.size() == 1) {
      data(i) = cell_points_ordered.at(0);
      continue;
    }

    std::sort(cell_points_ordered.begin(), cell_points_ordered.end());
    data(i) = cell_points_ordered[cell_points_ordered.size() / 2];
  }

  grid_map::Matrix& med_data = map["median"];
  std::vector<float> non_nan;
  non_nan.reserve(7 * 7);

  for(grid_map::SlidingWindowIterator iterator(map, "orig",
                                               grid_map::SlidingWindowIterator::EdgeHandling::EMPTY,
                                               7);
      !iterator.isPastEnd(); ++iterator) {
    Eigen::MatrixXf region_of_interest = iterator.getData();

    if(region_of_interest.array().isNaN().all())
    {
      continue;
    }

    non_nan.clear();
    for(int i = 0; i < region_of_interest.size(); ++i)
    {
      const float value = region_of_interest(i);
      if(!std::isnan(value))
      {
        non_nan.push_back(value);
      }
    }
    if(non_nan.size() < (7 * 7) / 2)
    {
      continue;
    }

    const auto median_it_position = non_nan.begin() + (non_nan.size() / 2);
    std::nth_element(non_nan.begin(), median_it_position, non_nan.end());

    const size_t linear_index = iterator.getLinearIndex();
    med_data(linear_index) = *median_it_position;
  }
/*
  sensor_msgs::PointCloud2Ptr points = boost::make_shared<sensor_msgs::PointCloud2>();
  grid_map::GridMapRosConverter::toPointCloud(map, "median", *points);

  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*points, pcl_cloud);

  const int write_success = pcl::io::savePLYFile("median_ground.ply", pcl_cloud);
  if(write_success != 0)
  {
    std::cout << "Couldn't save\n"; 
    return;
  }
*/

  CloudT::Ptr marked_ground(new CloudT());
  for (const auto& p : *ground) {
    grid_map::Position pos(p.x - adjust.x, p.y - adjust.y);
    if (!map.isInside(pos)) continue;
    grid_map::Index index;
    map.getIndex(pos, index);
    size_t linear_index = grid_map::getLinearIndexFromIndex(index, map.getSize());
    if (fabs(p.z - med_data(linear_index)) < 0.03) marked_ground->points.push_back(p);
  }
  pcl::io::savePCDFileBinary("marked_ground.pcd", *marked_ground);
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "USAGE: make_ground filename\n";
    return 1;
  }

  CloudT::Ptr points(new CloudT());
  const bool read_success = pcl::io::loadPCDFile(std::string(argv[1]), *points) == 0;
  if(!read_success)
  {
    std::cerr << "PCL failed to load file: " << argv[1] << std::endl;
    return 1;
  }

  if (argc == 2) {
    createGround(points);
  }

  return 0;
}
