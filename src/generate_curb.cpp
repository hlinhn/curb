#include <grid_map_core/GridMap.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <string>
#include <vector>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <grid_map_core/GridMapMath.hpp>

#include <curb/generate_curb.h>

bool operator<(HeightInd& lhs, HeightInd& rhs) {
  return (lhs.height < rhs.height);
}

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

CloudT::Ptr cutCloud(CloudT::Ptr points, PointT cmin, PointT cmax) {
  CloudT::Ptr box(new CloudT());
  pcl::ConditionalRemoval<PointT> condrem;
  pcl::ConditionAnd<PointT>::Ptr cond(new pcl::ConditionAnd<PointT>());
  cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr
                      (new pcl::FieldComparison<PointT>("x",
                                                        pcl::ComparisonOps::LE,
                                                        cmax.x)));
  cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr
                      (new pcl::FieldComparison<PointT>("x",
                                                        pcl::ComparisonOps::GE,
                                                        cmin.x)));
  cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr
                      (new pcl::FieldComparison<PointT>("y",
                                                        pcl::ComparisonOps::LE,
                                                        cmax.y)));
  cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr
                      (new pcl::FieldComparison<PointT>("y",
                                                        pcl::ComparisonOps::GE,
                                                        cmin.y)));

  condrem.setCondition(cond);
  condrem.setInputCloud(points);
  condrem.setKeepOrganized(true);
  condrem.filter(*box);

  return box;
}

std::vector<int> slidingWindow(std::vector<HeightInd> ordered,
                               double cluster_res, double cluster_dist, float& fin_height) {

  std::vector<HeightInd> cluster_count;
  cluster_count.reserve(ordered.size());

  float last_height = ordered.front().height;
  cluster_count.push_back(HeightInd(last_height, 0));

  for (auto& h : ordered) {
    float dist = h.height - last_height;
    if (dist < cluster_res) continue;
    cluster_count.push_back(HeightInd(h.height, 0));
    last_height = h.height;
  }

  size_t last_start = 0;
  size_t largest_count = 0;
  size_t largest_ind = 0;

  for (auto& h : ordered) {
    bool first_point = true;
    for (size_t i = last_start; i < cluster_count.size(); i++) {
      auto& cpair = cluster_count.at(i);
      float dist = cpair.height - h.height;

      if (std::fabs(dist) <= cluster_dist) {
          if (first_point) {
              last_start = i;
              first_point = false;
          }

          cpair.ind++;

          if (cpair.ind > largest_count) {
              largest_count = cpair.ind;
              largest_ind = i;
          }
      }

      else if (dist > cluster_dist) {
          break;
      }
    }
  }

  float best_height = cluster_count.at(largest_ind).height;
  fin_height = best_height;
  std::vector<int> chosen;
  for (auto& h : ordered) {
    if (std::fabs(h.height - best_height) <= cluster_dist)
      chosen.push_back(h.ind);
  }

  return chosen;
}

CloudT::Ptr getGround(CloudT::Ptr points, PointT cmin, PointT cmax,
                      float resolution = 0.1, float max_height = 1.,
                      float cluster_resolution = 0.005,
                      float cluster_distance = 0.1) {
  grid_map::GridMap map;
  map.setGeometry(grid_map::Length(cmax.x - cmin.x, cmax.y - cmin.y), resolution);
  map.add("lowest");

  std::vector<std::vector<HeightInd> > cell_points;
  cell_points.resize(map.getSize().x() * map.getSize().y());
  std::vector<int> chosen;

  PointT adjust;
  adjust.x = (cmax.x + cmin.x) / 2;
  adjust.y = (cmax.y + cmin.y) / 2;
  for (const auto& p : *points) {
    grid_map::Position pos(p.x - adjust.x, p.y - adjust.y);
    if (!map.isInside(pos)) continue;
    float val = map.atPosition("lowest", pos);
    if (std::isnan(val)) {
      map.atPosition("lowest", pos) = p.z;
      continue;
    }

    if (p.z < val) {
      map.atPosition("lowest", pos) = p.z;
    }
  }

  for (int i = 0; i < points->size(); i++) {
    PointT p = points->at(i);
    grid_map::Position pos(p.x - adjust.x, p.y - adjust.y);
    if (!map.isInside(pos)) continue;
    float val = map.atPosition("lowest", pos);
    assert(!std::isnan(val));
    if (p.z >= val + max_height || p.z >= 1.5) continue;
    grid_map::Index index;
    map.getIndex(pos, index);
    size_t linear_index = grid_map::getLinearIndexFromIndex(index, map.getSize());
    cell_points.at(linear_index).push_back(HeightInd(p.z, i));
  }

  for (size_t i = 0; i < cell_points.size(); i++) {
    auto& ordered = cell_points.at(i);
    if (ordered.empty()) continue;
    if (ordered.size() == 1) {
      chosen.push_back(ordered.at(0).ind);
      continue;
    }
    std::sort(ordered.begin(), ordered.end());
    float fin_height;
    std::vector<int> populated = slidingWindow(ordered, cluster_resolution,
                                               cluster_distance, fin_height);
    chosen.insert(chosen.end(), populated.begin(), populated.end());
  }

  CloudT::Ptr ground(new CloudT());

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(points);
  pcl::IndicesPtr chosen_ptr(new std::vector<int>());
  *chosen_ptr = chosen;
  extract.setIndices(chosen_ptr);
  extract.filter(*ground);

  return ground;
}

CloudT::Ptr getNonground(CloudT::Ptr points, CloudT::Ptr ground,
                         PointT cmin, PointT cmax,
                         float resolution = 0.1)
{
  grid_map::GridMap map;
  map.setGeometry(grid_map::Length(cmax.x - cmin.x, cmax.y - cmin.y), resolution);
  map.add("highest");
  map.add("erase");

  PointT adjust;
  adjust.x = (cmax.x + cmin.x) / 2;
  adjust.y = (cmax.y + cmin.y) / 2;
  for (const auto& p : *ground) {
    grid_map::Position pos(p.x - adjust.x, p.y - adjust.y);
    if (!map.isInside(pos)) continue;
    float val = map.atPosition("highest", pos);
    if (std::isnan(val)) {
      map.atPosition("highest", pos) = p.z;
      continue;
    }

    if (p.z > val) {
      map.atPosition("highest", pos) = p.z;
    }
  }

  for (const auto& p : *points) {
    grid_map::Position pos(p.x - adjust.x, p.y - adjust.y);
    if (!map.isInside(pos)) continue;
    float val = map.atPosition("highest", pos);
    if (std::isnan(val)) continue;
    if (p.z > val + 0.5 && p.z < val + 1) {
      float curval = map.atPosition("erase", pos);
      if (std::isnan(curval))
          map.atPosition("erase", pos) = 1;
      else
          map.atPosition("erase", pos) = curval + 1;
    }
  }

  std::vector<int> not_chosen;
  for (int i = 0; i < points->size(); i++) {
    PointT p = points->at(i);
    grid_map::Position pos(p.x - adjust.x, p.y - adjust.y);
    if (!map.isInside(pos)) continue;
    float val = map.atPosition("highest", pos);
    if (std::isnan(val)) continue;
    float inter = map.atPosition("erase", pos);
    if (!std::isnan(inter) && inter > 4) continue;
    if (p.z <= val + 1) not_chosen.push_back(i);
  }

  CloudT::Ptr nonground(new CloudT());

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(points);
  pcl::IndicesPtr chosen_ptr(new std::vector<int>());
  *chosen_ptr = not_chosen;
  extract.setIndices(chosen_ptr);
  extract.setNegative(true);
  extract.filter(*nonground);

  return nonground;
}

void markGround(CloudT::Ptr points, float size = 200.0) {
  std::pair<PointT, PointT> minmax = cloudMinmax(points);
  PointT min, max;
  min = minmax.first;
  max = minmax.second;

  float sizex = max.x - min.x;
  float sizey = max.y - min.y;

  int numx = std::floor(sizex / size);
  int numy = std::floor(sizey / size);

  if (numx == 0) numx = 1;
  if (numy == 0) numy = 1;

  CloudT::Ptr all_ground(new CloudT());
  CloudT::Ptr all_nonground(new CloudT());

  std::cout << numx << " " << numy << std::endl;
  for (int i = 0; i < numx; i++) {
    for (int j = 0; j < numy; j++) {
      PointT cmax, cmin;
      cmin.x = min.x + i * size;
      cmin.y = min.y + j * size;
      cmax.x = cmin.x + size;
      cmax.y = cmin.y + size;
      if (i == numx - 1) cmax.x = max.x;
      if (j == numy - 1) cmax.y = max.y;
      CloudT::Ptr cut_cloud = cutCloud(points, cmin, cmax);
      if (cut_cloud->size() < 10) continue;

      CloudT::Ptr current = getGround(cut_cloud, cmin, cmax);
      CloudT::Ptr current_nonground = getNonground(cut_cloud, current, cmin, cmax);
      *all_ground += *current;
      *all_nonground += *current_nonground;
    }
  }

  pcl::io::savePCDFile("ground.pcd", *all_ground);
  pcl::io::savePCDFile("nonground.pcd", *all_nonground);

}

// cut to pieces
// find height and mark ground for each piece
int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "USAGE: generate_curb filename\n";
    return 1;
  }

  CloudT::Ptr points(new CloudT());
  const bool read_success = pcl::io::loadPLYFile(std::string(argv[1]), *points) == 0;
  if(!read_success)
  {
    std::cerr << "PCL failed to load .ply file: " << argv[1] << std::endl;
    return 1;
  }

  if (argc == 2) {
    markGround(points);
  }

  return 0;
}
