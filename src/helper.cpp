#include <curb/helper.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/random_sample.h>
#include <sys/types.h>
#include <yaml-cpp/yaml.h>

CurbAggregateParam
readParamFile(std::string file_name)
{
  CurbAggregateParam param;
  const auto yaml = YAML::LoadFile(file_name);
  param.factor = yaml["factor"].as<double>();
  param.method = yaml["method"].as<std::string>();

  param.min_z = yaml["min_z"].as<double>();
  param.max_z = yaml["max_z"].as<double>();

  param.normal_sample = yaml["normal"]["subsample"].as<double>();
  param.normal_ksearch = yaml["normal"]["ksearch"].as<int>();
  param.normal_z_upper = yaml["normal"]["normal_z_upper"].as<double>();
  param.normal_z_lower = yaml["normal"]["normal_z_lower"].as<double>();
  param.normal_cur_upper = yaml["normal"]["normal_cur_upper"].as<double>();
  param.normal_cur_lower = yaml["normal"]["normal_cur_lower"].as<double>();

  param.image_half_size = yaml["image"]["half_size"].as<double>();
  param.image_resolution = yaml["image"]["resolution"].as<double>();
  param.image_min_num = yaml["image"]["min_num"].as<int>();
  param.image_max = yaml["image"]["max"].as<double>();
  param.image_min = yaml["image"]["min"].as<double>();
  param.image_threshold = yaml["image"]["threshold"].as<double>();
  return param;
}

Cloud::Ptr
subsample(Cloud::Ptr input, double ratio)
{
  Cloud::Ptr subsampled(new Cloud);
  pcl::RandomSample<Point> extract;
  extract.setInputCloud(input);
  extract.setSample(int(input->size() * ratio));
  extract.filter(*subsampled);
  return subsampled;
}

Cloud::Ptr
removeHighZ(Cloud::Ptr cloud, double min, double max)
{
  Cloud::Ptr filtered(new Cloud);
  pcl::ConditionalRemoval<Point> condrem;
  pcl::ConditionAnd<Point>::Ptr cond(new pcl::ConditionAnd<Point>());
  cond->addComparison(
      pcl::FieldComparison<Point>::ConstPtr(new pcl::FieldComparison<Point>("z", pcl::ComparisonOps::GT, min)));
  cond->addComparison(
      pcl::FieldComparison<Point>::ConstPtr(new pcl::FieldComparison<Point>("z", pcl::ComparisonOps::LT, max)));

  condrem.setCondition(cond);
  condrem.setInputCloud(cloud);
  condrem.setKeepOrganized(true);
  condrem.filter(*filtered);
  return filtered;
}
