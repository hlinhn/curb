#include <curb/helper.h>
#include <pcl/filters/random_sample.h>
#include <yaml-cpp/yaml.h>

CurbAggregateParam
readParamFile(std::string file_name)
{
  CurbAggregateParam param;
  const auto yaml = YAML::LoadFile(file_name);
  param.factor = yaml["factor"].as<double>();
  param.method = yaml["method"].as<std::string>();

  param.normal_sample = yaml["normal"]["subsample"].as<double>();
  param.normal_ksearch = yaml["normal"]["ksearch"].as<int>();
  param.normal_z_upper = yaml["normal"]["normal_z_upper"].as<double>();
  param.normal_z_lower = yaml["normal"]["normal_z_lower"].as<double>();
  param.normal_cur_upper = yaml["normal"]["normal_cur_upper"].as<double>();
  param.normal_cur_lower = yaml["normal"]["normal_cur_lower"].as<double>();

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
