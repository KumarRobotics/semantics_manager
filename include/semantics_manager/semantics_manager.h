#pragma once

namespace semantics_manager {

static std::string getMapPath(const std::string& config_path);
static std::string getClassesPath(const std::string& config_path);

struct ClassConfig {
  int num_classes;
  SemanticColorLut color_lut;
  Eigen::VectorXi class_to_flattened;
  Eigen::VectorXi flattened_to_class;
  Eigen::VectorXi exclusivity;
  Eigen::VectorXi traversabililty_diff;
  Eigen::VectorXf loc_weight;

  ClassConfig(const std::string& class_config_path);
}

struct MapConfig {
  std::string svg_path;
  std::string raster_path;
  std::string viz_path;
  float resolution;

  MapConfig(const std::string& map_config_path);
}

} // namespace semantics_manager
