#pragma once

#include "semantics_manager/semantic_color_lut.h"
#include <Eigen/Dense>

namespace semantics_manager {

std::string getMapPath(const std::string& config_path);
std::string getClassesPath(const std::string& config_path);

struct ClassConfig {
  int num_classes;
  SemanticColorLut color_lut;
  std::vector<int> class_to_flattened;
  std::vector<int> flattened_to_class;
  std::vector<int> exclusivity;
  std::vector<int> traversabililty_diff;
  std::vector<float> loc_weight;

  ClassConfig(const std::string& class_config_path);
};

struct MapConfig {
  bool dynamic;
  std::string svg_path{""};
  std::string raster_path{""};
  std::string color_path{""};
  std::string viz_path{""};
  float resolution;
  bool have_fixed_origin;
  Eigen::Vector2d fixed_origin_gps; // lat, long 

  MapConfig(const std::string& map_config_path);
};

} // namespace semantics_manager
