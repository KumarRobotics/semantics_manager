#include "semantics_manager/semantics_manager.h"

#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace semantics_manager {

std::string SemanticsManager::getMapPath(const std::string& config_path) {
  using namespace std::filesystem;
  const YAML::Node config_file = YAML::LoadFile(world_config_path);
  return (path(world_config_path).parent_path() / 
          path(config_file["map"].as<std::string>())).string();
}

std::string SemanticsManager::getClassesPath(const std::string& config_path) {
  using namespace std::filesystem;
  const YAML::Node config_file = YAML::LoadFile(world_config_path);
  return (path(world_config_path).parent_path() / 
          path(config_file["classes"].as<std::string>())).string();
}

ClassConfig::ClassConfig(const std::string& class_config_path) {
  const YAML::Node class_config_file = YAML::LoadFile(class_config_path);
  num_classes = class_config_file.

  // Initialize things
  flatten_lut_ = Eigen::VectorXi::Zero(256);

  // First pass to build class lookup dict
  int map_class_ind = 0;
  std::map<std::string, int> class_name_map;
  for (const auto& map_class : class_config_file) {
    // Classes not remapped are the ones we are going to end up with
    if (!map_class["remap"]) {
      class_name_map.emplace(map_class["name"].as<std::string>(), map_class_ind);
      if (map_class["exclusive"].as<bool>()) {
        map_params.exclusive_classes.push_back(map_class_ind);
      }
      ++map_class_ind;
    }
  }
  map_params.num_classes = map_class_ind;

  map_class_ind = 0;
  for (const auto& map_class : class_config_file) {
    auto color = map_class["color"];
    if (map_class["remap"]) {
      // remap stuff
      auto remap_class = class_name_map.find(map_class["remap"].as<std::string>());
      if (remap_class != class_name_map.end()) {
        flatten_lut_[map_class_ind] = remap_class->second + 1;
      }
    } else {
      flatten_lut_[map_class_ind] = map_class_ind + 1;
    }
    ++map_class_ind;
  }

  map_params.color_lut = SemanticColorLut(map_params.class_path);
  nh_.param<float>("out_of_bounds_const", map_params.out_of_bounds_const, 5);
}

MapConfig::MapConfig(const std::string& map_config_path) {
}

} // namespace semantics_manager
