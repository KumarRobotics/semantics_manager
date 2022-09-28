#include "semantics_manager/semantics_manager.h"

#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace semantics_manager {

std::string getMapPath(const std::string& config_path) {
  using namespace std::filesystem;
  const YAML::Node config_file = YAML::LoadFile(config_path);
  return path(config_path).parent_path() / 
         path(config_file["map"].as<std::string>());
}

std::string getClassesPath(const std::string& config_path) {
  using namespace std::filesystem;
  const YAML::Node config_file = YAML::LoadFile(config_path);
  return path(config_path).parent_path() / 
         path(config_file["classes"].as<std::string>());
}

ClassConfig::ClassConfig(const std::string& class_config_path) {
  const YAML::Node class_config_file = YAML::LoadFile(class_config_path);
  num_classes = class_config_file.size();

  // Initialize things
  exclusivity.resize(num_classes);
  traversabililty_diff.resize(num_classes);
  loc_weight.resize(num_classes);

  // First pass to build class lookup dict
  std::map<std::string, int> class_name_map;
  int flattened_class_ind = 0;
  for (const auto& map_class : class_config_file) {
    // Classes not remapped are the ones we are going to end up with
    if (!map_class["remap"]) {
      class_name_map.emplace(map_class["name"].as<std::string>(), flattened_class_ind);
      ++flattened_class_ind;
    }
  }

  class_to_flattened.resize(num_classes);
  flattened_to_class.resize(flattened_class_ind);

  int map_class_ind = 0;
  flattened_class_ind = 0;
  for (const auto& map_class : class_config_file) {
    exclusivity[map_class_ind] = map_class["exclusive"].as<bool>();
    traversabililty_diff[map_class_ind] = map_class["traversabililty_diff"].as<int>();    
    loc_weight[map_class_ind] = map_class["loc_weight"].as<float>();

    // Manage loading the remappings
    if (map_class["remap"]) {
      // remap stuff
      auto remap_class = class_name_map.find(map_class["remap"].as<std::string>());
      if (remap_class != class_name_map.end()) {
        class_to_flattened[map_class_ind] = remap_class->second;
      }
    } else {
      class_to_flattened[map_class_ind] = flattened_class_ind;
      flattened_to_class[flattened_class_ind] = map_class_ind;
      ++flattened_class_ind;
    }
    ++map_class_ind;
  }

  color_lut = SemanticColorLut(class_config_path);
}

MapConfig::MapConfig(const std::string& map_config_path) {
  const YAML::Node map_config_file = YAML::LoadFile(map_config_path);
  using namespace std::filesystem;
  auto parent_path = path(map_config_path).parent_path();

  resolution = map_config_file["resolution"].as<float>();
  dynamic = map_config_file["dynamic"].as<bool>();
  if (!dynamic) {
    if (map_config_file["svg"]) {
      svg_path = parent_path / path(map_config_file["svg"].as<std::string>());
    }
    if (map_config_file["raster"]) {
      raster_path = parent_path / path(map_config_file["raster"].as<std::string>());
    }
    if (map_config_file["viz"]) {
      viz_path = parent_path / path(map_config_file["viz"].as<std::string>());
    }
  }
}

} // namespace semantics_manager
