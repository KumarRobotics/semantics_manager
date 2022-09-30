#include "semantics_manager/semantics_manager.h"

#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace semantics_manager {

std::string getMapPath(const std::string& config_path) {
  using namespace std::filesystem;
  try {
    const YAML::Node config_file = YAML::LoadFile(config_path);
    return path(config_path).parent_path() / 
           path(config_file["map"].as<std::string>());
  } catch (const YAML::BadFile& ex) {
    throw std::invalid_argument("Could not find world config");
  } catch (const YAML::Exception& ex) {
    throw std::invalid_argument("Failed to load map config path from world config");
  }
}

std::string getClassesPath(const std::string& config_path) {
  using namespace std::filesystem;
  try {
    const YAML::Node config_file = YAML::LoadFile(config_path);
    return path(config_path).parent_path() / 
           path(config_file["classes"].as<std::string>());
  } catch (const YAML::BadFile& ex) {
    throw std::invalid_argument("Could not find world config");
  } catch (const YAML::Exception& ex) {
    throw std::invalid_argument("Failed to load class config path from world config");
  }
}

ClassConfig::ClassConfig(const std::string& class_config_path) {
  try {
    const YAML::Node class_config_file = YAML::LoadFile(class_config_path);
    num_classes = class_config_file.size();

    // Initialize things
    exclusivity.resize(num_classes);
    traversabililty_diff.resize(num_classes);
    loc_weight.resize(num_classes);

    class_to_flattened.resize(num_classes);
    flattened_to_class.resize(num_classes);
    // Init to all -1
    for (auto& cls : flattened_to_class) {
      cls = -1;
    }

    // First pass to build class lookup dict
    int num_flattened_classes = 0;
    int map_class_ind = 0;
    for (const auto& map_class : class_config_file) {
      exclusivity[map_class_ind] = map_class["exclusive"].as<bool>();
      traversabililty_diff[map_class_ind] = map_class["traversability_diff"].as<int>();    
      if (map_class["loc_weight"]) {
        loc_weight[map_class_ind] = map_class["loc_weight"].as<float>();
      } else {
        loc_weight[map_class_ind] = 0;
      }

      int flattened_class = map_class["localization_class"].as<int>();
      class_to_flattened[map_class_ind] = flattened_class;
      if (flattened_class >= 0 && flattened_to_class[flattened_class] < 0) {
        // Only allow setting once
        flattened_to_class[flattened_class] = map_class_ind;
      }
      if (flattened_class >= num_flattened_classes) {
        num_flattened_classes = flattened_class + 1;
      }
      ++map_class_ind;
    }
    flattened_to_class.resize(num_flattened_classes);

    color_lut = SemanticColorLut(class_config_path);
  } catch (const YAML::BadFile& ex) {
    throw std::invalid_argument("Could not find class config");
  } catch (const YAML::Exception& ex) {
    throw std::invalid_argument("Class config formatted incorrectly");
  }
}

MapConfig::MapConfig(const std::string& map_config_path) {
  try {
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
      } else if (raster_path != "") {
        viz_path = raster_path;
      }
    }
  } catch (const YAML::BadFile& ex) {
    throw std::invalid_argument("Could not find map config");
  } catch (const YAML::Exception& ex) {
    throw std::invalid_argument("Map config formatted incorrectly");
  }
}

} // namespace semantics_manager
