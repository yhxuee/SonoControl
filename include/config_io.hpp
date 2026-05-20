#pragma once

#include "config.hpp"

#include <filesystem>
#include <string>

namespace sonocontrol {

Config load_config_file(const std::filesystem::path& path);
void save_config_file(const std::filesystem::path& path, const Config& config, bool include_comments = true);
void write_config_template(const std::filesystem::path& path);
std::string config_to_text(const Config& config, bool include_comments = true);

} // namespace sonocontrol
