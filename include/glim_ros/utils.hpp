#pragma once

#include <string>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace glim_ros {

static std::string create_timestamped_dir(const std::string& base_path) {

    // Remove trailing slash if present
    std::string clean_base = base_path;
    if (!clean_base.empty() && clean_base.back() == '/') {
        clean_base.pop_back();
    }

    // Get current time
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm;
    localtime_r(&now_c, &tm);
    std::ostringstream oss;
    oss << clean_base << "/glim_dump_"
        << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S");

    return oss.str();
}

} // namespace glim_ros


