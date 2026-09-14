/// @file
/// 见 app/config.h。只认 `key: value` 与 `#` 行注释。

#include "app/config.h"

#include <cctype>
#include <fstream>
#include <unordered_map>

namespace servo {
namespace {

std::string Trim(std::string s) {
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) {
    s.erase(s.begin());
  }
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) {
    s.pop_back();
  }
  return s;
}

}  // namespace

bool LoadConfig(const std::string& path, ServoConfig* out) {
  if (out == nullptr) {
    return false;
  }
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  std::unordered_map<std::string, std::string> kv;
  std::string line;
  while (std::getline(in, line)) {
    line = Trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      return false;
    }
    const std::string key = Trim(line.substr(0, colon));
    const std::string value = Trim(line.substr(colon + 1));
    kv[key] = value;
  }
  try {
    out->pwmchip = kv.at("pwmchip");
    out->channel = std::stoi(kv.at("channel"));
    out->period_us = std::stoi(kv.at("period_us"));
    out->min_pulse_us = std::stoi(kv.at("min_pulse_us"));
    out->max_pulse_us = std::stoi(kv.at("max_pulse_us"));
  } catch (...) {
    return false;
  }
  return !out->pwmchip.empty();
}

}  // namespace servo
