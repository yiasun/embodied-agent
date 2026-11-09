#pragma once
#include <cmath>
#include <regex>
#include <string>

namespace embodied {
inline bool ValidMotionConfig(const std::string &token, double x, double low,
    double high, double speed, double duration, const std::string &pattern) {
  return std::regex_match(token,std::regex("[0-9a-f]{32}")) &&
      std::isfinite(x) && std::isfinite(low) && std::isfinite(high) &&
      std::isfinite(speed) && std::isfinite(duration) && x>=15.5 && x<=17.3 &&
      low==25.5 && high==26.5 && speed>=0.05 && speed<=0.3 && duration>=2 && duration<=30 &&
      (pattern=="cross" || pattern=="shuttle");
}
inline std::string StartupResult(bool config, bool entityName, bool link,
    bool robot, bool publication, bool subscription) {
  if (!config) return "INVALID_CONFIG";
  if (!entityName) return "ENTITY_NAME_MISMATCH";
  if (!link || !robot) return "MISSING_ENTITIES";
  if (!publication) return "STATE_PUBLICATION_FAILED";
  if (!subscription) return "CONTROL_SUBSCRIPTION_FAILED";
  return "READY";
}
}
