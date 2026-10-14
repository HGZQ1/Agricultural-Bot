#pragma once
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace agri {
struct GripContactPoint {
  std::array<double, 3> position{};
  double depth{};
};
struct GripEvidence {
  std::array<double, 3> centre{};
  double radius{}, tcpDistance{}, leftPosition{}, rightPosition{};
  double leftVelocity{}, rightVelocity{}, stableSeconds{};
  bool sensorsAvailable{}, unexpectedContact{};
  std::vector<GripContactPoint> left, right;
};
struct GripAssessment {
  bool valid{}, left{}, right{}, settled{};
  std::string state{"CONTACT_REJECTED"}, reason;
};
inline GripAssessment AssessGrip(const GripEvidence &e) {
  GripAssessment result;
  const double values[] = {e.centre[0], e.centre[1], e.centre[2], e.radius,
    e.tcpDistance, e.leftPosition, e.rightPosition, e.leftVelocity, e.rightVelocity};
  for (double value : values)
    if (!std::isfinite(value)) { result.reason = "nonfinite measured grip evidence"; return result; }
  if (!e.sensorsAvailable) { result.reason = "actual collision contact data unavailable"; return result; }
  if (e.radius <= 0 || e.tcpDistance > e.radius + 0.005 ||
      e.leftPosition < -0.0001 || e.rightPosition < -0.0001 ||
      e.leftPosition > 0.0351 || e.rightPosition > 0.0351) {
    result.reason = "fruit is outside the actual TCP/finger enclosure"; return result;
  }
  if (e.unexpectedContact) {
    result.reason = "selected fruit/fingers touch the palm or a foreign collision"; return result;
  }
  auto inspect = [&](const std::vector<GripContactPoint> &points, int side, bool &contact) {
    for (const auto &point : points) {
      if (!std::isfinite(point.depth) || point.depth < -0.0001 || point.depth > 0.002) {
        result.reason = "contact penetration exceeds numerical 2 mm limit"; return false;
      }
      double distance2 = 0;
      for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(point.position[axis])) {
          result.reason = "nonfinite actual contact point"; return false;
        }
        const double delta = point.position[axis] - e.centre[axis];
        distance2 += delta * delta;
      }
      const double radial = std::sqrt(distance2);
      const double jawSide = side * (point.position[2] - e.centre[2]);
      if (std::abs(radial - e.radius) > 0.002 || jawSide < 0.5 * e.radius) {
        result.reason = "contact is outside the selected fruit surface/opposing jaw side"; return false;
      }
      contact = true;
    }
    return true;
  };
  // The canonical left finger closes from negative gripper Z, right from positive Z.
  if (!inspect(e.left, -1, result.left) || !inspect(e.right, 1, result.right)) return result;
  result.valid = true;
  if (result.left && result.right) {
    result.settled = std::abs(e.leftVelocity) <= 0.002 &&
      std::abs(e.rightVelocity) <= 0.002 && e.stableSeconds >= 0.1;
    result.state = result.settled ? "CONTACT_CONFIRMED" : "CONTACT_SETTLING";
  } else if (result.left) result.state = "LEFT_CONTACT";
  else if (result.right) result.state = "RIGHT_CONTACT";
  else result.state = "NO_CONTACT";
  result.reason = result.settled ? "both actual jaw/fruit contacts stable for 0.1 simulation seconds" :
    "actual contact query; attachment has not occurred";
  return result;
}
}  // namespace agri
