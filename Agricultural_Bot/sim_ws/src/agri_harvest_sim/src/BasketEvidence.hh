#pragma once
#include <array>
#include <cmath>
#include <limits>

namespace agri {
constexpr double kBasketInnerHalfWidthX = 0.275;
constexpr double kBasketFloorMinimumY = 0.2005;
constexpr double kBasketMeshBottomY = 0.19059;
constexpr double kBasketMeshTopY = 0.45559;
constexpr double kBasketRegionMinimumY = kBasketMeshBottomY - 0.10;
constexpr double kBasketRegionMaximumY = (kBasketMeshBottomY + kBasketMeshTopY) / 2;
constexpr double kBasketRegionMinimumZ = -0.008;
constexpr double kBasketRegionMaximumZ = 0.351;

// CAD axes: X lateral, Y up (robot Z), Z forward. The user-selected ideal
// deposit region spans 10 cm below the basket through its lower half.
// Position in this region does not prove contact with the physical floor.
// The mesh's lateral inner walls lie at CAD X = +/-0.275 m. Check the
// whole sphere against those walls in the basket's own moving frame.
inline bool BasketContainsSphere(const std::array<double, 3> &p, double radius) {
  if (!std::isfinite(radius) || radius <= 0) return false;
  for (double value : p) if (!std::isfinite(value)) return false;
  return std::abs(p[0]) + radius <= kBasketInnerHalfWidthX &&
    p[1] - radius >= kBasketRegionMinimumY && p[1] + radius < kBasketRegionMaximumY &&
    p[2] - radius > kBasketRegionMinimumZ && p[2] + radius < kBasketRegionMaximumZ;
}

// Contact with any face, edge or corner of the same ideal deposit volume
// qualifies, even when the complete sphere is not contained by that volume.
inline bool BasketIntersectsRegion(const std::array<double, 3> &p, double radius) {
  if (!std::isfinite(radius) || radius <= 0) return false;
  for (double value : p) if (!std::isfinite(value)) return false;
  const std::array<double, 3> minimum{
    -kBasketInnerHalfWidthX, kBasketRegionMinimumY, kBasketRegionMinimumZ};
  const std::array<double, 3> maximum{
    kBasketInnerHalfWidthX, kBasketRegionMaximumY, kBasketRegionMaximumZ};
  std::array<double, 3> distance{};
  for (int axis = 0; axis < 3; ++axis) {
    if (p[axis] < minimum[axis]) distance[axis] = minimum[axis] - p[axis];
    else if (p[axis] > maximum[axis]) distance[axis] = p[axis] - maximum[axis];
  }
  // Include tangency despite floating-point subtraction at region faces.
  return std::hypot(distance[0], distance[1], distance[2]) <= radius + 1e-12;
}

struct BasketEvidence {
  std::array<double, 3> position{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  double relativeSpeed{std::numeric_limits<double>::quiet_NaN()};
  double stableSeconds{0}, previousTime{-1};
  bool inside{false};

  bool Confirmed() const { return inside && stableSeconds >= 0.5; }

  void Observe(const std::array<double, 3> &current, double radius,
               double simTime, bool paused) {
    const bool nextInside = BasketIntersectsRegion(current, radius);
    const bool rewind = previousTime >= 0 && simTime < previousTime;
    const double dt = previousTime >= 0 && !rewind ? simTime - previousTime : 0;
    if (rewind) relativeSpeed = std::numeric_limits<double>::quiet_NaN();
    if (!paused && dt > 0) {
      double distance2 = 0;
      for (int axis = 0; axis < 3; ++axis) {
        const double delta = current[axis] - position[axis];
        distance2 += delta * delta;
      }
      relativeSpeed = std::sqrt(distance2) / dt;
    }
    // Require continuous contact with the requested ideal deposit region;
    // rolling or touching its boundary does not interrupt the dwell.
    if (!nextInside || !inside || rewind) stableSeconds = 0;
    else if (!paused && dt > 0) stableSeconds += dt;
    position = current; inside = nextInside; previousTime = simTime;
  }
};
}  // namespace agri
