#include "../src/BasketEvidence.hh"
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  constexpr double observedRadius = .028031044266530716;
  const std::array<double, 3> penetrated{
    .00768355779502139, .20981804783442426, .17734143827867221};
  require(agri::BasketContainsSphere(penetrated, observedRadius),
    "the user-selected ideal region must accept the reported near-floor fruit");
  const std::array<double, 3> latestReported{
    .01317268162894214, .20930608759846336, .17749911657689932};
  require(agri::BasketContainsSphere(latestReported, observedRadius),
    "the latest reported near-floor fruit must also be in the ideal region");
  constexpr double edgeRadius = .026327331812614842;
  const std::array<double, 3> edgeReported{
    .24434856214129808, .23191498929366317, .11037605517234184};
  require(agri::BasketContainsSphere(edgeReported, edgeRadius),
    "the second fruit resting beside the real inner wall must be accepted");
  auto mirroredEdge = edgeReported; mirroredEdge[0] = -mirroredEdge[0];
  require(agri::BasketContainsSphere(mirroredEdge, edgeRadius),
    "the same physical deposit must be accepted beside either basket wall");
  for (double sign : {-1., 1.}) {
    auto edge = edgeReported;
    edge[0] = sign * (agri::kBasketInnerHalfWidthX - edgeRadius - 1e-6);
    require(agri::BasketContainsSphere(edge, edgeRadius),
      "a whole fruit inside the mesh wall must be accepted");
    edge[0] = sign * (agri::kBasketInnerHalfWidthX - edgeRadius + 1e-6);
    require(!agri::BasketContainsSphere(edge, edgeRadius),
      "a fruit extending past the mesh wall must still be rejected");
  }
  agri::BasketEvidence edgeEvidence;
  edgeEvidence.Observe(edgeReported, edgeRadius, 20., false);
  edgeEvidence.Observe(edgeReported, edgeRadius, 20.49, false);
  require(!edgeEvidence.Confirmed(),
    "the reported edge deposit still needs the full confirmation interval");
  edgeEvidence.Observe(edgeReported, edgeRadius, 20.5, false);
  require(edgeEvidence.Confirmed(),
    "the reported edge deposit must confirm after half a simulation second");
  auto escapedEdge = edgeReported;
  escapedEdge[0] = agri::kBasketInnerHalfWidthX + edgeRadius + 1e-6;
  edgeEvidence.Observe(escapedEdge, edgeRadius, 20.6, false);
  require(!edgeEvidence.Confirmed() && edgeEvidence.stableSeconds == 0,
    "losing all contact with the deposit region must revoke confirmation");
  auto supported = penetrated;
  supported[1] = .20559 + observedRadius;
  require(agri::BasketContainsSphere(supported, observedRadius),
    "a fruit supported by the solid floor must also be in the ideal region");
  require(std::abs(agri::kBasketRegionMinimumY - .09059) < 1e-12 &&
          std::abs(agri::kBasketRegionMaximumY - .32309) < 1e-12,
    "vertical bounds must span 10 cm below the basket to its half height");
  auto belowFloor = penetrated;
  belowFloor[1] = agri::kBasketRegionMinimumY + observedRadius + 1e-6;
  require(agri::BasketContainsSphere(belowFloor, observedRadius),
    "the entire sphere may be below the floor while within the ideal region");
  belowFloor[1] -= 2e-6;
  require(!agri::BasketContainsSphere(belowFloor, observedRadius),
    "a sphere below the new lower boundary must still be rejected");
  auto aboveRegion = penetrated;
  aboveRegion[1] = agri::kBasketRegionMaximumY - observedRadius - 1e-6;
  require(agri::BasketContainsSphere(aboveRegion, observedRadius),
    "a sphere below the half-height upper bound must be accepted");
  aboveRegion[1] += 2e-6;
  require(!agri::BasketContainsSphere(aboveRegion, observedRadius),
    "a sphere crossing the half-height upper boundary must be rejected");
  constexpr double radius = .02803;
  const std::array<double, 3> deposited{-.124, .23362, .2442};
  require(agri::BasketContainsSphere(deposited, radius),
    "measured deposited fruit must remain inside the ideal deposit region");
  agri::BasketEvidence evidence;
  evidence.Observe(deposited, radius, 10., false);
  require(!evidence.Confirmed(), "one position cannot establish continuous containment");
  evidence.Observe(deposited, radius, 10.49, false);
  require(!evidence.Confirmed(), "containment shorter than 0.5 simulation seconds is insufficient");
  auto rolling = deposited; rolling[0] += .04;
  evidence.Observe(rolling, radius, 10.5, false);
  require(evidence.Confirmed() && evidence.relativeSpeed > .03,
    "rolling inside the basket must not be rejected by a world-velocity gate");
  rolling[0] += .04;
  evidence.Observe(rolling, radius, 10.6, false);
  require(evidence.Confirmed(), "continued rolling cannot revoke deposit-region containment");
  auto outside = rolling;
  outside[0] = agri::kBasketInnerHalfWidthX + radius + 1e-6;
  evidence.Observe(outside, radius, 10.7, false);
  require(!evidence.Confirmed() && !evidence.inside && evidence.stableSeconds == 0,
    "a fruit actually leaving the basket must revoke confirmation immediately");
  evidence.Observe(deposited, radius, 10.8, false);
  require(!evidence.Confirmed(), "returning from outside requires a new containment interval");
  evidence.Observe(deposited, radius, 11.4, false);
  require(evidence.Confirmed(), "continuous re-entry must be confirmable");

  evidence.Observe(deposited, radius, 1., false);
  require(!evidence.Confirmed() && evidence.stableSeconds == 0,
    "a simulation clock rewind must reset containment dwell");
  evidence.Observe(deposited, radius, 1.2, false);
  evidence.Observe(deposited, radius, 2.2, true);
  require(!evidence.Confirmed() && std::abs(evidence.stableSeconds - .2) < 1e-9,
    "paused intervals cannot accumulate containment time");
  evidence.Observe(deposited, radius, 2.6, false);
  require(evidence.Confirmed(), "unpaused simulation time must resume the dwell");

  for (const auto &point : {std::array<double, 3>{.247, .23362, .17},
      std::array<double, 3>{0., .11, .17},
      std::array<double, 3>{0., .32, .17},
      std::array<double, 3>{0., .23362, .01},
      std::array<double, 3>{0., .23362, .33}})
    require(!agri::BasketContainsSphere(point, radius),
      "whole-sphere region boundaries must reject horizontal and vertical exits");
  require(!agri::BasketContainsSphere(deposited, -radius), "radius must be physical");

  const std::array<double, 3> lower{-agri::kBasketInnerHalfWidthX,
    agri::kBasketRegionMinimumY, agri::kBasketRegionMinimumZ};
  const std::array<double, 3> upper{agri::kBasketInnerHalfWidthX,
    agri::kBasketRegionMaximumY, agri::kBasketRegionMaximumZ};
  std::array<double, 3> middle;
  for (int axis = 0; axis < 3; ++axis)
    middle[axis] = (lower[axis] + upper[axis]) / 2;
  for (int axis = 0; axis < 3; ++axis) {
    for (double sign : {-1., 1.}) {
      auto faceContact = middle;
      const double bound = sign < 0 ? lower[axis] : upper[axis];
      faceContact[axis] = bound + sign * .03;
      // Measure from the stored coordinates to avoid rounding the exact tangent
      // to a side of the boundary in the test setup.
      const double tangentRadius = std::abs(faceContact[axis] - bound);
      require(agri::BasketIntersectsRegion(faceContact, tangentRadius),
        "a sphere tangent to any one of the six region faces must count as contact");
      require(!agri::BasketContainsSphere(faceContact, tangentRadius),
        "face contact need not place the whole fruit inside the region");
      auto entering = faceContact;
      entering[axis] -= sign * 1e-6;
      require(agri::BasketIntersectsRegion(entering, tangentRadius),
        "a sphere entering any region face must continue to count as contact");
      auto leaving = faceContact;
      leaving[axis] += sign * 1e-6;
      require(!agri::BasketIntersectsRegion(leaving, tangentRadius),
        "a sphere with a gap beside any region face must be rejected");
    }
  }
  for (double signX : {-1., 1.}) {
    for (double signY : {-1., 1.}) {
      auto cornerContact = middle;
      const double boundX = signX < 0 ? lower[0] : upper[0];
      const double boundY = signY < 0 ? lower[1] : upper[1];
      cornerContact[0] = boundX + signX * .6 * radius;
      cornerContact[1] = boundY + signY * .6 * radius;
      require(agri::BasketIntersectsRegion(cornerContact, radius),
        "a sphere overlapping a real two-face corner must count as contact");
      cornerContact[0] = boundX + signX * .8 * radius;
      cornerContact[1] = boundY + signY * .8 * radius;
      require(!agri::BasketIntersectsRegion(cornerContact, radius),
        "the expanded box alone must not accept a sphere missing a diagonal corner");
      for (double signZ : {-1., 1.}) {
        const double boundZ = signZ < 0 ? lower[2] : upper[2];
        cornerContact = {boundX + signX * .55 * radius,
          boundY + signY * .55 * radius, boundZ + signZ * .55 * radius};
        require(agri::BasketIntersectsRegion(cornerContact, radius),
          "a sphere overlapping a real three-face corner must count as contact");
        cornerContact = {boundX + signX * .6 * radius,
          boundY + signY * .6 * radius, boundZ + signZ * .6 * radius};
        require(!agri::BasketIntersectsRegion(cornerContact, radius),
          "a sphere separated from a three-face corner must not count as contact");
      }
    }
  }

  auto partial = middle;
  partial[0] = agri::kBasketInnerHalfWidthX + radius / 2;
  require(agri::BasketIntersectsRegion(partial, radius) &&
          !agri::BasketContainsSphere(partial, radius),
    "a fruit touching the region beside a wall need not be fully contained");
  agri::BasketEvidence contactEvidence;
  contactEvidence.Observe(partial, radius, 30., false);
  contactEvidence.Observe(partial, radius, 30.49, false);
  require(contactEvidence.inside && !contactEvidence.Confirmed(),
    "region contact shorter than 0.5 seconds must not confirm a partial deposit");
  contactEvidence.Observe(partial, radius, 30.5, false);
  require(contactEvidence.Confirmed(),
    "continuous partial region contact must confirm after 0.5 simulation seconds");
  contactEvidence.Observe(outside, radius, 30.6, false);
  require(!contactEvidence.inside && !contactEvidence.Confirmed() &&
          contactEvidence.stableSeconds == 0,
    "departing after a partial-contact confirmation must clear it immediately");
  contactEvidence.Observe(partial, radius, 30.7, false);
  contactEvidence.Observe(partial, radius, 31.1, false);
  require(!contactEvidence.Confirmed(),
    "a brief return must not retain time from the previous contact");
  contactEvidence.Observe(outside, radius, 31.2, false);
  require(!contactEvidence.Confirmed() && contactEvidence.stableSeconds == 0,
    "a fruit passing through the region for less than 0.5 seconds must not confirm");

  for (double invalidRadius : {0., -radius,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()}) {
    require(!agri::BasketIntersectsRegion(middle, invalidRadius),
      "nonpositive or nonfinite radii cannot establish region contact");
    agri::BasketEvidence invalidEvidence;
    invalidEvidence.Observe(middle, radius, 40., false);
    invalidEvidence.Observe(middle, radius, 40.5, false);
    require(invalidEvidence.Confirmed(), "invalid-data fixture must first be confirmed");
    invalidEvidence.Observe(middle, invalidRadius, 40.6, false);
    require(!invalidEvidence.inside && !invalidEvidence.Confirmed() &&
            invalidEvidence.stableSeconds == 0,
      "invalid radius feedback must reset an earlier region-contact confirmation");
  }
  for (int axis = 0; axis < 3; ++axis) {
    for (double value : {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()}) {
      auto invalidPoint = middle;
      invalidPoint[axis] = value;
      require(!agri::BasketIntersectsRegion(invalidPoint, radius),
        "a nonfinite coordinate on any axis cannot establish region contact");
      agri::BasketEvidence invalidEvidence;
      invalidEvidence.Observe(middle, radius, 50., false);
      invalidEvidence.Observe(middle, radius, 50.5, false);
      invalidEvidence.Observe(invalidPoint, radius, 50.6, false);
      require(!invalidEvidence.inside && !invalidEvidence.Confirmed() &&
              invalidEvidence.stableSeconds == 0,
        "nonfinite position feedback must reset an earlier region-contact confirmation");
    }
  }
  auto invalid = deposited; invalid[1] = std::numeric_limits<double>::quiet_NaN();
  evidence.Observe(invalid, radius, 2.7, false);
  require(!evidence.inside && !evidence.Confirmed(), "nonfinite position cannot prove containment");
  std::cout << "native basket containment, region contact, corners, rolling, exit, rewind and pause cases passed\n";
}
