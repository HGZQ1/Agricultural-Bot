#include "../src/GripEvidence.hh"
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}
agri::GripEvidence enclosed() {
  agri::GripEvidence e;
  e.centre = {-0.181066835, -0.004165375, 0.004101358};
  e.radius = 0.030318893;
  e.tcpDistance = 0.00594219;
  e.leftPosition = 0.011288627;
  e.rightPosition = 0.003085942;
  e.leftVelocity = e.rightVelocity = 0;
  e.stableSeconds = 0.2;
  e.sensorsAvailable = true;
  e.left = {{{e.centre[0], e.centre[1], e.centre[2] - e.radius}, 0.00000037}};
  e.right = {{{e.centre[0], e.centre[1], e.centre[2] + e.radius}, 0.00000040}};
  return e;
}
int main() {
  int count = 0;
  auto e = enclosed(); auto a = agri::AssessGrip(e);
  require(a.valid && a.settled && a.state == "CONTACT_CONFIRMED", "asymmetric actual two-jaw grip must pass"); ++count;
  e = enclosed(); e.left.clear(); e.right.clear(); a = agri::AssessGrip(e);
  require(a.valid && !a.settled && a.state == "NO_CONTACT", "closed positions alone must never prove contact"); ++count;
  e = enclosed(); e.left.clear(); a = agri::AssessGrip(e);
  require(a.valid && !a.settled && a.state == "RIGHT_CONTACT", "one jaw must not prove a grasp"); ++count;
  e = enclosed(); e.right.clear(); a = agri::AssessGrip(e);
  require(a.valid && !a.settled && a.state == "LEFT_CONTACT", "opposite single jaw must remain partial"); ++count;
  e = enclosed(); e.stableSeconds = 0.099; a = agri::AssessGrip(e);
  require(a.valid && !a.settled && a.state == "CONTACT_SETTLING", "transient contact must wait for physical stability"); ++count;
  e = enclosed(); e.rightVelocity = 0.003; a = agri::AssessGrip(e);
  require(a.valid && !a.settled && a.state == "CONTACT_SETTLING", "moving fingers cannot prove a grip"); ++count;
  e = enclosed(); e.left[0].depth = 0.00201; a = agri::AssessGrip(e);
  require(!a.valid && !a.settled, "deep penetration must fail closed"); ++count;
  e = enclosed(); e.right[0].position = e.left[0].position; a = agri::AssessGrip(e);
  require(!a.valid && !a.settled, "same-side contacts cannot prove enclosure"); ++count;
  e = enclosed(); e.unexpectedContact = true; a = agri::AssessGrip(e);
  require(!a.valid && !a.settled, "palm or foreign contacts must remain forbidden"); ++count;
  e = enclosed(); e.sensorsAvailable = false; a = agri::AssessGrip(e);
  require(!a.valid && !a.settled, "unavailable physical contact data must fail closed"); ++count;
  e = enclosed(); e.leftVelocity = std::numeric_limits<double>::quiet_NaN(); a = agri::AssessGrip(e);
  require(!a.valid && !a.settled, "nonfinite feedback must fail closed"); ++count;
  e = enclosed(); e.tcpDistance = 0.1; a = agri::AssessGrip(e);
  require(!a.valid && !a.settled, "contact outside TCP enclosure must fail closed"); ++count;
  e = enclosed(); e.right[0].position[1] += 0.02; a = agri::AssessGrip(e);
  require(!a.valid && !a.settled, "contact outside actual spherical surface must fail closed"); ++count;
  std::cout << count << " native grip evidence cases passed\n";
}
