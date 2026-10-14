// Explicit simulation idealization of one loaded steering actuator. No fake
// feedback: Physics consumes PositionReset and publishes its actual readbacks.
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

namespace agri {
namespace ideal_c = gz::sim::components;
using IdealJson = nlohmann::json;

class IdealSteeringSystem : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
  // Allows an offline fake-ECM fixture to inject packets without any network or
  // Gazebo runtime. Production callers only have the stamped transport topic.
  friend struct IdealSteeringSystemTestAccess;
  using Clock = std::chrono::steady_clock;
  struct Packet {
    uint64_t sequence{0}; int64_t stamp{0}; double angle{0}; bool valid{false};
    Clock::time_point received{}; std::string reason;
  };
  // No transport infrastructure exists until the explicit enabled Configure.
  std::unique_ptr<gz::transport::Node> node;
  std::mutex packetMutex, statusMutex;
  Packet incoming, target;
  std::string statusText{"{\"idealized\":true,\"ready\":false,\"active\":false}"};
  std::string robotName{"agri_robot"}, commandTopic, statusService;
  const std::string jointName{"r2_steer_joint"};
  gz::sim::Entity world{gz::sim::kNullEntity}, robot{gz::sim::kNullEntity},
                  joint{gz::sim::kNullEntity};
  bool configured{false}, enabled{false}, active{false}, ready{false};
  bool idealSimulation{false};
  int64_t maxAgeNs{200000000}, lastSimNs{-1}, lastAcceptedStamp{-1};
  uint64_t processedSequence{0}, appliedCount{0}, rejectedCount{0};
  double maxReceiptAgeWall{0.5}, startupError{0.03};
  std::string reason{"DISABLED"};

  static int64_t Ns(std::chrono::steady_clock::duration duration)
  { return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count(); }
  static IdealJson Number(double value)
  { return std::isfinite(value) ? IdealJson(value) : IdealJson(nullptr); }
  void Revoke(const std::string &why) { active = false; reason = why; }
  void OnCommand(const gz::msgs::Double &message)
  {
    Packet next; next.angle = message.data(); next.received = Clock::now();
    if (idealSimulation) {
      // This explicit simulation mode holds the last finite target. A message
      // timestamp describes the packet but is not a permission to actuate.
      next.valid = std::isfinite(next.angle);
      if (!next.valid) next.reason = "NONFINITE_TARGET";
      if (message.has_header() && message.header().has_stamp()) {
        const auto &stamp = message.header().stamp();
        if (stamp.sec() >= 0 && stamp.nsec() >= 0 && stamp.nsec() < 1000000000 &&
            stamp.sec() <= (std::numeric_limits<int64_t>::max()-stamp.nsec())/1000000000LL)
          next.stamp = int64_t(stamp.sec())*1000000000LL + stamp.nsec();
      }
    }
    else if (!message.has_header() || !message.header().has_stamp())
      next.reason = "MISSING_SIM_STAMP";
    else {
      const auto &stamp = message.header().stamp();
      if (stamp.sec() < 0 || stamp.nsec() < 0 || stamp.nsec() >= 1000000000 ||
          stamp.sec() > (std::numeric_limits<int64_t>::max()-stamp.nsec())/1000000000LL)
        next.reason = "INVALID_SIM_STAMP";
      else {
        next.stamp = int64_t(stamp.sec())*1000000000LL + stamp.nsec();
        if (next.stamp <= 0) next.reason = "ZERO_SIM_STAMP";
        else if (!std::isfinite(next.angle)) next.reason = "NONFINITE_TARGET";
        else next.valid = true;
      }
    }
    std::lock_guard<std::mutex> lock(packetMutex);
    // Identical duplicate packets cannot renew a wall lease. Same stamp with a
    // different angle is passed through and rejected as a replay in PreUpdate.
    if (next.valid && incoming.valid && next.stamp == incoming.stamp &&
        next.angle == incoming.angle) return;
    next.sequence = incoming.sequence + 1; incoming = next;
  }
  bool Status(const gz::msgs::StringMsg &request, gz::msgs::StringMsg &response)
  {
    const auto input = IdealJson::parse(request.data(), nullptr, false);
    if (input.is_discarded() || !input.is_object() || !input.contains("op") ||
        !input["op"].is_string() || input["op"].get<std::string>() != "status") {
      response.set_data("{\"idealized\":true,\"success\":false,\"reason\":\"READ_ONLY_STATUS_REQUEST_REQUIRED\"}");
      return true;
    }
    std::lock_guard<std::mutex> lock(statusMutex);
    response.set_data(statusText); return true;
  }
  void UpdateStatus(const gz::sim::UpdateInfo &info,
                    const gz::sim::EntityComponentManager &ecm)
  {
    const auto *position = ecm.Component<ideal_c::JointPosition>(joint);
    const auto *axis = ecm.Component<ideal_c::JointAxis>(joint);
    const auto *velocityCommand = ecm.Component<ideal_c::JointVelocityCmd>(joint);
    const IdealJson readiness{{"model_found", robot != gz::sim::kNullEntity && ecm.HasEntity(robot)},
      {"joint_found", joint != gz::sim::kNullEntity && ecm.HasEntity(joint)},
      {"joint_position_available", position && position->Data().size() == 1 && std::isfinite(position->Data()[0])},
      {"joint_axis_available", axis && std::isfinite(axis->Data().Lower()) &&
        std::isfinite(axis->Data().Upper()) && axis->Data().Lower() < axis->Data().Upper()},
      {"joint_velocity_command_available", velocityCommand && velocityCommand->Data().size() == 1}};
    auto value = [&](auto tag) -> IdealJson {
      using Component = decltype(tag);
      const auto *component = ecm.Component<Component>(joint);
      return component && component->Data().size() == 1 ? Number(component->Data()[0]) : IdealJson(nullptr);
    };
    IdealJson state{{"success", true}, {"idealized", true},
      {"system", "agri::IdealSteeringSystem"}, {"version", "r2_ideal_steering_v1"},
      {"enabled", enabled}, {"ready", ready}, {"active", active},
      {"ready_components", readiness},
      {"ideal_simulation", idealSimulation},
      {"command_policy", idealSimulation ? "last_finite_target_held" : "fresh_stamped_target_lease"},
      {"joint_name", jointName}, {"robot_name", robotName},
      {"command_topic", commandTopic}, {"status_service", statusService},
      {"expiry_reason", reason}, {"sim_ns", Ns(info.simTime)},
      {"post_update_iteration", info.iterations}, {"paused", info.paused},
      {"actual_position_rad", value(ideal_c::JointPosition())},
      {"actual_velocity_rps", value(ideal_c::JointVelocity())},
      {"target_rad", target.valid ? Number(target.angle) : IdealJson(nullptr)},
      {"target_stamp_ns", target.valid ? IdealJson(target.stamp) : IdealJson(nullptr)},
      {"max_sim_age_ns", idealSimulation ? IdealJson(nullptr) : IdealJson(maxAgeNs)},
      {"max_receipt_age_wall_s", idealSimulation ? IdealJson(nullptr) : IdealJson(maxReceiptAgeWall)},
      {"startup_error_limit_rad", idealSimulation ? IdealJson(nullptr) : IdealJson(startupError)},
      {"applied_count", appliedCount},
      {"rejected_count", rejectedCount},
      {"feedback_source", "Existing JointPosition/JointVelocity after Physics UpdateSim; sampling time is not a separate sensor timestamp. No feedback component is written"},
      {"control_handoff", idealSimulation ?
        "Last finite in-bounds target resets only r2 position and overrides only its native force/velocity commands. No feedback, other joint, drive wheel, or base pose is changed" :
        "Fresh targets reset only r2 position and zero only its existing native velocity command. No effort, other joint, drive wheel, or base pose is changed. This is an idealized simulation actuator, not hardware validation"}};
    std::lock_guard<std::mutex> lock(statusMutex); statusText = state.dump();
  }

  void PreUpdateIdeal(const gz::sim::UpdateInfo &info,
                      gz::sim::EntityComponentManager &ecm)
  {
    const auto now = Ns(info.simTime);
    const bool movedBackwards = lastSimNs >= 0 && now < lastSimNs;
    lastSimNs = now;
    if (robot == gz::sim::kNullEntity || !ecm.HasEntity(robot))
      robot = ecm.EntityByComponents(ideal_c::Model(), ideal_c::Name(robotName), ideal_c::ParentEntity(world));
    if (joint == gz::sim::kNullEntity || !ecm.HasEntity(joint))
      joint = ecm.EntityByComponents(ideal_c::Joint(), ideal_c::Name(jointName), ideal_c::ParentEntity(robot));
    const auto *position = ecm.Component<ideal_c::JointPosition>(joint);
    const auto *axis = ecm.Component<ideal_c::JointAxis>(joint);
    ready = robot != gz::sim::kNullEntity && joint != gz::sim::kNullEntity &&
      position && position->Data().size() == 1 &&
      std::isfinite(position->Data()[0]) && axis && std::isfinite(axis->Data().Lower()) &&
      std::isfinite(axis->Data().Upper()) && axis->Data().Lower() < axis->Data().Upper();
    if (!ready) {
      active = false;
      if (robot == gz::sim::kNullEntity) reason = "ROBOT_MODEL_NOT_FOUND";
      else if (joint == gz::sim::kNullEntity) reason = "R2_STEER_JOINT_NOT_FOUND";
      else if (!position || position->Data().size() != 1 || !std::isfinite(position->Data()[0]))
        reason = "ACTUAL_JOINT_POSITION_UNAVAILABLE";
      else reason = "IMPORTED_JOINT_AXIS_OR_LIMITS_INVALID";
      return;  // Retain the target and resume if the real components return.
    }
    // Force-based controllers need not create a velocity command themselves.
    // Add only this r2 command component after identifying the real joint and
    // validating its Physics-owned readback and imported limits. This remains
    // command preparation while paused; no feedback or position is invented.
    auto *velocityCommand = ecm.Component<ideal_c::JointVelocityCmd>(joint);
    if (!velocityCommand)
      velocityCommand = ecm.CreateComponent(joint, ideal_c::JointVelocityCmd({0.0}));
    else if (velocityCommand->Data().empty())
      velocityCommand->Data() = {0.0};
    ready = velocityCommand && velocityCommand->Data().size() == 1;
    if (!ready) {
      active = false; reason = "R2_NATIVE_VELOCITY_COMMAND_INVALID";
      return;
    }
    if (info.paused || Ns(info.dt) <= 0 || movedBackwards) {
      active = false;
      reason = movedBackwards ? "CLOCK_BACKWARD_IDEAL_TARGET_RETAINED" : "PAUSED_IDEAL_TARGET_RETAINED";
      return;
    }
    Packet latest;
    { std::lock_guard<std::mutex> lock(packetMutex); latest = incoming; }
    if (latest.sequence != processedSequence) {
      processedSequence = latest.sequence;
      if (!latest.valid || !std::isfinite(latest.angle)) {
        ++rejectedCount;
        reason = latest.reason.empty() ? "NONFINITE_TARGET" : latest.reason;
      }
      else if (latest.angle < axis->Data().Lower() || latest.angle > axis->Data().Upper()) {
        ++rejectedCount; reason = "TARGET_OUTSIDE_IMPORTED_JOINT_LIMITS";
      }
      else {
        target = latest; lastAcceptedStamp = latest.stamp;
      }
    }
    if (!target.valid) {
      active = false;
      if (rejectedCount == 0) reason = "WAITING_FOR_FIRST_FINITE_TARGET";
      return;
    }
    if (!std::isfinite(target.angle) || target.angle < axis->Data().Lower() ||
        target.angle > axis->Data().Upper()) {
      active = false; reason = "HELD_TARGET_OUTSIDE_IMPORTED_JOINT_LIMITS";
      return;
    }
    // Override only this explicitly selected simulation joint. Position and
    // velocity feedback components remain Physics-owned and are never written.
    if (auto *force = ecm.Component<ideal_c::JointForceCmd>(joint))
      force->Data() = {0.0};
    if (auto *reset = ecm.Component<ideal_c::JointVelocityReset>(joint))
      reset->Data() = {0.0};
    if (auto *reset = ecm.Component<ideal_c::JointPositionReset>(joint))
      reset->Data() = {target.angle};
    else
      ecm.CreateComponent(joint, ideal_c::JointPositionReset({target.angle}));
    velocityCommand->Data()[0] = 0.0;
    active = true; ++appliedCount; reason = "ACTIVE_IDEAL_LAST_FINITE_TARGET";
  }

public:
  ~IdealSteeringSystem() override { node.reset(); }
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    if (configured) { gzerr << "IdealSteeringSystem: duplicate Configure refused.\n"; return; }
    configured = true;
    if (!sdf || !ecm.Component<ideal_c::World>(entity)) {
      reason = "WORLD_CONFIGURATION_REQUIRED"; return;
    }
    if (sdf->HasElement("joint_name") && sdf->Get<std::string>("joint_name") != jointName) {
      reason = "ONLY_R2_STEER_JOINT_ALLOWED"; return;
    }
    if (sdf->HasElement("enabled")) enabled = sdf->Get<bool>("enabled");
    if (!enabled) { reason = "EXPLICIT_ENABLED_TRUE_REQUIRED"; return; }
    const char *idealEnvironment = std::getenv("AGRI_IDEAL_SIMULATION");
    idealSimulation = sdf->HasElement("ideal_simulation") ? sdf->Get<bool>("ideal_simulation") :
      idealEnvironment && std::string(idealEnvironment) == "1";
    if (sdf->HasElement("robot_name")) robotName = sdf->Get<std::string>("robot_name");
    if (robotName.empty()) { enabled = false; reason = "INVALID_ROBOT_NAME"; return; }
    const auto *name = ecm.Component<ideal_c::Name>(entity);
    if (!name || name->Data().empty()) { enabled = false; reason = "WORLD_NAME_REQUIRED"; return; }
    const std::string prefix = "/world/" + name->Data() + "/ideal_steering";
    commandTopic = prefix + "/r2_command"; statusService = prefix + "/status";
    if (sdf->HasElement("command_topic")) commandTopic = sdf->Get<std::string>("command_topic");
    const double age = sdf->HasElement("max_sim_age_s") ? sdf->Get<double>("max_sim_age_s") : .2;
    if (sdf->HasElement("max_receipt_age_wall_s")) maxReceiptAgeWall = sdf->Get<double>("max_receipt_age_wall_s");
    if (sdf->HasElement("startup_error_limit_rad")) startupError = sdf->Get<double>("startup_error_limit_rad");
    if (commandTopic.empty() || (!idealSimulation && (
        !std::isfinite(age) || age < .02 || age > 1 ||
        !std::isfinite(maxReceiptAgeWall) || maxReceiptAgeWall < .05 || maxReceiptAgeWall > 2 ||
        !std::isfinite(startupError) || startupError <= 0 || startupError > .03))) {
      enabled = false; reason = "INVALID_LEASE_OR_STARTUP_BOUNDS"; return;
    }
    maxAgeNs = idealSimulation ? 0 : static_cast<int64_t>(age*1e9); world = entity;
    node = std::make_unique<gz::transport::Node>();
    if (!node->Advertise(statusService, &IdealSteeringSystem::Status, this) ||
        !node->Subscribe(commandTopic, &IdealSteeringSystem::OnCommand, this)) {
      enabled = false; reason = "TRANSPORT_CONFIGURATION_FAILED"; return;
    }
    reason = idealSimulation ? "WAITING_FOR_FIRST_FINITE_TARGET" : "WAITING_FOR_JOINT_AND_FRESH_TARGET";
    gzmsg << "IdealSteeringSystem enabled explicitly for " << robotName << "::" << jointName
          << "; simulation idealization, command " << commandTopic << ".\n";
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (!enabled) return;
    if (idealSimulation) { PreUpdateIdeal(info, ecm); return; }
    const auto now = Ns(info.simTime);
    if (lastSimNs >= 0 && now < lastSimNs) {
      Revoke("CLOCK_BACKWARD_REVOKED"); target = Packet(); lastAcceptedStamp = -1;
      std::lock_guard<std::mutex> lock(packetMutex); incoming = Packet(); processedSequence = 0;
      lastSimNs = now; return;
    }
    lastSimNs = now;
    if (info.paused || Ns(info.dt) <= 0) { Revoke("PAUSED"); return; }
    if (robot == gz::sim::kNullEntity || !ecm.HasEntity(robot))
      robot = ecm.EntityByComponents(ideal_c::Model(), ideal_c::Name(robotName), ideal_c::ParentEntity(world));
    if (joint == gz::sim::kNullEntity || !ecm.HasEntity(joint))
      joint = ecm.EntityByComponents(ideal_c::Joint(), ideal_c::Name(jointName), ideal_c::ParentEntity(robot));
    const auto *position = ecm.Component<ideal_c::JointPosition>(joint);
    const auto *axis = ecm.Component<ideal_c::JointAxis>(joint);
    auto *velocityCommand = ecm.Component<ideal_c::JointVelocityCmd>(joint);
    ready = joint != gz::sim::kNullEntity && position && position->Data().size() == 1 &&
      std::isfinite(position->Data()[0]) && axis && std::isfinite(axis->Data().Lower()) &&
      std::isfinite(axis->Data().Upper()) && axis->Data().Lower() < axis->Data().Upper() &&
      velocityCommand && velocityCommand->Data().size() == 1;
    if (!ready) { Revoke("JOINT_LIMITS_OR_NATIVE_COMMAND_UNAVAILABLE"); return; }
    // Physics prioritizes any force component and ignores velocity commands
    // while VelocityReset exists. Do not compete with another control mode.
    if (ecm.Component<ideal_c::JointForceCmd>(joint) ||
        ecm.Component<ideal_c::JointVelocityReset>(joint)) {
      Revoke("NATIVE_FORCE_OR_VELOCITY_RESET_CONFLICT"); return;
    }
    Packet latest;
    { std::lock_guard<std::mutex> lock(packetMutex); latest = incoming; }
    if (latest.sequence != processedSequence) {
      processedSequence = latest.sequence;
      std::string rejected;
      if (!latest.valid) rejected = latest.reason;
      else if (latest.stamp <= lastAcceptedStamp) rejected = "NONMONOTONIC_TARGET_STAMP";
      else if (latest.stamp > now) rejected = "FUTURE_SIM_STAMP";
      else if (now-latest.stamp > maxAgeNs) rejected = "STALE_SIM_TARGET";
      else if (latest.angle < axis->Data().Lower() || latest.angle > axis->Data().Upper())
        rejected = "TARGET_OUTSIDE_IMPORTED_JOINT_LIMITS";
      else if (!active && std::abs(latest.angle-position->Data()[0]) > startupError)
        rejected = "STARTUP_TARGET_ACTUAL_MISMATCH";
      if (!rejected.empty()) { ++rejectedCount; target = Packet(); Revoke(rejected); return; }
      target = latest; lastAcceptedStamp = latest.stamp; active = true;
    }
    if (!target.valid || !active) return;
    if (target.stamp > now || now-target.stamp > maxAgeNs) { Revoke("SIM_LEASE_EXPIRED"); return; }
    if (std::chrono::duration<double>(Clock::now()-target.received).count() > maxReceiptAgeWall) {
      Revoke("WALL_LEASE_EXPIRED"); return;
    }
    if (ecm.Component<ideal_c::JointPositionReset>(joint)) {
      Revoke("EXISTING_POSITION_RESET_CONFLICT"); return;
    }
    // PositionReset is a one-physics-step native request, removed by Physics.
    // Zero the existing r2 velocity command so the SERVO does not pull the reset
    // angle back in this same step. No VelocityReset: it suppresses the velocity
    // command branch and can leave the earlier SERVO target in the engine.
    ecm.CreateComponent(joint, ideal_c::JointPositionReset({target.angle}));
    velocityCommand->Data()[0] = 0.0;
    ++appliedCount; reason = "ACTIVE_SIMULATION_IDEALIZATION";
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override
  { UpdateStatus(info, ecm); }
};
}
GZ_ADD_PLUGIN(agri::IdealSteeringSystem, gz::sim::System,
             agri::IdealSteeringSystem::ISystemConfigure,
             agri::IdealSteeringSystem::ISystemPreUpdate,
             agri::IdealSteeringSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(agri::IdealSteeringSystem, "agri::IdealSteeringSystem")
