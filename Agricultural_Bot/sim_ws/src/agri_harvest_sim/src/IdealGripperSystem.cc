// Explicit simulation-only position idealization for the two gripper fingers.
// Native resets change the physics joints; position/velocity feedback is read only.
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
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
#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <nlohmann/json.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace agri {
namespace gripper_c = gz::sim::components;
using GripperJson = nlohmann::json;

class IdealGripperSystem : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemConfigurePriority,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate {
  struct Commands {
    std::array<double, 2> target{};
    bool valid{false};
    uint64_t received{0}, rejected{0};
  };
  std::unique_ptr<gz::transport::Node> node;
  std::mutex mutex;
  Commands incoming;
  std::string statusText{"{\"success\":true,\"ready\":false}"};
  const std::array<std::string, 2> names{{"gripper_left_finger_joint",
                                          "gripper_right_finger_joint"}};
  gz::sim::Entity world{gz::sim::kNullEntity}, robot{gz::sim::kNullEntity};
  std::array<gz::sim::Entity, 2> joints{{gz::sim::kNullEntity, gz::sim::kNullEntity}};
  bool configured{false}, enabled{false}, ideal{false}, ready{false}, active{false};
  uint64_t applied{0};
  std::string robotName{"agri_robot"}, commandTopic, statusService, reason{"DISABLED"};

  static int64_t Ns(std::chrono::steady_clock::duration duration) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
  }
  Commands commandSnapshot() {
    std::lock_guard<std::mutex> lock(mutex); return incoming;
  }
  void onCommand(const gz::msgs::Double_V &message) {
    std::array<double, 2> next{};
    bool valid = message.data_size() == 2;
    if (valid) for (size_t i = 0; i < next.size(); ++i) {
      next[i] = message.data(static_cast<int>(i));
      valid = valid && std::isfinite(next[i]);
    }
    std::lock_guard<std::mutex> lock(mutex);
    ++incoming.received;
    if (valid) { incoming.target = next; incoming.valid = true; }
    else ++incoming.rejected;  // Malformed input never replaces the last valid pair.
  }
  void findJoints(gz::sim::EntityComponentManager &ecm) {
    if (robot == gz::sim::kNullEntity || !ecm.HasEntity(robot)) {
      robot = ecm.EntityByComponents(gripper_c::Model(), gripper_c::Name(robotName),
                                     gripper_c::ParentEntity(world));
      joints.fill(gz::sim::kNullEntity);
    }
    if (robot == gz::sim::kNullEntity) return;
    for (size_t i = 0; i < names.size(); ++i)
      if (joints[i] == gz::sim::kNullEntity || !ecm.HasEntity(joints[i]))
        joints[i] = ecm.EntityByComponents(gripper_c::Joint(), gripper_c::Name(names[i]),
                                           gripper_c::ParentEntity(robot));
  }
  void updateStatus(const gz::sim::UpdateInfo &info,
                    const gz::sim::EntityComponentManager &ecm) {
    const auto command = commandSnapshot();
    GripperJson position = GripperJson::array(), velocity = GripperJson::array();
    bool measured = true;
    for (const auto joint : joints) {
      const auto *p = ecm.Component<gripper_c::JointPosition>(joint);
      const auto *v = ecm.Component<gripper_c::JointVelocity>(joint);
      const bool pf = p && p->Data().size() == 1 && std::isfinite(p->Data()[0]);
      const bool vf = v && v->Data().size() == 1 && std::isfinite(v->Data()[0]);
      position.push_back(pf ? GripperJson(p->Data()[0]) : GripperJson(nullptr));
      velocity.push_back(vf ? GripperJson(v->Data()[0]) : GripperJson(nullptr));
      measured = measured && pf && vf;
    }
    GripperJson state{{"success", true}, {"idealized", true},
      {"system", "agri::IdealGripperSystem"}, {"version", "two_finger_ideal_gripper_v1"},
      {"enabled", enabled}, {"ideal_simulation", ideal}, {"ready", ready && measured},
      {"active", active}, {"reason", reason}, {"joint_names", names},
      {"robot_name", robotName}, {"command_topic", commandTopic}, {"status_service", statusService},
      {"command_policy", "last_finite_two_target_held"}, {"target_valid", command.valid},
      {"target_m", command.valid ? GripperJson(command.target) : GripperJson(nullptr)},
      {"actual_position_m", position}, {"actual_velocity_m_s", velocity},
      {"sim_ns", Ns(info.simTime)}, {"paused", info.paused}, {"applied_count", applied},
      {"received_count", command.received}, {"rejected_count", command.rejected},
      {"rejection_reason", command.rejected ? "TWO_FINITE_TARGETS_REQUIRED" : ""},
      {"feedback_source", "Existing JointPosition and JointVelocity after Physics UpdateSim; no feedback is written"},
      {"control_handoff", "Only the two gripper finger joints receive native position resets and zero velocity commands"}};
    std::lock_guard<std::mutex> lock(mutex); statusText = state.dump();
  }

public:
  ~IdealGripperSystem() override { node.reset(); }
  gz::sim::System::PriorityType ConfigurePriority() override { return 100; }
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    if (configured) { gzerr << "IdealGripperSystem: duplicate Configure refused.\n"; return; }
    configured = true; world = entity;
    if (!sdf || !ecm.Component<gripper_c::World>(entity)) {
      reason = "WORLD_CONFIGURATION_REQUIRED"; return;
    }
    enabled = sdf->HasElement("enabled") && sdf->Get<bool>("enabled");
    ideal = sdf->HasElement("ideal_simulation") && sdf->Get<bool>("ideal_simulation");
    if (sdf->HasElement("robot_name")) robotName = sdf->Get<std::string>("robot_name");
    const auto *name = ecm.Component<gripper_c::Name>(entity);
    if (!enabled || !ideal || robotName.empty() || !name || name->Data().empty()) {
      enabled = false; reason = "EXPLICIT_IDEAL_CONFIGURATION_REQUIRED"; return;
    }
    const std::string prefix = "/world/" + name->Data() + "/ideal_gripper";
    commandTopic = prefix + "/commands"; statusService = prefix + "/status";
    reason = "WAITING_FOR_FIRST_FINITE_TWO_TARGET";
    updateStatus(gz::sim::UpdateInfo{}, ecm);
    node = std::make_unique<gz::transport::Node>();
    if (!node->Advertise(statusService, &IdealGripperSystem::Status, this) ||
        !node->Subscribe(commandTopic, &IdealGripperSystem::onCommand, this)) {
      enabled = false; reason = "TRANSPORT_CONFIGURATION_FAILED";
      updateStatus(gz::sim::UpdateInfo{}, ecm); node.reset(); return;
    }
    gzmsg << "IdealGripperSystem enabled for " << robotName << ".\n";
  }
  bool Status(const gz::msgs::StringMsg &request, gz::msgs::StringMsg &response) {
    const auto input = GripperJson::parse(request.data(), nullptr, false);
    if (!input.is_object() || !input.contains("op") || !input.at("op").is_string() ||
        input.at("op").get<std::string>() != "status") {
      response.set_data("{\"success\":false,\"reason\":\"READ_ONLY_STATUS_REQUEST_REQUIRED\"}"); return true;
    }
    std::lock_guard<std::mutex> lock(mutex); response.set_data(statusText); return true;
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (!enabled) return;
    findJoints(ecm);
    ready = robot != gz::sim::kNullEntity && ecm.HasEntity(robot);
    for (const auto joint : joints) ready = ready && joint != gz::sim::kNullEntity && ecm.HasEntity(joint);
    active = false;
    if (!ready) { reason = "TWO_FINGER_JOINTS_UNAVAILABLE"; return; }
    if (info.paused || Ns(info.dt) <= 0) { reason = "PAUSED_OR_ZERO_DT_TARGET_RETAINED"; return; }
    const auto command = commandSnapshot();
    if (!command.valid) { reason = "WAITING_FOR_FIRST_FINITE_TWO_TARGET"; return; }
    for (size_t i = 0; i < joints.size(); ++i) {
      // Force commands override velocity commands in Physics; velocity resets
      // also make Physics ignore the velocity servo and emit a warning.
      // Explicit ideal ownership removes those competing commands only here.
      ecm.RemoveComponent<gripper_c::JointForceCmd>(joints[i]);
      ecm.RemoveComponent<gripper_c::JointVelocityReset>(joints[i]);
      if (auto *velocity = ecm.Component<gripper_c::JointVelocityCmd>(joints[i])) velocity->Data() = {0.0};
      else ecm.CreateComponent(joints[i], gripper_c::JointVelocityCmd({0.0}));
      if (auto *reset = ecm.Component<gripper_c::JointPositionReset>(joints[i])) reset->Data() = {command.target[i]};
      else ecm.CreateComponent(joints[i], gripper_c::JointPositionReset({command.target[i]}));
    }
    active = true; ++applied; reason = "ACTIVE_IDEAL_TWO_FINGERS";
  }
  void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override {
    if (enabled) updateStatus(info, ecm);
  }
};
}  // namespace agri

GZ_ADD_PLUGIN(agri::IdealGripperSystem, gz::sim::System,
             agri::IdealGripperSystem::ISystemConfigure,
             agri::IdealGripperSystem::ISystemConfigurePriority,
             agri::IdealGripperSystem::ISystemPreUpdate,
             agri::IdealGripperSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(agri::IdealGripperSystem, "agri::IdealGripperSystem")
