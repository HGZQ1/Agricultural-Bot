// Explicit simulation-only steering idealization for all four steering joints.
// It changes only native steering position commands; feedback, wheels, arm,
// gripper and base pose remain owned by Gazebo/ros2_control.
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
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
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>

namespace agri {
namespace c = gz::sim::components;
using Json = nlohmann::json;

class IdealFourWheelSteeringSystem : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate {
  std::unique_ptr<gz::transport::Node> node;
  std::mutex mutex;
  const std::array<std::string, 4> names{{"f1_steer_joint", "f2_steer_joint",
                                           "r1_steer_joint", "r2_steer_joint"}};
  gz::sim::Entity world{gz::sim::kNullEntity}, robot{gz::sim::kNullEntity};
  std::array<gz::sim::Entity, 4> joints{{gz::sim::kNullEntity, gz::sim::kNullEntity,
                                          gz::sim::kNullEntity, gz::sim::kNullEntity}};
  std::array<double, 4> target{{0., 0., 0., 0.}};
  bool targetValid{false}, configured{false}, enabled{false}, ideal{false};
  bool ready{false}, active{false};
  uint64_t sequence{0}, processed{0}, applied{0}, rejected{0};
  std::string robotName{"agri_robot"}, commandTopic, statusService;
  std::string reason{"DISABLED"}, statusText{"{\"idealized\":true,\"ready\":false}"};

  static int64_t Ns(std::chrono::steady_clock::duration d) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(d).count();
  }
  static Json number(double value) { return std::isfinite(value) ? Json(value) : Json(nullptr); }

  void onCommand(const gz::msgs::Double_V &message) {
    std::array<double, 4> next{};
    bool valid = message.data_size() == 4;
    if (valid) for (int i = 0; i < 4; ++i) {
      next[static_cast<size_t>(i)] = message.data(i);
      valid = valid && std::isfinite(next[static_cast<size_t>(i)]);
    }
    std::lock_guard<std::mutex> lock(mutex);
    ++sequence;
    if (valid) { target = next; targetValid = true; }
    else { ++rejected; reason = "FOUR_FINITE_TARGETS_REQUIRED"; }
  }

  void findJoints(gz::sim::EntityComponentManager &ecm) {
    if (robot == gz::sim::kNullEntity || !ecm.HasEntity(robot))
      robot = ecm.EntityByComponents(c::Model(), c::Name(robotName), c::ParentEntity(world));
    for (size_t i = 0; i < names.size(); ++i) {
      if (joints[i] == gz::sim::kNullEntity || !ecm.HasEntity(joints[i]))
        joints[i] = ecm.EntityByComponents(c::Joint(), c::Name(names[i]), c::ParentEntity(robot));
    }
  }

  void updateStatus(const gz::sim::UpdateInfo &info,
                    const gz::sim::EntityComponentManager &ecm) {
    Json components = Json::object();
    Json actual = Json::array();
    Json desired = Json::array();
    bool all = robot != gz::sim::kNullEntity && ecm.HasEntity(robot);
    for (size_t i = 0; i < names.size(); ++i) {
      const auto *p = ecm.Component<c::JointPosition>(joints[i]);
      const auto *a = ecm.Component<c::JointAxis>(joints[i]);
      const auto *v = ecm.Component<c::JointVelocityCmd>(joints[i]);
      const bool pf = p && p->Data().size() == 1 && std::isfinite(p->Data()[0]);
      const bool af = a && std::isfinite(a->Data().Lower()) && std::isfinite(a->Data().Upper()) &&
                      a->Data().Lower() < a->Data().Upper();
      const bool vf = v && v->Data().size() == 1;
      components[names[i]] = { {"position", pf}, {"axis", af}, {"velocity_command", vf} };
      actual.push_back(pf ? number(p->Data()[0]) : Json(nullptr));
      desired.push_back(targetValid ? number(target[i]) : Json(nullptr));
      all = all && pf && af && vf;
    }
    Json state{{"success", true}, {"idealized", true},
      {"system", "agri::IdealFourWheelSteeringSystem"},
      {"version", "four_wheel_ideal_steering_v1"}, {"enabled", enabled},
      {"ready", ready && all}, {"active", active}, {"ideal_simulation", ideal},
      {"command_policy", "last_finite_four_target_held"}, {"joint_names", names},
      {"robot_name", robotName}, {"command_topic", commandTopic},
      {"status_service", statusService}, {"expiry_reason", reason},
      {"sim_ns", Ns(info.simTime)}, {"paused", info.paused},
      {"ready_components", components}, {"actual_position_rad", actual},
      {"target_rad", desired}, {"applied_count", applied}, {"rejected_count", rejected},
      {"feedback_source", "Existing JointPosition after Physics UpdateSim; no feedback is written"},
      {"control_handoff", "Only the four steering joints receive native position resets; wheels, arm, gripper and base pose are untouched"}};
    std::lock_guard<std::mutex> lock(mutex);
    statusText = state.dump();
  }

  void preUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) {
    findJoints(ecm);
    bool componentsReady = robot != gz::sim::kNullEntity && ecm.HasEntity(robot);
    std::array<const c::JointAxis *, 4> axes{};
    for (size_t i = 0; i < names.size(); ++i) {
      auto *p = ecm.Component<c::JointPosition>(joints[i]);
      axes[i] = ecm.Component<c::JointAxis>(joints[i]);
      auto *v = ecm.Component<c::JointVelocityCmd>(joints[i]);
      const bool validPosition = p && p->Data().size() == 1 && std::isfinite(p->Data()[0]);
      const bool validAxis = axes[i] && std::isfinite(axes[i]->Data().Lower()) &&
        std::isfinite(axes[i]->Data().Upper()) && axes[i]->Data().Lower() < axes[i]->Data().Upper();
      if (!v && validPosition && validAxis)
        v = ecm.CreateComponent(joints[i], c::JointVelocityCmd({0.0}));
      else if (v && v->Data().empty()) v->Data() = {0.0};
      componentsReady = componentsReady && validPosition && validAxis && v && v->Data().size() == 1;
    }
    ready = componentsReady;
    if (!ready) { active = false; reason = "FOUR_STEERING_COMPONENTS_UNAVAILABLE"; return; }
    if (info.paused || Ns(info.dt) <= 0) { active = false; reason = "PAUSED_OR_ZERO_DT_TARGET_RETAINED"; return; }
    std::array<double, 4> local{}; bool valid = false;
    { std::lock_guard<std::mutex> lock(mutex); local = target; valid = targetValid; }
    if (!valid) { active = false; reason = "WAITING_FOR_FIRST_FINITE_FOUR_TARGET"; return; }
    for (size_t i = 0; i < names.size(); ++i) {
      if (local[i] < axes[i]->Data().Lower() || local[i] > axes[i]->Data().Upper()) {
        ++rejected; active = false; reason = "TARGET_OUTSIDE_IMPORTED_JOINT_LIMITS"; return;
      }
    }
    for (size_t i = 0; i < names.size(); ++i) {
      if (auto *force = ecm.Component<c::JointForceCmd>(joints[i])) force->Data() = {0.0};
      if (auto *reset = ecm.Component<c::JointPositionReset>(joints[i])) reset->Data() = {local[i]};
      else ecm.CreateComponent(joints[i], c::JointPositionReset({local[i]}));
      ecm.Component<c::JointVelocityCmd>(joints[i])->Data()[0] = 0.0;
    }
    active = true; ++applied; reason = "ACTIVE_IDEAL_FOUR_STEERING";
  }

public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    configured = true; world = entity;
    if (!sdf || !ecm.Component<c::World>(entity)) { reason = "WORLD_CONFIGURATION_REQUIRED"; return; }
    if (sdf->HasElement("enabled")) enabled = sdf->Get<bool>("enabled");
    if (sdf->HasElement("ideal_simulation")) ideal = sdf->Get<bool>("ideal_simulation");
    else { const char *env = std::getenv("AGRI_IDEAL_SIMULATION"); ideal = env && std::string(env) == "1"; }
    if (sdf->HasElement("robot_name")) robotName = sdf->Get<std::string>("robot_name");
    const auto *name = ecm.Component<c::Name>(entity);
    if (!enabled || !ideal || !name || name->Data().empty()) { enabled = false; reason = "EXPLICIT_IDEAL_CONFIGURATION_REQUIRED"; return; }
    const std::string prefix = "/world/" + name->Data() + "/ideal_steering";
    commandTopic = prefix + "/four_wheel_commands"; statusService = prefix + "/four_wheel_status";
    if (sdf->HasElement("command_topic")) commandTopic = sdf->Get<std::string>("command_topic");
    if (sdf->HasElement("status_service")) statusService = sdf->Get<std::string>("status_service");
    node = std::make_unique<gz::transport::Node>();
    if (!node->Advertise(statusService, &IdealFourWheelSteeringSystem::Status, this) ||
        !node->Subscribe(commandTopic, &IdealFourWheelSteeringSystem::onCommand, this)) {
      enabled = false; reason = "TRANSPORT_CONFIGURATION_FAILED"; return;
    }
    reason = "WAITING_FOR_FIRST_FINITE_FOUR_TARGET";
    gzmsg << "IdealFourWheelSteeringSystem enabled for " << robotName << ".\n";
  }
  bool Status(const gz::msgs::StringMsg &request, gz::msgs::StringMsg &response) {
    const auto input = Json::parse(request.data(), nullptr, false);
    if (!input.is_object() || input.value("op", "") != "status") {
      response.set_data("{\"success\":false,\"reason\":\"READ_ONLY_STATUS_REQUEST_REQUIRED\"}"); return true;
    }
    std::lock_guard<std::mutex> lock(mutex); response.set_data(statusText); return true;
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (enabled) preUpdate(info, ecm);
  }
  void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override {
    if (enabled) updateStatus(info, ecm);
  }
};
}

GZ_ADD_PLUGIN(agri::IdealFourWheelSteeringSystem, gz::sim::System,
             agri::IdealFourWheelSteeringSystem::ISystemConfigure,
             agri::IdealFourWheelSteeringSystem::ISystemPreUpdate,
             agri::IdealFourWheelSteeringSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(agri::IdealFourWheelSteeringSystem, "agri::IdealFourWheelSteeringSystem")
