// Explicitly enabled, bounded, feedback-only observer. Never create commands.
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fcntl.h>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace agri {
namespace arm_c = gz::sim::components;
using ArmJson = nlohmann::json;
using ArmEntity = gz::sim::Entity;
using ArmEcm = gz::sim::EntityComponentManager;
class ArmJointDiagnosticsV1 : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
  FILE *output{nullptr};
  std::chrono::steady_clock::time_point started;
  ArmEntity world{gz::sim::kNullEntity};
  std::map<std::string, ArmEntity> joints, collisions;
  std::set<ArmEntity> observedContacts;
  ArmJson beforePhysics;
  int64_t lastSample{-1};
  uint64_t feedbackReady{0}, sampleCount{0};
  static int64_t Ns(std::chrono::steady_clock::duration value) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(value).count();
  }
  double Wall() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
  }
  static ArmJson Number(double value) { return std::isfinite(value) ? ArmJson(value) : ArmJson(nullptr); }
  template<typename C> static ArmJson Existing(ArmEntity entity, const ArmEcm &ecm) {
    const auto *value = ecm.Component<C>(entity);
    if (!value || value->Data().empty()) return nullptr;
    ArmJson result = ArmJson::array();
    for (double item : value->Data()) result.push_back(Number(item));
    return result;
  }
  template<typename C> static bool Feedback(ArmEntity entity, ArmEcm &ecm) {
    if (entity == gz::sim::kNullEntity || ecm.Component<C>(entity)) return false;
    ecm.CreateComponent(entity, C());
    return true;
  }
  static ArmJson Vector(const gz::msgs::Vector3d &v) {
    return ArmJson::array({Number(v.x()), Number(v.y()), Number(v.z())});
  }
  static ArmJson Wrench(const gz::msgs::Wrench &w) {
    if (!w.has_force() && !w.has_torque()) return nullptr;
    return {{"force_N", w.has_force() ? Vector(w.force()) : ArmJson(nullptr)},
      {"torque_Nm", w.has_torque() ? Vector(w.torque()) : ArmJson(nullptr)}};
  }
  static FILE *Open(const std::string &rootText, const std::string &pathText) {
    namespace fs = std::filesystem;
    fs::path root(rootText), path(pathText);
    if (!root.is_absolute() || !path.is_absolute() || path.extension() != ".jsonl")
      throw std::runtime_error("absolute canonical workspace_root/output .jsonl required");
    root = root.lexically_normal(); path = path.lexically_normal();
    if (!fs::is_directory(root) || fs::canonical(root) != root)
      throw std::runtime_error("workspace_root must be a canonical directory");
    const auto relative = path.lexically_relative(root / "artifacts");
    if (relative.empty() || *relative.begin() == ".." || !fs::is_directory(path.parent_path()))
      throw std::runtime_error("new output requires existing parent under workspace_root/artifacts");
    for (auto p = path; !p.empty(); p = p.parent_path()) {
      if (fs::is_symlink(fs::symlink_status(p))) throw std::runtime_error("symlink output/ancestor refused");
      if (p == p.root_path()) break;
    }
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0600);
    if (fd < 0) throw std::runtime_error("exclusive output creation failed; no overwrite allowed");
    FILE *file = ::fdopen(fd, "w");
    if (!file) { ::close(fd); throw std::runtime_error("fdopen failed"); }
    return file;
  }
  void Write(const ArmJson &value) {
    if (!output) return;
    const auto line = value.dump()+"\n";
    if (std::fwrite(line.data(), 1, line.size(), output) != line.size()) {
      gzerr << "ArmJointDiagnosticsV1 output failed; observer disabled.\n";
      std::fclose(output); output = nullptr;
    } else std::fflush(output);
  }
  void Finish(const std::string &reason) {
    if (!output) return;
    Write({{"kind", "summary"}, {"reason", reason}, {"wall_elapsed_s", Wall()}, {"samples", sampleCount}});
    if (output) { std::fclose(output); output = nullptr; }
  }
  bool Expired() {
    if (!output) return true;
    if (Wall() >= 30.) { Finish("30_wall_seconds_reached"); return true; }
    return false;
  }
  ArmJson Contacts(ArmEntity entity, const ArmEcm &ecm) {
    const auto *component = ecm.Component<arm_c::ContactSensorData>(entity);
    ArmJson result{{"collision_id", entity}, {"component_present", component != nullptr},
      {"message_stamp_ns", nullptr}, {"contacts", nullptr}, {"ever_nonempty", false}};
    if (!component) return result;
    const auto &data = component->Data();
    if (data.has_header() && data.header().has_stamp())
      result["message_stamp_ns"] = int64_t(data.header().stamp().sec())*1000000000LL + data.header().stamp().nsec();
    if (data.contact_size()) observedContacts.insert(entity);
    result["ever_nonempty"] = observedContacts.count(entity) > 0;
    // Empty newly requested feedback is unknown, not proof of contact absence.
    if (!observedContacts.count(entity)) return result;
    result["contacts"] = ArmJson::array();
    for (const auto &contact : data.contact()) {
      auto label = [&ecm](const gz::msgs::Entity &e) {
        return ArmJson{{"id", e.id()}, {"name", e.name().empty() && ecm.HasEntity(e.id())
          ? gz::sim::scopedName(e.id(), ecm) : e.name()}};
      };
      ArmJson entry{{"collision1", contact.has_collision1() ? label(contact.collision1()) : ArmJson(nullptr)},
        {"collision2", contact.has_collision2() ? label(contact.collision2()) : ArmJson(nullptr)},
        {"message_stamp_ns", nullptr}, {"positions_world_m", ArmJson::array()},
        {"depths_m", ArmJson::array()}, {"wrenches_world", ArmJson::array()}};
      if (contact.has_header() && contact.header().has_stamp())
        entry["message_stamp_ns"] = int64_t(contact.header().stamp().sec())*1000000000LL + contact.header().stamp().nsec();
      for (const auto &v : contact.position()) entry["positions_world_m"].push_back(Vector(v));
      for (double depth : contact.depth()) entry["depths_m"].push_back(Number(depth));
      for (const auto &w : contact.wrench()) entry["wrenches_world"].push_back({
        {"body1", w.has_body_1_wrench() ? Wrench(w.body_1_wrench()) : ArmJson(nullptr)},
        {"body2", w.has_body_2_wrench() ? Wrench(w.body_2_wrench()) : ArmJson(nullptr)}});
      result["contacts"].push_back(entry);
    }
    return result;
  }
public:
  ~ArmJointDiagnosticsV1() override { Finish("plugin_destroyed"); }
  void Configure(const ArmEntity &entity, const std::shared_ptr<const sdf::Element> &sdf,
      ArmEcm &ecm, gz::sim::EventManager &) override {
    try {
      if (!sdf || !sdf->HasElement("enabled") || !sdf->Get<bool>("enabled") ||
          !ecm.Component<arm_c::World>(entity)) return;
      if (!sdf->HasElement("workspace_root") || !sdf->HasElement("output"))
        throw std::runtime_error("explicit workspace_root and output required");
      world = entity; started = std::chrono::steady_clock::now();
      output = Open(sdf->Get<std::string>("workspace_root"), sdf->Get<std::string>("output"));
      Write({{"kind", "metadata"}, {"system", "agri::ArmJointDiagnosticsV1"},
        {"readonly", true}, {"wall_duration_s", 30}, {"sample_period_sim_s", .1},
        {"joints", {"arm_joint5", "cr5_joint6"}}, {"selected_fruit", "tomato_20_fruit_2"},
        {"command_sampling", "existing slots in PreUpdate; same iteration PostUpdate feedback; ordering before observer matters"},
        {"wrench", "JointTransmittedWrench is reaction in joint frame, not applied motor effort"},
        {"contact_freshness", "stamps recorded when available; empty/cached data does not independently prove current no-contact"}});
    } catch (const std::exception &error) {
      gzerr << "ArmJointDiagnosticsV1 configuration refused: " << error.what() << "\n";
      Finish("configuration_error");
    }
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, ArmEcm &ecm) override {
    beforePhysics = nullptr;
    if (Expired() || info.paused || Ns(info.dt) <= 0) return;
    const auto robot = ecm.EntityByComponents(arm_c::Model(), arm_c::Name("agri_robot"), arm_c::ParentEntity(world));
    if (robot == gz::sim::kNullEntity) return;
    bool requested = false;
    for (const std::string name : {"arm_joint5", "cr5_joint6"}) {
      const auto joint = ecm.EntityByComponents(arm_c::Joint(), arm_c::Name(name), arm_c::ParentEntity(robot));
      if (joint == gz::sim::kNullEntity) continue;
      joints[name] = joint;
      requested |= Feedback<arm_c::JointPosition>(joint, ecm);
      requested |= Feedback<arm_c::JointVelocity>(joint, ecm);
      requested |= Feedback<arm_c::JointTransmittedWrench>(joint, ecm);
    }
    const auto fruit = ecm.EntityByComponents(arm_c::Model(), arm_c::Name("tomato_20_fruit_2"), arm_c::ParentEntity(world));
    collisions.clear();
    ecm.Each<arm_c::Collision, arm_c::ParentEntity>([&](ArmEntity entity, const arm_c::Collision *, const arm_c::ParentEntity *parent) {
      const auto link = parent->Data();
      const auto *name = ecm.Component<arm_c::Name>(link);
      const auto *model = ecm.Component<arm_c::ParentEntity>(link);
      if (!name || !model || !ecm.Component<arm_c::Link>(link)) return true;
      if ((model->Data() == robot && (name->Data() == "gripper_base_link" ||
          name->Data() == "gripper_left_finger_link" || name->Data() == "gripper_right_finger_link")) ||
          (fruit != gz::sim::kNullEntity && model->Data() == fruit)) {
        collisions[gz::sim::scopedName(entity, ecm)] = entity;
        requested |= Feedback<arm_c::ContactSensorData>(entity, ecm);
      }
      return true;
    });
    if (requested) feedbackReady = info.iterations+2;
    const auto now = Ns(info.simTime);
    if (lastSample >= 0 && now >= lastSample && now-lastSample < 100000000) return;
    lastSample = now;
    beforePhysics = {{"iteration", info.iterations}, {"sim_ns", now}, {"joints", ArmJson::object()}};
    for (const auto &[name, entity] : joints) beforePhysics["joints"][name] = {
      {"JointVelocityCmd_rps", Existing<arm_c::JointVelocityCmd>(entity, ecm)},
      {"JointForceCmd_Nm", Existing<arm_c::JointForceCmd>(entity, ecm)},
      {"JointVelocityReset_present", ecm.Component<arm_c::JointVelocityReset>(entity) != nullptr},
      {"JointPositionReset_present", ecm.Component<arm_c::JointPositionReset>(entity) != nullptr},
      {"position_rad", Existing<arm_c::JointPosition>(entity, ecm)},
      {"velocity_rps", Existing<arm_c::JointVelocity>(entity, ecm)}};
  }
  void PostUpdate(const gz::sim::UpdateInfo &info, const ArmEcm &ecm) override {
    if (Expired() || beforePhysics.is_null() || info.paused ||
        beforePhysics["iteration"] != info.iterations) return;
    ArmJson result{{"kind", "sample"}, {"iteration", info.iterations}, {"sim_ns", Ns(info.simTime)},
      {"wall_elapsed_s", Wall()}, {"before_physics", beforePhysics},
      {"joints", ArmJson::object()}, {"contacts", ArmJson::object()},
      {"detachable_joints", ArmJson::array()}, {"feedback_ready", info.iterations >= feedbackReady}};
    for (const auto &[name, entity] : joints) {
      const auto *axis = ecm.Component<arm_c::JointAxis>(entity);
      const auto *wrench = ecm.Component<arm_c::JointTransmittedWrench>(entity);
      ArmJson torque = nullptr;
      if (axis && wrench && wrench->Data().has_torque() && info.iterations >= feedbackReady) {
        const auto &a = axis->Data().Xyz(); const auto &t = wrench->Data().torque();
        torque = Number(a.X()*t.x()+a.Y()*t.y()+a.Z()*t.z());
      }
      result["joints"][name] = {{"entity_id", entity},
        {"position_rad", Existing<arm_c::JointPosition>(entity, ecm)},
        {"velocity_rps", Existing<arm_c::JointVelocity>(entity, ecm)},
        {"reaction_wrench_joint_frame", wrench && info.iterations >= feedbackReady ? Wrench(wrench->Data()) : ArmJson(nullptr)},
        {"reaction_axis_torque_Nm", torque}, {"axis", axis ? ArmJson{
          {"xyz", {Number(axis->Data().Xyz().X()), Number(axis->Data().Xyz().Y()), Number(axis->Data().Xyz().Z())}},
          {"lower", Number(axis->Data().Lower())}, {"upper", Number(axis->Data().Upper())},
          {"effort", Number(axis->Data().Effort())}, {"velocity", Number(axis->Data().MaxVelocity())},
          {"damping", Number(axis->Data().Damping())}, {"friction", Number(axis->Data().Friction())}} : ArmJson(nullptr)}};
    }
    for (const auto &[name, entity] : collisions) result["contacts"][name] = Contacts(entity, ecm);
    ecm.Each<arm_c::DetachableJoint>([&](ArmEntity entity, const arm_c::DetachableJoint *joint) {
      const auto &value = joint->Data();
      result["detachable_joints"].push_back({{"entity_id", entity},
        {"parent_link", value.parentLink}, {"child_link", value.childLink}, {"joint_type", value.jointType}});
      return true;
    });
    Write(result); ++sampleCount;
  }
};
}
GZ_ADD_PLUGIN(agri::ArmJointDiagnosticsV1, gz::sim::System,
    agri::ArmJointDiagnosticsV1::ISystemConfigure, agri::ArmJointDiagnosticsV1::ISystemPreUpdate,
    agri::ArmJointDiagnosticsV1::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(agri::ArmJointDiagnosticsV1, "agri::ArmJointDiagnosticsV1")
