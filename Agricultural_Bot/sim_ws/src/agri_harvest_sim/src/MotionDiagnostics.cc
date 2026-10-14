// Simulation-only observer: request feedback, never create command components.
// PreUpdate copies existing command slots; PostUpdate records physics readbacks.
// Physics consumes and clears command slots during Update, so those stages differ.
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularAcceleration.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointEffortLimitsCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocityLimitsCmd.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fcntl.h>
#include <filesystem>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace agri
{
namespace c = gz::sim::components;
using Json = nlohmann::json;
using Entity = gz::sim::Entity;
using Ecm = gz::sim::EntityComponentManager;
namespace fs = std::filesystem;

// The distinct C++ type and plugin name avoid a live loader resolving an older
// MotionDiagnostics class which was already registered by a prior library.
class SteeringDiagnosticsV9 : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate,
                          public gz::sim::ISystemPostUpdate
{
  struct Sample { int64_t ns; Json data; bool written{false}; };
  FILE *output{nullptr};
  std::chrono::steady_clock::time_point started;
  Entity world{gz::sim::kNullEntity}, robot{gz::sim::kNullEntity}, base{gz::sim::kNullEntity};
  std::string robotName{"agri_robot"};
  std::string startScope{"continuation_from_actual_stopped_pose_not_normal_spawn"};
  std::map<std::string, Entity> joints;
  std::map<std::string, std::vector<Entity>> wheels;
  std::set<Entity> contactObserved;
  std::deque<Sample> history;
  Json beforePhysicsCommands = nullptr;
  double wallDuration{180.0}, trigger{0.025}, peak{0};
  int64_t periodNs{20000000}, windowNs{500000000};
  int64_t lastNs{-1}, lastRegularNs{-1}, captureUntil{-1};
  uint64_t readyIteration{0}, samples{0}, records{0}, events{0}, historyDrops{0};
  bool wasAbove{false}, firstPose{true};

  static int64_t Ns(std::chrono::steady_clock::duration duration)
  { return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count(); }
  double Wall() const
  { return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(); }
  static Json Number(double x) { return std::isfinite(x) ? Json(x) : Json(nullptr); }
  template<typename V> static Json Vector(const V &v)
  {
    return std::isfinite(v.X()) && std::isfinite(v.Y()) && std::isfinite(v.Z())
      ? Json::array({v.X(), v.Y(), v.Z()}) : Json(nullptr);
  }
  static Json MsgVector(const gz::msgs::Vector3d &v)
  { return Json::array({Number(v.x()), Number(v.y()), Number(v.z())}); }
  static Json Wrench(const gz::msgs::Wrench &w)
  {
    if (!w.has_force() && !w.has_torque()) return nullptr;
    return {{"force_N", w.has_force() ? MsgVector(w.force()) : Json(nullptr)},
            {"torque_Nm", w.has_torque() ? MsgVector(w.torque()) : Json(nullptr)}};
  }
  template<typename C> Json FeedbackVector(Entity entity, const Ecm &ecm,
                                           uint64_t iteration) const
  {
    const auto *component = ecm.Component<C>(entity);
    return component && iteration >= readyIteration ? Vector(component->Data()) : Json(nullptr);
  }
  template<typename C> static bool Request(Entity entity, Ecm &ecm, const C &value = C())
  {
    if (entity == gz::sim::kNullEntity || ecm.Component<C>(entity)) return false;
    ecm.CreateComponent(entity, value); // Feedback-only component; no commands.
    return true;
  }
  // Inspect existing command components only. Never request or create them.
  template<typename C> static Json ExistingArray(Entity entity, const Ecm &ecm)
  {
    const auto *component = ecm.Component<C>(entity);
    if (!component || component->Data().empty()) return nullptr;
    Json values = Json::array();
    for (double value : component->Data()) values.push_back(Number(value));
    return values;
  }
  template<typename C> static Json ExistingLimitPairs(Entity entity, const Ecm &ecm)
  {
    const auto *component = ecm.Component<C>(entity);
    if (!component || component->Data().empty()) return nullptr;
    Json values = Json::array();
    for (const auto &value : component->Data())
      values.push_back(Json::array({Number(value.X()), Number(value.Y())}));
    return values;
  }
  static Json ContactStamp(const gz::msgs::Header &header)
  {
    return header.has_stamp() ? Json(int64_t(header.stamp().sec()) * 1000000000LL +
      header.stamp().nsec()) : Json(nullptr);
  }
  Json ExistingCommands(const gz::sim::UpdateInfo &info, const Ecm &ecm) const
  {
    Json result{{"iteration", info.iterations}, {"sim_ns", Ns(info.simTime)},
                {"physics_dt_ns", Ns(info.dt)}, {"paused", info.paused},
                {"stage", "observer_PreUpdate_before_Physics_Update"},
                {"joints", Json::object()}};
    for (const auto &[name, entity] : joints) {
      if (name.find("_steer_joint") == std::string::npos &&
          name.find("_wheel_joint") == std::string::npos) continue;
      result["joints"][name] = {
        {"existing_JointForceCmd_Nm", ExistingArray<c::JointForceCmd>(entity, ecm)},
        {"existing_JointVelocityCmd_rps", ExistingArray<c::JointVelocityCmd>(entity, ecm)},
        {"pending_JointEffortLimitsCmd_pairs", ExistingLimitPairs<c::JointEffortLimitsCmd>(entity, ecm)},
        {"pending_JointVelocityLimitsCmd_pairs", ExistingLimitPairs<c::JointVelocityLimitsCmd>(entity, ecm)},
        {"pre_update_joint_position_rad", ExistingArray<c::JointPosition>(entity, ecm)},
        {"pre_update_joint_velocity_rps", ExistingArray<c::JointVelocity>(entity, ecm)}};
    }
    return result;
  }
  void Write(const Json &record)
  {
    if (!output) return;
    const auto line = record.dump() + "\n";
    if (std::fwrite(line.data(), 1, line.size(), output) != line.size()) {
      gzerr << "MotionDiagnostics: output write failed; observer disabled.\n";
      std::fclose(output); output = nullptr;
    }
  }
  void Finish(const std::string &reason)
  {
    if (!output) return;
    Write({{"kind", "summary"}, {"reason", reason}, {"wall_elapsed_s", Wall()},
           {"physics_samples", samples}, {"sample_records", records},
           {"spike_events", events}, {"peak_abs_q2_velocity_rps", peak},
           {"history_dropped_samples", historyDrops}});
    if (output) { std::fflush(output); std::fclose(output); output = nullptr; }
  }
  bool Expired()
  {
    if (!output) return true;
    if (Wall() >= wallDuration) { Finish("wall_duration_reached"); return true; }
    return false;
  }
  static FILE *Open(const std::string &rootText, const std::string &pathText)
  {
    fs::path root(rootText), path(pathText);
    if (!root.is_absolute() || !path.is_absolute() || path.extension() != ".jsonl")
      throw std::runtime_error("workspace_root and output must be absolute; output must end .jsonl");
    root = root.lexically_normal(); path = path.lexically_normal();
    if (!fs::is_directory(root) || fs::canonical(root) != root)
      throw std::runtime_error("workspace_root must be an existing canonical directory");
    const auto relative = path.lexically_relative(root / "artifacts");
    if (relative.empty() || *relative.begin() == "..")
      throw std::runtime_error("output must be under workspace_root/artifacts");
    if (!fs::is_directory(path.parent_path()))
      throw std::runtime_error("output parent must already exist");
    for (auto p = path; !p.empty(); p = p.parent_path()) {
      if (fs::is_symlink(fs::symlink_status(p)))
        throw std::runtime_error("output and its ancestors must not be symlinks");
      if (p == p.root_path()) break;
    }
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0600);
    if (fd < 0) throw std::runtime_error("cannot exclusively create output; existing files are refused");
    FILE *file = ::fdopen(fd, "w");
    if (!file) { ::close(fd); throw std::runtime_error("fdopen failed"); }
    return file;
  }
  Json Contacts(Entity collision, const Ecm &ecm)
  {
    const auto *component = ecm.Component<c::ContactSensorData>(collision);
    Json result{{"collision_id", collision}, {"collision_name", gz::sim::scopedName(collision, ecm)},
                {"component_present", component != nullptr}, {"feedback_observed", false},
                {"message_stamp_ns", nullptr},
                {"contacts", nullptr}};
    if (!component) return result;
    const auto &data = component->Data();
    if (data.has_header()) result["message_stamp_ns"] = ContactStamp(data.header());
    if (data.contact_size() > 0) contactObserved.insert(collision);
    result["feedback_observed"] = contactObserved.count(collision) > 0;
    // A newly requested empty message does not establish engine support or no contact.
    if (!contactObserved.count(collision)) return result;
    result["contacts"] = Json::array();
    for (const auto &contact : data.contact()) {
      auto entity = [&ecm](const gz::msgs::Entity &value) -> Json {
        std::string name = value.name();
        if (name.empty() && ecm.HasEntity(value.id())) name = gz::sim::scopedName(value.id(), ecm);
        return {{"id", value.id()}, {"name", name.empty() ? Json(nullptr) : Json(name)}};
      };
      Json entry{{"collision1", contact.has_collision1() ? entity(contact.collision1()) : Json(nullptr)},
                 {"collision2", contact.has_collision2() ? entity(contact.collision2()) : Json(nullptr)},
                 {"message_stamp_ns", contact.has_header() ? ContactStamp(contact.header()) : Json(nullptr)},
                 {"positions_world_m", contact.position_size() ? Json::array() : Json(nullptr)},
                 {"normals_world", contact.normal_size() ? Json::array() : Json(nullptr)},
                 {"depths_m", contact.depth_size() ? Json::array() : Json(nullptr)},
                 {"wrenches_world", contact.wrench_size() ? Json::array() : Json(nullptr)}};
      for (const auto &v : contact.position()) entry["positions_world_m"].push_back(MsgVector(v));
      for (const auto &v : contact.normal()) entry["normals_world"].push_back(MsgVector(v));
      for (double v : contact.depth()) entry["depths_m"].push_back(Number(v));
      for (const auto &w : contact.wrench())
        entry["wrenches_world"].push_back({
          {"body1", w.has_body_1_wrench() ? Wrench(w.body_1_wrench()) : Json(nullptr)},
          {"body2", w.has_body_2_wrench() ? Wrench(w.body_2_wrench()) : Json(nullptr)}});
      result["contacts"].push_back(entry);
    }
    return result;
  }
  Json Snapshot(const gz::sim::UpdateInfo &info, const Ecm &ecm)
  {
    Json record{{"kind", "sample"}, {"sim_ns", Ns(info.simTime)}, {"physics_dt_ns", Ns(info.dt)},
                {"iteration", info.iterations}, {"wall_elapsed_s", Wall()},
                {"base_world_pose", nullptr}, {"joints", Json::object()},
                {"wheels", Json::object()}, {"steering_joint_observations", Json::object()},
                {"before_physics_commands", nullptr},
                {"arm_joint2_transmitted_wrench", nullptr}};
    // Never attach a cached command snapshot from another physics iteration.
    if (!beforePhysicsCommands.is_null() &&
        beforePhysicsCommands["iteration"] == info.iterations &&
        beforePhysicsCommands["sim_ns"] == Ns(info.simTime))
      record["before_physics_commands"] = beforePhysicsCommands;
    if (base != gz::sim::kNullEntity && ecm.Component<c::Pose>(base)) {
      const auto pose = gz::sim::worldPose(base, ecm);
      record["base_world_pose"] = {{"position_m", Vector(pose.Pos())},
        {"quaternion_xyzw", Json::array({Number(pose.Rot().X()), Number(pose.Rot().Y()),
                                       Number(pose.Rot().Z()), Number(pose.Rot().W())})},
        {"rpy_rad", Vector(pose.Rot().Euler())}};
    }
    record["base_world_linear_velocity_mps"] = FeedbackVector<c::WorldLinearVelocity>(base, ecm, info.iterations);
    record["base_world_angular_velocity_rps"] = FeedbackVector<c::WorldAngularVelocity>(base, ecm, info.iterations);
    record["base_world_linear_acceleration_mps2"] = FeedbackVector<c::WorldLinearAcceleration>(base, ecm, info.iterations);
    record["base_world_angular_acceleration_rps2"] = FeedbackVector<c::WorldAngularAcceleration>(base, ecm, info.iterations);
    for (const auto &[name, entity] : joints) {
      const auto *position = ecm.Component<c::JointPosition>(entity);
      const auto *velocity = ecm.Component<c::JointVelocity>(entity);
      Json p = nullptr, v = nullptr;
      if (position && !position->Data().empty()) { p = Json::array(); for (double x : position->Data()) p.push_back(Number(x)); }
      if (velocity && !velocity->Data().empty()) { v = Json::array(); for (double x : velocity->Data()) v.push_back(Number(x)); }
      record["joints"][name] = {{"position_rad", p}, {"velocity_rps", v}};
      if (name.find("_steer_joint") != std::string::npos) {
        const auto *w = ecm.Component<c::JointTransmittedWrench>(entity);
        const auto *axis = ecm.Component<c::JointAxis>(entity);
        Json torque = nullptr;
        if (w && axis && w->Data().has_torque()) {
          const auto &t = w->Data().torque();
          const auto &a = axis->Data().Xyz();
          torque = Number(t.x()*a.X() + t.y()*a.Y() + t.z()*a.Z());
        }
        record["steering_joint_observations"][name] = {
          {"reaction_transmitted_wrench_joint_frame", w ? Wrench(w->Data()) : Json(nullptr)},
          {"reaction_torque_along_joint_axis_Nm", torque},
          {"joint_axis_xyz", axis ? Vector(axis->Data().Xyz()) : Json(nullptr)},
          {"imported_axis_limits", axis ? Json{
            {"effort_magnitude_Nm", Number(axis->Data().Effort())},
            {"velocity_magnitude_rps", Number(axis->Data().MaxVelocity())},
            {"position_lower_rad", Number(axis->Data().Lower())},
            {"position_upper_rad", Number(axis->Data().Upper())},
            {"damping", Number(axis->Data().Damping())},
            {"friction", Number(axis->Data().Friction())},
            {"xyz_expressed_in", axis->Data().XyzExpressedIn()}} : Json(nullptr)},
          {"existing_JointForceCmd_Nm", ExistingArray<c::JointForceCmd>(entity, ecm)},
          {"existing_JointVelocityCmd_rps", ExistingArray<c::JointVelocityCmd>(entity, ecm)},
          {"pending_JointEffortLimitsCmd_pairs", ExistingLimitPairs<c::JointEffortLimitsCmd>(entity, ecm)},
          {"pending_JointVelocityLimitsCmd_pairs", ExistingLimitPairs<c::JointVelocityLimitsCmd>(entity, ecm)},
          {"JointPositionCmd_rad", nullptr}};
      }
      if (name == "arm_joint2") {
        const auto *w = ecm.Component<c::JointTransmittedWrench>(entity);
        if (w) record["arm_joint2_transmitted_wrench"] = Wrench(w->Data());
      }
    }
    for (const auto &[name, collisions] : wheels) {
      record["wheels"][name] = collisions.empty() ? Json(nullptr) : Json::array();
      for (Entity entity : collisions) record["wheels"][name].push_back(Contacts(entity, ecm));
    }
    return record;
  }
  void Emit(Sample &sample, const std::string &capture)
  {
    if (sample.written || !output) return;
    sample.data["capture"] = capture;
    Write(sample.data); sample.written = true; ++records;
  }

public:
  ~SteeringDiagnosticsV9() override { Finish("plugin_destroyed"); }
  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 Ecm &ecm, gz::sim::EventManager &) override
  {
    try {
      if (output) throw std::runtime_error("observer already configured");
      if (!ecm.Component<c::World>(entity)) throw std::runtime_error("attach observer to the world entity");
      if (!sdf || !sdf->HasElement("workspace_root") || !sdf->HasElement("output"))
        throw std::runtime_error("explicit workspace_root and output are required");
      if (sdf->HasElement("robot_name")) robotName = sdf->Get<std::string>("robot_name");
      if (sdf->HasElement("start_scope")) startScope = sdf->Get<std::string>("start_scope");
      if (startScope.empty() || startScope.size() > 256)
        throw std::runtime_error("start_scope must be a nonempty description up to 256 characters");
      if (sdf->HasElement("wall_duration_s")) wallDuration = sdf->Get<double>("wall_duration_s");
      if (sdf->HasElement("regular_period_s")) {
        const double value = sdf->Get<double>("regular_period_s");
        if (!std::isfinite(value) || value < 0.001 || value > 0.1)
          throw std::runtime_error("regular_period_s must be finite and in [0.001,0.1]");
        periodNs = static_cast<int64_t>(value * 1e9);
      }
      if (sdf->HasElement("spike_window_s")) {
        const double value = sdf->Get<double>("spike_window_s");
        if (!std::isfinite(value) || value < 0 || value > 1)
          throw std::runtime_error("spike_window_s must be finite and in [0,1]");
        windowNs = static_cast<int64_t>(value * 1e9);
      }
      if (sdf->HasElement("spike_velocity_rps")) trigger = sdf->Get<double>("spike_velocity_rps");
      if (!std::isfinite(wallDuration) || wallDuration <= 0 || wallDuration > 180 ||
          !std::isfinite(trigger) || trigger <= 0 || periodNs < 1000000 || periodNs > 100000000 ||
          windowNs < 0 || windowNs > 1000000000)
        throw std::runtime_error("invalid bounds: wall (0,180], period [0.001,0.1], window [0,1], trigger >0");
      output = Open(sdf->Get<std::string>("workspace_root"), sdf->Get<std::string>("output"));
      started = std::chrono::steady_clock::now(); world = entity;
      Write({{"kind", "metadata"}, {"observer", "agri::SteeringDiagnosticsV9"},
        {"observer_version", "steering_v9_pre_physics_commands"}, {"read_only_feedback", true},
        {"start_scope", startScope}, {"robot_name", robotName},
        {"wall_duration_s", wallDuration}, {"regular_period_ns", periodNs}, {"spike_window_ns", windowNs},
        {"spike_velocity_rps", trigger}, {"history_max_samples", 5000},
        {"wrench_frames", {{"joint", "arm_joint2 frame, at joint origin"}, {"contacts", "world"}}},
        {"steering_observation", "transmitted wrench is reaction; existing command components are only read. gz-sim8 has no JointPositionCmd component; position references remain ROS controller commands"},
        {"command_sampling", "before_physics_commands copies existing slots at observer PreUpdate and is attached only to the same PostUpdate iteration. All system PreUpdates precede Physics Update. Controller writes must occur before observer PreUpdate to be included; slots can be zero between controller updates. PostUpdate existing command slots are normally cleared by Physics and do not prove command absence. No command slot is created or modified"},
        {"limit_observation", "imported_axis_limits are SDF values, not a native force-limit or applied actuator-effort readback. Pending limit commands are read only if already present; physics normally consumes and clears them"},
        {"contact_freshness", "contact message timestamps are recorded when present; null timestamps or unchanged cached messages do not independently establish current contact force"},
        {"world_vector_freshness", "ECM world velocity/acceleration can retain prior values when link poses stop changing; verify against actual WorldPose differences"},
        {"missing_feedback", "null; empty contact component is unknown until nonempty data was observed"},
        {"ordering", "spike history can be appended retrospectively; sort sample records by iteration"}});
      if (output) std::fflush(output);
    } catch (const std::exception &error) {
      gzerr << "MotionDiagnostics refused configuration: " << error.what() << "\n";
      Finish("configuration_error");
    }
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, Ecm &ecm) override
  {
    beforePhysicsCommands = nullptr;
    if (Expired()) return;
    if (robot == gz::sim::kNullEntity)
      robot = ecm.EntityByComponents(c::Model(), c::Name(robotName), c::ParentEntity(world));
    if (robot == gz::sim::kNullEntity) return;
    if (base == gz::sim::kNullEntity)
      base = ecm.EntityByComponents(c::Link(), c::Name("base_footprint"), c::ParentEntity(robot));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const gz::math::Vector3d unknown(nan, nan, nan);
    bool requested = Request<c::WorldPose>(base, ecm);
    requested |= Request<c::WorldLinearVelocity>(base, ecm, c::WorldLinearVelocity(unknown));
    requested |= Request<c::WorldAngularVelocity>(base, ecm, c::WorldAngularVelocity(unknown));
    requested |= Request<c::WorldLinearAcceleration>(base, ecm, c::WorldLinearAcceleration(unknown));
    requested |= Request<c::WorldAngularAcceleration>(base, ecm, c::WorldAngularAcceleration(unknown));
    const std::vector<std::string> names = {"arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "cr5_joint6",
      "f1_steer_joint", "f2_steer_joint", "r1_steer_joint", "r2_steer_joint", "f1_wheel_joint", "f2_wheel_joint", "r1_wheel_joint", "r2_wheel_joint"};
    for (const auto &name : names) {
      if (!joints.count(name)) {
        const auto entity = ecm.EntityByComponents(c::Joint(), c::Name(name), c::ParentEntity(robot));
        if (entity != gz::sim::kNullEntity) joints[name] = entity;
      }
      if (!joints.count(name)) continue;
      Request<c::JointPosition>(joints[name], ecm); Request<c::JointVelocity>(joints[name], ecm);
      if (name == "arm_joint2" || name.find("_steer_joint") != std::string::npos)
        Request<c::JointTransmittedWrench>(joints[name], ecm);
    }
    for (const std::string name : {"f1_wheel_link", "f2_wheel_link", "r1_wheel_link", "r2_wheel_link"}) {
      const auto link = ecm.EntityByComponents(c::Link(), c::Name(name), c::ParentEntity(robot));
      wheels[name].clear();
      ecm.Each<c::Collision, c::ParentEntity>([&](Entity collision, const c::Collision *, const c::ParentEntity *parent) {
        if (link != gz::sim::kNullEntity && parent->Data() == link) {
          wheels[name].push_back(collision); Request<c::ContactSensorData>(collision, ecm);
        }
        return true;
      });
    }
    if (requested) readyIteration = info.iterations + 2;
    beforePhysicsCommands = ExistingCommands(info, ecm);
  }
  void PostUpdate(const gz::sim::UpdateInfo &info, const Ecm &ecm) override
  {
    if (Expired() || info.paused || Ns(info.dt) <= 0) return;
    const auto ns = Ns(info.simTime);
    if (lastNs >= 0 && ns <= lastNs) {
      Write({{"kind", "clock_discontinuity"}, {"previous_sim_ns", lastNs}, {"sim_ns", ns}});
      history.clear(); lastRegularNs = -1; captureUntil = -1; wasAbove = false;
    }
    lastNs = ns; ++samples;
    auto record = Snapshot(info, ecm);
    if (firstPose && !record["base_world_pose"].is_null()) {
      Write({{"kind", "initial_actual_pose"}, {"sim_ns", ns}, {"base_world_pose", record["base_world_pose"]},
             {"start_scope", startScope}});
      firstPose = false;
    }
    double qd = 0; bool measured = false;
    const auto q2 = joints.find("arm_joint2");
    if (q2 != joints.end()) {
      const auto *velocity = ecm.Component<c::JointVelocity>(q2->second);
      if (velocity && !velocity->Data().empty() && std::isfinite(velocity->Data()[0])) {
        qd = velocity->Data()[0]; measured = true; peak = std::max(peak, std::abs(qd));
      }
    }
    const bool above = measured && std::abs(qd) >= trigger;
    history.push_back({ns, std::move(record)});
    while (!history.empty() && history.front().ns < ns - windowNs) history.pop_front();
    while (history.size() > 5000) { history.pop_front(); ++historyDrops; }
    if (above) {
      captureUntil = ns + windowNs;
      if (!wasAbove) {
        ++events;
        Write({{"kind", "spike_event"}, {"sim_ns", ns}, {"iteration", info.iterations},
               {"arm_joint2_velocity_rps", qd}, {"threshold_rps", trigger}});
        for (auto &entry : history) Emit(entry, "spike_history");
      }
    }
    wasAbove = above;
    const bool regular = lastRegularNs < 0 || ns - lastRegularNs >= periodNs;
    if (ns <= captureUntil) Emit(history.back(), "spike_window");
    else if (regular) Emit(history.back(), "regular");
    if (regular) { lastRegularNs = ns; if (output) std::fflush(output); }
  }
};
}
GZ_ADD_PLUGIN(agri::SteeringDiagnosticsV9, gz::sim::System,
             agri::SteeringDiagnosticsV9::ISystemConfigure, agri::SteeringDiagnosticsV9::ISystemPreUpdate,
             agri::SteeringDiagnosticsV9::ISystemPostUpdate)
// Compatibility for offline fixtures / fresh processes; live loads use V9.
GZ_ADD_PLUGIN_ALIAS(agri::SteeringDiagnosticsV9, "agri::MotionDiagnostics")
GZ_ADD_PLUGIN_ALIAS(agri::SteeringDiagnosticsV9, "agri::SteeringDiagnosticsV9")
