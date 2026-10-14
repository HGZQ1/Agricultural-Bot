// One-shot numerical limit tolerance for the existing physical joint6 servo.
// Never writes position, velocity, effort, reset or feedback components.
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionLimitsCmd.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>

namespace agri {
namespace roundoff_c = gz::sim::components;
using RoundoffJson = nlohmann::json;

class NativeJointLimitRoundoffRecovery : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
  using Clock = std::chrono::steady_clock;
  gz::transport::Node node;
  std::mutex statusMutex;
  std::string statusText{"{\"enabled\":false,\"phase\":\"NOT_CONFIGURED\"}"};
  const std::string robotName{"agri_robot"}, jointName{"cr5_joint6"};
  const double pad{1e-9}, exitMargin{1e-4}, wallLimit{15.0};
  const int64_t simLimitNs{2000000000LL};
  gz::sim::Entity world{gz::sim::kNullEntity}, joint{gz::sim::kNullEntity};
  Clock::time_point started{Clock::now()};
  bool enabled{false}, ready{false}, padded{false}, padConsumed{false};
  bool restoreRequested{false}, restoreIssued{false}, restored{false};
  bool exited{false}, failed{false}, finished{false};
  double lower{0}, upper{0}, initialQ{0}, actualQ{NAN}, actualQd{NAN}, preCommand{NAN};
  uint64_t padIteration{0}, restoreIteration{0};
  int64_t firstSimNs{-1}, lastSimNs{-1};
  std::string phase{"DISABLED"}, reason;

  static int64_t Ns(std::chrono::steady_clock::duration duration)
  { return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count(); }
  double Wall() const
  { return std::chrono::duration<double>(Clock::now()-started).count(); }
  static RoundoffJson Number(double value)
  { return std::isfinite(value) ? RoundoffJson(value) : RoundoffJson(nullptr); }
  void Status()
  {
    RoundoffJson report{{"system", "agri::NativeJointLimitRoundoffRecovery"},
      {"enabled", enabled}, {"robot_name", robotName}, {"joint_name", jointName},
      {"ready", ready}, {"active", padded && !restored && !finished},
      {"phase", phase}, {"reason", reason}, {"success", finished && !failed && exited && restored},
      {"original_limits", ready ? RoundoffJson::array({lower, upper}) : RoundoffJson(nullptr)},
      {"padding_rad", pad}, {"exit_margin_rad", exitMargin},
      {"initial_q_rad", ready ? Number(initialQ) : RoundoffJson(nullptr)},
      {"actual_q_rad", Number(actualQ)}, {"actual_qd_rps", Number(actualQd)},
      {"before_physics_velocity_command_rps", Number(preCommand)},
      {"padding_command_issued", padded}, {"padding_command_consumed", padConsumed},
      {"restore_requested", restoreRequested}, {"restore_command_issued", restoreIssued},
      {"restored", restored}, {"measured_exited_original_lower", exited},
      {"padding_iteration", padded ? RoundoffJson(padIteration) : RoundoffJson(nullptr)},
      {"restore_iteration", restoreIssued ? RoundoffJson(restoreIteration) : RoundoffJson(nullptr)},
      {"wall_elapsed_s", Wall()}, {"wall_timeout_s", wallLimit}, {"sim_timeout_s", 2.0},
      {"last_sim_ns", lastSimNs}, {"no_position_velocity_effort_reset_or_feedback_writes", true},
      {"engine_limit_readback_available", false},
      {"ack_boundary", "consumed means the existing JointPositionLimitsCmd slot was observed empty "
        "after Physics in the exact issuance iteration, not an independent engine-limit getter"}};
    std::lock_guard<std::mutex> guard(statusMutex);statusText = report.dump();
  }
  bool OnStatus(const gz::msgs::StringMsg &request, gz::msgs::StringMsg &response)
  {
    try {
      const auto input = RoundoffJson::parse(request.data());
      if (input.size() != 1 || input.value("op", "") != "status") {
        response.set_data("{\"success\":false,\"reason\":\"status_only\"}");return true;
      }
    } catch (...) {
      response.set_data("{\"success\":false,\"reason\":\"invalid_status_request\"}");return true;
    }
    std::lock_guard<std::mutex> guard(statusMutex);response.set_data(statusText);return true;
  }
  void Fail(const std::string &why)
  {
    failed = true;reason = why;
    if (padded) { restoreRequested = true;phase = "RESTORE_PENDING_FAILURE"; }
    else { finished = true;phase = "FAILED_NO_LIMIT_CHANGE"; }
  }
  bool WriteLimits(gz::sim::EntityComponentManager &ecm, double minimum)
  {
    auto *slot = ecm.Component<roundoff_c::JointPositionLimitsCmd>(joint);
    // Do not overwrite any pending limits command from another writer.
    if (slot && !slot->Data().empty()) return false;
    const std::vector<gz::math::Vector2d> limits{{minimum, upper}};
    if (slot) slot->Data() = limits;
    else ecm.CreateComponent(joint, roundoff_c::JointPositionLimitsCmd(limits));
    return true;
  }
  static bool Consumed(const gz::sim::EntityComponentManager &ecm, gz::sim::Entity entity)
  {
    const auto *slot = ecm.Component<roundoff_c::JointPositionLimitsCmd>(entity);
    return slot && slot->Data().empty();
  }

public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    started = Clock::now();world = entity;
    enabled = sdf && sdf->HasElement("enabled") && sdf->Get<bool>("enabled");
    const auto *worldName = ecm.Component<roundoff_c::Name>(world);
    if (!ecm.Component<roundoff_c::World>(world) || !worldName) {
      enabled = false;reason = "world_entity_required";Status();return;
    }
    const std::string service = "/world/" + worldName->Data() +
      "/native_joint_limit_roundoff_recovery/status";
    if (!node.Advertise(service, &NativeJointLimitRoundoffRecovery::OnStatus, this)) {
      enabled = false;reason = "status_service_advertisement_failed";Status();return;
    }
    if (enabled) phase = "WAITING_FOR_EXISTING_ACTUAL_FEEDBACK";
    Status();
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
  {
    if (!enabled || finished) return;
    const int64_t now = Ns(info.simTime);
    if (firstSimNs < 0) firstSimNs = now;
    if (lastSimNs >= 0 && now < lastSimNs) Fail("clock_backjump");
    lastSimNs = now;
    if (Wall() >= wallLimit || now-firstSimNs >= simLimitNs) Fail("bounded_recovery_timeout");
    if (finished) { Status();return; }
    if (joint == gz::sim::kNullEntity) {
      const auto robot = ecm.EntityByComponents(roundoff_c::Model(),
        roundoff_c::Name(robotName), roundoff_c::ParentEntity(world));
      if (robot != gz::sim::kNullEntity)
        joint = ecm.EntityByComponents(roundoff_c::Joint(),
          roundoff_c::Name(jointName), roundoff_c::ParentEntity(robot));
      if (joint == gz::sim::kNullEntity) { Status();return; }
    }
    if (restoreRequested) {
      if (!restoreIssued && WriteLimits(ecm, lower)) {
        restoreIssued = true;restoreIteration = info.iterations;phase = "RESTORE_COMMAND_ISSUED";
      }
      Status();return;
    }
    if (padded || info.paused) { Status();return; }
    const auto *axis = ecm.Component<roundoff_c::JointAxis>(joint);
    const auto *position = ecm.Component<roundoff_c::JointPosition>(joint);
    const auto *velocity = ecm.Component<roundoff_c::JointVelocity>(joint);
    const auto *command = ecm.Component<roundoff_c::JointVelocityCmd>(joint);
    if (!axis || !position || position->Data().size() != 1 ||
        !velocity || velocity->Data().size() != 1) {
      Fail("existing_actual_joint_feedback_missing");Status();return;
    }
    lower = axis->Data().Lower();upper = axis->Data().Upper();
    actualQ = position->Data()[0];actualQd = velocity->Data()[0];
    preCommand = command && command->Data().size() == 1 ? command->Data()[0] : NAN;
    if (!std::isfinite(lower) || !std::isfinite(upper) ||
        std::abs(lower+3.1416) > 1e-12 || std::abs(upper-3.1416) > 1e-12 ||
        !std::isfinite(actualQ) || !std::isfinite(actualQd) ||
        actualQ != std::nextafter(lower, -std::numeric_limits<double>::infinity()) ||
        !(actualQ < lower && lower-actualQ <= 1e-6)) {
      Fail("not_exact_one_ulp_lower_roundoff");Status();return;
    }
    if (ecm.Component<roundoff_c::JointForceCmd>(joint) ||
        ecm.Component<roundoff_c::JointPositionReset>(joint) ||
        ecm.Component<roundoff_c::JointVelocityReset>(joint)) {
      Fail("foreign_effort_or_reset_component_present");Status();return;
    }
    if (command && !command->Data().empty() &&
        (command->Data().size() != 1 || !std::isfinite(preCommand))) {
      Fail("invalid_existing_original_velocity_command");Status();return;
    }
    if (!command || command->Data().empty() || preCommand <= 0.001) {
      phase = "WAITING_FOR_INWARD_ORIGINAL_SERVO";Status();return;
    }
    initialQ = actualQ;ready = true;
    if (!WriteLimits(ecm, lower-pad)) { Fail("foreign_pending_position_limits");Status();return; }
    padded = true;padIteration = info.iterations;phase = "TEMPORARY_LIMIT_COMMAND_ISSUED";
    gzmsg << "Joint6 numerical-limit recovery: engine minimum padding " << pad
          << " rad only; original servo supplies all motion.\n";
    Status();
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override
  {
    if (!enabled || finished || joint == gz::sim::kNullEntity) return;
    const auto *position = ecm.Component<roundoff_c::JointPosition>(joint);
    const auto *velocity = ecm.Component<roundoff_c::JointVelocity>(joint);
    actualQ = position && position->Data().size() == 1 ? position->Data()[0] : NAN;
    actualQd = velocity && velocity->Data().size() == 1 ? velocity->Data()[0] : NAN;
    if (padded && !info.paused && info.iterations == padIteration)
      padConsumed = Consumed(ecm, joint);
    if (padded && !restoreRequested && padConsumed &&
        std::isfinite(actualQ) && actualQ >= lower+exitMargin && actualQ <= upper) {
      exited = true;restoreRequested = true;phase = "MEASURED_EXIT_RESTORE_PENDING";
    }
    if (restoreIssued && !info.paused && info.iterations == restoreIteration && Consumed(ecm, joint)) {
      restored = true;finished = true;
      if (!exited || !std::isfinite(actualQ) || actualQ < lower+exitMargin || actualQ > upper)
        failed = true;
      phase = failed ? "FAILED_OR_TIMEOUT_LIMIT_COMMAND_RESTORED" : "SUCCESS_LIMIT_COMMAND_RESTORED";
      gzmsg << "Joint6 numerical-limit recovery completed: " << phase << "; actual q=" << actualQ << "\n";
    }
    Status();
  }
};
}  // namespace agri

GZ_ADD_PLUGIN(agri::NativeJointLimitRoundoffRecovery, gz::sim::System,
  agri::NativeJointLimitRoundoffRecovery::ISystemConfigure,
  agri::NativeJointLimitRoundoffRecovery::ISystemPreUpdate,
  agri::NativeJointLimitRoundoffRecovery::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(agri::NativeJointLimitRoundoffRecovery,
                    "agri::NativeJointLimitRoundoffRecovery")
