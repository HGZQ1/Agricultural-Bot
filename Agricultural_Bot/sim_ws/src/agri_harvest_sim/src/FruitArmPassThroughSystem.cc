// Explicit demo collision filtering. Geometry, joint feedback and grasp
// evidence are untouched; fruit still contacts gripper, base, ground and basket.
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/CollisionBitmask.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <nlohmann/json.hpp>
#include <chrono>
#include <memory>
#include <mutex>
#include <regex>
#include <set>
#include <string>

namespace agri {
namespace fc = gz::sim::components;
class FruitArmPassThroughSystem : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
  gz::sim::Entity world{gz::sim::kNullEntity};
  std::string robotName{"agri_robot"};
  bool enabled{false};
  std::unique_ptr<gz::transport::Node> node;
  std::mutex statusMutex;
  std::string statusText{"{\"ready\":false}"};
  std::set<gz::sim::Entity> applied;
  const std::set<std::string> armLinks{
      "arm_link1", "arm_link2", "arm_link3", "arm_link4", "arm_link5"};
  const std::regex fruitName{"tomato_[0-9]+_fruit_[0-9]+"};
  bool Status(const gz::msgs::StringMsg &request, gz::msgs::StringMsg &response)
  {
    const auto input = nlohmann::json::parse(request.data(), nullptr, false);
    if (input.is_discarded() || input.value("op", std::string()) != "status") {
      response.set_data("{\"success\":false,\"reason\":\"status required\"}");
      return true;
    }
    std::lock_guard<std::mutex> lock(statusMutex);
    response.set_data(statusText); return true;
  }
public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    if (!sdf || !sdf->HasElement("enabled") || !sdf->Get<bool>("enabled") ||
        !ecm.Component<fc::World>(entity)) return;
    world = entity;
    if (sdf->HasElement("robot_name")) robotName = sdf->Get<std::string>("robot_name");
    const auto *name = ecm.Component<fc::Name>(world);
    if (!name || robotName.empty()) return;
    node = std::make_unique<gz::transport::Node>();
    enabled = node->Advertise("/world/" + name->Data() +
        "/fruit_arm_pass_through/status", &FruitArmPassThroughSystem::Status, this);
    if (enabled) gzmsg << "Demo fruit/arm pass-through enabled; gripper contacts preserved.\n";
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (!enabled || info.paused) return;
    size_t fruits = 0, arms = 0;
    ecm.Each<fc::Collision, fc::ParentEntity>(
        [&](const gz::sim::Entity &collision, const fc::Collision *,
            const fc::ParentEntity *parent) {
      const auto link = parent->Data();
      const auto *linkName = ecm.Component<fc::Name>(link);
      const auto *modelParent = ecm.Component<fc::ParentEntity>(link);
      if (!ecm.Component<fc::Link>(link) || !linkName || !modelParent) return true;
      const auto model = modelParent->Data();
      const auto *modelName = ecm.Component<fc::Name>(model);
      const auto *worldParent = ecm.Component<fc::ParentEntity>(model);
      if (!ecm.Component<fc::Model>(model) || !modelName || !worldParent ||
          worldParent->Data() != world) return true;
      uint16_t mask = 0;
      if (std::regex_match(modelName->Data(), fruitName)) { mask = 0x0002; ++fruits; }
      else if (modelName->Data() == robotName && armLinks.count(linkName->Data())) {
        mask = 0x0001; ++arms;
      }
      if (mask && !applied.count(collision)) {
        // Both category and collide are needed for the native OR filter rule.
        ecm.SetComponentData<fc::CategoryBitmaskCmd>(collision, mask);
        ecm.SetComponentData<fc::CollideBitmaskCmd>(collision, mask);
        applied.insert(collision);
      }
      return true;
    });
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    nlohmann::json status{{"success", true}, {"enabled", true},
      {"system", "agri::FruitArmPassThroughSystem"}, {"sim_ns", ns},
      {"fruit_collisions", fruits}, {"arm_collisions", arms},
      {"ready", fruits > 0 && arms >= armLinks.size()},
      {"fruit_mask", 2}, {"arm_mask", 1}, {"arm_links", armLinks},
      {"gripper_contacts_preserved", true}};
    std::lock_guard<std::mutex> lock(statusMutex); statusText = status.dump();
  }
};
}
GZ_ADD_PLUGIN(agri::FruitArmPassThroughSystem, gz::sim::System,
              agri::FruitArmPassThroughSystem::ISystemConfigure,
              agri::FruitArmPassThroughSystem::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(agri::FruitArmPassThroughSystem, "agri::FruitArmPassThroughSystem")
