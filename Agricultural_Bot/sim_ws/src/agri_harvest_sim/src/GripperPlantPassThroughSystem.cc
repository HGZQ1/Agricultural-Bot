// The current plants have visual-only leaves and one narrow stem proxy box.
// Exempt only that plant body / gripper pair. Fruit, basket, floor and chassis
// contact remain enabled, including the native evidence used by HarvestSystem.
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
#include <cstdint>
#include <memory>
#include <mutex>
#include <regex>
#include <set>
#include <string>

namespace agri {
namespace gpc = gz::sim::components;
class GripperPlantPassThroughSystem : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
  friend struct GripperPlantPassThroughTestAccess;
  gz::sim::Entity world{gz::sim::kNullEntity};
  std::string robotName{"agri_robot"};
  bool enabled{false};
  std::unique_ptr<gz::transport::Node> node;
  std::mutex statusMutex;
  std::string statusText{"{\"ready\":false}"};
  std::set<gz::sim::Entity> applied;
  const std::set<std::string> gripperLinks{
      "gripper_base_link", "gripper_left_finger_link", "gripper_right_finger_link"};
  const std::regex plantName{"tomato_[0-9]+"};
  // FruitArmPassThroughSystem already owns category bits 0x1 (arm) and 0x2
  // (fruit). Use separate bits so both independent policies compose.
  static constexpr uint16_t plantCategory = 0x0004;
  static constexpr uint16_t gripperCategory = 0x0008;
  static constexpr uint16_t plantCollide = 0xfff7;
  static constexpr uint16_t gripperCollide = 0xfffb;

  bool Status(const gz::msgs::StringMsg &request, gz::msgs::StringMsg &response) {
    const auto input = nlohmann::json::parse(request.data(), nullptr, false);
    if (!input.is_object() || !input.contains("op") || !input["op"].is_string() ||
        input["op"].get<std::string>() != "status") {
      response.set_data("{\"success\":false,\"reason\":\"status required\"}");
      return true;
    }
    std::lock_guard<std::mutex> lock(statusMutex);
    response.set_data(statusText);
    return true;
  }

public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    if (!sdf || !sdf->HasElement("enabled") || !sdf->Get<bool>("enabled") ||
        !ecm.Component<gpc::World>(entity)) return;
    world = entity;
    if (sdf->HasElement("robot_name")) robotName = sdf->Get<std::string>("robot_name");
    const auto *name = ecm.Component<gpc::Name>(world);
    if (!name || robotName.empty()) return;
    node = std::make_unique<gz::transport::Node>();
    enabled = node->Advertise("/world/" + name->Data() +
        "/gripper_plant_pass_through/status", &GripperPlantPassThroughSystem::Status, this);
    if (enabled) gzmsg << "Demo gripper/plant-body pass-through enabled; fruit contacts preserved.\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override {
    if (!enabled || info.paused) return;
    size_t plants = 0, grippers = 0;
    std::set<std::string> foundGripperLinks;
    ecm.Each<gpc::Collision, gpc::ParentEntity>(
        [&](const gz::sim::Entity &collision, const gpc::Collision *,
            const gpc::ParentEntity *parent) {
      const auto link = parent->Data();
      const auto *linkName = ecm.Component<gpc::Name>(link);
      const auto *modelParent = ecm.Component<gpc::ParentEntity>(link);
      if (!ecm.Component<gpc::Link>(link) || !linkName || !modelParent) return true;
      const auto model = modelParent->Data();
      const auto *modelName = ecm.Component<gpc::Name>(model);
      const auto *worldParent = ecm.Component<gpc::ParentEntity>(model);
      if (!ecm.Component<gpc::Model>(model) || !modelName || !worldParent ||
          worldParent->Data() != world) return true;
      uint16_t category = 0, collide = 0;
      if (std::regex_match(modelName->Data(), plantName)) {
        category = plantCategory; collide = plantCollide; ++plants;
      } else if (modelName->Data() == robotName && gripperLinks.count(linkName->Data())) {
        category = gripperCategory; collide = gripperCollide; ++grippers;
        foundGripperLinks.insert(linkName->Data());
      }
      if (category && !applied.count(collision)) {
        // Gazebo's native OR rule needs BOTH directions disabled for a pair:
        // (category_A & collide_B) || (category_B & collide_A).
        ecm.SetComponentData<gpc::CategoryBitmaskCmd>(collision, category);
        ecm.SetComponentData<gpc::CollideBitmaskCmd>(collision, collide);
        applied.insert(collision);
      }
      return true;
    });
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    nlohmann::json status{{"success", true}, {"enabled", true},
      {"system", "agri::GripperPlantPassThroughSystem"}, {"sim_ns", ns},
      {"plant_collisions", plants}, {"gripper_collisions", grippers},
      {"ready", plants > 0 && foundGripperLinks == gripperLinks},
      {"plant_category", plantCategory}, {"plant_collide", plantCollide},
      {"gripper_category", gripperCategory}, {"gripper_collide", gripperCollide},
      {"gripper_links", gripperLinks}, {"plant_models", "tomato_[0-9]+"},
      {"plant_collision_scope", "top_level_tomato_body_only"},
      {"fruit_contacts_preserved", true}, {"basket_chassis_ground_contacts_preserved", true}};
    std::lock_guard<std::mutex> lock(statusMutex);
    statusText = status.dump();
  }
};
}
GZ_ADD_PLUGIN(agri::GripperPlantPassThroughSystem, gz::sim::System,
    agri::GripperPlantPassThroughSystem::ISystemConfigure,
    agri::GripperPlantPassThroughSystem::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(agri::GripperPlantPassThroughSystem, "agri::GripperPlantPassThroughSystem")
