// Offline component-manager test: no Gazebo world, transport or robot runs.
#include "../src/GripperPlantPassThroughSystem.cc"
#include <gz/sim/EventManager.hh>
#include <cassert>
#include <map>
#include <utility>

namespace agri {
struct GripperPlantPassThroughTestAccess {
  static void Enable(GripperPlantPassThroughSystem &system, gz::sim::Entity world) {
    system.enabled = true; system.world = world;
  }
  static nlohmann::json Status(GripperPlantPassThroughSystem &system) {
    gz::msgs::StringMsg request, response;
    request.set_data("{\"op\":\"status\"}");
    assert(system.Status(request, response));
    return nlohmann::json::parse(response.data());
  }
};
}

int main() {
  namespace c = gz::sim::components;
  using Access = agri::GripperPlantPassThroughTestAccess;
  gz::sim::EntityComponentManager ecm;
  auto child = [&](gz::sim::Entity parent, const std::string &name) {
    const auto id = ecm.CreateEntity(); ecm.SetParentEntity(id, parent);
    ecm.CreateComponent(id, c::ParentEntity(parent));
    ecm.CreateComponent(id, c::Name(name));
    return id;
  };
  const auto world = ecm.CreateEntity(); ecm.CreateComponent(world, c::World());
  auto model = [&](gz::sim::Entity parent, const std::string &name) {
    const auto id = child(parent, name); ecm.CreateComponent(id, c::Model()); return id;
  };
  auto collision = [&](gz::sim::Entity owner, const std::string &name) {
    const auto link = child(owner, name); ecm.CreateComponent(link, c::Link());
    const auto id = child(link, "collision"); ecm.CreateComponent(id, c::Collision()); return id;
  };
  const auto robot = model(world, "agri_robot");
  std::map<std::string, gz::sim::Entity> ids;
  for (const auto &link : {"gripper_base_link", "gripper_left_finger_link",
        "gripper_right_finger_link", "arm_link1", "tomato_box_link", "base_link"})
    ids[link] = collision(robot, link);
  ids["plant"] = collision(model(world, "tomato_20"), "link");
  ids["fruit"] = collision(model(world, "tomato_20_fruit_2"), "fruit_link");
  ids["floor"] = collision(model(world, "ground_plane"), "ground_link");
  ids["unrelated_model"] = collision(model(world, "wooden_post"), "link");
  ids["other_robot"] = collision(model(world, "different_robot"), "gripper_left_finger_link");
  const auto otherWorld = ecm.CreateEntity(); ecm.CreateComponent(otherWorld, c::World());
  ids["other_world_plant"] = collision(model(otherWorld, "tomato_21"), "link");

  auto step = [&](agri::GripperPlantPassThroughSystem &system, bool paused = false) {
    gz::sim::UpdateInfo info; info.paused = paused; system.PreUpdate(info, ecm);
  };
  agri::GripperPlantPassThroughSystem disabled;
  step(disabled);
  for (const auto &[_, id] : ids) assert(!ecm.Component<c::CategoryBitmaskCmd>(id));
  agri::GripperPlantPassThroughSystem system; Access::Enable(system, world);
  step(system, true);
  for (const auto &[_, id] : ids) assert(!ecm.Component<c::CategoryBitmaskCmd>(id));
  step(system);
  auto status = Access::Status(system);
  assert(status["ready"] == true && status["plant_collisions"] == 1);
  assert(status["gripper_collisions"] == 3);
  for (const auto &[name, id] : ids) {
    const bool expected = name == "plant" || name.find("gripper_") == 0;
    assert(bool(ecm.Component<c::CategoryBitmaskCmd>(id)) == expected);
    assert(bool(ecm.Component<c::CollideBitmaskCmd>(id)) == expected);
  }

  // Compose the existing independent fruit/arm policy with this new pair
  // exemption. Every other native category defaults to 0xffff.
  ecm.SetComponentData<c::CategoryBitmaskCmd>(ids["fruit"], uint16_t{2});
  ecm.SetComponentData<c::CollideBitmaskCmd>(ids["fruit"], uint16_t{2});
  ecm.SetComponentData<c::CategoryBitmaskCmd>(ids["arm_link1"], uint16_t{1});
  ecm.SetComponentData<c::CollideBitmaskCmd>(ids["arm_link1"], uint16_t{1});
  auto bits = [&](const std::string &name) {
    const auto id = ids.at(name);
    const auto *category = ecm.Component<c::CategoryBitmaskCmd>(id);
    const auto *collide = ecm.Component<c::CollideBitmaskCmd>(id);
    return std::pair<uint16_t, uint16_t>{category ? category->Data() : 0xffff,
                                       collide ? collide->Data() : 0xffff};
  };
  auto contacts = [&](const std::string &first, const std::string &second) {
    const auto a = bits(first), b = bits(second);
    return (a.first & b.second) != 0 || (b.first & a.second) != 0;
  };
  assert(!contacts("fruit", "arm_link1"));
  for (const auto &gripper : {"gripper_base_link", "gripper_left_finger_link",
                             "gripper_right_finger_link"}) {
    assert(!contacts("plant", gripper));
    for (const auto &protectedBody : {"fruit", "floor", "arm_link1", "base_link",
          "tomato_box_link", "unrelated_model", "other_robot", "other_world_plant"})
      assert(contacts(protectedBody, gripper));
  }
  for (const auto &protectedBody : {"fruit", "floor", "arm_link1", "base_link",
                                   "tomato_box_link", "unrelated_model", "other_robot"})
    assert(contacts("plant", protectedBody));

  // Later model creation is also filtered; neither old fruit/arm category is
  // overwritten by the new filter on subsequent updates.
  ids["new_plant"] = collision(model(world, "tomato_21"), "link");
  step(system); status = Access::Status(system);
  assert(status["plant_collisions"] == 2 && status["ready"] == true);
  assert(!contacts("new_plant", "gripper_left_finger_link"));
  assert(bits("fruit").first == 2 && bits("arm_link1").first == 1);
}
