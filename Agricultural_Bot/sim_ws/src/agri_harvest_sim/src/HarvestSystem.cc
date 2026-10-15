// Physical interaction only. This system never publishes perception targets.
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <sdf/Sphere.hh>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include "GripEvidence.hh"
#include "BasketEvidence.hh"

using Json = nlohmann::json;
namespace agri {
class HarvestSystem : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
  struct Fruit {
    std::string name, plant, state{"ON_PLANT"}, work, target;
    double radius{}, releasedAt{-1}, gripStableSince{-1}; bool ripe{};
    gz::sim::Entity link{gz::sim::kNullEntity}, joint{gz::sim::kNullEntity};
    gz::math::Vector3d previousPosition; double previousTime{-1}, speed{1e9};
    gz::math::Vector3d attachedLocal;
    BasketEvidence basket;
  };
  struct Pending { Json input, output; bool done{false}, cancelled{false};
    std::mutex mutex; std::condition_variable cv; };
  gz::transport::Node transport;
  std::mutex queueMutex; std::deque<std::shared_ptr<Pending>> queue;
  std::vector<Fruit> fruits;
  std::string robotName{"agri_robot"}, service;
  gz::sim::Entity world{gz::sim::kNullEntity};
  gz::math::Vector3d tcpOffset{-0.18, 0, 0};
  double simTime{0};

  gz::sim::Entity ModelNamed(gz::sim::EntityComponentManager &ecm,
                             const std::string &name) const {
    return ecm.EntityByComponents(gz::sim::components::Model(),
       gz::sim::components::Name(name), gz::sim::components::ParentEntity(world));
  }
  gz::sim::Entity LinkNamed(gz::sim::EntityComponentManager &ecm,
                            gz::sim::Entity model, const std::string &name) const {
    return ecm.EntityByComponents(gz::sim::components::Link(),
      gz::sim::components::Name(name), gz::sim::components::ParentEntity(model));
  }
  Json PoseJson(const gz::math::Pose3d &pose) const {
    return {{"xyz", {pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()}},
      {"xyzw", {pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W()}}};
  }
  gz::math::Pose3d BasePose(gz::sim::EntityComponentManager &ecm,
                           gz::sim::Entity robot) const {
    auto footprint = LinkNamed(ecm, robot, "base_footprint");
    return gz::sim::worldPose(footprint != gz::sim::kNullEntity ? footprint : robot, ecm);
  }
  gz::math::Pose3d BasketCadPose(gz::sim::EntityComponentManager &ecm,
                               gz::sim::Entity robot) const {
    auto base = LinkNamed(ecm, robot, "base_link");
    if (base != gz::sim::kNullEntity) return gz::sim::worldPose(base, ecm);
    // URDF->SDF fixed-joint reduction folds CAD base_link into base_footprint.
    // Recover the documented rigid transform instead of requiring a removed Link.
    return BasePose(ecm, robot) * gz::math::Pose3d(0, 0, 0, M_PI/2, 0, M_PI/2);
  }
  double Finger(gz::sim::EntityComponentManager &ecm, gz::sim::Entity model,
                 const std::string &name) const {
    auto joint = ecm.EntityByComponents(gz::sim::components::Name(name),
      gz::sim::components::ParentEntity(model));
    auto position = ecm.Component<gz::sim::components::JointPosition>(joint);
    return position && !position->Data().empty() ? position->Data()[0] : NAN;
  }
  double FingerVelocity(gz::sim::EntityComponentManager &ecm, gz::sim::Entity model,
                         const std::string &name) const {
    auto joint = ecm.EntityByComponents(gz::sim::components::Name(name),
      gz::sim::components::ParentEntity(model));
    auto velocity = ecm.Component<gz::sim::components::JointVelocity>(joint);
    return velocity && !velocity->Data().empty() ? velocity->Data()[0] : NAN;
  }
  gz::sim::Entity CollisionFor(gz::sim::EntityComponentManager &ecm,
                               gz::sim::Entity link) const {
    if (link == gz::sim::kNullEntity) return gz::sim::kNullEntity;
    return ecm.EntityByComponents(gz::sim::components::Collision(),
      gz::sim::components::ParentEntity(link));
  }
  void RequestContacts(gz::sim::EntityComponentManager &ecm, gz::sim::Entity collision) {
    if (collision != gz::sim::kNullEntity &&
        !ecm.Component<gz::sim::components::ContactSensorData>(collision))
      ecm.CreateComponent(collision, gz::sim::components::ContactSensorData());
  }
  GripEvidence Grip(gz::sim::EntityComponentManager &ecm, const Fruit &fruit) {
    GripEvidence evidence;
    auto robot = ModelNamed(ecm, robotName);
    auto hand = LinkNamed(ecm, robot, "gripper_base_link");
    evidence.leftPosition = Finger(ecm, robot, "gripper_left_finger_joint");
    evidence.rightPosition = Finger(ecm, robot, "gripper_right_finger_joint");
    evidence.leftVelocity = FingerVelocity(ecm, robot, "gripper_left_finger_joint");
    evidence.rightVelocity = FingerVelocity(ecm, robot, "gripper_right_finger_joint");
    if (hand == gz::sim::kNullEntity || fruit.link == gz::sim::kNullEntity) return evidence;
    auto handPose = gz::sim::worldPose(hand, ecm);
    auto centre = handPose.Inverse().CoordPositionAdd(gz::sim::worldPose(fruit.link, ecm).Pos());
    evidence.centre = {centre.X(), centre.Y(), centre.Z()};
    evidence.tcpDistance = (centre - tcpOffset).Length();
    evidence.stableSeconds = fruit.gripStableSince < 0 ? 0 : simTime - fruit.gripStableSince;
    auto left = CollisionFor(ecm, LinkNamed(ecm, robot, "gripper_left_finger_link"));
    auto right = CollisionFor(ecm, LinkNamed(ecm, robot, "gripper_right_finger_link"));
    auto target = CollisionFor(ecm, fruit.link);
    auto leftData = ecm.Component<gz::sim::components::ContactSensorData>(left);
    auto rightData = ecm.Component<gz::sim::components::ContactSensorData>(right);
    auto targetData = ecm.Component<gz::sim::components::ContactSensorData>(target);
    auto geometry = ecm.Component<gz::sim::components::Geometry>(target);
    evidence.sensorsAvailable = leftData && rightData && targetData && geometry &&
      geometry->Data().SphereShape() &&
      std::abs(geometry->Data().SphereShape()->Radius() - fruit.radius) < 1e-9;
    evidence.radius = evidence.sensorsAvailable ? geometry->Data().SphereShape()->Radius() : fruit.radius;
    if (!evidence.sensorsAvailable) return evidence;
    auto readFinger = [&](const auto &data, gz::sim::Entity finger,
                          std::vector<GripContactPoint> &points) {
      for (const auto &contact : data.contact()) {
        const auto first = contact.collision1().id(), second = contact.collision2().id();
        if (!((first == finger && second == target) || (second == finger && first == target))) {
          evidence.unexpectedContact = true; continue;
        }
        for (int index = 0; index < contact.position_size(); ++index) {
          const auto &point = contact.position(index);
          auto local = handPose.Inverse().CoordPositionAdd({point.x(), point.y(), point.z()});
          points.push_back({{local.X(), local.Y(), local.Z()},
            index < contact.depth_size() ? contact.depth(index) : NAN});
        }
      }
    };
    readFinger(leftData->Data(), left, evidence.left);
    readFinger(rightData->Data(), right, evidence.right);
    for (const auto &contact : targetData->Data().contact()) {
      auto first = contact.collision1().id(), second = contact.collision2().id();
      if (!((first == target && (second == left || second == right)) ||
            (second == target && (first == left || first == right))))
        evidence.unexpectedContact = true;
    }
    return evidence;
  }
  Json GripJson(gz::sim::EntityComponentManager &ecm, const Fruit &fruit,
                 const GripEvidence &evidence, const GripAssessment &assessment) {
    auto robot = ModelNamed(ecm, robotName);
    auto collision = [&](const char *name) { return CollisionFor(ecm, LinkNamed(ecm, robot, name)); };
    Json points = {{"left", Json::array()}, {"right", Json::array()}};
    for (const auto &point : evidence.left)
      points["left"].push_back({{"position_hand", point.position}, {"penetration_m", point.depth}});
    for (const auto &point : evidence.right)
      points["right"].push_back({{"position_hand", point.position}, {"penetration_m", point.depth}});
    return {{"entity_id", fruit.name}, {"state", assessment.state},
      {"left_contact", assessment.left}, {"right_contact", assessment.right},
      {"finger_positions", {evidence.leftPosition, evidence.rightPosition}},
      {"finger_velocities", {evidence.leftVelocity, evidence.rightVelocity}},
      {"stable_s", evidence.stableSeconds}, {"centre_hand", evidence.centre},
      {"tcp_distance_m", evidence.tcpDistance}, {"radius_m", evidence.radius},
      {"sensors_available", evidence.sensorsAvailable}, {"unexpected_contact", evidence.unexpectedContact},
      {"collision_ids", {collision("gripper_left_finger_link"),
                         collision("gripper_right_finger_link"), CollisionFor(ecm, fruit.link)}},
      {"actual_contact_points", points}, {"reason", assessment.reason}};
  }
  gz::sim::Entity Attach(gz::sim::EntityComponentManager &ecm,
                          gz::sim::Entity parent, gz::sim::Entity child) {
    auto joint = ecm.CreateEntity();
    ecm.CreateComponent(joint, gz::sim::components::DetachableJoint({parent, child, "fixed"}));
    return joint;
  }
  Json Failure(const std::string &message) const {
    return {{"success", false}, {"state", "UNAVAILABLE"}, {"entity_id", ""}, {"message", message}};
  }
  Json FruitResult(const Fruit &fruit, bool success, const std::string &message) const {
    return {{"success", success}, {"state", fruit.state},
      {"entity_id", fruit.name}, {"message", message}};
  }
  Json BasketJson(const Fruit &fruit) const {
    Json result = {{"entity_id", fruit.name}, {"state", fruit.state},
      {"sim_time", simTime}, {"basket_position_cad_m", fruit.basket.position},
      {"radius_m", fruit.radius},
      {"basket_criterion", "ideal_lower_half_and_10cm_below_contact"},
      {"basket_lateral_bounds_cad_x_m", {-kBasketInnerHalfWidthX, kBasketInnerHalfWidthX}},
      {"basket_lateral_clearance_m", kBasketInnerHalfWidthX -
        std::abs(fruit.basket.position[0]) - fruit.radius},
      {"basket_vertical_bounds_cad_y_m", {kBasketRegionMinimumY, kBasketRegionMaximumY}},
      {"basket_longitudinal_bounds_cad_z_m", {kBasketRegionMinimumZ, kBasketRegionMaximumZ}},
      {"basket_floor_clearance_m", fruit.basket.position[1] - fruit.radius -
        kBasketFloorMinimumY},
      {"basket_relative_speed_m_s", fruit.basket.relativeSpeed},
      {"basket_inside", fruit.basket.inside},
      {"basket_region_contact", fruit.basket.inside},
      {"basket_fully_inside", BasketContainsSphere(fruit.basket.position, fruit.radius)},
      {"basket_stable_s", fruit.basket.stableSeconds},
      {"world_speed_m_s", fruit.speed}, {"released_elapsed_sim_s", nullptr}};
    if (fruit.releasedAt >= 0)
      result["released_elapsed_sim_s"] = std::max(0.0, simTime - fruit.releasedAt);
    return result;
  }
  Json Snapshot(gz::sim::EntityComponentManager &ecm) {
    Json result = {{"success", true}, {"sim_time", simTime}, {"fruits", Json::array()},
      {"grip_diagnostics", Json::array()}};
    auto robot = ModelNamed(ecm, robotName);
    if (robot == gz::sim::kNullEntity) return Failure("robot model is absent");
    result["base_pose"] = PoseJson(BasePose(ecm, robot));
    result["finger_positions"] = {Finger(ecm, robot, "gripper_left_finger_joint"),
                                   Finger(ecm, robot, "gripper_right_finger_joint")};
    result["finger_velocities"] = {FingerVelocity(ecm, robot, "gripper_left_finger_joint"),
                                   FingerVelocity(ecm, robot, "gripper_right_finger_joint")};
    auto hand = LinkNamed(ecm, robot, "gripper_base_link");
    if (hand != gz::sim::kNullEntity)
      result["hand_pose"] = PoseJson(gz::sim::worldPose(hand, ecm));
    for (const auto &fruit : fruits) {
      if (fruit.link == gz::sim::kNullEntity) continue;
      auto item = PoseJson(gz::sim::worldPose(fruit.link, ecm));
      item.update({{"entity_id", fruit.name}, {"plant_id", fruit.plant},
        {"radius", fruit.radius}, {"ripe", fruit.ripe}, {"state", fruit.state},
        {"work_id", fruit.work}, {"target_id", fruit.target}, {"speed", fruit.speed},
        {"basket_diagnostics", BasketJson(fruit)}});
      result["fruits"].push_back(item);
      if (hand != gz::sim::kNullEntity &&
          (gz::sim::worldPose(hand, ecm).CoordPositionAdd(tcpOffset) -
           gz::sim::worldPose(fruit.link, ecm).Pos()).Length() <= fruit.radius + 0.05) {
        auto evidence = Grip(ecm, fruit);
        result["grip_diagnostics"].push_back(GripJson(ecm, fruit, evidence, AssessGrip(evidence)));
      }
    }
    return result;
  }
  Json Process(const Json &input, gz::sim::EntityComponentManager &ecm) {
    const auto operation = input.value("operation", std::string());
    if (operation == "scene") return Snapshot(ecm);
    const auto work = input.value("work_id", std::string());
    const auto target = input.value("target_id", std::string());
    const auto plant = input.value("plant_id", std::string());
    if (work.empty() || target.empty() || plant.empty()) return Failure("observation/work identity is required");
    Fruit *bound = nullptr;
    for (auto &fruit : fruits)
      if (fruit.work == work && fruit.target == target && fruit.plant == plant) bound = &fruit;
    if (operation == "status") {
      if (!bound) return Failure("no fruit bound to this observation/work");
      auto diagnostic = BasketJson(*bound);
      diagnostic["message"] = "ideal basket-region contact state";
      return FruitResult(*bound, true, diagnostic.dump());
    }
    auto robot = ModelNamed(ecm, robotName);
    auto hand = LinkNamed(ecm, robot, "gripper_base_link");
    if (robot == gz::sim::kNullEntity || hand == gz::sim::kNullEntity)
      return Failure("robot/gripper link is absent");
    const auto left = Finger(ecm, robot, "gripper_left_finger_joint");
    const auto right = Finger(ecm, robot, "gripper_right_finger_joint");
    if (!std::isfinite(left) || !std::isfinite(right)) return Failure("measured finger positions are unavailable");
    if (operation == "release") {
      if (!bound || bound->state != "ATTACHED") return Failure("matching fruit is not attached");
      if (std::abs(left) > 0.002 || std::abs(right) > 0.002)
        return FruitResult(*bound, false, "release requires measured open fingers");
      // CAD basket coordinates: X lateral, Y up, Z forward. Do not use map/world alignment.
      auto basketPoint = BasketCadPose(ecm, robot).Inverse().CoordPositionAdd(
        gz::sim::worldPose(bound->link, ecm).Pos());
      if (std::abs(basketPoint.X()) + bound->radius > kBasketInnerHalfWidthX ||
          basketPoint.Z() - bound->radius < kBasketRegionMinimumZ ||
          basketPoint.Z() + bound->radius > kBasketRegionMaximumZ ||
          basketPoint.Y() < 0.46 || basketPoint.Y() > 0.80)
        return FruitResult(*bound, false, "fruit is outside the basket release volume");
      ecm.RequestRemoveEntity(bound->joint); bound->joint = gz::sim::kNullEntity;
      bound->state = "RELEASED"; bound->releasedAt = simTime;
      bound->basket = BasketEvidence();
      bound->basket.Observe({basketPoint.X(), basketPoint.Y(), basketPoint.Z()},
                            bound->radius, simTime, true);
      return FruitResult(*bound, true, "detached; basket outcome still pending");
    }
    if (operation != "attach" && operation != "check_grip") return Failure("unknown operation");
    if (!input.contains("expected_entity_id") || input.at("expected_entity_id").get<std::string>().empty())
      return Failure("matching contact-selected expected entity is required");
    if (bound && operation == "attach")
      return FruitResult(*bound, bound->state == "ATTACHED", "idempotent observation/work query");
    for (const auto &fruit : fruits)
      if (fruit.state == "ATTACHED") return Failure("another fruit is already attached");
    auto xyz = input.at("position").get<std::vector<double>>();
    if (xyz.size() != 3 || !std::all_of(xyz.begin(), xyz.end(), [](double x){return std::isfinite(x);}))
      return Failure("finite position in base_footprint is required");
    auto observedWorld = BasePose(ecm, robot).CoordPositionAdd({xyz[0], xyz[1], xyz[2]});
    std::vector<std::pair<double, Fruit *>> matches;
    for (auto &fruit : fruits) {
      if (fruit.plant != plant || !fruit.ripe || fruit.state != "ON_PLANT" ||
          fruit.link == gz::sim::kNullEntity) continue;
      double gap = (gz::sim::worldPose(fruit.link, ecm).Pos() - observedWorld).Length();
      if (gap <= fruit.radius + 0.03) matches.emplace_back(gap, &fruit);
    }
    std::sort(matches.begin(), matches.end(), [](const auto &a, const auto &b){return a.first < b.first;});
    if (matches.empty()) return Failure("visual surface observation does not match an unpicked ripe fruit");
    if (matches.size() > 1 && matches[1].first - matches[0].first < 0.005)
      return Failure("ambiguous nearby fruit; need a better visual observation");
    auto &fruit = *matches.front().second;
    if (input.contains("expected_entity_id") && input.at("expected_entity_id") != fruit.name)
      return Failure("visual association changed after contact selection");
    auto evidence = Grip(ecm, fruit);
    auto assessment = AssessGrip(evidence);
    auto diagnostic = GripJson(ecm, fruit, evidence, assessment);
    if (operation == "check_grip")
      return {{"success", assessment.valid}, {"state", assessment.state},
        {"entity_id", fruit.name}, {"message", diagnostic.dump()}};
    if (!assessment.valid || !assessment.settled)
      return FruitResult(fruit, false, "strict actual contact attachment check failed: " + diagnostic.dump());
    ecm.RequestRemoveEntity(fruit.joint);
    fruit.joint = Attach(ecm, hand, fruit.link);
    fruit.attachedLocal = gz::sim::worldPose(hand, ecm).Inverse().CoordPositionAdd(
      gz::sim::worldPose(fruit.link, ecm).Pos());
    fruit.work = work; fruit.target = target; fruit.state = "ATTACHED";
    return FruitResult(fruit, true, "stem joint removed and gripper fixed joint created");
  }
 public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
     gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    world = entity;
    auto settings = sdf->FindElement("config")->Get<std::string>();
    auto config = Json::parse(settings);
    robotName = config.value("robot_model", "agri_robot");
    auto offset = config.value("tcp_offset", std::vector<double>{-0.18, 0, 0});
    tcpOffset.Set(offset.at(0), offset.at(1), offset.at(2));
    for (const auto &item : config.at("fruits")) {
      Fruit fruit; fruit.name = item.at("id"); fruit.plant = item.at("plant_id");
      fruit.radius = item.at("radius"); fruit.ripe = item.at("ripe"); fruits.push_back(fruit);
    }
    service = config.value("service", "/world/field/harvest/command");
    transport.Advertise(service, &HarvestSystem::Command, this);
  }
  bool Command(const gz::msgs::StringMsg &request, gz::msgs::StringMsg &reply) {
    auto pending = std::make_shared<Pending>();
    try { pending->input = Json::parse(request.data()); }
    catch (...) { reply.set_data(Failure("invalid command JSON").dump()); return true; }
    { std::lock_guard<std::mutex> guard(queueMutex); queue.push_back(pending); }
    std::unique_lock<std::mutex> lock(pending->mutex);
    if (!pending->cv.wait_for(lock, std::chrono::seconds(2), [&]{return pending->done;})) {
      pending->cancelled = true;
      reply.set_data(Failure("simulation did not process command before deadline").dump());
    } else reply.set_data(pending->output.dump());
    return true;
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    simTime = std::chrono::duration<double>(info.simTime).count();
    // Physics produces joint measurements only when a consumer requests the components.
    auto robot = ModelNamed(ecm, robotName);
    if (robot != gz::sim::kNullEntity) {
      for (const auto &name : {"gripper_left_finger_joint", "gripper_right_finger_joint"}) {
        auto joint = ecm.EntityByComponents(gz::sim::components::Name(name),
          gz::sim::components::ParentEntity(robot));
        if (joint != gz::sim::kNullEntity && !ecm.Component<gz::sim::components::JointPosition>(joint))
          ecm.CreateComponent(joint, gz::sim::components::JointPosition());
        if (joint != gz::sim::kNullEntity && !ecm.Component<gz::sim::components::JointVelocity>(joint))
          ecm.CreateComponent(joint, gz::sim::components::JointVelocity());
      }
      for (const auto &name : {"gripper_left_finger_link", "gripper_right_finger_link"})
        RequestContacts(ecm, CollisionFor(ecm, LinkNamed(ecm, robot, name)));
    }
    auto handEntity = robot == gz::sim::kNullEntity ? gz::sim::kNullEntity :
      LinkNamed(ecm, robot, "gripper_base_link");
    auto currentTcp = handEntity == gz::sim::kNullEntity ? gz::math::Vector3d::Zero :
      gz::sim::worldPose(handEntity, ecm).CoordPositionAdd(tcpOffset);
    for (auto &fruit : fruits) {
      if (fruit.link == gz::sim::kNullEntity) {
        fruit.link = LinkNamed(ecm, ModelNamed(ecm, fruit.name), "fruit_link");
        auto stem = LinkNamed(ecm, ModelNamed(ecm, fruit.plant), "link");
        if (fruit.link != gz::sim::kNullEntity && stem != gz::sim::kNullEntity)
          fruit.joint = Attach(ecm, stem, fruit.link);
        else { fruit.link = gz::sim::kNullEntity; continue; }
      }
      auto current = gz::sim::worldPose(fruit.link, ecm).Pos();
      if (!info.paused && fruit.previousTime >= 0 && simTime > fruit.previousTime)
        fruit.speed = (current - fruit.previousPosition).Length() / (simTime - fruit.previousTime);
      fruit.previousPosition = current; fruit.previousTime = simTime;
      if (handEntity != gz::sim::kNullEntity && (currentTcp - current).Length() < fruit.radius + 0.08) {
        RequestContacts(ecm, CollisionFor(ecm, fruit.link));
        auto evidence = Grip(ecm, fruit);
        auto assessment = AssessGrip(evidence);
        if (!info.paused && assessment.valid && assessment.left && assessment.right &&
            std::abs(evidence.leftVelocity) <= 0.002 && std::abs(evidence.rightVelocity) <= 0.002) {
          if (fruit.gripStableSince < 0) fruit.gripStableSince = simTime;
        } else fruit.gripStableSince = -1;
      } else fruit.gripStableSince = -1;
      if (fruit.state == "ATTACHED") {
        auto hand = LinkNamed(ecm, robot, "gripper_base_link");
        if (hand == gz::sim::kNullEntity ||
            (gz::sim::worldPose(hand, ecm).CoordPositionAdd(fruit.attachedLocal)-current).Length() > 0.02)
          fruit.state = "LOST";
      }
      if (fruit.releasedAt < 0) continue;
      auto robot = ModelNamed(ecm, robotName);
      if (robot == gz::sim::kNullEntity) {
        fruit.basket = BasketEvidence();
        if (fruit.state == "IN_BASKET") fruit.state = "RELEASED";
        continue;
      }
      auto p = BasketCadPose(ecm, robot).Inverse().CoordPositionAdd(current);
      fruit.basket.Observe({p.X(), p.Y(), p.Z()}, fruit.radius, simTime, info.paused);
      if (fruit.state != "RELEASED" && fruit.state != "IN_BASKET") continue;
      fruit.state = fruit.basket.Confirmed() ? "IN_BASKET" : "RELEASED";
      if (current.Z() < 0.12 && !fruit.basket.inside) fruit.state = "LOST";
    }
    std::deque<std::shared_ptr<Pending>> work;
    { std::lock_guard<std::mutex> guard(queueMutex); work.swap(queue); }
    for (auto &pending : work) {
      std::lock_guard<std::mutex> lock(pending->mutex);
      if (pending->cancelled) continue;
      try { pending->output = Process(pending->input, ecm); }
      catch (const std::exception &error) { pending->output = Failure(error.what()); }
      pending->done = true; pending->cv.notify_all();
    }
  }
};
}
GZ_ADD_PLUGIN(agri::HarvestSystem, gz::sim::System,
             agri::HarvestSystem::ISystemConfigure, agri::HarvestSystem::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(agri::HarvestSystem, "agri::HarvestSystem")
