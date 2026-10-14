// Offline fake-ECM test. No publisher, service call, world, or robot is started.
#include "../src/IdealSteeringSystem.cc"
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <cassert>
#include <iostream>

namespace agri {
struct IdealSteeringSystemTestAccess {
  static void Enable(IdealSteeringSystem &s, gz::sim::Entity world) {
    s.enabled=true; s.configured=true; s.world=world;
    s.commandTopic="offline_fixture_no_transport"; s.statusService="offline_fixture_no_transport";
  }
  static void Input(IdealSteeringSystem &s, double angle, int64_t ns, bool header=true) {
    gz::msgs::Double message;message.set_data(angle);
    if(header){message.mutable_header()->mutable_stamp()->set_sec(ns/1000000000LL);
      message.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000LL);}
    s.OnCommand(message);
  }
  static IdealJson Status(IdealSteeringSystem &s, const std::string &request="{\"op\":\"status\"}") {
    gz::msgs::StringMsg input,output;input.set_data(request);assert(s.Status(input,output));
    return IdealJson::parse(output.data());
  }
  static void MakeWallStale(IdealSteeringSystem &s) {
    s.target.received-=std::chrono::seconds(1);s.incoming.received-=std::chrono::seconds(1);
  }
};
}
using c=gz::sim::components::Joint;
namespace comp=gz::sim::components;
using Access=agri::IdealSteeringSystemTestAccess;
using Json=nlohmann::json;

int main() {
 gz::sim::EntityComponentManager e;auto world=e.CreateEntity();e.CreateComponent(world,comp::World());e.CreateComponent(world,comp::Name("field"));
 auto child=[&](gz::sim::Entity parent,const std::string&name){auto id=e.CreateEntity();e.SetParentEntity(id,parent);e.CreateComponent(id,comp::ParentEntity(parent));e.CreateComponent(id,comp::Name(name));return id;};
 auto robot=child(world,"agri_robot");e.CreateComponent(robot,comp::Model());auto base=child(robot,"base_footprint");e.CreateComponent(base,comp::Link());e.CreateComponent(base,comp::Pose(gz::math::Pose3d(2,3,.36,0,0,.5)));
 sdf::JointAxis axis;assert(axis.SetXyz(gz::math::Vector3d(0,0,-1)).empty());axis.SetLower(-3.1416);axis.SetUpper(3.1416);
 std::map<std::string,gz::sim::Entity> joints;
 for(const std::string name:{"r2_steer_joint","f1_steer_joint","r2_wheel_joint","arm_joint2"}){
  auto id=child(robot,name);joints[name]=id;e.CreateComponent(id,comp::Joint());e.CreateComponent(id,comp::JointAxis(axis));e.CreateComponent(id,comp::JointPosition({-.8}));e.CreateComponent(id,comp::JointVelocity({.012}));e.CreateComponent(id,comp::JointVelocityCmd({1.2}));
 }
 auto r2=joints.at("r2_steer_joint");agri::IdealSteeringSystem s;Access::Enable(s,world);
 auto step=[&](int64_t ns,bool paused=false){gz::sim::UpdateInfo info;info.iterations=ns/1000000;info.simTime=std::chrono::nanoseconds(ns);info.dt=std::chrono::milliseconds(1);info.paused=paused;s.PreUpdate(info,e);s.PostUpdate(info,e);return Access::Status(s);};
 auto consume=[&]{auto *reset=e.Component<comp::JointPositionReset>(r2);assert(reset);double q=reset->Data().at(0);e.RemoveComponent<comp::JointPositionReset>(r2);return q;};
 auto untouched=[&]{for(const auto &[name,id]:joints){assert(!e.Component<comp::JointVelocityReset>(id));assert(!e.Component<comp::JointForceCmd>(id));assert(e.Component<comp::JointPosition>(id)->Data()==std::vector<double>{-.8});assert(e.Component<comp::JointVelocity>(id)->Data()==std::vector<double>{.012});if(name!="r2_steer_joint"){assert(!e.Component<comp::JointPositionReset>(id));assert(e.Component<comp::JointVelocityCmd>(id)->Data()==std::vector<double>{1.2});}}assert(e.Component<comp::Pose>(base)->Data()==gz::math::Pose3d(2,3,.36,0,0,.5));};
 assert(!step(1000000000)["active"].get<bool>());assert(!e.Component<comp::JointPositionReset>(r2));
 Access::Input(s,-.79,1000000000);auto state=step(1000000000);assert(state["ready"]==true&&state["active"]==true&&state["idealized"]==true);assert(state["actual_position_rad"]==-.8);assert(state["actual_velocity_rps"]==.012);assert(state["target_rad"]==-.79);assert(consume()==-.79);assert(e.Component<comp::JointVelocityCmd>(r2)->Data()==std::vector<double>{0});untouched();
 // A continuously active lease copies the already filtered upstream reference,
 // without additional slew. Initial activation is guarded within .03 rad.
 Access::Input(s,-.5,1001000000);state=step(1001000000);assert(state["active"]==true);assert(consume()==-.5);untouched();
 // Duplicate header+payload must not renew either clock lease.
 Access::MakeWallStale(s);Access::Input(s,-.5,1001000000);e.Component<comp::JointVelocityCmd>(r2)->Data()[0]=1.2;state=step(1002000000);assert(state["active"]==false&&state["expiry_reason"]=="WALL_LEASE_EXPIRED");assert(!e.Component<comp::JointPositionReset>(r2));assert(e.Component<comp::JointVelocityCmd>(r2)->Data()[0]==1.2);
 Access::Input(s,-.79,1003000000);state=step(1003000000);assert(state["active"]==true);consume();
 state=step(1204000000);assert(state["active"]==false&&state["expiry_reason"]=="SIM_LEASE_EXPIRED");assert(!e.Component<comp::JointPositionReset>(r2));
 auto rejected=[&](double angle,int64_t stamp,int64_t now,const std::string &why,bool header=true){e.Component<comp::JointVelocityCmd>(r2)->Data()[0]=1.2;Access::Input(s,angle,stamp,header);auto r=step(now);assert(r["active"]==false);assert(r["expiry_reason"]==why);assert(!e.Component<comp::JointPositionReset>(r2));assert(e.Component<comp::JointVelocityCmd>(r2)->Data()[0]==1.2);untouched();};
 rejected(-.79,1205000000,1204000000,"FUTURE_SIM_STAMP");
 rejected(-.79,0,1204000000,"ZERO_SIM_STAMP");
 rejected(-.79,0,1204000000,"MISSING_SIM_STAMP",false);
 rejected(std::numeric_limits<double>::quiet_NaN(),1204000000,1204000000,"NONFINITE_TARGET");
 rejected(4,1204000000,1204000000,"TARGET_OUTSIDE_IMPORTED_JOINT_LIMITS");
 rejected(-.5,1204000000,1204000000,"STARTUP_TARGET_ACTUAL_MISMATCH");
 rejected(-.79,900000000,1204000000,"NONMONOTONIC_TARGET_STAMP");
 Access::Input(s,-.79,1205000000);state=step(1205000000);assert(state["active"]==true);consume();
 // A malformed new target immediately revokes an otherwise active lease.
 rejected(std::numeric_limits<double>::infinity(),1206000000,1206000000,"NONFINITE_TARGET");
 Access::Input(s,-.79,1206500000);state=step(1206500000);assert(state["active"]==true);consume();
 rejected(-.78,1206500000,1206600000,"NONMONOTONIC_TARGET_STAMP");
 Access::Input(s,-.79,1207000000);state=step(1207000000);assert(state["active"]==true);consume();
 state=step(500000000);assert(state["active"]==false&&state["expiry_reason"]=="CLOCK_BACKWARD_REVOKED");assert(!e.Component<comp::JointPositionReset>(r2));
 Access::Input(s,-.79,501000000);state=step(501000000);assert(state["active"]==true);consume();
 state=step(502000000,true);assert(state["active"]==false&&state["expiry_reason"]=="PAUSED");assert(!e.Component<comp::JointPositionReset>(r2));
 Access::Input(s,-.79,503000000);e.CreateComponent(r2,comp::JointPositionReset({-.1}));state=step(503000000);assert(state["active"]==false&&state["expiry_reason"]=="EXISTING_POSITION_RESET_CONFLICT");assert(consume()==-.1);
 // A foreign force or VelocityReset would suppress SERVO0 in Physics. Refuse
 // that control-mode conflict and leave the foreign command untouched.
 e.CreateComponent(r2,comp::JointForceCmd({7}));Access::Input(s,-.79,503500000);state=step(503500000);assert(state["active"]==false&&state["expiry_reason"]=="NATIVE_FORCE_OR_VELOCITY_RESET_CONFLICT");assert(e.Component<comp::JointForceCmd>(r2)->Data()==std::vector<double>{7});assert(!e.Component<comp::JointPositionReset>(r2));e.RemoveComponent<comp::JointForceCmd>(r2);
 e.CreateComponent(r2,comp::JointVelocityReset({.13}));Access::Input(s,-.79,503600000);state=step(503600000);assert(state["active"]==false&&state["expiry_reason"]=="NATIVE_FORCE_OR_VELOCITY_RESET_CONFLICT");assert(e.Component<comp::JointVelocityReset>(r2)->Data()==std::vector<double>{.13});assert(!e.Component<comp::JointPositionReset>(r2));e.RemoveComponent<comp::JointVelocityReset>(r2);
 e.RemoveComponent<comp::JointVelocityCmd>(r2);Access::Input(s,-.79,504000000);state=step(504000000);assert(state["active"]==false&&state["ready"]==false);assert(!e.Component<comp::JointVelocityCmd>(r2));assert(!e.Component<comp::JointPositionReset>(r2));
 assert(Access::Status(s,"{\"op\":17}")["success"]==false);assert(Access::Status(s,"{\"op\":\"enable\"}")["success"]==false);
 agri::IdealSteeringSystem disabled;gz::sim::EventManager events;auto config=std::make_shared<sdf::Element>();config->SetName("plugin");disabled.Configure(world,config,e,events);gz::sim::UpdateInfo info;info.simTime=std::chrono::seconds(2);info.dt=std::chrono::milliseconds(1);Access::Input(disabled,-.79,2000000000);disabled.PreUpdate(info,e);assert(!e.Component<comp::JointPositionReset>(r2));
 auto wrongJoint=std::make_shared<sdf::Element>();wrongJoint->SetName("plugin");auto jointElement=std::make_shared<sdf::Element>();jointElement->SetName("joint_name");jointElement->AddValue("string","f1_steer_joint",false);wrongJoint->InsertElement(jointElement);agri::IdealSteeringSystem forbidden;forbidden.Configure(world,wrongJoint,e,events);forbidden.PostUpdate(info,e);assert(Access::Status(forbidden)["expiry_reason"]=="ONLY_R2_STEER_JOINT_ALLOWED");assert(Access::Status(forbidden)["enabled"]==false);
 std::cout<<"PASS ideal steering: r2-only native reset/control handoff; no fake feedback/base/other-joint changes; limits, startup guard, dual-clock leases, replay/future/NaN/clockback/pause/conflict fail closed; status read only.\n";
}
