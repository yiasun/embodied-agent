// Runtime-only test entity plugin; never modifies the formal world or robot.
#include "BoundedProfile.hh"
#include "StartupCheck.hh"
#include "HeartbeatWatchdog.hh"
#include <atomic>
#include <chrono>
#include <mutex>
#include <sstream>
#include <regex>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <gz/common/Console.hh>

namespace embodied {
using Steady=std::chrono::steady_clock;
class BoundedMotionHeartbeatV2 : public gz::sim::System, public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate {
  gz::sim::Entity entity{}, link{}, robot{};
  gz::transport::Node node; gz::transport::Node::Publisher publisher;
  BoundedProfile profile; std::string token;
  std::string entityName, configureStatus{"FAILED"};
  std::atomic<bool> configured{false};
  std::mutex mutex; bool stopRequested{false};
  HeartbeatWatchdog heartbeat;
  Steady::time_point firstHeartbeat{};
  double previousPreWall{-1}, preWallGap{-1}, maxPreWallGap{-1};
  double lastPublish{-1}; unsigned long sequence{0};
  bool contactsMeasured{false}, unsafeContact{false}; unsigned int robotContacts{0};

  bool Below(gz::sim::Entity child, gz::sim::Entity parent, const gz::sim::EntityComponentManager &ecm) const {
    for (int i=0;i<10 && child;i++) {
      if (child==parent) return true;
      const auto *p=ecm.Component<gz::sim::components::ParentEntity>(child);
      if (!p) return false;
      child=p->Data();
    }
    return false;
  }
 public:
  void Configure(const gz::sim::Entity &e, const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    entity=e; gzmsg << "[EA_MOTION_INIT] entity=" << entity << " phase=BEGIN\n";
    auto fail=[&](const std::string &reason) {
      profile.Stop(reason); configureStatus="FAILED"; configured=false;
      gzerr << "[EA_MOTION_INIT] entity=" << entity << " phase=FAILED reason=" << reason
            << " link=" << link << " robot=" << robot << "\n";
    };
    if (!sdf || !sdf->HasElement("token")) { fail("INVALID_CONFIG"); return; }
    token=sdf->Get<std::string>("token");
    if (!std::regex_match(token,std::regex("[0-9a-f]{32}"))) { fail("INVALID_CONFIG"); return; }
    // Publish failures too, even when link/robot lookup or configuration fails.
    const std::string root="/ea_stage62/"+token;
    publisher=node.Advertise<gz::msgs::StringMsg>(root+"/state");
    const auto *name=ecm.Component<gz::sim::components::Name>(entity);
    entityName=name?name->Data():"";
    std::string pattern;
    try {
      for (const auto &key:{"x","y_min","y_max","speed","duration","pattern"})
        if (!sdf->HasElement(key)) { fail("INVALID_CONFIG"); return; }
      profile.x=sdf->Get<double>("x"); profile.low=sdf->Get<double>("y_min"); profile.high=sdf->Get<double>("y_max");
      profile.speed=sdf->Get<double>("speed"); profile.duration=sdf->Get<double>("duration");
      pattern=sdf->Get<std::string>("pattern"); profile.shuttle=pattern=="shuttle";
    } catch (const std::exception &) { fail("INVALID_CONFIG"); return; }
    if (!ValidMotionConfig(token,profile.x,profile.low,profile.high,profile.speed,profile.duration,pattern)) {
      fail("INVALID_CONFIG"); return;
    }
    link=gz::sim::Model(entity).LinkByName(ecm,"ea_link_"+token);
    robot=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name("turtlebot3_waffle"));
    const auto result=StartupResult(true,entityName=="ea_stage61_"+token,link,robot,publisher.Valid(),true);
    if (result!="READY") { fail(result); return; }
    std::function<void(const gz::msgs::StringMsg &)> callback=[this](const gz::msgs::StringMsg &msg) {
      std::lock_guard<std::mutex> guard(mutex);
      if (msg.data()=="stop") stopRequested=true; // Latched; no re-arm operation.
      std::uint64_t command=0,state=0;
      if (ParseHeartbeat(msg.data(),command,state) && configured.load()) {
        const auto now=Steady::now();
        const bool first=heartbeat.authorized==0;
        if (heartbeat.ReceiveProof(std::chrono::duration<double>(now.time_since_epoch()).count(),command,state) && first)
          firstHeartbeat=now;
      }
    };
    if (!node.Subscribe(root+"/control",callback)) { fail("CONTROL_SUBSCRIPTION_FAILED"); return; }
    configureStatus="READY";configured=true;
    gzmsg << "[EA_MOTION_INIT] entity=" << entity << " phase=READY link=" << link << " robot=" << robot << "\n";
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    const auto now=Steady::now();
    const double wallNow=std::chrono::duration<double>(now.time_since_epoch()).count();
    if (previousPreWall>=0) { preWallGap=wallNow-previousPreWall;maxPreWallGap=std::max(maxPreWallGap,preWallGap); }
    previousPreWall=wallNow;
    if (!link || !configured.load()) return;
    if (!ecm.HasEntity(robot)) profile.Stop("ROBOT_ENTITY_MISSING");
    bool fresh=false; double wall=0;
    {
      std::lock_guard<std::mutex> guard(mutex);
      if (stopRequested) profile.Stop("EXPLICIT_STOP");
      const auto checked=Steady::now();
      fresh=heartbeat.Fresh(std::chrono::duration<double>(checked.time_since_epoch()).count());
      if (heartbeat.authorized) wall=std::chrono::duration<double>(checked-firstHeartbeat).count();
    }
    const auto pose=gz::sim::worldPose(entity,ecm); const auto rp=gz::sim::worldPose(robot,ecm);
    if (std::abs(pose.Rot().Roll())>0.02 || std::abs(pose.Rot().Pitch())>0.02) profile.Stop("ATTITUDE_VIOLATION");
    const double t=std::chrono::duration<double>(info.simTime).count();
    double v=0;
    if (!info.paused) v=profile.Step(t,pose.Pos().X(),pose.Pos().Y(),pose.Pos().Z(),pose.Rot().Yaw(),
                                   rp.Pos().X(),rp.Pos().Y(),fresh,unsafeContact,wall);
    else if (profile.armed && !fresh) profile.Stop("HEARTBEAT_EXPIRED");
    gz::sim::Link body(link);
    body.SetLinearVelocity(ecm,pose.Rot().Inverse().RotateVector(gz::math::Vector3d(0,v,0)));
    body.SetAngularVelocity(ecm,gz::math::Vector3d::Zero);
  }
  void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override {
    if (!publisher.Valid()) return;
    const double t=std::chrono::duration<double>(info.simTime).count();
    contactsMeasured=false; robotContacts=0; unsafeContact=false;
    const auto floor=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name("floor"));
    ecm.Each<gz::sim::components::ContactSensorData>([&](const auto &id, const auto *data) {
      if (!Below(id,entity,ecm)) return true;
      const auto &msg=data->Data();
      const double stamp=msg.header().stamp().sec()+msg.header().stamp().nsec()*1e-9;
      if (stamp<=0 || std::abs(t-stamp)>0.1) return true;
      contactsMeasured=true;
      for (const auto &c:msg.contact()) {
        if (!c.collision1().id() || !c.collision2().id()) {
          contactsMeasured=false; unsafeContact=true; continue;
        }
        if ((Below(c.collision1().id(),entity,ecm) && !Below(c.collision2().id(),floor,ecm)) ||
            (Below(c.collision2().id(),entity,ecm) && !Below(c.collision1().id(),floor,ecm))) unsafeContact=true;
        if ((Below(c.collision1().id(),entity,ecm) && Below(c.collision2().id(),robot,ecm)) ||
            (Below(c.collision2().id(),entity,ecm) && Below(c.collision1().id(),robot,ecm))) ++robotContacts;
      }
      return true;
    });
    // Publish at 20 sim-Hz. Paused clock intentionally produces no fresh samples.
    if (lastPublish>=0 && t-lastPublish<0.05) return;
    lastPublish=t;
    const bool poseValid=ecm.HasEntity(entity);
    const auto p=poseValid?gz::sim::worldPose(entity,ecm):gz::math::Pose3d::Zero;
    double heartbeatAge=-1,proofAge=-1,receiveGap=-1,maxReceiveGap=-1;
    std::uint64_t receivedCount=0,authorizedCount=0,rejectedCount=0;
    const auto publishedSequence=++sequence;
    {
      std::lock_guard<std::mutex> guard(mutex);
      const double wallNow=std::chrono::duration<double>(Steady::now().time_since_epoch()).count();
      heartbeatAge=heartbeat.Age(wallNow);proofAge=heartbeat.ProofAge(wallNow);
      receiveGap=heartbeat.lastGap;maxReceiveGap=heartbeat.maxGap;receivedCount=heartbeat.received;
      authorizedCount=heartbeat.authorized;rejectedCount=heartbeat.rejected;
      heartbeat.Published(publishedSequence,wallNow);
    }
    auto number=[](double value) { return value<0?std::string("null"):std::to_string(value); };
    std::ostringstream out; out.precision(17);
    out<<"{\"protocol\":\"ea-motion-heartbeat-v2\",\"frame\":\"gazebo_world\",\"configured\":"<<(configured.load()?"true":"false")
       <<",\"configure_status\":\""<<configureStatus<<"\",\"pose_valid\":"<<(poseValid?"true":"false")
       <<",\"entity_name\":\""<<(entityName=="ea_stage61_"+token?entityName:"")<<"\",\"token\":\""<<token<<"\",\"entity_id\":"<<entity<<",\"sequence\":"<<publishedSequence
       <<",\"sim_time\":"<<t<<",\"position\":["<<p.Pos().X()<<","<<p.Pos().Y()<<","<<p.Pos().Z()
       <<"],\"yaw\":"<<p.Rot().Yaw()<<",\"commanded_vy\":"<<profile.velocity
       <<",\"stopped\":"<<(profile.stopped?"true":"false")<<",\"reason\":\""<<profile.reason
       <<"\",\"contact_measured\":"<<(contactsMeasured?"true":"false")<<",\"robot_contacts\":"<<robotContacts
       <<",\"heartbeat_received_count\":"<<receivedCount
       <<",\"heartbeat_authorized_count\":"<<authorizedCount<<",\"heartbeat_rejected_count\":"<<rejectedCount
       <<",\"heartbeat_authorization_age_wall_s\":"<<number(proofAge)
       <<",\"heartbeat_age_wall_s\":"<<number(heartbeatAge)<<",\"heartbeat_receive_gap_wall_s\":"<<number(receiveGap)
       <<",\"heartbeat_receive_max_gap_wall_s\":"<<number(maxReceiveGap)
       <<",\"preupdate_gap_wall_s\":"<<number(preWallGap)<<",\"preupdate_max_gap_wall_s\":"<<number(maxPreWallGap)<<"}";
    gz::msgs::StringMsg msg; msg.set_data(out.str()); publisher.Publish(msg);
  }
};
}
GZ_ADD_PLUGIN(embodied::BoundedMotionHeartbeatV2,gz::sim::System,gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(embodied::BoundedMotionHeartbeatV2,"embodied::BoundedMotionHeartbeatV2")
