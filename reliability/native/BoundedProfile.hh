#pragma once
#include <algorithm>
#include <cmath>
#include <string>

namespace embodied {
// Independent of Gazebo: exercised by an offline compiled C++ unit harness.
struct BoundedProfile {
  double x{}, low{25.5}, high{26.5}, speed{0.15}, duration{15};
  double velocity{0}, elapsed{0}, previousTime{-1}, previousX{}, previousY{};
  int direction{1}; bool shuttle{false}, armed{false}, stopped{false};
  std::string reason{"WAITING_FOR_HEARTBEAT"};
  void Stop(const std::string &why) { stopped=true; velocity=0; reason=why; }
  double Step(double t, double px, double py, double pz, double yaw,
              double rx, double ry, bool heartbeat, bool contact, double wallElapsed) {
    if (stopped) return 0;
    if (!std::isfinite(t+px+py+pz+yaw+rx+ry+wallElapsed)) { Stop("NONFINITE_STATE"); return 0; }
    if (previousTime >= 0 && (t < previousTime || t-previousTime > 0.05)) {
      Stop("SIM_CLOCK_DISCONTINUITY"); return 0;
    }
    const double dt=previousTime < 0 ? 0 : t-previousTime;
    if (previousTime >= 0 && std::hypot(px-previousX,py-previousY)>speed*dt+0.03) {
      Stop("POSE_JUMP"); return 0;
    }
    previousTime=t; previousX=px; previousY=py;
    if (std::abs(px-x)>0.04 || py<low-0.04 || py>high+0.04 || std::abs(pz-0.6)>0.04 || std::abs(yaw)>0.02) {
      Stop("ROUTE_OR_ATTITUDE_VIOLATION"); return 0;
    }
    const double dx=std::max(std::abs(rx-px)-0.2,0.0);
    const double dy=std::max(std::abs(ry-py)-0.2,0.0);
    if (std::hypot(dx,dy)-0.22<0.35 || contact) { Stop(contact?"CONTACT_DETECTED":"ROBOT_CLEARANCE_GUARD"); return 0; }
    if (!heartbeat) {
      if (armed) Stop("HEARTBEAT_EXPIRED");
      return 0;
    }
    armed=true; elapsed+=dt;
    if (elapsed>=duration || wallElapsed>=60) { Stop("DURATION_COMPLETE"); return 0; }
    double remaining=direction>0?high-py:py-low;
    if (remaining<0.015 && std::abs(velocity)<0.12) {
      if (!shuttle) { Stop("TRAJECTORY_COMPLETE"); return 0; }
      direction=-direction; velocity=0;
      remaining=direction>0?high-py:py-low;
    }
    // D3 changes the speed envelope halfway through its sim-time lease.
    const double cap=shuttle && elapsed>=duration/2 ? speed/2 : speed;
    const double target=direction*std::min(cap,std::sqrt(std::max(0.0,2*0.4*remaining)));
    const double change=std::clamp(target-velocity,-0.4*dt,0.4*dt);
    velocity+=change; reason="MOVING"; return velocity;
  }
};
}
