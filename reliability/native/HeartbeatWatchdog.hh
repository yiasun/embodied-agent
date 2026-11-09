#pragma once
#include <cmath>
#include <cstdint>
#include <charconv>
#include <deque>
#include <string>

namespace embodied {
// Native steady-clock seconds only. Epochs are never compared across processes.
struct HeartbeatWatchdog {
  static constexpr double Timeout=0.5;
  std::uint64_t received{0}, authorized{0}, rejected{0}, lastCommand{0};
  double last{0}, lastProof{0}, lastGap{-1}, maxGap{-1};
  std::deque<std::pair<std::uint64_t,double>> states;
  void ObserveArrival(double now) {
    if (received) { lastGap=now-last; if(lastGap>maxGap) maxGap=lastGap; }
    last=now; ++received;
  }
  void Receive(double now) { ObserveArrival(now);lastProof=now;++authorized; } // deterministic core tests
  void Published(std::uint64_t sequence,double now) {
    states.emplace_back(sequence,now);while(states.size()>64) states.pop_front();
  }
  bool ReceiveProof(double now,std::uint64_t command,std::uint64_t state) {
    ObserveArrival(now);
    for (const auto &entry:states) {
      if (entry.first==state && command>lastCommand && now>=entry.second && now-entry.second<=0.25) {
        lastProof=entry.second;lastCommand=command;++authorized;return true;
      }
    }
    ++rejected;return false;
  }
  double Age(double now) const { return received?now-last:-1; }
  double ProofAge(double now) const { return authorized?now-lastProof:-1; }
  bool Fresh(double now) const { return authorized && std::isfinite(now) && now>=lastProof && now-lastProof<Timeout; }
};

inline bool ParseHeartbeat(const std::string &data,std::uint64_t &command,std::uint64_t &state) {
  const std::string prefix="heartbeat:";
  if (data.size()>96 || data.compare(0,prefix.size(),prefix)!=0) return false;
  const auto split=data.find(':',prefix.size());if(split==std::string::npos) return false;
  auto first=std::from_chars(data.data()+prefix.size(),data.data()+split,command);
  auto second=std::from_chars(data.data()+split+1,data.data()+data.size(),state);
  return first.ec==std::errc() && first.ptr==data.data()+split && second.ec==std::errc() &&
      second.ptr==data.data()+data.size() && command>0 && state>0;
}
}
