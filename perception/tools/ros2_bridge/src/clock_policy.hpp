#pragma once
#include <cstdint>
#include <optional>
#include <string>
namespace wbr::bridge {
// ROS-free policy; one binding per object lifetime, restart after a clock fault.
class ClockPolicy {
 public:
  ClockPolicy(bool sim, int64_t max_step):sim_(sim),max_step_(max_step) {}
  std::string observe(int64_t now, bool sim, bool changed=false) {
    if (changed || sim!=sim_ || now<0 || (last_ && now<*last_)) fault_=true;
    // A configured maximum ordinary replay increment is required. Smaller
    // seeks/source replacement must also be explicitly signalled by the owner.
    if (sim_ && max_step_<=0) return "clock_policy_unconfigured";
    if (sim_ && last_ && *last_>0 && now>*last_ && now-*last_>max_step_) fault_=true;
    const bool paused=sim_ && last_ && now==*last_;
    last_=now;
    if (fault_) return "clock_jump_restart_required";
    if (sim_ && now==0) return "waiting_for_clock";
    if (paused) return "clock_paused";
    return "";
  }
 private:
  bool sim_,fault_=false;
  int64_t max_step_;
  std::optional<int64_t> last_;
};
}
