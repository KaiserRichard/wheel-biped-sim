#include "mapping.hpp"
#include "clock_policy.hpp"
#include <cassert>
#include <iomanip>
#include <iostream>
using namespace wbr::bridge;
using namespace wbr::vio;

static Gate gate() {return {"boot","device","ros_live",2,100,10};}
static EstimateSnapshot snapshot() {
  EstimateSnapshot s;
  s.source=Stamp{100,"device"}; s.sequence=1; s.state=State::TRACKING;
  s.valid=true; s.reasons=NONE; s.map_id=7; s.T_world_camera=Transform{};
  return s;
}
static ClockEvidence clock_evidence() {return {100,"device","boot","ros_live",2,10,1234567890,20,3};}
static void policy_checks() {
  auto s=snapshot(); auto e=clock_evidence(); auto g=gate();
  assert(g.advance(s,e).pose);
  assert(g.advance(s,e).reason=="duplicate");
  e.age_upper_ns=101;
  assert(g.advance(s,e).reason=="stale_source");
  s.valid=false; s.reasons=RESET; ++s.reset_counter; ++s.epoch_id;
  assert(g.advance(s,e).reason=="reset_boundary");
  s.valid=true; s.reasons=NONE; ++s.sequence; s.source->ns=200;
  e.source_ns=200; e.age_upper_ns=20;
  assert(g.advance(s,e).pose);
  s.epoch_id=0;
  assert(g.advance(s,e).reason=="regression_latched");
  s.epoch_id=1; ++s.sequence; s.source->ns=300; e.source_ns=300;
  assert(g.advance(s,e).reason=="regression_latched");
  s=snapshot(); e=clock_evidence();
  {auto f=gate(); assert(f.advance(s,std::nullopt).reason=="clock_unavailable");}
  e.session="old";
  {auto f=gate(); assert(f.advance(s,e).reason=="clock_identity_mismatch");}
  e=clock_evidence(); e.uncertainty_ns=11;
  {auto f=gate(); assert(f.advance(s,e).reason=="clock_uncertain");}
  e=clock_evidence(); s.T_world_camera->parent="odom";
  {auto f=gate(); assert(f.advance(s,e).reason=="bad_frame");}
  s=snapshot(); s.T_world_camera->matrix[0]=2;
  {auto f=gate(); assert(f.advance(s,e).reason=="bad_pose");}
  s=snapshot(); s.covariance_availability=Availability::AVAILABLE;
  {auto f=gate(); assert(f.advance(s,e).reason=="bad_covariance_metadata");}
  s=snapshot(); e=clock_evidence();
  {auto f=gate(); assert(f.advance(s,e).pose);
    ++s.sequence; s.source=Stamp{0,"other"};
    assert(f.advance(s,e).reason=="bad_source");
    s.source=Stamp{200,"device"}; e.source_ns=200;
    assert(f.advance(s,e).pose);}
}
static void clock_policy_checks() {
  ClockPolicy c(true,100);
  Gate g{"boot","device","ros_replay",2,100,10};
  auto s=snapshot(); auto e=clock_evidence(); e.target_domain="ros_replay"; e.ros_ns=50;
  auto step=[&](int64_t now) {
    const auto reason=c.observe(now,true);
    auto decision=g.advance(s,e); // Consume snapshots even while paused.
    return reason.empty()?decision:Decision{std::nullopt,reason};
  };
  assert(step(0).reason=="waiting_for_clock");
  ++s.sequence; s.source->ns=200; e.source_ns=200;
  assert(step(100).pose);
  ++s.sequence; s.source->ns=300; e.source_ns=300;
  assert(step(100).reason=="clock_paused"); // fresh ingress, nonzero frozen ROS time
  assert(step(110).reason=="duplicate"); // resume cannot publish a paused snapshot
  ++s.sequence; s.source->ns=400; e.source_ns=400;
  assert(step(120).pose);
  assert(step(1000).reason=="clock_jump_restart_required");
  ++s.sequence; s.source->ns=500; e.source_ns=500;
  assert(step(1010).reason=="clock_jump_restart_required");
  ClockPolicy rebound(true,100); Gate new_gate{"boot","device","ros_replay",3,100,10};
  assert(rebound.observe(1010,true).empty());
  assert(new_gate.advance(s,e).reason=="clock_identity_mismatch");
  ++s.sequence; s.source->ns=600; e.source_ns=600; e.generation=3;
  assert(new_gate.advance(s,e).pose); // fresh estimate + newly mapped generation
  assert(rebound.observe(1020,true,true)=="clock_jump_restart_required"); // publisher/seek notification
  assert(rebound.observe(1030,true)=="clock_jump_restart_required");
  ClockPolicy small_seek(true,100); small_seek.observe(100,true);
  assert(small_seek.observe(101,true,true)=="clock_jump_restart_required");
  ClockPolicy replacement(true,100); replacement.observe(100,true);
  assert(replacement.observe(100,true,true)=="clock_jump_restart_required");
  ClockPolicy mode(true,100);
  assert(mode.observe(100,false)=="clock_jump_restart_required");
  ClockPolicy backward(true,100); backward.observe(100,true);
  assert(backward.observe(99,true)=="clock_jump_restart_required");
  ClockPolicy unconfigured(true,-1);
  assert(unconfigured.observe(100,true)=="clock_policy_unconfigured");
}
int main() {
  policy_checks();
  clock_policy_checks();
  int64_t ns;
  Transform t;
  if (!(std::cin>>ns)) return 0;
  for (double &v:t.matrix) if (!(std::cin>>v)) return 2;
  if (!valid_pose(t) || ns<0 || ns>2147483647999999999LL) return 2;
  auto p=pose_fields(t,ns);
  std::cout<<p.sec<<' '<<p.nanosec<<std::setprecision(17);
  for (double v:p.position) std::cout<<' '<<v;
  for (double v:p.xyzw) std::cout<<' '<<v;
  std::cout<<'\n';
}
