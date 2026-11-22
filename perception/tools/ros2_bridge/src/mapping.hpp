#pragma once
// ROS-free C++ counterpart to mapping.py; actual adapter contract, no engine link.
#include "adapter.h"
#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

namespace wbr::bridge {
struct ClockEvidence {
  int64_t source_ns;
  std::string source_domain, session, target_domain;
  uint64_t generation, revision;
  int64_t ros_ns, age_upper_ns, uncertainty_ns;
};
struct PoseFields {
  int32_t sec;
  uint32_t nanosec;
  std::array<double, 3> position;
  std::array<double, 4> xyzw;
};
struct Decision {
  std::optional<PoseFields> pose;
  std::string reason;
};
inline bool valid_pose(const vio::Transform &t) {
  const auto &m = t.matrix;
  for (double v : m) if (!std::isfinite(v)) return false;
  for (int j=0; j<4; ++j) if (std::abs(m[12+j]-(j==3?1:0))>1e-6) return false;
  for (int i=0; i<3; ++i) for (int j=0; j<3; ++j) {
    double dot=0;
    for (int k=0; k<3; ++k) dot+=m[4*k+i]*m[4*k+j];
    if (std::abs(dot-(i==j?1:0))>1e-4) return false;
  }
  double det=m[0]*(m[5]*m[10]-m[6]*m[9])-m[1]*(m[4]*m[10]-m[6]*m[8])
      +m[2]*(m[4]*m[9]-m[5]*m[8]);
  return std::abs(det-1)<1e-4;
}
inline PoseFields pose_fields(const vio::Transform &t, int64_t ns) {
  const auto &m=t.matrix;
  std::array<double,4> q{};
  const double trace=m[0]+m[5]+m[10];
  if (trace>0) {
    double scale=2*std::sqrt(trace+1);
    q={(m[9]-m[6])/scale,(m[2]-m[8])/scale,(m[4]-m[1])/scale,scale/4};
  } else {
    int i=0;
    if (m[5]>m[0]) i=1;
    if (m[10]>m[5*i]) i=2;
    const int j=(i+1)%3, k=(i+2)%3;
    double scale=2*std::sqrt(1+m[5*i]-m[5*j]-m[5*k]);
    q[i]=scale/4;
    q[j]=(m[4*i+j]+m[4*j+i])/scale;
    q[k]=(m[4*i+k]+m[4*k+i])/scale;
    q[3]=(m[4*k+j]-m[4*j+k])/scale;
  }
  double norm=0;
  for (double v:q) norm+=v*v;
  norm=std::sqrt(norm)*(q[3]<0?-1:1);
  for (double &v:q) v/=norm;
  return {static_cast<int32_t>(ns/1000000000),static_cast<uint32_t>(ns%1000000000),
          {m[3],m[7],m[11]},q};
}
class Gate {
 public:
  Gate(std::string session, std::string domain, std::string target, uint64_t generation,
       int64_t max_age, int64_t max_uncertainty)
    : session_(std::move(session)), domain_(std::move(domain)), target_(std::move(target)),
      generation_(generation), max_age_(max_age), max_uncertainty_(max_uncertainty) {}
  Decision advance(const vio::EstimateSnapshot &s, const std::optional<ClockEvidence> &e) {
    auto reject=[](const char *reason) {return Decision{std::nullopt,reason};};
    if (blocked_) return reject("regression_latched");
    if (session_.empty() || domain_.empty() || (target_!="ros_live" && target_!="ros_replay") ||
        max_age_<=0 || max_uncertainty_<0) return reject("unconfigured");
    if (s.source && (s.source->ns<0 || s.source->domain!=domain_)) return reject("bad_source");
    const Boundary b{s.reset_counter,s.epoch_id,s.map_id};
    if (s.sequence<sequence_ || (boundary_ && (s.reset_counter<std::get<0>(*boundary_) ||
        s.epoch_id<std::get<1>(*boundary_)))) {
      blocked_=true; return reject("regression_latched");
    }
    const bool changed=boundary_ && b!=*boundary_;
    boundary_=b;
    const bool fresh=s.sequence>sequence_;
    if (fresh) {
      sequence_=s.sequence;
      if (s.source) {
        if (source_ && s.source->ns<=*source_) {
          blocked_=true; return reject("regression_latched");
        }
        source_=s.source->ns;
      }
    }
    if (changed) return reject("reset_boundary");
    if (s.state!=vio::State::TRACKING || !s.valid || s.reasons!=vio::NONE)
      return reject("invalid_snapshot");
    if (!s.source || s.source->domain!=domain_ || s.source->ns<0) return reject("bad_source");
    if (!e) return reject("clock_unavailable");
    if (e->source_ns!=s.source->ns || e->source_domain!=domain_ || e->session!=session_ ||
        e->target_domain!=target_ || e->generation!=generation_) return reject("clock_identity_mismatch");
    if (e->age_upper_ns<0 || e->uncertainty_ns<0) return reject("clock_unavailable");
    if (e->uncertainty_ns>max_uncertainty_) return reject("clock_uncertain");
    if (e->age_upper_ns>max_age_) return reject("stale_source");
    const bool available=s.covariance_availability==vio::Availability::AVAILABLE;
    if (!available && s.covariance_availability!=vio::Availability::UNAVAILABLE)
      return reject("bad_covariance_metadata");
    if ((available && !s.covariance) || (!available && s.covariance))
      return reject("bad_covariance_metadata");
    if (s.covariance) for (double v:*s.covariance)
      if (!std::isfinite(v)) return reject("bad_covariance_metadata");
    if (!fresh) return reject("duplicate");
    if (!s.T_world_camera) return reject("bad_pose");
    const auto &t=*s.T_world_camera;
    if (t.parent!="slam_world" || t.child!="camera_left_optical") return reject("bad_frame");
    if (!valid_pose(t)) return reject("bad_pose");
    if (e->ros_ns<0 || e->ros_ns>2147483647999999999LL) return reject("bad_ros_stamp");
    return {pose_fields(t,e->ros_ns),"pose_published"};
  }
 private:
  using Boundary=std::tuple<uint64_t,uint64_t,std::optional<uint64_t>>;
  std::string session_,domain_,target_;
  uint64_t generation_,sequence_=0;
  int64_t max_age_,max_uncertainty_;
  std::optional<int64_t> source_;
  std::optional<Boundary> boundary_;
  bool blocked_=false;
};
}  // namespace wbr::bridge
