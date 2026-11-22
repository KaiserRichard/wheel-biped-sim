// Built ONLY when WBR_WITH_ORBSLAM3 is enabled. Pinned API citations: SOURCES.md.
#include "orb_engine.h"
#include "orb_imu.hpp"
#ifndef WBR_WITH_ORBSLAM3
#error "orb_engine.cpp requires WBR_WITH_ORBSLAM3"
#endif
#include <System.h>
#include <Tracking.h>
#include <opencv2/core.hpp>
#include <utility>
namespace wbr::vio {
OrbEngine::OrbEngine(ORB_SLAM3::System& s,OrbMapObserver o):system_(s),observer_(std::move(o)) {}
static State state(int value) {
    using T=ORB_SLAM3::Tracking;
    switch(value) {
    case T::SYSTEM_NOT_READY: case T::NO_IMAGES_YET: return State::STARTING;
    case T::NOT_INITIALIZED: return State::INITIALIZING;
    case T::OK: case T::OK_KLT: return State::TRACKING;
    case T::RECENTLY_LOST: case T::LOST: return State::LOST;
    default:return State::FAULT;
    }
}
EngineResult OrbEngine::track(const Stereo& p,const std::vector<Combined>& batch) {
    if (!origin_) origin_=std::min(p.left.stamp.ns,batch.front().stamp.ns);
    auto seconds=[&](int64_t ns) { return double(ns-*origin_)*1e-9; };
    std::vector<ORB_SLAM3::IMU::Point> imu;
    for (const auto& m:batch) {
        const auto v=orb_motion(m); // range_error -> Adapter ENGINE_FAULT, no TrackStereo.
        imu.emplace_back(v[0],v[1],v[2],v[3],v[4],v[5],seconds(m.stamp.ns));
    }
    cv::Mat left(p.left.height,p.left.width,CV_8UC1,const_cast<uint8_t*>(p.left.y8.data()));
    cv::Mat right(p.right.height,p.right.width,CV_8UC1,const_cast<uint8_t*>(p.right.y8.data()));
    // System.h 107-110 / System.cc 316-325: returned Tcw maps world into camera.
    auto Tcw=system_.TrackStereo(left,right,seconds(p.left.stamp.ns),imu);
    EngineResult r; r.source=p.left.stamp; r.state=state(system_.GetTrackingState());
    r.discontinuity=!observer_ && system_.MapChanged(); // Loop/global BA, not a reset log.
    // INFERRED conservative boundary: reinitialization after loss/tracking.
    r.discontinuity=r.discontinuity ||
        ((previous_state_==State::TRACKING || previous_state_==State::LOST) &&
         (r.state==State::STARTING || r.state==State::INITIALIZING));
    previous_state_=r.state;
    if (observer_) {
        auto meta=observer_(); r.map_id=meta.map_id;
        if (previous_) r.discontinuity=r.discontinuity || meta.map_id!=previous_->map_id ||
            meta.reset_generation!=previous_->reset_generation || meta.big_change_index!=previous_->big_change_index;
        previous_=meta;
    }
    if (r.state==State::TRACKING) {
        Transform t; const auto Twc=Tcw.inverse().matrix();
        for (int i=0;i<4;++i) for (int j=0;j<4;++j) t.matrix[i*4+j]=Twc(i,j);
        r.T_world_camera=t;
    }
    // Stock ORB exposes no qualified pose covariance at this boundary.
    return r;
}
void OrbEngine::reset() { system_.ResetActiveMap(); previous_.reset(); previous_state_=State::INITIALIZING; }
}
