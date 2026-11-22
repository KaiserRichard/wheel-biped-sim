#include "adapter.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <utility>

namespace wbr::vio {
bool valid_transform(const Transform& t) {
    const auto& m=t.matrix;
    for (double v:m) if (!std::isfinite(v)) return false;
    if (t.parent.empty() || t.child.empty() || t.parent==t.child) return false;
    for (int j=0;j<4;++j) if (std::abs(m[12+j]-(j==3?1:0))>1e-6) return false;
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) {
        double dot=0; for (int k=0;k<3;++k) dot+=m[4*k+i]*m[4*k+j];
        if (std::abs(dot-(i==j?1:0))>1e-4) return false;
    }
    double det=m[0]*(m[5]*m[10]-m[6]*m[9])-m[1]*(m[4]*m[10]-m[6]*m[8])+m[2]*(m[4]*m[9]-m[5]*m[8]);
    return std::abs(det-1)<1e-4;
}
Transform inverse(const Transform& t) {
    if (!valid_transform(t)) throw std::invalid_argument("invalid named SE(3)");
    Transform out; out.parent=t.child; out.child=t.parent;
    for (int i=0;i<3;++i) {
        for (int j=0;j<3;++j) out.matrix[4*i+j]=t.matrix[4*j+i];
        out.matrix[4*i+3]=0;
        for (int j=0;j<3;++j) out.matrix[4*i+3]-=out.matrix[4*i+j]*t.matrix[4*j+3];
    }
    return out;
}
Adapter::Adapter(Engine& e,std::string d,Limits l):engine_(e),domain_(std::move(d)),limits_(l) {
    if (domain_.empty() || l.imu<2 || l.images<1 || l.max_gap_ns<=0) throw std::invalid_argument("invalid domain/limits");
}
bool Adapter::append(Motion m,std::deque<Motion>& q,std::optional<int64_t>& last,uint64_t& accepted) {
    if (finished_) throw std::logic_error("feed after finish");
    if (m.stamp.domain!=domain_) { ++counts_.domain; return false; }
    if (m.stamp.ns<0 || (last && m.stamp.ns<=*last)) { ++counts_.order; return false; }
    if (!std::isfinite(m.value.x)||!std::isfinite(m.value.y)||!std::isfinite(m.value.z)) { ++counts_.nonfinite; return false; }
    if (last && m.stamp.ns-*last>limits_.max_gap_ns) { ++counts_.gap; return false; }
    if (q.size()>=limits_.imu) { ++counts_.overflow; return false; }
    last=m.stamp.ns; q.push_back(std::move(m)); ++accepted; drain(); return true;
}
bool Adapter::append(Image m,std::deque<Image>& q,std::optional<int64_t>& last,uint64_t& accepted) {
    if (finished_) throw std::logic_error("feed after finish");
    if (m.stamp.domain!=domain_) { ++counts_.domain; return false; }
    if (m.stamp.ns<0 || (last && m.stamp.ns<=*last) || (last_processed_ && m.stamp.ns<=*last_processed_)) { ++counts_.order; return false; }
    if (!m.width || !m.height || m.width>8192 || m.height>8192 || m.y8.size()!=uint64_t(m.width)*m.height) { ++counts_.image; return false; }
    if (q.size()>=limits_.images) { ++counts_.overflow; return false; }
    last=m.stamp.ns; q.push_back(std::move(m)); ++accepted; drain(); return true;
}
bool Adapter::feed_accel(Motion m) { return append(std::move(m),accel_,last_accel_,counts_.accel); }
bool Adapter::feed_gyro(Motion m) { return append(std::move(m),gyro_,last_gyro_,counts_.gyro); }
bool Adapter::feed_left(Image m) { return append(std::move(m),left_,last_left_,counts_.left); }
bool Adapter::feed_right(Image m) { return append(std::move(m),right_,last_right_,counts_.right); }
bool Adapter::feed_stereo(Stereo p) {
    if (finished_) throw std::logic_error("feed after finish");
    if (p.left.stamp.domain!=domain_ || p.right.stamp.domain!=domain_) { ++counts_.domain; return false; }
    const auto t=p.left.stamp.ns;
    if (t!=p.right.stamp.ns) { ++counts_.unmatched; return false; }
    if (t<0 || (last_left_ && t<=*last_left_) || (last_right_ && t<=*last_right_) ||
        (last_processed_ && t<=*last_processed_)) { ++counts_.order; return false; }
    const auto good=[](const Image& i) { return i.width && i.height && i.width<=8192 && i.height<=8192 && i.y8.size()==uint64_t(i.width)*i.height; };
    if (!good(p.left) || !good(p.right) || p.left.width!=p.right.width || p.left.height!=p.right.height) { ++counts_.image; return false; }
    if (left_.size()>=limits_.images || right_.size()>=limits_.images) { ++counts_.overflow; return false; }
    last_left_=t; last_right_=t;
    left_.push_back(std::move(p.left)); right_.push_back(std::move(p.right));
    ++counts_.left; ++counts_.right; drain(); return true;
}
void Adapter::invalidate(uint32_t reason) {
    snapshot_.valid=false; snapshot_.reasons=reason; snapshot_.T_world_camera.reset();
    snapshot_.covariance.reset(); snapshot_.covariance_availability=Availability::UNAVAILABLE;
}
void Adapter::publish(EngineResult r,Stamp expected) {
    if (r.source.domain!=domain_ || r.source.ns!=expected.ns ||
        (snapshot_.source && r.source.ns<=snapshot_.source->ns)) {
        ++counts_.engine_rejected; invalidate(STALE_ENGINE); snapshot_.state=State::FAULT; return;
    }
    bool changed=r.discontinuity || (snapshot_.map_id && r.map_id && snapshot_.map_id!=r.map_id);
    if (changed) { ++snapshot_.reset_counter; ++snapshot_.epoch_id; }
    snapshot_.source=r.source; ++snapshot_.sequence; snapshot_.map_id=r.map_id; snapshot_.state=r.state;
    invalidate(changed?MAP_CHANGE:NOT_TRACKING);
    if (changed || r.state!=State::TRACKING) return;
    if (!r.T_world_camera) { invalidate(BAD_POSE); return; }
    const auto& t=*r.T_world_camera;
    if (t.parent!="slam_world" || t.child!="camera_left_optical") { invalidate(BAD_FRAME); ++counts_.engine_rejected; return; }
    if (!valid_transform(t)) { invalidate(BAD_POSE); ++counts_.engine_rejected; return; }
    snapshot_.T_world_camera=t; snapshot_.valid=true; snapshot_.reasons=NONE;
    if (r.covariance) {
        bool finite=true; for (double v:*r.covariance) finite=finite && std::isfinite(v);
        if (finite) { snapshot_.covariance=r.covariance; snapshot_.covariance_availability=Availability::AVAILABLE; }
    }
}
void Adapter::drain() {
    // Adapted from first-party d435i_input.h (2026-10-13 prep), see SOURCES.md.
    // Integer ns replace double seconds; bracket, prefix and endpoint intent retained.
    while (!accel_.empty() && !gyro_.empty() && gyro_.front().stamp.ns<accel_.front().stamp.ns) {
        gyro_.pop_front(); ++counts_.prefix_gyro;
    }
    while (!left_.empty() && !right_.empty()) {
        auto t=left_.front().stamp.ns;
        if (t!=right_.front().stamp.ns) {
            if (t<right_.front().stamp.ns) left_.pop_front(); else right_.pop_front();
            ++counts_.unmatched; continue;
        }
        if (!gyro_.empty() && !last_processed_ && t<gyro_.front().stamp.ns) {
            left_.pop_front(); right_.pop_front(); ++counts_.prefix_pairs; continue;
        }
        if (accel_.size()<2 || gyro_.empty()) return;
        auto end=gyro_.begin(); while (end!=gyro_.end() && end->stamp.ns<t) ++end;
        if (end==gyro_.end() || accel_.front().stamp.ns>gyro_.front().stamp.ns || accel_.back().stamp.ns<end->stamp.ns) return;
        Stereo pair{std::move(left_.front()),std::move(right_.front())}; left_.pop_front(); right_.pop_front();
        if (pair.left.width!=pair.right.width || pair.left.height!=pair.right.height) { ++counts_.image; continue; }
        std::vector<Combined> batch;
        do {
            auto g=gyro_.front(); gyro_.pop_front();
            while (accel_.size()>2 && accel_[1].stamp.ns<=g.stamp.ns) accel_.pop_front();
            const auto& a=accel_[0]; const auto& b=accel_[1];
            double u=double(g.stamp.ns-a.stamp.ns)/double(b.stamp.ns-a.stamp.ns);
            Vec3 v{(1-u)*a.value.x+u*b.value.x,(1-u)*a.value.y+u*b.value.y,(1-u)*a.value.z+u*b.value.z};
            batch.push_back({g.stamp,v,g.value});
        } while (batch.back().stamp.ns<t);
        ++counts_.pairs; counts_.combined+=batch.size(); last_processed_=t;
        try { publish(engine_.track(pair,batch),pair.left.stamp); }
        catch (const std::exception&) { ++counts_.engine_rejected; snapshot_.state=State::FAULT; invalidate(ENGINE_FAULT); }
        if (on_snapshot) on_snapshot(snapshot_);
    }
}
void Adapter::discard_pending() {
    while (!left_.empty() && !right_.empty()) {
        if (left_.front().stamp.ns==right_.front().stamp.ns) { ++counts_.missing_coverage; left_.pop_front(); right_.pop_front(); }
        else { ++counts_.unmatched; if (left_.front().stamp.ns<right_.front().stamp.ns) left_.pop_front(); else right_.pop_front(); }
    }
    counts_.unmatched+=left_.size()+right_.size(); counts_.tail_accel+=accel_.size(); counts_.tail_gyro+=gyro_.size();
    left_.clear(); right_.clear(); accel_.clear(); gyro_.clear();
}
void Adapter::reset() {
    if (finished_) throw std::logic_error("reset after finish");
    ++snapshot_.reset_counter; ++snapshot_.epoch_id;
    snapshot_.state=State::INITIALIZING; invalidate(RESET); snapshot_.map_id.reset();
    // Keep watermarks/source/sequence: reset is not permission to replay stale epochs.
    discard_pending();
    try { engine_.reset(); }
    catch (const std::exception&) { ++counts_.engine_rejected; snapshot_.state=State::FAULT; invalidate(ENGINE_FAULT); }
}
void Adapter::finish() {
    if (finished_) return;
    drain(); discard_pending(); finished_=true;
}
const char* state_name(State s) {
    switch(s) { case State::STARTING:return "STARTING"; case State::INITIALIZING:return "INITIALIZING";
    case State::TRACKING:return "TRACKING"; case State::LOST:return "LOST"; case State::FAULT:return "FAULT"; }
    return "FAULT";
}
}
