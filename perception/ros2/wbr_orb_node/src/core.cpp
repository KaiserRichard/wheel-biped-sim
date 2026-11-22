#include "wbr_orb_node/core.hpp"
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace wbr::orb_node {
namespace {
int64_t stamp(const Input& input) {
    return std::visit([](const auto& value) -> int64_t {
        using T=std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T,Imu>) return value.stamp.ns;
        else return value.image.stamp.ns;
    }, input);
}
const char* sender_state(vio::State state) {
    // sender.py consumes stock ORB tracking names, not adapter enum labels.
    switch (state) {
        case vio::State::STARTING: case vio::State::FAULT: return "SYSTEM_NOT_READY";
        case vio::State::INITIALIZING: return "NOT_INITIALIZED";
        case vio::State::TRACKING: return "OK";
        case vio::State::LOST: return "LOST";
    }
    return "SYSTEM_NOT_READY";
}
bool finite(vio::Vec3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }
void validate(const Config& c) {
    if (!c.ingress_capacity || !c.image_capacity || c.imu_capacity<2 || !c.max_image_bytes ||
        c.max_imu_gap_ns<=0 || c.source_domain.empty() || c.source_session.empty())
        throw std::invalid_argument("invalid core bounds/clock identity");
    // Exercise the recorder's token validation before producers start.
    observer::Recorder<1> token_check(c.source_session.c_str());
    observer::SourceEpoch epoch{0,c.source_domain.c_str(),c.source_session.c_str()};
    if (!token_check.record(observer::Kind::frame_received,0,&epoch))
        throw std::invalid_argument("invalid recorder clock token");
}
}
Ingress::Ingress(const Config& c, Recorder& e):config_(c),events_(e) { validate(c); }
uint64_t Ingress::received(int64_t source_ns) {
    const auto id=next_.fetch_add(1);
    observer::SourceEpoch epoch{source_ns,config_.source_domain.c_str(),config_.source_session.c_str()};
    events_.record(observer::Kind::frame_received,id,&epoch);
    return id;
}
bool Ingress::submit(Input input,std::optional<uint64_t> receipt_id) {
    const auto id=receipt_id?*receipt_id:received(stamp(input));
    observer::SourceEpoch epoch{stamp(input),config_.source_domain.c_str(),config_.source_session.c_str()};
    std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
    if (!lock.owns_lock()) {
        ++contention_;
        if (std::holds_alternative<Imu>(input)) ++imu_loss_;
        events_.record(observer::Kind::dropped,id,&epoch,"ingress_contention");
        return false;
    }
    if (queue_.size()==config_.ingress_capacity) {
        const auto& old=queue_.front();
        observer::SourceEpoch old_epoch{stamp(old.input),config_.source_domain.c_str(),config_.source_session.c_str()};
        events_.record(observer::Kind::dropped,old.id,&old_epoch,"ingress_drop_oldest");
        if (std::holds_alternative<Imu>(old.input)) ++imu_loss_;
        queue_.pop_front(); ++oldest_;
    }
    queue_.push_back({id,std::move(input)});
    events_.record(observer::Kind::queued,id,&epoch);
    events_.record(observer::Kind::queue_depth,0,nullptr,nullptr,queue_.size());
    return true;
}
std::optional<Message> Ingress::pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) return std::nullopt;
    auto value=std::move(queue_.front()); queue_.pop_front();
    events_.record(observer::Kind::queue_depth,0,nullptr,nullptr,queue_.size());
    return value;
}
QueueCounts Ingress::counts() const { return {oldest_.load(),contention_.load(),imu_loss_.load()}; }
size_t Ingress::depth() const { std::lock_guard<std::mutex> lock(mutex_); return queue_.size(); }
Core::Core(vio::Engine& engine,const Config& c,Recorder& events)
    :config_(c),events_(events),timed_(engine,*this),
     adapter_(timed_,c.source_domain,{c.imu_capacity+2,1,c.max_imu_gap_ns}) {
    validate(c);
    adapter_.on_snapshot=[this](const vio::EstimateSnapshot& snapshot) {
        ++counts_.processed;
        if (on_output) on_output({active_id_,snapshot,active_source_,config_.source_session});
    };
}
void Core::event(observer::Kind kind,uint64_t id,int64_t ns,const char* reason) {
    observer::SourceEpoch epoch{ns,config_.source_domain.c_str(),config_.source_session.c_str()};
    events_.record(kind,id,&epoch,reason);
}
void Core::drop(const ImageEntry& entry,const char* reason) {
    event(observer::Kind::dropped,entry.id,entry.image.stamp.ns,reason);
}
void Core::input_imu_loss() { if (!fault_) { fault_=true; ++counts_.source_fault; } }
void Core::image(ImageEntry entry,bool left) {
    auto& watermark=left?last_left_:last_right_;
    auto& queue=left?left_:right_;
    const auto& i=entry.image;
    if (i.stamp.domain!=config_.source_domain || i.stamp.ns<0 ||
        (watermark && i.stamp.ns<=*watermark)) {
        ++counts_.invalid; input_imu_loss(); drop(entry,"image_clock_fault"); return;
    }
    if (!i.width || !i.height || uint64_t(i.width)*i.height>config_.max_image_bytes ||
        i.y8.size()!=uint64_t(i.width)*i.height) {
        ++counts_.invalid; drop(entry,"invalid_image"); return;
    }
    watermark=i.stamp.ns;
    if (queue.size()==config_.image_capacity) {
        drop(queue.front(),"image_drop_oldest"); queue.pop_front();
        if (left) ++counts_.left_oldest; else ++counts_.right_oldest;
    }
    queue.push_back(std::move(entry));
}
void Core::consume(Message message) {
    if (finished_) throw std::logic_error("consume after finish");
    if (fault_) {
        event(observer::Kind::dropped,message.id,stamp(message.input),"session_fault"); return;
    }
    if (auto* i=std::get_if<Imu>(&message.input)) {
        if (i->stamp.domain!=config_.source_domain || i->stamp.ns<0 ||
            (last_imu_ && (i->stamp.ns<=*last_imu_ || i->stamp.ns-*last_imu_>config_.max_imu_gap_ns)) ||
            !finite(i->accel) || !finite(i->gyro)) {
            ++counts_.invalid; input_imu_loss();
            event(observer::Kind::dropped,message.id,i->stamp.ns,"imu_clock_or_value_fault"); return;
        }
        last_imu_=i->stamp.ns;
        if (imu_.size()==config_.imu_capacity) {
            const auto old=imu_.front(); imu_.pop_front(); ++counts_.imu_oldest; input_imu_loss();
            event(observer::Kind::dropped,old.id,old.sample.stamp.ns,"imu_drop_oldest_fault");
        }
        imu_.push_back({message.id,*i});
    } else if (auto* l=std::get_if<Left>(&message.input)) image({message.id,std::move(l->image)},true);
    else image({message.id,std::move(std::get<Right>(message.input).image)},false);
    if (!fault_) pump();
}
void Core::pump() {
    while (!left_.empty() && !right_.empty()) {
        const auto t=left_.front().image.stamp.ns, r=right_.front().image.stamp.ns;
        // Transport pairing only. The adapter validates the final atomic stereo
        // pair and owns all interpolation, coverage, prefix and output semantics.
        if (t!=r) {
            auto& queue=t<r?left_:right_;
            drop(queue.front(),"stereo_unmatched"); queue.pop_front(); ++counts_.unmatched; continue;
        }
        if (imu_.empty() || imu_.back().sample.stamp.ns<t || (fed_imu_+imu_.size()<2)) return;
        // Supply only through the first gyro endpoint at/after this image.
        // Co-stamped sensor_msgs/Imu provides the matching accel bracket.
        bool endpoint=false;
        do {
            auto sample=imu_.front().sample; imu_.pop_front();
            endpoint=sample.stamp.ns>=t;
            bool accepted=adapter_.feed_accel({sample.stamp,sample.accel});
            accepted=adapter_.feed_gyro({sample.stamp,sample.gyro}) && accepted;
            ++fed_imu_;
            if (!accepted) { input_imu_loss(); return; }
        } while ((!endpoint || fed_imu_<2) && !imu_.empty());
        active_id_=left_.front().id; active_source_=left_.front().image.stamp;
        const auto before=adapter_.counters();
        vio::Stereo pair{std::move(left_.front().image),std::move(right_.front().image)};
        left_.pop_front(); right_.pop_front();
        if (!adapter_.feed_stereo(std::move(pair))) {
            // A rejected stereo pair is an image/association fault, not evidence
            // that an IMU sample was lost.  Source clock/value faults still fail
            // closed through their own validation paths above.
            ++counts_.invalid;
            if (adapter_.counters().image==before.image)
                event(observer::Kind::dropped,active_id_,t,"adapter_stereo_rejected");
        }
        if (adapter_.counters().prefix_pairs>before.prefix_pairs)
            event(observer::Kind::dropped,active_id_,t,"adapter_prefix_pair");
        if (adapter_.counters().image>before.image)
            event(observer::Kind::dropped,active_id_,t,"adapter_image_invalid");
    }
}
vio::EngineResult Core::TimedEngine::track(const vio::Stereo& pair,const std::vector<vio::Combined>& imu) {
    core_.event(observer::Kind::track_start,core_.active_id_,pair.left.stamp.ns);
    try {
        auto result=engine_.track(pair,imu);
        core_.event(observer::Kind::track_end,core_.active_id_,pair.left.stamp.ns);
        return result;
    } catch (...) {
        core_.event(observer::Kind::track_end,core_.active_id_,pair.left.stamp.ns); throw;
    }
}
void Core::output_written(const Output& output) {
    event(observer::Kind::snapshot_published,output.frame_id,output.source_image.ns);
    ++counts_.output;
}
void Core::finish() {
    if (finished_) return;
    counts_.shutdown_images=left_.size()+right_.size(); counts_.shutdown_imu_tail=imu_.size();
    for (const auto& entry:left_) drop(entry,"shutdown_pending");
    for (const auto& entry:right_) drop(entry,"shutdown_pending");
    left_.clear(); right_.clear(); imu_.clear(); adapter_.finish(); finished_=true;
}
std::optional<bridge::PoseFields> odometry_pose(const vio::EstimateSnapshot& s,const vio::Transform& extrinsic) {
    if (!s.valid || s.state!=vio::State::TRACKING || s.reasons!=vio::NONE || !s.source ||
        s.source->ns<0 || s.source->ns>2147483647999999999LL || !s.T_world_camera ||
        s.T_world_camera->parent!="slam_world" || s.T_world_camera->child!="camera_left_optical" ||
        extrinsic.parent!="camera_left_optical" || extrinsic.child!="imu_camera" ||
        !vio::valid_transform(extrinsic) || !vio::valid_transform(*s.T_world_camera)) return std::nullopt;
    auto pose=*s.T_world_camera;
    pose.child="imu_camera";
    for (int i=0;i<4;++i) for (int j=0;j<4;++j) {
        pose.matrix[4*i+j]=0;
        for (int k=0;k<4;++k) pose.matrix[4*i+j]+=s.T_world_camera->matrix[4*i+k]*extrinsic.matrix[4*k+j];
    }
    return bridge::pose_fields(pose,s.source->ns); // quaternion/stamp extraction is reused
}
std::string health_json(const Output& out,const std::string& mapping_json,bool fake) {
    std::ostringstream stream;
    stream << "{\"pose_source_d435i_ns\":" << out.source_image.ns
           << ",\"mapping\":" << mapping_json
           << ",\"tracking_state\":\"" << sender_state(out.snapshot.state)
           << "\",\"adapter_tracking_state\":\"" << vio::state_name(out.snapshot.state)
           << "\",\"map_reset_counter\":" << out.snapshot.reset_counter
           // Adapter does not expose engine IMU initialization or feature counts.
           // False/zero are conservative UNKNOWN sentinels, not measured health.
           << ",\"imu_initialized\":false,\"tracked_features\":0,\"degraded\":true"
           << ",\"imu_initialized_evidence\":\"UNKNOWN\",\"tracked_features_evidence\":\"UNKNOWN\""
           << ",\"engine\":\"" << (fake?"fake":"ORB_SLAM3")
           << "\",\"evidence\":\"" << (fake?"ASSUMED synthetic estimate":"INFERRED adapter estimate")
           << "\",\"source_clock_domain\":\"" << out.source_image.domain
           << "\",\"source_session\":\"" << out.source_session
           << "\",\"sequence\":" << out.snapshot.sequence << ",\"epoch_id\":" << out.snapshot.epoch_id
           << ",\"pose_valid\":" << (out.snapshot.valid?"true":"false") << "}";
    return stream.str();
}
} // namespace wbr::orb_node
