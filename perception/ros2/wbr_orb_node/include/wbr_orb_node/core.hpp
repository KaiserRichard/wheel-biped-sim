#pragma once
#include "adapter.h"
#include "event_recorder.h"
#include "mapping.hpp"
#include "defaults.hpp"
#include <atomic>
#include <deque>
#include <functional>
#include <mutex>
#include <variant>

namespace wbr::orb_node {
using Recorder = observer::Recorder<defaults::event_capacity>;
struct Config {
    size_t ingress_capacity=defaults::ingress_capacity, image_capacity=defaults::image_capacity,
           imu_capacity=defaults::imu_capacity, max_image_bytes=defaults::max_image_bytes;
    int64_t max_imu_gap_ns=defaults::max_imu_gap_ns;
    std::string source_domain="ros_header", source_session="unbound";
};
struct Imu { vio::Stamp stamp; vio::Vec3 accel, gyro; };
struct Left { vio::Image image; };
struct Right { vio::Image image; };
using Input = std::variant<Left, Right, Imu>;
struct Message { uint64_t id; Input input; };
struct QueueCounts { uint64_t oldest=0, contention=0, imu_loss=0; };
// Callbacks only try_lock and retain an owning message; no engine or stream I/O.
class Ingress {
 public:
    Ingress(const Config&, Recorder&);
    uint64_t received(int64_t source_ns); // callback entry, before image copy
    bool submit(Input, std::optional<uint64_t> receipt_id=std::nullopt);
    std::optional<Message> pop(); // runner only
    QueueCounts counts() const;
    size_t depth() const;
 private:
    Config config_; Recorder& events_; mutable std::mutex mutex_;
    std::deque<Message> queue_; std::atomic<uint64_t> next_{1}, oldest_{0}, contention_{0}, imu_loss_{0};
};
struct Counts {
    uint64_t unmatched=0, left_oldest=0, right_oldest=0, imu_oldest=0, invalid=0,
             source_fault=0, processed=0, output=0, shutdown_images=0, shutdown_imu_tail=0;
};
struct Output {
    uint64_t frame_id; vio::EstimateSnapshot snapshot;
    vio::Stamp source_image; std::string source_session;
};
// Single tracking runner owns this class and the borrowed engine.
class Core {
 public:
    Core(vio::Engine&, const Config&, Recorder&);
    void consume(Message);
    void finish();
    void input_imu_loss(); // fail closed if callback mailbox lost an IMU sample
    std::function<void(const Output&)> on_output;
    const Counts& counts() const { return counts_; }
    const vio::Counters& adapter_counts() const { return adapter_.counters(); }
    bool faulted() const { return fault_; }
    void output_written(const Output&); // caller marks actual publication/write boundary
 private:
    struct ImageEntry { uint64_t id; vio::Image image; };
    struct ImuEntry { uint64_t id; Imu sample; };
    class TimedEngine : public vio::Engine {
     public:
        TimedEngine(vio::Engine& engine, Core& core):engine_(engine),core_(core) {}
        vio::EngineResult track(const vio::Stereo&, const std::vector<vio::Combined>&) override;
        void reset() override { engine_.reset(); }
     private: vio::Engine& engine_; Core& core_;
    };
    Config config_; Recorder& events_; Counts counts_;
    TimedEngine timed_; vio::Adapter adapter_;
    std::deque<ImageEntry> left_, right_; std::deque<ImuEntry> imu_;
    std::optional<int64_t> last_left_,last_right_,last_imu_;
    uint64_t active_id_=0; vio::Stamp active_source_{0,""}; size_t fed_imu_=0; bool fault_=false,finished_=false;
    void image(ImageEntry, bool left);
    void pump();
    void event(observer::Kind, uint64_t, int64_t, const char* reason=nullptr);
    void drop(const ImageEntry&, const char*);
};
// Compose the adapter's optical camera pose with measured T_camera_left_optical_imu_camera.
// Uses tools/frames naming/direction. odom is a task-specific diagnostic view only.
std::optional<bridge::PoseFields> odometry_pose(const vio::EstimateSnapshot&,
                                               const vio::Transform& camera_from_imu);
std::string health_json(const Output&, const std::string& mapping_json, bool fake);
} // namespace wbr::orb_node
