#pragma once
#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace wbr::vio {
struct Stamp { int64_t ns; std::string domain; };
struct Vec3 { double x, y, z; };
struct Motion { Stamp stamp; Vec3 value; };
struct Combined { Stamp stamp; Vec3 accel, gyro; };
struct Image { Stamp stamp; uint32_t width, height; std::vector<uint8_t> y8; };
struct Stereo { Image left, right; };
enum class State { STARTING, INITIALIZING, TRACKING, LOST, FAULT };
enum class Availability { UNAVAILABLE, AVAILABLE };
enum Reason : uint32_t { NONE=0, NO_ESTIMATE=1, NOT_TRACKING=2, RESET=4,
    MAP_CHANGE=8, STALE_ENGINE=16, BAD_FRAME=32, BAD_POSE=64, ENGINE_FAULT=128 };
struct Transform {
    std::string parent="slam_world", child="camera_left_optical";
    std::array<double,16> matrix{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
};
bool valid_transform(const Transform&);
Transform inverse(const Transform&);
struct EngineResult {
    Stamp source;
    State state=State::STARTING;
    std::optional<Transform> T_world_camera;
    bool discontinuity=false;
    std::optional<uint64_t> map_id;
    // Covariance order: x,y,z,rotation x,y,z in world tangent coordinates.
    std::optional<std::array<double,36>> covariance;
};
class Engine {
public:
    virtual ~Engine()=default;
    virtual EngineResult track(const Stereo&, const std::vector<Combined>&)=0;
    virtual void reset()=0;
};
struct EstimateSnapshot {
    std::optional<Stamp> source;
    uint64_t sequence=0, reset_counter=0, epoch_id=0;
    std::optional<uint64_t> map_id;
    State state=State::STARTING;
    bool valid=false;
    uint32_t reasons=NO_ESTIMATE;
    std::optional<Transform> T_world_camera;
    Availability covariance_availability=Availability::UNAVAILABLE;
    std::optional<std::array<double,36>> covariance;
};
struct Counters {
    uint64_t accel=0, gyro=0, left=0, right=0, pairs=0, combined=0;
    uint64_t domain=0, order=0, nonfinite=0, gap=0, overflow=0, image=0;
    uint64_t unmatched=0, prefix_gyro=0, prefix_pairs=0, missing_coverage=0;
    uint64_t tail_accel=0, tail_gyro=0, engine_rejected=0;
};
struct Limits { size_t imu=4096, images=8; int64_t max_gap_ns=50000000; };
// Single-owner synchronous API: caller serializes feed/reset/snapshot.
class Adapter {
public:
    Adapter(Engine&, std::string domain, Limits={});
    bool feed_accel(Motion); bool feed_gyro(Motion);
    bool feed_left(Image); bool feed_right(Image);
    bool feed_stereo(Stereo); // atomic validation before either side is enqueued
    EstimateSnapshot snapshot() const { return snapshot_; }
    EstimateSnapshot heartbeat() const { return snapshot_; }
    const Counters& counters() const { return counts_; }
    void reset();
    std::function<void(const EstimateSnapshot&)> on_snapshot; // optional synchronous observer
    void finish(); // classify pending unmatched images/coverage and unused IMU tail
private:
    Engine& engine_; std::string domain_; Limits limits_; Counters counts_;
    std::deque<Motion> accel_, gyro_; std::deque<Image> left_, right_;
    std::optional<int64_t> last_accel_, last_gyro_, last_left_, last_right_, last_processed_;
    EstimateSnapshot snapshot_; bool finished_=false;
    bool append(Motion, std::deque<Motion>&, std::optional<int64_t>&, uint64_t&);
    bool append(Image, std::deque<Image>&, std::optional<int64_t>&, uint64_t&);
    void discard_pending();
    void drain(); void publish(EngineResult, Stamp); void invalidate(uint32_t);
};
const char* state_name(State);
} // namespace wbr::vio
