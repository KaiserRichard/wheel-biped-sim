#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace wbr::orb_node {
using MapXYZ = std::array<float, 3>;
struct MapSnapshot {
    std::vector<MapXYZ> points;
    uint64_t map_id = 0, init_kf_id = 0, candidates = 0;
    int big_change = 0, map_change = 0;
    bool truncated = false;
};

inline bool finite_point(const MapXYZ& p) {
    return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

// Wall/steady time bounds live work even when bag timestamps jump forward.
// A source-time gate also avoids repeated snapshots when playback is slowed.
class MapSnapshotRate {
 public:
    explicit MapSnapshotRate(double hz) {
        if (!std::isfinite(hz) || hz < 0 || hz > 1 || (hz > 0 && hz < 0.001))
            throw std::invalid_argument("map_points_hz must be 0 (off) or 0.001..1 Hz");
        period_ = hz == 0 ? 0 : static_cast<int64_t>(std::ceil(1e9 / hz));
    }
    bool due(int64_t source_ns, int64_t steady_ns) {
        if (!period_) return false;
        if (started_ && (source_ns - source_ < period_ || steady_ns - steady_ < period_))
            return false;
        started_ = true;
        source_ = source_ns;
        steady_ = steady_ns;
        return true;
    }
 private:
    int64_t period_ = 0, source_ = 0, steady_ = 0;
    bool started_ = false;
};

// Kept ROS-independent so the exact byte writer can also run on the host.
// Cloud is sensor_msgs::msg::PointCloud2 in production and in the ROS test.
template<class Cloud>
Cloud map_points_cloud(const MapSnapshot& snapshot, int32_t sec, uint32_t nanosec,
                       const std::string& frame) {
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
    if (frame.empty() || sec < 0 || nanosec >= 1000000000 || snapshot.points.size() > 200000)
        throw std::invalid_argument("invalid map cloud header or point limit");
    Cloud cloud;
    cloud.header.stamp.sec = sec;
    cloud.header.stamp.nanosec = nanosec;
    cloud.header.frame_id = frame;
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 12;
    cloud.fields.resize(3);
    for (uint32_t axis = 0; axis < 3; ++axis) {
        auto& field = cloud.fields[axis];
        field.name = std::string(1, "xyz"[axis]);
        field.offset = axis * 4;
        field.datatype = 7;  // sensor_msgs/PointField.FLOAT32
        field.count = 1;
    }
    cloud.data.reserve(snapshot.points.size() * cloud.point_step);
    for (const auto& point : snapshot.points) {
        if (!finite_point(point)) continue;
        for (float coordinate : point) {
            uint32_t bits;
            std::memcpy(&bits, &coordinate, sizeof(bits));
            for (unsigned shift = 0; shift < 32; shift += 8)
                cloud.data.push_back(static_cast<uint8_t>(bits >> shift));
        }
    }
    cloud.width = static_cast<uint32_t>(cloud.data.size() / cloud.point_step);
    cloud.row_step = cloud.width * cloud.point_step;
    return cloud;
}
}  // namespace wbr::orb_node
