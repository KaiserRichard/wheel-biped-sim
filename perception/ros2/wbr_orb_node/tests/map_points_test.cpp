#include "wbr_orb_node/map_points.hpp"
#include <iostream>
#include <utility>
#ifdef WBR_TEST_ROS_CLOUD
#include <sensor_msgs/msg/point_cloud2.hpp>
using Cloud = sensor_msgs::msg::PointCloud2;
#else
// Host fixture for the message API. Pi CTest instantiates the same byte writer
// and assertions with the actual generated ROS PointCloud2 type.
struct Cloud {
    struct Stamp { int32_t sec = 0; uint32_t nanosec = 0; };
    struct Header { Stamp stamp; std::string frame_id; } header;
    struct Field { std::string name; uint32_t offset = 0, count = 0; uint8_t datatype = 0; };
    uint32_t height = 0, width = 0, point_step = 0, row_step = 0;
    bool is_bigendian = false, is_dense = false;
    std::vector<Field> fields;
    std::vector<uint8_t> data;
};
#endif
#define CHECK(condition) do { if (!(condition)) throw std::runtime_error(#condition); } while (false)
using namespace wbr::orb_node;
template<class F> void rejects(F fn) {
    bool rejected = false;
    try { fn(); } catch (const std::invalid_argument&) { rejected = true; }
    CHECK(rejected);
}
int main() try {
    MapSnapshot snapshot;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    snapshot.points = {{1.0f, -2.0f, 0.5f}, {nan, 1, 2}, {1, inf, 2}, {1, 2, -inf}};
    const auto cloud = map_points_cloud<Cloud>(snapshot, 1792548001, 123456789, "slam_world");
    CHECK(cloud.header.stamp.sec == 1792548001 && cloud.header.stamp.nanosec == 123456789);
    CHECK(cloud.header.frame_id == "slam_world");
    CHECK(cloud.height == 1 && cloud.width == 1 && cloud.point_step == 12 && cloud.row_step == 12);
    CHECK(!cloud.is_bigendian && cloud.is_dense && cloud.fields.size() == 3);
    for (size_t i = 0; i < 3; ++i) {
        CHECK(cloud.fields[i].name == std::string(1, "xyz"[i]));
        CHECK(cloud.fields[i].offset == i * 4 && cloud.fields[i].datatype == 7 && cloud.fields[i].count == 1);
    }
    const std::vector<uint8_t> expected{0,0,128,63, 0,0,0,192, 0,0,0,63};
    CHECK(cloud.data == expected);
    snapshot.points.clear();
    const auto empty = map_points_cloud<Cloud>(snapshot, 1, 0, "custom_world");
    CHECK(empty.width == 0 && empty.row_step == 0 && empty.data.empty() && empty.fields.size() == 3);
    CHECK(empty.header.frame_id == "custom_world");
    rejects([&] { map_points_cloud<Cloud>(snapshot, 1, 0, ""); });
    rejects([&] { map_points_cloud<Cloud>(snapshot, -1, 0, "world"); });
    rejects([&] { map_points_cloud<Cloud>(snapshot, 1, 1000000000, "world"); });
    snapshot.points.resize(200001);
    rejects([&] { map_points_cloud<Cloud>(snapshot, 1, 0, "world"); });
    MapSnapshotRate rate(1);
    CHECK(rate.due(1000000000, 1000000000));
    CHECK(!rate.due(2000000000, 1999999999)); // accelerated replay wall bound
    CHECK(rate.due(2000000000, 2000000000));
    CHECK(!rate.due(2999999999, 4000000000)); // slowed replay source bound
    CHECK(rate.due(3000000000, 4000000000));
    CHECK(!rate.due(0, 5000000000)); // no cloud after backwards source seek
    MapSnapshotRate disabled(0), slow(0.5);
    CHECK(!disabled.due(0, 0));
    CHECK(slow.due(0, 0) && !slow.due(1000000000, 1000000000));
    CHECK(slow.due(2000000000, 2000000000));
    for (double hz : {-1., 1.01, 0.0001, static_cast<double>(nan), static_cast<double>(inf)})
        rejects([&] { MapSnapshotRate invalid(hz); });
    std::cout << "PASS: XYZ layout, endian bytes, finite filtering, exact stamp/frame, empty replacement, bounds, rate gates\n";
    return 0;
} catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
}
