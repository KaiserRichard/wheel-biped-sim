// Visual-only entry point in the existing wrapper. No fake engine or body pose.
#include <System.h>
#include "wbr_orb_node/orb_map_snapshot.hpp"
#include <Tracking.h>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace wbr::orb_node {
using Image = sensor_msgs::msg::Image;

class D435Node : public rclcpp::Node {
 public:
    D435Node() : Node("d435_orb") {
        const auto mode = declare_parameter<std::string>("mode", "stereo");
        if (mode != "stereo" && mode != "rgbd")
            throw std::invalid_argument("mode must be stereo or rgbd");
        stereo_ = mode == "stereo";
        const auto vocabulary = declare_parameter<std::string>("orb_vocabulary", "");
        const auto settings = declare_parameter<std::string>("orb_settings", "");
        if (!std::filesystem::is_regular_file(vocabulary) ||
            !std::filesystem::is_regular_file(settings))
            throw std::invalid_argument("existing orb_vocabulary and orb_settings files required");
        queue_size_ = declare_parameter<int>("queue_size", 5);
        max_skew_ns_ = declare_parameter<int64_t>("max_skew_ns", stereo_ ? 0 : 10000000);
        world_frame_ = declare_parameter<std::string>("world_frame", "slam_world");
        if (queue_size_ < 1 || queue_size_ > 100 || max_skew_ns_ < 0 ||
            max_skew_ns_ > 100000000 || world_frame_.empty())
            throw std::invalid_argument("invalid queue_size, max_skew_ns or world_frame");
        if (stereo_ && max_skew_ns_ != 0)
            throw std::invalid_argument("D435 stereo requires equal source stamps");

        map_rate_ = std::make_unique<MapSnapshotRate>(declare_parameter<double>("map_points_hz", 1.0));
        const auto point_limit = declare_parameter<int>("map_points_max", 50000);
        if (point_limit < 1 || point_limit > 200000)
            throw std::invalid_argument("map_points_max must be 1..200000");
        map_limit_ = static_cast<size_t>(point_limit);

        cv::FileStorage config(settings, cv::FileStorage::READ);
        if (!config.isOpened()) throw std::invalid_argument("cannot read ORB settings");
        width_ = static_cast<int>(config["Camera.width"]);
        height_ = static_cast<int>(config["Camera.height"]);
        const auto type = static_cast<std::string>(config["Camera.type"]);
        if (width_ <= 0 || height_ <= 0 ||
            (stereo_ ? type != "Rectified" : type != "PinHole"))
            throw std::invalid_argument("use camera_day SDK converter settings for this mode");
        if (!stereo_ && (config["RGBD.DepthMapFactor"].empty() ||
                        static_cast<double>(config["RGBD.DepthMapFactor"]) != 1.0 ||
                        static_cast<int>(config["Camera.RGB"]) != 1))
            throw std::invalid_argument("RGB-D input is RGB8 and normalized metres: factor=1, RGB=1");
        system_ = std::make_unique<ORB_SLAM3::System>(
            vocabulary, settings, stereo_ ? ORB_SLAM3::System::STEREO : ORB_SLAM3::System::RGBD,
            false);
        pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
        status_ = create_publisher<std_msgs::msg::String>("~/status", 10);
        // A late subscriber receives the latest complete snapshot while alive.
        const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
        map_points_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/map_points", map_qos);
        map_info_ = create_publisher<std_msgs::msg::String>("~/map_points_info", map_qos);
        const auto first = declare_parameter<std::string>(
            "image_topic", stereo_ ? "/camera/camera/infra1/image_rect_raw" :
                                    "/camera/camera/color/image_raw");
        const auto second = declare_parameter<std::string>(
            "second_topic", stereo_ ? "/camera/camera/infra2/image_rect_raw" :
                                     "/camera/camera/aligned_depth_to_color/image_raw");
        auto qos = rclcpp::SensorDataQoS().keep_last(queue_size_);
        first_ = create_subscription<Image>(first, qos, [this](Image::ConstSharedPtr msg) {
            receive(std::move(msg), 0);
        });
        second_ = create_subscription<Image>(second, qos, [this](Image::ConstSharedPtr msg) {
            receive(std::move(msg), 1);
        });
        RCLCPP_INFO(get_logger(), "%s: %dx%d; max pair skew %ld ns; viewer disabled",
                    mode.c_str(), width_, height_, static_cast<long>(max_skew_ns_));
        RCLCPP_WARN(get_logger(),
            "Diagnostic camera optical pose only; SLAM world may jump on reset/loop closure; no TF/body output");
    }

    ~D435Node() override { if (system_) system_->Shutdown(); }

 private:
    bool stereo_ = true;
    int queue_size_ = 5, width_ = 0, height_ = 0;
    int64_t max_skew_ns_ = 0, last_received_[2] = {-1, -1}, origin_ns_ = -1;
    uint64_t dropped_ = 0, processed_ = 0;
    std::string world_frame_;
    std::deque<Image::ConstSharedPtr> queues_[2];
    std::unique_ptr<ORB_SLAM3::System> system_;
    rclcpp::Subscription<Image>::SharedPtr first_, second_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_info_;
    std::unique_ptr<MapSnapshotRate> map_rate_;
    size_t map_limit_ = 50000;
    uint64_t map_sequence_ = 0;

    void publish_map(const Image& source, bool tracked) {
        const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        // This check precedes all map access/copying, including on lost frames.
        if (!map_rate_->due(stamp(source), steady_ns)) return;
        MapSnapshot snapshot;
        if (!snapshot_active_map(*system_, map_limit_, snapshot)) return;
        if (!tracked) snapshot.points.clear();  // replace stale display on lost tracking
        auto cloud = map_points_cloud<sensor_msgs::msg::PointCloud2>(
            snapshot, source.header.stamp.sec, source.header.stamp.nanosec, world_frame_);
        std::ostringstream record;
        record << "{\"source_ns\":" << stamp(source) << ",\"sequence\":" << ++map_sequence_
               << ",\"map_id\":" << snapshot.map_id << ",\"init_kf_id\":" << snapshot.init_kf_id
               << ",\"big_change\":" << snapshot.big_change
               << ",\"map_change\":" << snapshot.map_change << ",\"tracked\":"
               << (tracked ? "true" : "false") << ",\"candidates\":" << snapshot.candidates
               << ",\"points\":" << cloud.width << ",\"truncated\":"
               << (snapshot.truncated ? "true" : "false") << "}";
        std_msgs::msg::String info;
        info.data = record.str();
        map_info_->publish(info);
        map_points_->publish(cloud);
    }

    static int64_t stamp(const Image& msg) {
        if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000)
            throw std::invalid_argument("invalid source image stamp");
        return int64_t(msg.header.stamp.sec) * 1000000000 + msg.header.stamp.nanosec;
    }

    void receive(Image::ConstSharedPtr msg, int side) {
        // Single-threaded executor owns both queues and ORB. Tracking blocks callbacks;
        // DDS SensorDataQoS and these queues bound buffering, not latency or DDS loss.
        const auto ns = stamp(*msg);
        if (ns < last_received_[side])
            throw std::runtime_error("source time reversed: restart node before replay/seek");
        if (ns == last_received_[side]) { ++dropped_; return; }
        last_received_[side] = ns;
        const bool mono = stereo_ && (msg->encoding == "mono8" || msg->encoding == "8UC1");
        const bool color = !stereo_ && side == 0 &&
                           (msg->encoding == "rgb8" || msg->encoding == "bgr8");
        const bool depth = !stereo_ && side == 1 && msg->encoding == "16UC1";
        const auto bytes_per_pixel = mono ? 1u : color ? 3u : 2u;
        if ((!mono && !color && !depth) || msg->width != uint32_t(width_) ||
            msg->height != uint32_t(height_) || msg->header.frame_id.empty() ||
            msg->step < msg->width * bytes_per_pixel ||
            uint64_t(msg->step) * msg->height != msg->data.size()) {
            throw std::invalid_argument("image encoding/size/stride/frame differs from D435 input contract");
        }
        auto& queue = queues_[side];
        queue.push_back(std::move(msg));
        if (queue.size() > static_cast<size_t>(queue_size_)) { queue.pop_front(); ++dropped_; }
        while (!queues_[0].empty() && !queues_[1].empty()) {
            const auto difference = stamp(*queues_[0].front()) - stamp(*queues_[1].front());
            if (difference < -max_skew_ns_) { queues_[0].pop_front(); ++dropped_; }
            else if (difference > max_skew_ns_) { queues_[1].pop_front(); ++dropped_; }
            else {
                auto a = queues_[0].front(), b = queues_[1].front();
                queues_[0].pop_front(); queues_[1].pop_front();
                track(a, b);
            }
        }
    }

    void track(const Image::ConstSharedPtr& a, const Image::ConstSharedPtr& b) {
        const auto ns = stamp(*a);
        if (origin_ns_ < 0) origin_ns_ = ns;
        const double seconds = double(ns - origin_ns_) * 1e-9;
        // Keep integer epochs for publication; ORB gets relative seconds.
        cv::Mat first, second;
        if (stereo_) {
            first = cv_bridge::toCvCopy(a)->image;
            second = cv_bridge::toCvCopy(b)->image;
        } else {
            if (a->header.frame_id != b->header.frame_id)
                throw std::invalid_argument("RGB-D depth must be aligned to the color optical frame");
            first = cv_bridge::toCvCopy(a, "rgb8")->image;
            // realsense2_camera publishes 16UC1 in millimetres, independent of
            // native SDK depth units. ORB settings use DepthMapFactor=1.
            cv_bridge::toCvCopy(b)->image.convertTo(second, CV_32F, 0.001);
        }
        const auto Tcw = stereo_ ? system_->TrackStereo(first, second, seconds) :
                                   system_->TrackRGBD(first, second, seconds);
        const auto state = system_->GetTrackingState();
        const auto Twc = Tcw.inverse();
        bool published = false;
        if ((state == ORB_SLAM3::Tracking::OK || state == ORB_SLAM3::Tracking::OK_KLT) &&
            Twc.matrix().allFinite()) {
            geometry_msgs::msg::PoseStamped out;
            out.header.stamp = a->header.stamp;
            out.header.frame_id = world_frame_;
            const auto p = Twc.translation();
            const auto q = Twc.unit_quaternion();
            out.pose.position.x = p.x(); out.pose.position.y = p.y(); out.pose.position.z = p.z();
            out.pose.orientation.x = q.x(); out.pose.orientation.y = q.y();
            out.pose.orientation.z = q.z(); out.pose.orientation.w = q.w();
            pose_->publish(out);
            published = true;
        }
        std::ostringstream record;
        record << "{\"source_ns\":" << ns << ",\"second_ns\":" << stamp(*b)
               << ",\"tracking_state\":" << state << ",\"pose_published\":"
               << (published ? "true" : "false") << ",\"processed\":" << ++processed_
               << ",\"application_dropped\":" << dropped_ << "}";
        std_msgs::msg::String status;
        status.data = record.str();
        status_->publish(status);
        publish_map(*a, published);
    }
};
}  // namespace wbr::orb_node

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int code = 0;
    try {
        // Do not use a multithreaded executor with this synchronous ORB owner.
        rclcpp::spin(std::make_shared<wbr::orb_node::D435Node>());
    } catch (const std::exception& error) {
        std::cerr << "d435_node: " << error.what() << '\n';
        code = 1;
    }
    rclcpp::shutdown();
    return code;
}
