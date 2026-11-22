// ROS 2 Jazzy integration: deliberately not built on the development Mac.
#include "wbr_orb_node/core.hpp"
#include "clock_policy.hpp"
#include "fake_engine.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <regex>
#include <sstream>
#include <thread>
#ifdef WBR_WITH_ORBSLAM3
#include "System.h"
#include "orb_engine.h"
#endif

namespace wbr::orb_node {
namespace {
using ImageMsg=sensor_msgs::msg::Image;
using ImuMsg=sensor_msgs::msg::Imu;
std::string unique_session() {
    std::random_device random;
    std::ostringstream value;
    value << "run-" << std::hex << random() << random() << random();
    return value.str();
}
int64_t source_ns(const builtin_interfaces::msg::Time& t) {
    if (t.sec<0 || t.nanosec>=1000000000) throw std::invalid_argument("invalid ROS header stamp");
    return int64_t(t.sec)*1000000000LL+t.nanosec;
}
bool token(const std::string& value) { return std::regex_match(value,std::regex("[A-Za-z0-9_.-]{1,47}")); }
struct Mapping {
    std::string json="{\"valid\":false,\"reason\":\"mapping_unavailable\"}";
    int64_t lower=0,upper=-1;
    std::string at(int64_t ns,bool replay,bool bound) const {
        if (replay || !bound || ns<lower || ns>upper)
            return "{\"valid\":false,\"reason\":\"replay_or_unbound_or_outside_interval\"}";
        return json;
    }
};
Mapping load_mapping(const std::string& path,const Config& config,const std::string& pi_session) {
    if (path.empty()) return {};
    const auto data=YAML::LoadFile(path);
    if (data["prepared_by"].as<std::string>()!="wbr_orb_node.prepare_mapping.v1" ||
        !data["valid"].as<bool>() || data["camera_domain"].as<std::string>()!=config.source_domain ||
        data["camera_session"].as<std::string>()!=config.source_session ||
        data["pi_domain"].as<std::string>()!="CLOCK_MONOTONIC_RAW" ||
        data["pi_session"].as<std::string>()!=pi_session || !token(pi_session))
        throw std::invalid_argument("mapping was not prepared for this source/Pi clock session");
    Mapping result;
    result.lower=data["valid_source_min_ns"].as<int64_t>();
    result.upper=data["valid_source_max_ns"].as<int64_t>();
    if (result.lower<0 || result.upper<result.lower) throw std::invalid_argument("invalid mapping interval");
    std::ostringstream out;
    out << "{\"valid\":true,\"camera_domain\":\"" << config.source_domain
        << "\",\"camera_session\":\"" << config.source_session
        << "\",\"pi_domain\":\"CLOCK_MONOTONIC_RAW\",\"pi_session\":\"" << pi_session
        << "\",\"exchanges_ns\":[";
    bool first=true;
    for (const auto& row:data["exchanges_ns"]) {
        if (!row.IsSequence() || row.size()!=4) throw std::invalid_argument("invalid exchange row");
        if (!first) out << ',';
        first=false; out << '[';
        for (size_t i=0;i<4;++i) { if (i) out << ','; out << row[i].as<int64_t>(); }
        out << ']';
    }
    if (first) throw std::invalid_argument("empty clock exchanges");
    out << "]}"; result.json=out.str(); return result;
}
}

class OrbNode final : public rclcpp::Node {
 public:
    OrbNode():Node("wbr_orb_node") {
        config_.source_domain=declare_parameter<std::string>("source_domain","ros_header");
        config_.source_session=declare_parameter<std::string>("source_session","unbound");
        if (config_.source_session=="unbound") throw std::invalid_argument("bind a unique source_session before launch");
        auto positive=[this](const std::string& name,int64_t fallback) {
            auto value=declare_parameter<int64_t>(name,fallback);
            if (value<=0) throw std::invalid_argument("positive bound required: "+name);
            return value;
        };
        config_.ingress_capacity=positive("ingress_capacity",defaults::ingress_capacity);
        config_.image_capacity=positive("image_capacity",defaults::image_capacity);
        config_.imu_capacity=positive("imu_capacity",defaults::imu_capacity);
        config_.max_image_bytes=positive("max_image_bytes",defaults::max_image_bytes);
        config_.max_imu_gap_ns=positive("max_imu_gap_ns",defaults::max_imu_gap_ns);
        poll_=std::chrono::milliseconds(positive("worker_poll_ms",defaults::worker_poll_ms));
        const auto qos_depth=positive("qos_depth",defaults::qos_depth);
        sim_=get_parameter("use_sim_time").as_bool();
        policy_=std::make_unique<bridge::ClockPolicy>(sim_,positive("max_ros_step_ns",defaults::max_ros_step_ns));
        header_bound_=declare_parameter<bool>("source_header_is_d435i_time",false);
        auto output=declare_parameter<std::string>("output_directory","");
        if (output.empty() || !std::filesystem::create_directory(output))
            throw std::invalid_argument("output_directory must be a NEW directory with an existing parent");
        directory_=output;
        poses_.open(directory_/"pose_events.jsonl");
        if (!poses_) throw std::runtime_error("cannot open pose events");
        host_session_=unique_session();
        events_=std::make_unique<Recorder>(host_session_.c_str());
        ingress_=std::make_unique<Ingress>(config_,*events_);
        const auto mapping_path=declare_parameter<std::string>("clockmap_file","");
        const auto pi_session=declare_parameter<std::string>("pi_clock_session","");
        if (!sim_ && !mapping_path.empty()) {
            std::ifstream boot("/proc/sys/kernel/random/boot_id"); std::string actual;
            std::getline(boot,actual);
            if (!boot || actual!=pi_session) throw std::invalid_argument("pi_clock_session must equal Linux boot_id");
            mapping_=load_mapping(mapping_path,config_,pi_session);
        }
        const auto extrinsic_provenance=declare_parameter<std::string>("extrinsic_provenance","");
        auto extrinsic=declare_parameter<std::vector<double>>("T_camera_left_optical_imu_camera",std::vector<double>{});
        if (!extrinsic.empty()) {
            if (extrinsic_provenance.empty()) throw std::invalid_argument("extrinsic_provenance required with camera/IMU transform");
            if (extrinsic.size()!=16) throw std::invalid_argument("camera/IMU extrinsic requires 16 row-major values");
            vio::Transform transform; transform.parent="camera_left_optical"; transform.child="imu_camera";
            std::copy(extrinsic.begin(),extrinsic.end(),transform.matrix.begin());
            if (!vio::valid_transform(transform)) throw std::invalid_argument("invalid camera/IMU extrinsic");
            extrinsic_=transform;
        }
#ifdef WBR_WITH_ORBSLAM3
        const auto vocabulary=declare_parameter<std::string>("orb_vocabulary","");
        const auto settings=declare_parameter<std::string>("orb_settings","");
        if (vocabulary.empty() || settings.empty()) throw std::invalid_argument("qualified ORB vocabulary/settings required");
        system_=std::make_unique<ORB_SLAM3::System>(vocabulary,settings,ORB_SLAM3::System::IMU_STEREO,false);
        engine_=std::make_unique<vio::OrbEngine>(*system_);
#else
        engine_=std::make_unique<vio::FakeEngine>();
        RCLCPP_WARN(get_logger(),"Fake engine: all estimates are SYNTHETIC");
#endif
        core_=std::make_unique<Core>(*engine_,config_,*events_);
        publisher_=create_publisher<nav_msgs::msg::Odometry>("diagnostic_odometry",rclcpp::SensorDataQoS().keep_last(qos_depth));
        core_->on_output=[this](const Output& out) { publish(out); };
        const auto left=declare_parameter<std::string>("left_topic","/camera/camera/infra1/image_rect_raw");
        const auto right=declare_parameter<std::string>("right_topic","/camera/camera/infra2/image_rect_raw");
        const auto imu=declare_parameter<std::string>("imu_topic","/camera/camera/imu");
        auto qos=rclcpp::SensorDataQoS().keep_last(qos_depth);
        left_=create_subscription<ImageMsg>(left,qos,[this](ImageMsg::ConstSharedPtr msg){ image(*msg,true); });
        right_=create_subscription<ImageMsg>(right,qos,[this](ImageMsg::ConstSharedPtr msg){ image(*msg,false); });
        imu_=create_subscription<ImuMsg>(imu,qos,[this](ImuMsg::ConstSharedPtr msg){ motion(*msg); });
        // Wall timer detects replay pause/seek even without new sensor traffic.
        clock_timer_=create_wall_timer(poll_,[this]{ check_clock(); });
        RCLCPP_WARN(get_logger(),"Diagnostic odom -> imu_camera only; no TF, no continuous odometry qualification");
        if (!extrinsic_) RCLCPP_WARN(get_logger(),"No measured optical-to-camera-IMU transform; odometry withheld");
        save_manifest();
        worker_=std::thread([this]{ work(); });
    }
    void stop() {
        stopping_=true;
        if (worker_.joinable()) worker_.join();
    }
    ~OrbNode() override {
        stop();
#ifdef WBR_WITH_ORBSLAM3
        if (system_) system_->Shutdown();
#endif
    }
    bool failed() const { return worker_failed_.load() || clock_fault_.load() || core_->faulted(); }
 private:
    Config config_; Mapping mapping_; std::string host_session_; std::optional<vio::Transform> extrinsic_;
    std::filesystem::path directory_; std::ofstream poses_;
    std::chrono::milliseconds poll_{defaults::worker_poll_ms};
    std::unique_ptr<Recorder> events_; std::unique_ptr<Ingress> ingress_;
    std::unique_ptr<vio::Engine> engine_; std::unique_ptr<Core> core_;
#ifdef WBR_WITH_ORBSLAM3
    std::unique_ptr<ORB_SLAM3::System> system_;
#endif
    std::unique_ptr<bridge::ClockPolicy> policy_;
    bool sim_=false,header_bound_=false;
    std::atomic<bool> stopping_{false},worker_failed_{false},clock_fault_{false};
    std::atomic<uint64_t> rejected_images_{0},rejected_imu_{0},clock_withheld_{0};
    std::thread worker_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<ImageMsg>::SharedPtr left_,right_;
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_;
    rclcpp::TimerBase::SharedPtr clock_timer_;
    bool check_clock() {
        const auto reason=policy_->observe(now().nanoseconds(),get_parameter("use_sim_time").as_bool());
        if (reason=="clock_jump_restart_required" || reason=="clock_policy_unconfigured") clock_fault_=true;
        return !clock_fault_ && reason!="waiting_for_clock";
    }
    void image(const ImageMsg& msg,bool left) {
        if (!check_clock()) { ++clock_withheld_; return; }
        // MONO8/Y8 only, including padded rows; conversion never opens a camera.
        if ((msg.encoding!="mono8" && msg.encoding!="8UC1") || !msg.width || !msg.height ||
            uint64_t(msg.width)*msg.height>config_.max_image_bytes || msg.step<msg.width ||
            uint64_t(msg.step)*msg.height!=msg.data.size()) { ++rejected_images_; return; }
        try {
            const auto ns=source_ns(msg.header.stamp);
            const auto receipt=ingress_->received(ns);
            vio::Image value{{ns,config_.source_domain},msg.width,msg.height,{}};
            value.y8.reserve(size_t(msg.width)*msg.height);
            for (uint32_t row=0;row<msg.height;++row)
                value.y8.insert(value.y8.end(),msg.data.begin()+size_t(row)*msg.step,
                                msg.data.begin()+size_t(row)*msg.step+msg.width);
            if (left) ingress_->submit(Left{std::move(value)},receipt);
            else ingress_->submit(Right{std::move(value)},receipt);
        } catch (const std::invalid_argument&) { ++rejected_images_; clock_fault_=true; }
    }
    void motion(const ImuMsg& msg) {
        if (!check_clock()) { ++clock_withheld_; return; }
        // A single qualified, combined D435i IMU topic, never the STM32 body IMU.
        if (msg.linear_acceleration_covariance[0]<0 || msg.angular_velocity_covariance[0]<0) {
            ++rejected_imu_; clock_fault_=true; return;
        }
        try {
            ingress_->submit(Imu{{source_ns(msg.header.stamp),config_.source_domain},
                {msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z},
                {msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z}});
        } catch (const std::invalid_argument&) { ++rejected_imu_; clock_fault_=true; }
    }
    void publish(const Output& out) {
#ifdef WBR_WITH_ORBSLAM3
        constexpr bool fake=false;
#else
        constexpr bool fake=true;
#endif
        if (clock_fault_) return;
        const auto mapping=mapping_.at(out.source_image.ns,sim_,header_bound_ && !fake);
        poses_ << health_json(out,mapping,fake) << '\n'; poses_.flush();
        if (!poses_) throw std::runtime_error("pose event write failure");
        if (extrinsic_ && rclcpp::ok() && out.snapshot.reset_counter==0 && out.snapshot.epoch_id==0) {
            auto pose=odometry_pose(out.snapshot,*extrinsic_);
            if (pose) {
                nav_msgs::msg::Odometry msg;
                msg.header.frame_id="odom"; msg.child_frame_id="imu_camera";
                msg.header.stamp.sec=pose->sec; msg.header.stamp.nanosec=pose->nanosec;
                msg.pose.pose.position.x=pose->position[0]; msg.pose.pose.position.y=pose->position[1];
                msg.pose.pose.position.z=pose->position[2];
                msg.pose.pose.orientation.x=pose->xyzw[0]; msg.pose.pose.orientation.y=pose->xyzw[1];
                msg.pose.pose.orientation.z=pose->xyzw[2]; msg.pose.pose.orientation.w=pose->xyzw[3];
                // NaN explicitly marks unavailable covariance and twist. Never
                // feed this diagnostic topic into navigation/state estimation.
                const auto unknown=std::numeric_limits<double>::quiet_NaN();
                msg.pose.covariance.fill(unknown); msg.twist.covariance.fill(unknown);
                msg.twist.twist.linear.x=msg.twist.twist.linear.y=msg.twist.twist.linear.z=unknown;
                msg.twist.twist.angular.x=msg.twist.twist.angular.y=msg.twist.twist.angular.z=unknown;
                publisher_->publish(msg);
            }
        }
        core_->output_written(out);
    }
    void work() noexcept {
        try {
            while (!stopping_) {
                if (ingress_->counts().imu_loss || clock_fault_) core_->input_imu_loss();
                auto message=ingress_->pop();
                if (message) core_->consume(std::move(*message));
                else std::this_thread::sleep_for(poll_);
            }
            while (auto message=ingress_->pop()) {
                if (ingress_->counts().imu_loss || clock_fault_) core_->input_imu_loss();
                core_->consume(std::move(*message));
            }
            core_->finish();
        } catch (const std::exception& error) {
            worker_failed_=true;
            RCLCPP_ERROR(get_logger(),"tracking runner failed: %s",error.what());
        }
        try { save(); } catch (const std::exception& error) {
            worker_failed_=true; RCLCPP_ERROR(get_logger(),"evidence write failed: %s",error.what());
        }
    }
    void save_manifest() {
        YAML::Emitter manifest;
        manifest << YAML::BeginMap << YAML::Key << "tool_version" << YAML::Value << "0.1.0"
                 << YAML::Key << "evidence" << YAML::Value << "MEASURED configured runtime; hardware qualification UNKNOWN"
                 << YAML::Key << "host_session" << YAML::Value << host_session_
                 << YAML::Key << "runtime_clock" << YAML::Value << "Linux CLOCK_MONOTONIC (observer host_steady)"
                 << YAML::Key << "image_clock" << YAML::Value << "ROS header.stamp retained verbatim"
                 << YAML::Key << "odom_semantics" << YAML::Value << "diagnostic initial slam_world axes; no TF or continuity qualification"
                 << YAML::Key << "parameters" << YAML::Value << YAML::BeginMap;
        for (const auto& parameter:get_parameters(list_parameters({},0).names))
            manifest << YAML::Key << parameter.get_name() << YAML::Value << parameter.value_to_string();
        manifest << YAML::EndMap << YAML::EndMap;
        std::ofstream output(directory_/"run_manifest.yaml"); output << manifest.c_str() << '\n'; output.flush();
        if (!manifest.good() || !output) throw std::runtime_error("cannot write run manifest");
    }
    void save() {
        std::ofstream events(directory_/"events.csv"); events_->dump_csv(events); events.flush();
        std::ofstream summary(directory_/"counts.json");
        const auto queue=ingress_->counts(); const auto c=core_->counts(); const auto a=core_->adapter_counts();
        summary << "{\"evidence\":\"MEASURED application counts; DDS losses UNKNOWN\","
                << "\"ingress_drop_oldest\":" << queue.oldest << ",\"ingress_contention\":" << queue.contention
                << ",\"ingress_imu_loss\":" << queue.imu_loss << ",\"left_drop_oldest\":" << c.left_oldest
                << ",\"right_drop_oldest\":" << c.right_oldest << ",\"imu_drop_oldest\":" << c.imu_oldest
                << ",\"unmatched\":" << c.unmatched << ",\"invalid\":" << c.invalid
                << ",\"shutdown_pending_images\":" << c.shutdown_images
                << ",\"shutdown_imu_tail\":" << c.shutdown_imu_tail
                << ",\"source_faults\":" << c.source_fault
                << ",\"processed\":" << c.processed << ",\"output\":" << c.output
                << ",\"adapter_pairs\":" << a.pairs << ",\"adapter_missing_coverage\":" << a.missing_coverage
                << ",\"adapter_overflow\":" << a.overflow << ",\"adapter_prefix_pairs\":" << a.prefix_pairs
                << ",\"adapter_engine_rejected\":" << a.engine_rejected
                << ",\"rejected_ros_images\":" << rejected_images_.load() << ",\"rejected_ros_imu\":" << rejected_imu_.load()
                << ",\"clock_withheld_messages\":" << clock_withheld_.load()
                << ",\"session_fault\":" << (core_->faulted()||clock_fault_?"true":"false")
                << ",\"worker_failed\":" << (worker_failed_?"true":"false") << "}\n";
        summary.flush();
        if (!events || !summary) throw std::runtime_error("runtime evidence stream failure");
    }
};
} // namespace wbr::orb_node
int main(int argc,char** argv) {
    rclcpp::init(argc,argv);
    int status=0;
    try {
        auto node=std::make_shared<wbr::orb_node::OrbNode>();
        rclcpp::spin(node); node->stop(); status=node->failed()?1:0;
        node.reset(); // join tracking and save evidence before context shutdown
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; status=3; }
    rclcpp::shutdown(); return status;
}
