// UNBUILT ROS 2 Jazzy skeleton. main() intentionally has no snapshot producer.
// API citations and missing integration work: ../DESIGN.md.
#include "mapping.hpp"
#include "clock_policy.hpp"
#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

class SnapshotBridge : public rclcpp::Node {
 public:
  SnapshotBridge() : Node("snapshot_bridge"), updater_(this) {
    session_=declare_parameter<std::string>("adapter_session", "");
    const auto domain=declare_parameter<std::string>("source_domain", "");
    sim_time_=get_parameter("use_sim_time").as_bool();
    const auto target=sim_time_?"ros_replay":"ros_live";
    const auto generation=declare_parameter<int64_t>("clock_generation", 0);
    const auto max_age=declare_parameter<int64_t>("max_age_ns", -1);
    const auto max_uncertainty=declare_parameter<int64_t>("max_uncertainty_ns", -1);
    if (generation<0) throw std::invalid_argument("negative clock generation");
    gate_=std::make_unique<wbr::bridge::Gate>(session_,domain,target,generation,max_age,max_uncertainty);
    const auto max_step=declare_parameter<int64_t>("max_forward_step_ns", -1);
    clock_policy_=std::make_unique<wbr::bridge::ClockPolicy>(sim_time_,max_step);
    // Jazzy rclcpp/clock.hpp L212-238; rcl/time.h L100-111.
    // The callback also catches jumps that reverse again between wall polls.
    rcl_jump_threshold_t threshold{};
    threshold.on_clock_change=true;
    threshold.min_backward.nanoseconds=-1;
    threshold.min_forward.nanoseconds=max_step>0?max_step:0;
    jump_handler_=get_clock()->create_jump_callback(nullptr,
        [this](const rcl_time_jump_t &) noexcept {
          if (clock_started_.load()) clock_discontinuity_.store(true);
        },threshold);
    pose_=create_publisher<geometry_msgs::msg::PoseStamped>(
        "slam/pose",rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
    updater_.setHardwareID("unbound_adapter:"+session_);
    updater_.add("snapshot_bridge",this,&SnapshotBridge::diagnostic);
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this] {tick();});
    // Updater's own timer follows ROS time. Force visibility during a /clock pause.
    diagnostics_wall_=create_wall_timer(std::chrono::seconds(1),[this] {
      const auto stamp=now().nanoseconds();
      if (sim_time_ && last_diag_ros_ && stamp==*last_diag_ros_) updater_.force_update();
      last_diag_ros_=stamp;
    });
    RCLCPP_WARN(get_logger(),"UNBUILT skeleton: no snapshot ingress or clock mapper wired");
  }
  // Integration seam only. A serialized adapter owner must copy snapshots here.
  // evidence.age_upper_ns is evaluated at this call; include mapping uncertainty.
  // Do not reuse cached evidence on a heartbeat. Overflow latches a fault so an
  // unread reset/invalid event cannot disappear behind a newer healthy snapshot.
  void submit(wbr::vio::EstimateSnapshot snapshot,
              std::optional<wbr::bridge::ClockEvidence> evidence) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_unread_) ingress_overrun_=true;
    pending_=std::move(snapshot);
    evidence_=std::move(evidence);
    received_=std::chrono::steady_clock::now();
    pending_unread_=true;
  }
  // The trusted replay owner MUST call this before every seek (including
  // sub-threshold forward seeks) or /clock publisher replacement. ROS clock
  // callbacks cannot identify replacement by another publisher at equal time.
  void notify_clock_discontinuity() noexcept {clock_discontinuity_.store(true);}
 private:
  void tick() {
    const auto ros_now=now().nanoseconds();
    const auto clock_reason=clock_policy_->observe(ros_now,
        get_parameter("use_sim_time").as_bool(),clock_discontinuity_.load());
    if (ros_now>0) clock_started_.store(true);
    if (clock_reason=="clock_jump_restart_required") {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_.reset(); evidence_.reset(); last_evidence_.reset();
      pending_unread_=false;
      reason_=clock_reason; level_=2; return; // New generation + restart/rebind required.
    }
    std::optional<wbr::vio::EstimateSnapshot> s;
    std::optional<wbr::bridge::ClockEvidence> e;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (ingress_overrun_) {reason_="ingress_overrun_restart_required"; level_=2; return;}
      s=pending_; e=evidence_;
      pending_unread_=false;
      if (e) {
        const auto elapsed=std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now()-received_).count();
        if (elapsed<0 || e->age_upper_ns<0 ||
            e->age_upper_ns>std::numeric_limits<int64_t>::max()-elapsed) e.reset();
        else e->age_upper_ns+=elapsed; // Same steady domain, never ROS-minus-device.
      }
    }
    if (!s) {reason_=clock_reason.empty()?"no_snapshot_ingress":clock_reason; level_=3; return;}
    latest_=*s;
    if (e && (e->ros_ns<0 || e->ros_ns>ros_now)) e.reset();
    const auto decision=gate_->advance(*s,e);
    last_evidence_=e;
    // Consume paused ingress through Gate, so resume cannot publish cached poses
    // or hide reset/regression events. Only a later fresh estimate may publish.
    if (!clock_reason.empty()) {reason_=clock_reason; level_=3; return;}
    if (clock_discontinuity_.load()) {
      reason_="clock_jump_restart_required"; level_=2; return;
    }
    if (decision.reason!="duplicate") reason_=decision.reason;
    if (decision.pose) {
      const auto &p=*decision.pose;
      geometry_msgs::msg::PoseStamped message;
      message.header.stamp.sec=p.sec; message.header.stamp.nanosec=p.nanosec;
      message.header.frame_id="slam_world";
      message.pose.position.x=p.position[0]; message.pose.position.y=p.position[1];
      message.pose.position.z=p.position[2];
      message.pose.orientation.x=p.xyzw[0]; message.pose.orientation.y=p.xyzw[1];
      message.pose.orientation.z=p.xyzw[2]; message.pose.orientation.w=p.xyzw[3];
      pose_->publish(message);
    }
    level_=reason_=="pose_published"?1:(reason_=="stale_source"?3:2);
  }
  void diagnostic(diagnostic_updater::DiagnosticStatusWrapper &status) {
    status.summary(level_,reason_);
    status.add("build_status","UNBUILT ROS skeleton");
    status.add("adapter_session",session_);
    status.add("pose_child_frame","camera_left_optical");
    status.add("covariance_availability",latest_.covariance_availability==
        wbr::vio::Availability::AVAILABLE?"AVAILABLE (not published)":"UNAVAILABLE");
    status.add("sequence",latest_.sequence);
    status.add("reset_counter",latest_.reset_counter);
    status.add("epoch_id",latest_.epoch_id);
    status.add("map_id",latest_.map_id?std::to_string(*latest_.map_id):"UNAVAILABLE");
    status.add("source_domain",latest_.source?latest_.source->domain:"UNAVAILABLE");
    status.add("source_ns",latest_.source?std::to_string(latest_.source->ns):"UNAVAILABLE");
    status.add("source_age_upper_ns",last_evidence_?std::to_string(last_evidence_->age_upper_ns):"UNAVAILABLE");
    status.add("mapping_revision",last_evidence_?std::to_string(last_evidence_->revision):"UNAVAILABLE");
    status.add("mapping_uncertainty_ns",last_evidence_?std::to_string(last_evidence_->uncertainty_ns):"UNAVAILABLE");
    status.add("mapped_source_ros_ns",last_evidence_?std::to_string(last_evidence_->ros_ns):"UNAVAILABLE");
    status.add("clock_generation",last_evidence_?std::to_string(last_evidence_->generation):"UNAVAILABLE");
    status.add("ros_clock_domain",sim_time_?"ros_replay":"ros_live");
    status.add("tracking_state",static_cast<int>(latest_.state));
    status.add("adapter_reasons",latest_.reasons);
    status.add("mcu_authority","independent; bridge is diagnostic only");
  }
  std::mutex mutex_;
  std::optional<wbr::vio::EstimateSnapshot> pending_;
  std::optional<wbr::bridge::ClockEvidence> evidence_;
  std::optional<wbr::bridge::ClockEvidence> last_evidence_;
  bool pending_unread_=false,ingress_overrun_=false,sim_time_=false;
  std::chrono::steady_clock::time_point received_;
  wbr::vio::EstimateSnapshot latest_;
  std::unique_ptr<wbr::bridge::Gate> gate_;
  std::string session_,reason_="no_snapshot_ingress";
  int level_=3;
  std::unique_ptr<wbr::bridge::ClockPolicy> clock_policy_;
  std::atomic<bool> clock_discontinuity_{false},clock_started_{false};
  rclcpp::JumpHandler::SharedPtr jump_handler_;
  std::optional<int64_t> last_diag_ros_;
  diagnostic_updater::Updater updater_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_wall_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<SnapshotBridge>());
  rclcpp::shutdown();
  return 0;
}
