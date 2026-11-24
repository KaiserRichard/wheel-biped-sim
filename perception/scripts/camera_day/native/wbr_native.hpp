// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "metrics.hpp"
#include "orb_access.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <thread>
#include <unistd.h>

namespace wbr {
using Clock = std::chrono::steady_clock;
inline double env_seconds(const char* name, double fallback) {
    const char* value = std::getenv(name);
    if (!value) return fallback;
    char* end = nullptr;
    double number = std::strtod(value, &end);
    if (end == value || *end || !std::isfinite(number) || number <= 0)
        throw std::runtime_error(std::string(name) + " must be finite and positive");
    return number;
}
struct Runtime {
    Clock::time_point epoch = Clock::now();
    double duration = env_seconds("WBR_DURATION_S", 60);
    double grace = env_seconds("WBR_SHUTDOWN_GRACE_S", 30);
    double startup_limit = env_seconds("WBR_STARTUP_LIMIT_S", 120);
    std::string output, serial, mode;
    int fps = 30, features = 0;
    double active_start = -1, active_end = -1, depth_units = 0;
    double previous_source = -1, previous_start = -1;
    uint64_t previous_frame = 0;
    Counts counts;
    size_t attempted_track_calls = 0, failed_track_calls = 0;
    std::ofstream csv, attempts;
    std::mutex watch_mutex;
    std::condition_variable watch_cv;
    bool watch_done = false;
    double hard_deadline = 0;
    std::thread watchdog;

    double elapsed() const { return std::chrono::duration<double>(Clock::now() - epoch).count(); }
    Runtime(const char* yaml, const char* mode_name) : mode(mode_name) {
        cv::FileStorage settings(yaml, cv::FileStorage::READ);
        if (!settings.isOpened()) throw std::runtime_error("cannot read settings");
        fps = static_cast<int>(settings["Camera.fps"]);
        features = static_cast<int>(settings["ORBextractor.nFeatures"]);
        settings["WBR.serial"] >> serial;
        if ((fps != 15 && fps != 30) || features <= 0 || serial.empty() ||
            static_cast<int>(settings["Camera.width"]) != 640 ||
            static_cast<int>(settings["Camera.height"]) != 480)
            throw std::runtime_error("require 640x480, FPS 15/30, positive features, WBR.serial");
        if (serial.find_first_not_of("0123456789") != std::string::npos ||
            !settings["System.SaveAtlasToFile"].empty())
            throw std::runtime_error("require decimal serial and no upstream automatic atlas save");
        if (mode == "rgbd" && static_cast<double>(settings["RGBD.DepthMapFactor"]) != 1.0)
            throw std::runtime_error("native helper converts SDK Z16 units to metres; DepthMapFactor must be 1");
        const char* path = std::getenv("WBR_OUTPUT");
        if (!path || !*path) throw std::runtime_error("WBR_OUTPUT must name a NEW output directory");
        output = path;
        if (!std::filesystem::create_directory(output))
            throw std::runtime_error("WBR_OUTPUT exists; refusing to overwrite evidence");
        csv.exceptions(std::ios::failbit | std::ios::badbit);
        csv.open(output + "/frames.csv");
        csv << std::setprecision(17);
        csv_header(csv);
        csv.flush();
        attempts.exceptions(std::ios::failbit | std::ios::badbit);
        attempts.open(output + "/attempts.csv");
        attempts << std::setprecision(17) << "attempt_index,source_timestamp_s,attempt_elapsed_s\n";
        attempts.flush();
        hard_deadline = startup_limit;
        watchdog = std::thread([this] {
            std::unique_lock<std::mutex> lock(watch_mutex);
            while (!watch_done) {
                if (elapsed() >= hard_deadline) {
                    // No unsafe call into ORB/SDK from this thread. Preserve an
                    // unmistakable failure marker; never claim graceful export.
                    std::ofstream marker(output + "/INCOMPLETE-watchdog.txt");
                    marker << "Startup, Track, SDK stop or ORB shutdown exceeded bound; exit 124.\n";
                    marker.close();
                    const char msg[] = "WBR watchdog: incomplete run, exit 124\n";
                    ::write(STDERR_FILENO, msg, sizeof(msg) - 1);
                    std::_Exit(124);
                }
                watch_cv.wait_for(lock, std::chrono::milliseconds(50));
            }
        });
    }
    ~Runtime() {
        { std::lock_guard<std::mutex> lock(watch_mutex); watch_done = true; }
        watch_cv.notify_all();
        if (watchdog.joinable()) watchdog.join();
    }
    void arm() {
        active_start = elapsed();
        std::lock_guard<std::mutex> lock(watch_mutex);
        hard_deadline = active_start + duration + grace;
    }
    bool within_duration() const { return active_start < 0 || elapsed() - active_start < duration; }
    void stopping() {
        active_end = elapsed();
        std::lock_guard<std::mutex> lock(watch_mutex);
        hard_deadline = std::min(hard_deadline, active_end + grace);
    }
    double begin(double source) {
        const double start = elapsed();
        // Persist the attempt before entering Track: a hung call has no invented
        // end/state/duration but still leaves a timestamped CSV receipt.
        attempts << ++attempted_track_calls << ',' << source << ',' << start << '\n';
        attempts.flush();
        return elapsed(); // Track duration excludes the attempt-receipt flush.
    }
    void record(double source, double start, double end, int state, int imu,
                unsigned overwrites, uint64_t frame_number, int domain, size_t imu_samples,
                bool completed = true) {
        if (completed) counts.add(state);
        if (!completed) ++failed_track_calls;
        csv << attempted_track_calls << ',';
        if (completed) csv << counts.processed;
        csv << ',' << source << ',' << start << ',' << end << ','
            << end - start << ',' << state << ',' << (state == 2) << ',' << imu << ',';
        if (previous_source >= 0) csv << source - previous_source;
        csv << ',';
        if (previous_start >= 0) csv << start - previous_start;
        csv << ',' << overwrites << ',' << frame_number << ',';
        if (previous_frame) csv << static_cast<int64_t>(frame_number) - static_cast<int64_t>(previous_frame);
        csv << ',' << domain << ',' << imu_samples << ',' << completed << '\n';
        csv.flush();
        previous_source = source; previous_start = start; previous_frame = frame_number;
    }
    size_t check_tum(const std::string& path) {
        std::ifstream input(path);
        if (!input) throw std::runtime_error("missing trajectory: " + path);
        size_t rows = 0;
        std::string line;
        while (std::getline(input, line)) {
            std::istringstream values(line);
            for (int i = 0; i < 8; ++i) {
                double value;
                if (!(values >> value) || !std::isfinite(value))
                    throw std::runtime_error("invalid TUM row: " + path);
            }
            std::string extra;
            if (values >> extra) throw std::runtime_error("extra TUM columns: " + path);
            ++rows;
        }
        if (!input.eof()) throw std::runtime_error("cannot read complete trajectory: " + path);
        return rows;
    }
    void finish(ORB_SLAM3::System& slam, const char* reason) {
        // Caller stops the pipeline first. No new Track calls after this point.
        slam.Shutdown();
        while (!workers_finished(slam)) std::this_thread::sleep_for(std::chrono::milliseconds(10));
        const bool have_keyframes = !atlas(slam)->GetAllKeyFrames().empty();
        if (have_keyframes) {
            slam.SaveTrajectoryTUM(output + "/CameraTrajectory.txt");
            slam.SaveKeyFrameTrajectoryTUM(output + "/KeyFrameTrajectory.txt");
        } else {
            // The installed SaveTrajectoryTUM dereferences keyframes[0].
            std::ofstream(output + "/CameraTrajectory.txt");
            std::ofstream(output + "/KeyFrameTrajectory.txt");
        }
        const size_t camera_rows = check_tum(output + "/CameraTrajectory.txt");
        const size_t keyframe_rows = check_tum(output + "/KeyFrameTrajectory.txt");
        if (have_keyframes && keyframe_rows == 0)
            throw std::runtime_error("keyframe export is unexpectedly empty");
        std::ofstream summary(output + "/summary.json");
        summary.exceptions(std::ios::failbit | std::ios::badbit);
        summary << std::setprecision(17) << "{\n\"mode\":\"" << mode
                << "\",\n\"serial\":\"" << serial << "\",\n\"requested_fps\":" << fps
                << ",\n\"feature_count\":" << features << ",\n\"duration_requested_s\":" << duration
                << ",\n\"native_depth_units_m\":" << depth_units
                << ",\n\"stop_reason\":\"" << reason << "\",\n\"shutdown_complete\":true,\n"
                << "\"failed_track_calls\":" << failed_track_calls << ",\n"
                << "\"attempted_track_calls\":" << attempted_track_calls << ",\n"
                << "\"camera_trajectory_rows\":" << camera_rows << ",\n"
                << "\"keyframe_trajectory_rows\":" << keyframe_rows << ",\n"
                << "\"trajectory_has_keyframes\":" << (have_keyframes ? "true" : "false") << ",\n";
        counts.json(summary, std::max(0.0, active_end - active_start));
        summary << "\n}\n";
        summary.close();
    }
};
} // namespace wbr
