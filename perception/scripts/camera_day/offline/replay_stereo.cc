// SPDX-License-Identifier: GPL-3.0-or-later
// Headless recorded-pair runner. No camera, ROS, motors or calibration writes.
#include <System.h>
#include "orb_access.hpp"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

using Clock = std::chrono::steady_clock;
static int64_t steady_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}
static std::string source_time(int64_t ns) {
    std::ostringstream out;
    out << ns / 1000000000 << '.' << std::setw(9) << std::setfill('0') << ns % 1000000000;
    return out.str();
}
int main(int argc, char** argv) try {
    if (argc != 7) throw std::runtime_error("Usage: replay VOCAB SETTINGS PAIRS_DIRECTORY NEW_OUTPUT EXPECTED_PAIRS PACE_RATE");
    const std::filesystem::path input(argv[3]), output(argv[4]);
    const size_t expected = std::stoull(argv[5]);
    const double rate = std::stod(argv[6]);
    if (!std::isfinite(rate) || rate <= 0) throw std::runtime_error("pace rate must be positive");
    if (!std::filesystem::create_directory(output)) throw std::runtime_error("output exists; refusing overwrite");
    std::ifstream pairs(input / "pairs.csv");
    if (!pairs) throw std::runtime_error("cannot open pairs.csv");
    std::ofstream frames(output / "frames.csv"), poses(output / "CameraTrajectory.txt"), maps(output / "map-events.csv"), attempts(output / "attempts.csv");
    frames.exceptions(std::ios::badbit | std::ios::failbit);
    poses.exceptions(std::ios::badbit | std::ios::failbit);
    attempts.exceptions(std::ios::badbit | std::ios::failbit);
    attempts << "attempt_index,source_ns,processing_start_steady_ns\n";
    frames << "source_ns,tracking_state,pose_published,processed,application_dropped,processing_start_steady_ns,processing_end_steady_ns,map_id\n";
    maps << "source_ns,map_id,event\n";
    poses << std::setprecision(17);
    ORB_SLAM3::System slam(argv[1], argv[2], ORB_SLAM3::System::STEREO, false);
    std::string line;
    std::getline(pairs, line);
    if (line != "source_ns,left,right") throw std::runtime_error("unexpected pair manifest header");
    size_t processed = 0, tracked = 0, losses = 0, map_changes = 0;
    bool was_tracked = false;
    int64_t first_source = -1, previous_source = -1, first_start = 0, final_end = 0, sum_track = 0;
    long previous_map = -1;
    const auto pacing_epoch = Clock::now();
    while (std::getline(pairs, line)) {
        std::istringstream row(line);
        std::string stamp, left, right;
        if (!std::getline(row, stamp, ',') || !std::getline(row, left, ',') || !std::getline(row, right) || right.find(',') != std::string::npos)
            throw std::runtime_error("invalid pair row");
        const int64_t ns = std::stoll(stamp);
        if (ns <= previous_source) throw std::runtime_error("non-increasing pair timestamp");
        if (first_source < 0) first_source = ns;
        previous_source = ns;
        const auto im_left = cv::imread((input / left).string(), cv::IMREAD_UNCHANGED);
        const auto im_right = cv::imread((input / right).string(), cv::IMREAD_UNCHANGED);
        if (im_left.type() != CV_8UC1 || im_right.type() != CV_8UC1 || im_left.rows != 480 || im_left.cols != 640 || im_right.size() != im_left.size())
            throw std::runtime_error("invalid stereo pixels");
        std::this_thread::sleep_until(pacing_epoch + std::chrono::nanoseconds(static_cast<int64_t>((ns - first_source) / rate)));
        attempts << processed + 1 << ',' << ns << ',' << steady_ns() << '\n';
        attempts.flush();
        const int64_t start = steady_ns();
        if (!processed) first_start = start;
        const auto tcw = slam.TrackStereo(im_left, im_right, static_cast<double>(ns) * 1e-9);
        const int64_t end = steady_ns();
        sum_track += end - start;
        final_end = end;
        const int state = slam.GetTrackingState();
        const auto twc = tcw.inverse();
        const bool success = state == 2 && twc.matrix().allFinite();
        if (was_tracked && !success) ++losses;
        was_tracked = success;
        ++processed;
        if (success) {
            ++tracked;
            const auto t = twc.translation();
            const auto q = twc.unit_quaternion();
            poses << source_time(ns) << ' ' << t.x() << ' ' << t.y() << ' ' << t.z() << ' '
                  << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
        }
        const auto map = wbr::atlas(slam)->GetCurrentMap();
        const long map_id = map ? static_cast<long>(map->GetId()) : -1;
        if (map_id != previous_map) {
            maps << ns << ',' << map_id << ',' << (previous_map < 0 ? "initial_map" : "current_map_changed") << '\n';
            if (previous_map >= 0) ++map_changes;
            previous_map = map_id;
        }
        frames << ns << ',' << state << ',' << (success ? "true" : "false") << ',' << processed << ",0," << start << ',' << end << ',' << map_id << '\n';
        frames.flush();
        poses.flush();
        if (processed % 150 == 0) std::cout << "PROGRESS pairs=" << processed << " tracked=" << tracked << std::endl;
    }
    slam.Shutdown();
    const auto deadline = Clock::now() + std::chrono::seconds(60);
    while (!wbr::workers_finished(slam)) {
        if (Clock::now() > deadline) throw std::runtime_error("ORB workers did not finish within60s");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    slam.SaveTrajectoryTUM((output / "FinalAtlasTrajectory.txt").string());
    slam.SaveKeyFrameTrajectoryTUM((output / "KeyFrameTrajectory.txt").string());
    if (processed != expected) throw std::runtime_error("processed pair count disagrees with expected");
    std::ofstream summary(output / "summary.json");
    summary << std::setprecision(17) << "{\n\"processed\":" << processed << ",\n\"tracked\":" << tracked
            << ",\n\"loss_transitions\":" << losses << ",\n\"current_map_changes\":" << map_changes
            << ",\n\"first_source_ns\":" << first_source << ",\n\"last_source_ns\":" << previous_source
            << ",\n\"steady_span_s\":" << (final_end - first_start) * 1e-9
            << ",\n\"sum_track_s\":" << sum_track * 1e-9
            << ",\n\"tracked_fps_paced_span\":" << tracked / ((final_end - first_start) * 1e-9)
            << ",\n\"tracked_per_track_compute_second\":" << tracked / (sum_track * 1e-9)
            << ",\n\"pace_rate\":" << rate << ",\n\"all_pairs_processed\":true,\n\"shutdown_complete\":true\n}\n";
    return 0;
} catch (const std::exception& error) {
    std::cerr << "Replay failed: " << error.what() << std::endl;
    return 1;
}
