// SPDX-License-Identifier: GPL-3.0-or-later
// Metrics for the isolated ORB-SLAM3 examples. No camera or ORB dependencies.
#pragma once
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <ostream>

namespace wbr {
struct Counts {
    std::size_t processed = 0, tracked = 0, startup = 0, untracked_after_start = 0;
    std::size_t loss_transitions = 0, lost_state_frames = 0;
    bool established = false, previous_tracked = false;
    void add(int state) {
        const bool ok = state == 2; // ORB Tracking::OK; pose alone is not success.
        ++processed;
        if (ok) { ++tracked; established = true; }
        else if (!established) ++startup;
        else ++untracked_after_start;
        if (established && previous_tracked && !ok) ++loss_transitions;
        if (state == 3 || state == 4) ++lost_state_frames;
        previous_tracked = ok;
    }
    void json(std::ostream& out, double window_seconds) const {
        out << "\"processed_frames\":" << processed
            << ",\n\"tracked_frames\":" << tracked
            << ",\n\"startup_untracked_frames\":" << startup
            << ",\n\"untracked_after_established_frames\":" << untracked_after_start
            << ",\n\"recently_lost_or_lost_state_frames\":" << lost_state_frames
            << ",\n\"loss_transitions_after_established\":" << loss_transitions
            << ",\n\"processing_window_s\":" << window_seconds
            << ",\n\"processed_fps\":" << (window_seconds > 0 ? processed / window_seconds : 0)
            << ",\n\"successfully_tracked_fps\":" << (window_seconds > 0 ? tracked / window_seconds : 0);
    }
};
inline void csv_header(std::ostream& out) {
    out << "track_call_index,processed_frame_index,source_timestamp_s,track_start_elapsed_s,track_end_elapsed_s,"
           "track_duration_s,tracking_state,tracked,imu_initialized,source_delta_s,"
           "processed_start_delta_s,callback_overwrites,source_frame_number,"
           "source_frame_number_delta,source_timestamp_domain,imu_samples,track_completed\n";
}
} // namespace wbr
