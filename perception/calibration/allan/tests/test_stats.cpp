// SYNTHETIC ONLY: directly exercises recorder counters with the local SDK stub.
#define ALLAN_RECORDER_TEST
#include "../d435i_imu_record.cpp"
#include <cassert>

int main() {
    Stats stats;
    const auto hardware = RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK;
    const auto system = RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME;
    stats.add(Sample{0, 1000.0, hardware, 1, 0, {}});
    stats.add(Sample{0, 1.0, system, 2, 0, {}});
    assert(stats.domain_changes == 1);
    assert(stats.backwards == 0);
    assert(stats.duplicates == 0);
    assert(stats.max_gap_ms == 0.0);

    stats.add(Sample{0, 2.0, system, 3, 0, {}});
    assert(stats.domain_changes == 1);
    assert(stats.backwards == 0);
    assert(stats.max_gap_ms == 1.0);

    stats.add(Sample{0, 2.0, system, 4, 0, {}});
    assert(stats.duplicates == 1);

    // A forward cross-domain jump must not inflate the previous maximum.
    const auto global = RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME;
    stats.add(Sample{0, 1.0e12, global, 7, 0, {}});
    assert(stats.domain_changes == 2);
    assert(stats.max_gap_ms == 1.0);
    assert(stats.frame_gaps == 2); // Frame counters remain independent of clocks.
    stats.add(Sample{0, 1.0e12 + 5.0, global, 8, 0, {}});
    assert(stats.max_gap_ms == 5.0); // Transition sample established the baseline.
    stats.add(Sample{0, 1.0e12 + 4.0, global, 9, 0, {}});
    assert(stats.backwards == 1); // Same-domain reversal is still counted.

    // Equal numeric values in different domains are not duplicate timestamps.
    stats.add(Sample{0, 1.0e12 + 4.0, hardware, 9, 0, {}});
    assert(stats.domain_changes == 3);
    assert(stats.duplicates == 1);
    assert(stats.frame_reversals == 1);
    assert(stats.count == 8);
}
