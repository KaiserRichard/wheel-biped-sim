#pragma once
#include "adapter.h"
#ifdef WBR_WITH_ORBSLAM3
#include <functional>
namespace ORB_SLAM3 { class System; }
namespace wbr::vio {
struct OrbMapMetadata {
    uint64_t map_id, reset_generation;
    int big_change_index;
};
// Optional reviewed engine extension: return copied metadata under engine locks.
// Stock System has no public Atlas getter; never bypass its private members.
using OrbMapObserver=std::function<OrbMapMetadata()>;
class OrbEngine final : public Engine {
public:
    // Borrow an IMU_STEREO System; owner handles Shutdown and serializes all calls.
    explicit OrbEngine(ORB_SLAM3::System&, OrbMapObserver={});
    EngineResult track(const Stereo&, const std::vector<Combined>&) override;
    void reset() override;
private:
    ORB_SLAM3::System& system_; OrbMapObserver observer_;
    std::optional<OrbMapMetadata> previous_;
    std::optional<int64_t> origin_;
    State previous_state_=State::STARTING;
};
}
#endif
