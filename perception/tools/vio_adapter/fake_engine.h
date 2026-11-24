#pragma once
#include "adapter.h"
#include <stdexcept>
#include <algorithm>
#include <utility>
namespace wbr::vio {
// Script states and map events; default replay script is intentionally synthetic.
struct FakeStep { State state; uint64_t map_id=1; bool discontinuity=false; };
class FakeEngine final : public Engine {
public:
    explicit FakeEngine(std::vector<FakeStep> script={{State::INITIALIZING},{State::TRACKING},
        {State::LOST},{State::INITIALIZING,2,true},{State::TRACKING,2}}):script_(std::move(script)) {
        if (script_.empty()) throw std::invalid_argument("empty fake script");
    }
    EngineResult track(const Stereo& p,const std::vector<Combined>& imu) override {
        if (imu.empty() || imu.back().stamp.ns<p.left.stamp.ns) throw std::runtime_error("missing IMU endpoint");
        auto step=script_[std::min(index_++,script_.size()-1)];
        EngineResult r; r.source=p.left.stamp; r.state=step.state; r.map_id=step.map_id; r.discontinuity=step.discontinuity;
        if (step.state==State::TRACKING) { Transform t; t.matrix[3]=double(index_); r.T_world_camera=t; }
        return r;
    }
    void reset() override { index_=0; }
private:
    std::vector<FakeStep> script_; size_t index_=0;
};
}
