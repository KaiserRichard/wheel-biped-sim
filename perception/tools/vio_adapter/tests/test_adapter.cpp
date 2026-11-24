#include "adapter.h"
#include "fake_engine.h"
#include "orb_imu.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace wbr::vio;
static Stamp stamp(int64_t n) { return {n,"device"}; }
static Motion m(int64_t n,double x=0) { return {stamp(n),{x,2,-2}}; }
static Image image(int64_t n) { return {stamp(n),2,1,{10,20}}; }
class Probe : public Engine {
public:
    std::vector<std::vector<Combined>> batches;
    EngineResult result;
    bool stale=false, wrong_domain=false, throwing=false, reset_throw=false;
    EngineResult track(const Stereo& p,const std::vector<Combined>& imu) override {
        if (throwing) throw std::runtime_error("injected");
        batches.push_back(imu); auto r=result; r.source=p.left.stamp;
        if(stale) --r.source.ns; if(wrong_domain) r.source.domain="other";
        return r;
    }
    void reset() override { if(reset_throw) throw std::runtime_error("injected reset failure"); }
    Probe() { result.state=State::TRACKING; result.T_world_camera=Transform{}; }
};
static void pair(Adapter& a,int64_t n) { assert(a.feed_left(image(n))); assert(a.feed_right(image(n))); }
static void covered(Adapter& a,int64_t start) {
    assert(a.feed_accel(m(start,0))); assert(a.feed_accel(m(start+10000000,10)));
    assert(a.feed_gyro(m(start+3000000))); assert(a.feed_gyro(m(start+7000000)));
    pair(a,start+5000000);
}
int main() {
    { // Exactly the helper called immediately before ORB IMU::Point construction.
        for (int component=0; component<6; ++component) {
            for (double value : {1e40, -1e40, 1e-50, -1e-50,
                    std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::quiet_NaN()}) {
                Combined sample{stamp(0),{0,0,0},{0,0,0}};
                double* fields[]={&sample.accel.x,&sample.accel.y,&sample.accel.z,
                                  &sample.gyro.x,&sample.gyro.y,&sample.gyro.z};
                *fields[component]=value;
                bool rejected=false;
                try { (void)orb_motion(sample); }
                catch(const std::range_error&) { rejected=true; }
                assert(rejected);
            }
        }
        const double largest=std::numeric_limits<float>::max();
        const double smallest=std::numeric_limits<float>::denorm_min();
        auto values=orb_motion({stamp(0),{largest,-largest,smallest},{-smallest,0,1.25}});
        assert(values[0]==largest && values[1]==-largest && values[2]==smallest);
        assert(values[3]==-smallest && values[4]==0 && values[5]==1.25f);
        class BoundaryEngine : public Probe {
            EngineResult track(const Stereo& p,const std::vector<Combined>& batch) override {
                for(const auto& sample:batch) (void)orb_motion(sample);
                return Probe::track(p,batch);
            }
        } e;
        Adapter a(e,"device");
        a.feed_accel(m(0)); a.feed_accel(m(10000000));
        a.feed_gyro(m(3000000,1e40)); a.feed_gyro(m(7000000,1e40));
        pair(a,5000000);
        assert(a.snapshot().state==State::FAULT && !a.snapshot().valid);
        assert(a.snapshot().reasons==ENGINE_FAULT && a.counters().engine_rejected==1);
        assert(e.batches.empty());
    }

    { // Original d435i_input test intent: prefix, interpolation, endpoint once, no extrapolation.
        Probe e; Adapter a(e,"device"); a.feed_gyro(m(999000000));
        a.feed_accel(m(1000000000,0)); a.feed_accel(m(1010000000,10));
        assert(a.counters().prefix_gyro==1);
        a.feed_gyro(m(1003000000)); a.feed_gyro(m(1007000000)); pair(a,1005000000);
        assert(e.batches.size()==1 && e.batches[0].size()==2);
        assert(e.batches[0][1].stamp.ns==1007000000);
        assert(std::abs(e.batches[0][0].accel.x-3)<1e-12);
        assert(std::abs(e.batches[0][1].accel.x-7)<1e-12);
        auto old=a.snapshot(); for(int i=0;i<100;++i) assert(a.heartbeat().sequence==old.sequence && a.heartbeat().source->ns==old.source->ns);
        a.feed_gyro(m(1012000000)); pair(a,1011000000); assert(e.batches.size()==1);
        a.feed_accel(m(1020000000,20)); assert(e.batches.size()==2 && e.batches[1].size()==1);
        assert(e.batches[1][0].stamp.ns==1012000000 && std::abs(e.batches[1][0].accel.x-12)<1e-12);
        pair(a,1030000000); a.finish(); assert(a.counters().missing_coverage==1);
    }
    {
        Probe e; Adapter a(e,"device"); a.feed_accel(m(10));
        assert(!a.feed_accel(m(10))); assert(!a.feed_accel(m(9)));
        auto other=m(11); other.stamp.domain="other"; assert(!a.feed_accel(other));
        assert(!a.feed_accel(m(50000011)));
        auto bad=m(11); bad.value.x=std::numeric_limits<double>::quiet_NaN(); assert(!a.feed_accel(bad));
        bad.value.x=std::numeric_limits<double>::infinity(); assert(!a.feed_accel(bad));
        assert(a.counters().order==2 && a.counters().domain==1 && a.counters().gap==1 && a.counters().nonfinite==2);
    }
    {
        Probe e; Adapter a(e,"device",{2,1,50000000});
        a.feed_accel(m(1)); a.feed_accel(m(2)); assert(!a.feed_accel(m(3)));
        a.feed_left(image(10)); assert(!a.feed_left(image(11))); a.feed_right(image(12));
        assert(a.counters().unmatched==1); a.finish(); assert(a.counters().unmatched==2);
        assert(a.counters().overflow==2);
    }
    {
        Probe e; Adapter a(e,"device"); auto bad=image(10); bad.y8.pop_back(); assert(!a.feed_left(bad));
        a.feed_accel(m(0)); a.feed_accel(m(10)); a.feed_gyro(m(5)); pair(a,3);
        assert(e.batches.empty() && a.counters().prefix_pairs==1);
    }
    {
        FakeEngine e; Adapter a(e,"device");
        State states[]={State::INITIALIZING,State::TRACKING,State::LOST,State::INITIALIZING,State::TRACKING};
        for(int i=0;i<5;++i) {
            covered(a,i*20000000); auto s=a.snapshot(); assert(s.sequence==uint64_t(i+1) && s.state==states[i]);
            assert(s.valid==(i==1||i==4)); assert(s.covariance_availability==Availability::UNAVAILABLE);
            if (i==3) assert(s.reset_counter==1 && s.epoch_id==1 && s.map_id==2 && !s.T_world_camera);
        }
        auto old=a.snapshot(); a.reset(); auto reset=a.snapshot();
        assert(reset.reset_counter==2 && reset.epoch_id==2 && !reset.valid && !reset.T_world_camera && reset.reasons==RESET);
        assert(reset.sequence==old.sequence && reset.source->ns==old.source->ns);
        assert(!a.feed_left(image(old.source->ns))); // Stale even after reset.
        covered(a,100000000); assert(a.snapshot().sequence==6 && !a.snapshot().valid);
    }
    {
        Probe e; Adapter a(e,"device"); covered(a,0); auto old=a.snapshot();
        e.stale=true; covered(a,20000000); auto s=a.snapshot();
        assert(!s.valid && s.reasons==STALE_ENGINE && s.sequence==old.sequence && s.source->ns==old.source->ns);
        e.stale=false; e.wrong_domain=true; covered(a,40000000); assert(a.snapshot().sequence==old.sequence);
        e.wrong_domain=false; e.result.T_world_camera->child="base_link"; covered(a,60000000);
        assert(!a.snapshot().valid && a.snapshot().reasons==BAD_FRAME);
        e.result.T_world_camera->child="camera_left_optical"; e.result.T_world_camera->matrix[0]=2; covered(a,80000000);
        assert(a.snapshot().reasons==BAD_POSE);
        e.throwing=true; covered(a,100000000); assert(a.snapshot().state==State::FAULT && a.snapshot().reasons==ENGINE_FAULT);
    }
    { // Nonidentity inversion and named frames, never just negate translation.
        Transform t; t.parent="camera_left_optical"; t.child="slam_world";
        t.matrix={0,-1,0,1, 1,0,0,2, 0,0,1,3, 0,0,0,1};
        auto inv=inverse(t); assert(inv.parent=="slam_world" && inv.child=="camera_left_optical");
        assert(inv.matrix[3]==-2 && inv.matrix[7]==1 && inv.matrix[11]==-3);
        auto again=inverse(inv); assert(again.matrix==t.matrix);
    }
    {
        Probe e; Adapter a(e,"device"); e.result.map_id=1; covered(a,0);
        e.result.map_id=2; covered(a,20000000); assert(!a.snapshot().valid && a.snapshot().reset_counter==1);
        covered(a,40000000); assert(a.snapshot().valid && a.snapshot().reset_counter==1);
        e.result.covariance=std::array<double,36>{}; covered(a,60000000);
        assert(a.snapshot().covariance_availability==Availability::AVAILABLE);
    }
    { // Large integer epoch retains one-ns distinctions.
        Probe e; Adapter a(e,"device"); covered(a,1791950284500000000LL);
        assert(a.snapshot().source->ns==1791950284505000000LL);
    }
    {
        Probe e; Adapter a(e,"device");
        auto left=image(5), right=image(6);
        assert(!a.feed_stereo({left,right})); assert(a.counters().left==0 && a.counters().right==0);
        right=image(5); right.y8.clear(); assert(!a.feed_stereo({left,right}));
        a.feed_accel(m(0)); a.feed_accel(m(10)); a.feed_gyro(m(3)); a.feed_gyro(m(7));
        assert(a.feed_stereo({image(5),image(5)}) && a.snapshot().valid);
        assert(!a.feed_stereo({image(5),image(5)}));
    }
    {
        Probe e; Adapter a(e,"device");
        auto initial=a.heartbeat(); assert(initial.state==State::STARTING && !initial.source && initial.sequence==0);
        covered(a,0); auto old=a.snapshot(); e.reset_throw=true; a.reset();
        auto fault=a.heartbeat(); assert(fault.state==State::FAULT && !fault.valid && !fault.T_world_camera);
        assert(fault.reasons==ENGINE_FAULT && fault.sequence==old.sequence && fault.source->ns==old.source->ns);
        assert(fault.reset_counter==1 && a.counters().engine_rejected==1);
        a.finish(); a.finish();
        bool threw=false; try { a.feed_gyro(m(20000000)); } catch(const std::logic_error&) { threw=true; } assert(threw);
    }
    std::cout<<"MEASURED: 12 synthetic adapter fixture groups passed; ORB/hardware UNKNOWN\n";
}
