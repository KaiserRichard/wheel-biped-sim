#include "wbr_orb_node/core.hpp"
#include "clock_policy.hpp"
#include "fake_engine.h"
#include <atomic>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>
using namespace wbr;
using namespace wbr::orb_node;
namespace {
void require(bool value,const char* reason) { if (!value) throw std::runtime_error(reason); }
Config config() { Config c; c.source_domain="global_time"; c.source_session="synthetic-session"; return c; }
vio::Image image(int64_t ns) { return {{ns,"global_time"},1,1,{42}}; }
Imu imu(int64_t ns) { return {{ns,"global_time"},{0,0,9.81},{0,0,0}}; }
struct Harness {
    Config c=config(); std::unique_ptr<Recorder> events=std::make_unique<Recorder>("synthetic-host");
    vio::FakeEngine engine{{{vio::State::TRACKING}}};
    Core core{engine,c,*events}; uint64_t id=1; std::vector<Output> outputs;
    Harness() { core.on_output=[this](const Output& out){ outputs.push_back(out); core.output_written(out); }; }
    void feed(Input input) { core.consume({id++,std::move(input)}); }
};
void pairing() {
    Harness h;
    h.feed(Left{image(10)}); h.feed(Right{image(11)});
    h.feed(imu(0)); h.feed(imu(20));
    require(h.outputs.empty(),"nonexact stereo was processed");
    h.feed(Left{image(11)});
    require(h.outputs.size()==1,"exact pair did not process");
    require(h.core.counts().unmatched==1,"unmatched drop not counted");
    require(h.outputs[0].snapshot.source->ns==11,"source stamp changed");
    h.core.finish();
}
void coverage() {
    Harness h;
    h.feed(Left{image(15)}); h.feed(Right{image(15)}); h.feed(imu(0)); h.feed(imu(10));
    require(h.outputs.empty(),"image processed without trailing IMU endpoint");
    h.feed(imu(20));
    require(h.outputs.size()==1 && h.core.adapter_counts().combined==3,"coverage/interpolation not delegated");
    require(h.outputs[0].snapshot.source->ns==15,"coverage gate restamped image");
    h.core.finish();
}
void image_rejection_does_not_invent_imu_fault() {
    Harness h;
    h.feed(imu(0)); h.feed(imu(20));
    auto mismatched=image(15);
    mismatched.width=2; mismatched.y8={42,43};
    h.feed(Left{image(15)}); h.feed(Right{mismatched});
    require(h.outputs.empty(),"rejected stereo pair produced output");
    require(!h.core.faulted(),"image rejection was misclassified as IMU loss");
    require(h.core.counts().invalid==1 && h.core.adapter_counts().image==1,
            "stereo image rejection was not counted");
    h.core.finish();
    std::ostringstream trace; h.events->dump_csv(trace);
    const auto csv=trace.str();
    const auto first=csv.find(",adapter_image_invalid,");
    require(first!=std::string::npos && csv.find(",adapter_image_invalid,",first+1)==std::string::npos,
            "stereo image rejection emitted duplicate drop events");
}
void drops() {
    Config c=config(); c.image_capacity=2; c.imu_capacity=2; c.ingress_capacity=2;
    auto events=std::make_unique<Recorder>("synthetic-host");
    vio::FakeEngine engine{{{vio::State::TRACKING}}}; Core core(engine,c,*events);
    core.consume({1,Left{image(10)}}); core.consume({2,Left{image(20)}}); core.consume({3,Left{image(30)}});
    require(core.counts().left_oldest==1,"image queue failed to drop oldest");
    core.consume({4,imu(0)}); core.consume({5,imu(1)}); core.consume({6,imu(2)});
    require(core.counts().imu_oldest==1 && core.faulted(),"IMU oldest loss must fault session");
    core.finish();
    Ingress ingress(c,*events);
    ingress.submit(Left{image(100)}); ingress.submit(Right{image(100)}); ingress.submit(imu(100));
    require(ingress.counts().oldest==1 && ingress.depth()==2,"ingress bound/drop count wrong");
    auto message=ingress.pop(); require(message && std::holds_alternative<Right>(message->input),"ingress dropped newest");
    ingress.submit(imu(101)); ingress.submit(imu(102));
    require(ingress.counts().imu_loss==1,"lost queued IMU was not counted");
}
void stamps_and_frames() {
    Harness h;
    const int64_t t=1791950284587092529LL; // real saved source epoch, synthetic pixels/IMU
    h.feed(imu(t-10)); h.feed(imu(t+10)); h.feed(Left{image(t)}); h.feed(Right{image(t)});
    require(h.outputs.size()==1,"large epoch not processed");
    auto out=h.outputs[0];
    vio::Transform extrinsic; extrinsic.parent="camera_left_optical"; extrinsic.child="imu_camera";
    extrinsic.matrix[3]=2;
    auto pose=odometry_pose(out.snapshot,extrinsic);
    require(pose && int64_t(pose->sec)*1000000000LL+pose->nanosec==t,"ROS stamp not source epoch");
    require(pose->position[0]==3,"camera IMU lever arm composition wrong");
    // Nonidentity rotation verifies the lever arm is rotated into the world.
    auto& m=out.snapshot.T_world_camera->matrix;
    m[0]=0; m[1]=-1; m[4]=1; m[5]=0;
    pose=odometry_pose(out.snapshot,extrinsic);
    require(pose && pose->position[0]==1 && pose->position[1]==2,"rotated lever arm incorrect");
    require(std::abs(pose->xyzw[2]-std::sqrt(0.5))<1e-12,"bridge quaternion mapping incorrect");
    extrinsic.child="base_link";
    require(!odometry_pose(out.snapshot,extrinsic),"robot body mislabeled camera IMU");
    out.snapshot.valid=false;
    require(!odometry_pose(out.snapshot,extrinsic),"invalid snapshot published");
    h.core.finish();
}
void faults() {
    Harness h; h.feed(imu(10)); h.feed(imu(9));
    require(h.core.faulted(),"IMU reversal not latched");
    h.feed(Left{image(20)}); h.feed(Right{image(20)}); h.feed(imu(30));
    require(h.outputs.empty(),"fault session resumed"); h.core.finish();
    Harness gap; gap.feed(imu(0)); gap.feed(imu(gap.c.max_imu_gap_ns+1));
    require(gap.core.faulted(),"IMU gap accepted"); gap.core.finish();
    Harness domain; auto wrong=imu(0); wrong.stamp.domain="stm32_timer"; domain.feed(wrong);
    require(domain.core.faulted(),"cross-domain IMU accepted"); domain.core.finish();
    bridge::ClockPolicy policy(true,defaults::max_ros_step_ns);
    require(policy.observe(0,true)=="waiting_for_clock","uninitialized replay clock accepted");
    policy.observe(100,true); require(policy.observe(99,true)=="clock_jump_restart_required","replay seek not latched");
}
void states() {
    auto events=std::make_unique<Recorder>("synthetic-host"); auto c=config();
    vio::FakeEngine engine; Core core(engine,c,*events); std::vector<Output> outputs;
    core.on_output=[&](const Output& out){ outputs.push_back(out); core.output_written(out); };
    uint64_t id=1;
    for (int64_t t=10;t<=50;t+=10) {
        if (t==10) core.consume({id++,imu(0)});
        core.consume({id++,imu(t)}); core.consume({id++,Left{image(t)}}); core.consume({id++,Right{image(t)}});
    }
    require(outputs.size()==5,"script stream incomplete");
    require(!outputs[0].snapshot.valid && outputs[1].snapshot.valid && !outputs[2].snapshot.valid &&
            !outputs[3].snapshot.valid && outputs[4].snapshot.valid,"tracking/loss/reset validity wrong");
    require(outputs[3].snapshot.reset_counter==1 && outputs[4].snapshot.reset_counter==1,"reset counter lost");
    core.finish();
}
void concurrent_worker() {
    struct SlowEngine : vio::Engine {
        std::atomic<bool> entered{false},release{false};
        vio::FakeEngine fake{{{vio::State::TRACKING}}};
        vio::EngineResult track(const vio::Stereo& pair,const std::vector<vio::Combined>& samples) override {
            entered=true;
            while (!release) std::this_thread::yield();
            return fake.track(pair,samples);
        }
        void reset() override { fake.reset(); }
    } engine;
    auto c=config(); c.ingress_capacity=2;
    auto events=std::make_unique<Recorder>("synthetic-host");
    Ingress ingress(c,*events); Core core(engine,c,*events);
    ingress.submit(imu(0)); ingress.submit(imu(20));
    while (auto message=ingress.pop()) core.consume(std::move(*message));
    ingress.submit(Left{image(15)}); ingress.submit(Right{image(15)});
    std::thread runner([&]{ while (auto message=ingress.pop()) core.consume(std::move(*message)); });
    while (!engine.entered) std::this_thread::yield();
    // A tracking call is still blocked. Callback enqueue and drop-oldest must
    // finish before we let that call return; no shared tracking/ingress lock.
    ingress.submit(Left{image(30)}); ingress.submit(Right{image(30)}); ingress.submit(Left{image(40)});
    const auto drops=ingress.counts().oldest;
    engine.release=true; runner.join();
    require(drops==1,"slow engine prevented bounded callback progress"); core.finish();
}
void engine_fault() {
    struct BadEngine : vio::Engine {
        vio::EngineResult track(const vio::Stereo&,const std::vector<vio::Combined>&) override {
            throw std::runtime_error("injected tracking fault");
        }
        void reset() override {}
    } engine;
    auto c=config(); auto events=std::make_unique<Recorder>("synthetic-host");
    Core core(engine,c,*events); std::vector<Output> outputs;
    core.on_output=[&](const Output& output){ outputs.push_back(output); core.output_written(output); };
    core.consume({1,imu(0)}); core.consume({2,imu(20)});
    core.consume({3,Left{image(15)}}); core.consume({4,Right{image(15)}});
    require(outputs.size()==1 && outputs[0].snapshot.state==vio::State::FAULT,"engine fault not surfaced");
    require(outputs[0].source_image.ns==15 && !outputs[0].snapshot.source,"fault image/source identity incorrect");
    const auto health=health_json(outputs[0],"{\"valid\":false}",true);
    require(health.find("SYSTEM_NOT_READY")!=std::string::npos,"engine fault incompatible with sender");
    core.finish();
}
void schema() {
    for (auto state:{vio::State::STARTING,vio::State::INITIALIZING,vio::State::TRACKING,vio::State::LOST,vio::State::FAULT}) {
        Output out{}; out.snapshot.state=state; out.source_image={15,"global_time"}; out.source_session="synthetic-session";
        std::cout << health_json(out,"{\"valid\":false}",true) << '\n';
    }
}
void emit(const std::string& directory) {
    auto c=config(); auto events=std::make_unique<Recorder>("synthetic-host");
    vio::FakeEngine engine{{{vio::State::TRACKING}}}; Core core(engine,c,*events); Ingress ingress(c,*events);
    std::ofstream json(directory+"/pose_events.jsonl");
    core.on_output=[&](const Output& out){ json << health_json(out,"{\"valid\":false}",true) << '\n'; core.output_written(out); };
    ingress.submit(imu(0)); ingress.submit(imu(20)); ingress.submit(Left{image(15)}); ingress.submit(Right{image(15)});
    while (auto value=ingress.pop()) core.consume(std::move(*value));
    core.finish();
    std::ofstream csv(directory+"/events.csv"); events->dump_csv(csv);
    require(json.good() && csv.good(),"trace write failed");
}
void real_epochs(const std::string& path) {
    std::ifstream input(path); require(bool(input),"missing real epoch fixture");
    Harness h; int64_t ns,previous=0; size_t rows=0;
    while (input>>ns) {
        if (!rows) { h.feed(imu(ns-2)); h.feed(imu(ns+2)); }
        else {
            // Fill synthetic IMU through the measured camera interval. Values
            // and pixels are deliberately synthetic; only epochs are MEASURED.
            for (int64_t t=previous+2+10000000;t<ns;t+=10000000) h.feed(imu(t));
            h.feed(imu(ns+2));
        }
        h.feed(Left{image(ns)}); h.feed(Right{image(ns)});
        require(h.outputs.size()==rows+1,"real epoch pair failed");
        require(h.outputs.back().snapshot.source->ns==ns,"real epoch lost precision");
        previous=ns; ++rows;
    }
    require(rows>1,"real fixture is empty"); h.core.finish();
}
}
int main(int argc,char** argv) {
    try {
        if (argc<2) throw std::invalid_argument("scenario required");
        std::string scenario=argv[1];
        if (scenario=="pairing" || scenario=="all") pairing();
        if (scenario=="coverage" || scenario=="all") coverage();
        if (scenario=="image_rejection" || scenario=="all") image_rejection_does_not_invent_imu_fault();
        if (scenario=="drops" || scenario=="all") drops();
        if (scenario=="frames" || scenario=="all") stamps_and_frames();
        if (scenario=="faults" || scenario=="all") faults();
        if (scenario=="states" || scenario=="all") states();
        if (scenario=="concurrent" || scenario=="all") concurrent_worker();
        if (scenario=="engine_fault" || scenario=="all") engine_fault();
        if (scenario=="schema") schema();
        if (scenario=="emit") { if (argc!=3) throw std::invalid_argument("output dir required"); emit(argv[2]); }
        if (scenario=="real") { if (argc!=3) throw std::invalid_argument("epoch file required"); real_epochs(argv[2]); }
        if (scenario=="fail") require(false,"injected fixture failure");
        std::cout << "PASS " << scenario << " (synthetic software checks; no hardware gate)\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << "FAIL " << error.what() << '\n'; return 1; }
}
