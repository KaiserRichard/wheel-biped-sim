#include "adapter.h"
#include "fake_engine.h"
#include <iostream>
#include <stdexcept>
#include <iomanip>
#include <cstring>
#include <cctype>
using namespace wbr::vio;
// Private driver protocol: little-endian integers/double and length-prefixed domain.
// Every record carries its own clock domain; there is no implicit conversion.
static uint64_t integer(size_t n) {
    uint64_t v=0;
    for (size_t i=0;i<n;++i) { int c=std::cin.get(); if(c==EOF) throw std::runtime_error("truncated event"); v|=uint64_t(c)<<(8*i); }
    return v;
}
static double real() {
    uint64_t bits=integer(8); double v; static_assert(sizeof(v)==8);
    std::memcpy(&v,&bits,8); return v;
}
static Stamp stamp() {
    auto ns=integer(8); auto n=integer(2);
    std::string domain(n,' '); std::cin.read(domain.data(),n);
    if (!std::cin || ns>uint64_t(INT64_MAX)) throw std::runtime_error("invalid stamp");
    return {int64_t(ns),domain};
}
static void log_snapshot(const EstimateSnapshot& s) {
    std::cout<<"{\"evidence\":\"MEASURED association; SYNTHETIC fake estimate\",\"source_ns\":";
    if (s.source) std::cout<<s.source->ns; else std::cout<<"null";
    std::cout<<",\"domain\":\""<<(s.source?s.source->domain:"")<<"\",\"sequence\":"<<s.sequence
        <<",\"state\":\""<<state_name(s.state)<<"\",\"valid\":"<<(s.valid?"true":"false")
        <<",\"reasons\":"<<s.reasons<<",\"reset_counter\":"<<s.reset_counter<<",\"epoch_id\":"<<s.epoch_id
        <<",\"map_id\":"; if (s.map_id) std::cout<<*s.map_id; else std::cout<<"null";
    std::cout<<",\"covariance_availability\":\"UNAVAILABLE\",\"T_world_camera\":";
    if (s.T_world_camera) {
        std::cout<<"{\"parent\":\""<<s.T_world_camera->parent<<"\",\"child\":\""<<s.T_world_camera->child<<"\",\"matrix\":[";
        for (size_t i=0;i<16;++i) { if(i)std::cout<<","; std::cout<<s.T_world_camera->matrix[i]; } std::cout<<"]}";
    } else std::cout<<"null";
    std::cout<<"}\n";
}
int main(int argc,char** argv) {
    try {
        if(argc!=2) throw std::runtime_error("usage: replay_fake CLOCK_DOMAIN < events.bin");
        std::string domain=argv[1];
        for (char c:domain) if (!(std::isalnum(static_cast<unsigned char>(c)) || c=='_' || c=='-' || c=='.')) throw std::runtime_error("unsafe domain spelling");
        FakeEngine engine; Adapter adapter(engine,domain); adapter.on_snapshot=log_snapshot;
        int kind;
        while ((kind=std::cin.get())!=EOF) {
            auto s=stamp();
            if (kind=='A' || kind=='G') {
                Motion m{s,{real(),real(),real()}};
                if(kind=='A') adapter.feed_accel(m); else adapter.feed_gyro(m);
            } else if (kind=='L' || kind=='R') {
                auto w=integer(4),h=integer(4);
                if (!w || !h || w>8192 || h>8192) throw std::runtime_error("invalid dimensions");
                Image image{s,uint32_t(w),uint32_t(h),std::vector<uint8_t>(w*h)};
                std::cin.read(reinterpret_cast<char*>(image.y8.data()),image.y8.size());
                if (!std::cin) throw std::runtime_error("truncated Y8 image");
                if(kind=='L') adapter.feed_left(std::move(image)); else adapter.feed_right(std::move(image));
            } else throw std::runtime_error("unknown event kind");
        }
        adapter.finish(); const auto& c=adapter.counters();
        std::cerr<<"{";
#define COUNT(name) std::cerr<<"\"" #name "\":"<<c.name<<",";
        COUNT(accel) COUNT(gyro) COUNT(left) COUNT(right) COUNT(pairs) COUNT(combined)
        COUNT(domain) COUNT(order) COUNT(nonfinite) COUNT(gap) COUNT(overflow) COUNT(image)
        COUNT(unmatched) COUNT(prefix_gyro) COUNT(prefix_pairs) COUNT(missing_coverage)
        COUNT(tail_accel) COUNT(tail_gyro)
#undef COUNT
        std::cerr<<"\"engine_rejected\":"<<c.engine_rejected<<"}\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<"\n"; return 2; }
}
