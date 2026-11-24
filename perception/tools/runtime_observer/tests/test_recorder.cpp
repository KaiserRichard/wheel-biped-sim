#include "../event_recorder.h"

#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <new>
#include <sstream>
#include <streambuf>
#include <thread>
#include <vector>

namespace {
std::atomic<bool> watch_allocations{false};
std::atomic<unsigned> allocations{0};
}
void* operator new(std::size_t n) {
    if (watch_allocations.load()) ++allocations;
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

using namespace wbr::observer;

struct BlockingBuffer : std::streambuf {
    std::atomic<bool> entered{false}, release{false};
    std::streamsize xsputn(const char*, std::streamsize n) override {
        entered = true;
        while (!release.load()) std::this_thread::yield();
        return n;
    }
    int overflow(int c) override { return c; }
};

int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "--dump") {
        Recorder<16> recorder("test-run");
        SourceEpoch source{100, "device", "device-boot"};
        for (auto kind : {Kind::frame_received, Kind::queued, Kind::track_start,
                          Kind::track_end, Kind::snapshot_published})
            assert(recorder.record(kind, 1, &source));
        assert(recorder.record(Kind::dropped, 2, nullptr, "backlog"));
        assert(recorder.record(Kind::queue_depth, 0, nullptr, nullptr, 3));
        std::ofstream out(argv[2]);
        recorder.dump_csv(out);
        out.close();
        return out ? 0 : 1;
    }
    Recorder<3> ring("run-a");
    SourceEpoch source{1700000000000000001LL, "global_time", "device-a"};
    watch_allocations = true;
    assert(ring.record(Kind::frame_received, 1, &source));
    assert(ring.record(Kind::queued, 1));
    assert(ring.record(Kind::track_start, 1));
    assert(ring.record(Kind::track_end, 1));
    assert(!ring.record(Kind::dropped, 2));
    assert(!ring.record(Kind::queue_depth, 0, nullptr, nullptr, -1));
    assert(!ring.record(Kind::dropped, 2, nullptr, "bad,reason"));
    assert(!ring.record(Kind::queued, 2, nullptr, nullptr, 1));
    SourceEpoch invalid{0, "a-name-that-is-longer-than-forty-seven-characters-in-total", "boot"};
    assert(!ring.record(Kind::queued, 2, &invalid));
    assert(!ring.record(static_cast<Kind>(123), 2));
    watch_allocations = false;
    assert(allocations == 0);
    std::ostringstream out;
    ring.dump_csv(out);
    assert(out.str().find("overwritten=1 contention=0 invalid=6 clock_errors=0") != std::string::npos);
    assert(out.str().find(",frame_received,") == std::string::npos);
    assert(out.str().find(",queued,") < out.str().find(",track_start,"));
    assert(out.str().find(",track_start,") < out.str().find(",track_end,"));

    Recorder<2> copy("copy");
    char domain[] = "global_time";
    SourceEpoch mutable_source{1700000000000000001LL, domain, "boot"};
    assert(copy.record(Kind::frame_received, 9, &mutable_source));
    domain[0] = 'x';
    out.str("");
    copy.dump_csv(out);
    assert(out.str().find("1700000000000000001,global_time,boot") != std::string::npos);

    // Deliberately hold the dump mutex to prove record() makes one nonblocking
    // attempt and loss is counted, without a scheduler-dependent contention race.
    BlockingBuffer buffer;
    std::ostream blocked(&buffer);
    std::thread dumper([&] { copy.dump_csv(blocked); });
    while (!buffer.entered.load()) std::this_thread::yield();
    assert(!copy.record(Kind::queued, 9));
    buffer.release = true;
    dumper.join();
    out.str("");
    copy.dump_csv(out);
    assert(out.str().find("contention=1") != std::string::npos);

    Recorder<4000> concurrent("threads");
    std::vector<std::thread> threads;
    std::atomic<unsigned> accepted{0};
    for (unsigned t = 0; t < 4; ++t)
        threads.emplace_back([&, t] {
            for (unsigned i = 0; i < 1000; ++i)
                if (concurrent.record(Kind::frame_received, t * 1000 + i)) ++accepted;
        });
    for (auto& thread : threads) thread.join();
    out.str("");
    concurrent.dump_csv(out);
    const auto trace = out.str();
    const auto begin = trace.find("contention=") + std::string("contention=").size();
    const auto lost = std::stoul(trace.substr(begin));
    assert(accepted + lost == 4000);
    std::istringstream lines(trace);
    std::string line;
    std::getline(lines, line); std::getline(lines, line);
    std::int64_t previous = 0;
    unsigned count = 0;
    while (std::getline(lines, line)) {
        const auto ns = std::stoll(line.substr(0, line.find(',')));
        assert(ns >= previous && ns > 0);
        previous = ns;
        ++count;
    }
    assert(count == accepted);
    std::cout << "Recorder checks passed (allocation, wrap, validation, copied epochs, contention, concurrency).\n";
}
