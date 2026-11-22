#ifndef WBR_RUNTIME_OBSERVER_EVENT_RECORDER_H
#define WBR_RUNTIME_OBSERVER_EVENT_RECORDER_H

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <ostream>
#include <stdexcept>
#include <time.h>

namespace wbr::observer {

enum class Kind { frame_received, queued, track_start, track_end,
                  snapshot_published, dropped, queue_depth };

inline const char* name(Kind kind) noexcept {
    switch (kind) {
        case Kind::frame_received: return "frame_received";
        case Kind::queued: return "queued";
        case Kind::track_start: return "track_start";
        case Kind::track_end: return "track_end";
        case Kind::snapshot_published: return "snapshot_published";
        case Kind::dropped: return "dropped";
        case Kind::queue_depth: return "queue_depth";
    }
    return nullptr;
}

struct SourceEpoch {
    std::int64_t ns;
    const char* domain;
    const char* session;
};

// No owned strings, allocation, I/O, or waits in record(). Tokens have at most
// 47 characters; rejecting invalid/overlong tokens prevents silent truncation.
template <std::size_t Capacity>
class Recorder {
    static_assert(Capacity > 0, "ring capacity must be positive");
    static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
                  "loss counters require lock-free uint64 atomics");
    struct Event {
        std::int64_t host_ns = 0, source_ns = 0, depth = -1;
        std::uint64_t frame_id = 0;
        Kind kind = Kind::frame_received;
        bool has_source = false;
        char source_domain[48]{}, source_session[48]{}, reason[48]{};
    };
    std::array<Event, Capacity> events_{};
    char session_[48]{};
    std::mutex mutex_;
    std::size_t next_ = 0, size_ = 0;
    std::atomic<std::uint64_t> overwritten_{0}, contention_{0}, invalid_{0}, clock_errors_{0};

    static bool token(char (&out)[48], const char* in) noexcept {
        if (!in || !*in) return false;
        std::size_t n = 0;
        for (; n < 48 && in[n]; ++n) {
            const char c = in[n];
            if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                  (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.'))
                return false;
            out[n] = c;
        }
        if (n == 48) return false;
        out[n] = '\0';
        return true;
    }

public:
    explicit Recorder(const char* host_session) {
        if (!token(session_, host_session))
            throw std::invalid_argument("host session must be a 1..47 character token");
    }
    Recorder(const Recorder&) = delete;
    Recorder& operator=(const Recorder&) = delete;

    // One try_lock attempt. true means retained now, but later writes can
    // overwrite it. false means a counted rejection. Mutex/platform scheduling
    // cost is NOT a proven real-time bound. Frame IDs must be unique per session.
    bool record(Kind kind, std::uint64_t frame_id, const SourceEpoch* source = nullptr,
                const char* reason = nullptr, std::int64_t depth = -1) {
        Event event;
        event.kind = kind;
        event.frame_id = frame_id;
        event.depth = depth;
        if (!name(kind) || (kind == Kind::queue_depth ? depth < 0 : depth != -1) ||
            (kind == Kind::dropped ? !token(event.reason, reason) : reason != nullptr) ||
            (source && (!token(event.source_domain, source->domain) ||
                        !token(event.source_session, source->session)))) {
            invalid_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        if (source) { event.source_ns = source->ns; event.has_source = true; }
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {
            contention_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        timespec ts{};
        if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
            clock_errors_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        event.host_ns = static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
        events_[next_] = event;
        next_ = (next_ + 1) % Capacity;
        if (size_ < Capacity) ++size_;
        else overwritten_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    // Call after stopping/joining all producers, at normal exit. Explicit dump
    // keeps streams out of the hot path. Not async-signal-safe; crashes may lose
    // the in-memory trace. The caller owns the output stream and checks errors.
    void dump_csv(std::ostream& out) {
        std::lock_guard<std::mutex> lock(mutex_);
        out << "# observer_loss overwritten=" << overwritten_.load()
            << " contention=" << contention_.load() << " invalid=" << invalid_.load()
            << " clock_errors=" << clock_errors_.load() << '\n';
        out << "host_ns,host_domain,host_session,frame_id,event,source_ns,source_domain,source_session,reason,depth\n";
        const auto first = (next_ + Capacity - size_) % Capacity;
        for (std::size_t i = 0; i < size_; ++i) {
            const auto& e = events_[(first + i) % Capacity];
            out << e.host_ns << ",host_steady," << session_ << ',' << e.frame_id << ',' << name(e.kind) << ',';
            if (e.has_source) out << e.source_ns;
            out << ',' << e.source_domain << ',' << e.source_session << ',' << e.reason << ',';
            if (e.kind == Kind::queue_depth) out << e.depth;
            out << '\n';
        }
    }
};
}  // namespace wbr::observer
#endif
