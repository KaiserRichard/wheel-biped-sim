// Status: OPEN. D435i imu_sensor / imu_camera only; future Pi-local acquisition.
// Pi build: g++ -std=c++17 -O2 -Wall -Wextra -pthread d435i_imu_record.cpp -o d435i_imu_record -lrealsense2
#include <librealsense2/rs.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t interrupted = 0;
void on_signal(int sig) { interrupted = sig; }
std::string utc() {
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{}; gmtime_r(&t, &tm);
    char b[32]; std::strftime(b, sizeof(b), "%FT%TZ", &tm); return b;
}
std::string quote(const std::string& s) {
    std::ostringstream out; out << '"';
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
        else out << c;
    }
    out << '"'; return out.str();
}
std::string profiles(const rs2::pipeline_profile& p) {
    std::ostringstream o; o << '['; bool first = true;
    bool gyro = false, accel = false;
    const auto streams = p.get_streams();
    for (const auto& s : streams) {
        const auto type = s.stream_type();
        if (s.format() != RS2_FORMAT_MOTION_XYZ32F ||
            !((type == RS2_STREAM_GYRO && s.fps() == 200 && !gyro) ||
              (type == RS2_STREAM_ACCEL && s.fps() == 100 && !accel)))
            throw std::runtime_error("resolved profile differs from IMU-only gyro/200 accel/100");
        gyro |= type == RS2_STREAM_GYRO; accel |= type == RS2_STREAM_ACCEL;
        if (!first) o << ',';
        first = false;
        o << "{\"stream\":" << quote(s.stream_name()) << ",\"index\":" << s.stream_index()
          << ",\"format\":\"MOTION_XYZ32F\",\"fps\":" << s.fps() << '}';
    }
    if (streams.size() != 2 || !gyro || !accel) throw std::runtime_error("expected exactly two motion streams");
    o << ']'; return o.str();
}
std::string options(rs2::device dev) {
    std::ostringstream o; o << '['; bool first = true;
    for (auto sensor : dev.query_sensors()) {
        if (!first) o << ',';
        first = false;
        o << "{\"sensor\":" << quote(sensor.get_info(RS2_CAMERA_INFO_NAME)) << ",\"global_time_enabled\":";
        if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) o << sensor.get_option(RS2_OPTION_GLOBAL_TIME_ENABLED);
        else o << "null";
        o << '}';
    }
    o << ']'; return o.str();
}
struct Sample {
    int sensor; double stamp; rs2_timestamp_domain domain; unsigned long long number;
    long long host; rs2_vector xyz;
};
struct Stats {
    unsigned long long count = 0, frame_gaps = 0, frame_reversals = 0;
    unsigned long long backwards = 0, duplicates = 0, domain_changes = 0;
    double last_stamp = 0, max_gap_ms = 0;
    unsigned long long last_number = 0;
    std::string domain = "UNKNOWN";
    void add(const Sample& s) {
        const std::string d = rs2_timestamp_domain_to_string(s.domain);
        if (count) {
            if (d != domain) {
                ++domain_changes;
            } else {
                const double dt = s.stamp - last_stamp;
                if (dt < 0) ++backwards;
                if (dt == 0) ++duplicates;
                max_gap_ms = std::max(max_gap_ms, dt);
            }
            if (s.number > last_number && s.number - last_number > 1) frame_gaps += s.number - last_number - 1;
            if (s.number <= last_number) ++frame_reversals;
        }
        ++count; last_stamp = s.stamp; last_number = s.number; domain = d;
    }
    std::string json() const {
        std::ostringstream o; o << std::setprecision(17)
          << "{\"sample_count\":" << count << ",\"max_gap_ms\":" << max_gap_ms
          << ",\"domain_changes\":" << domain_changes << ",\"backward_stamps\":" << backwards
          << ",\"duplicate_stamps\":" << duplicates << ",\"frame_number_gaps\":" << frame_gaps
          << ",\"frame_number_nonincreasing\":" << frame_reversals << ",\"last_domain\":" << quote(domain) << '}';
        return o.str();
    }
};
class Writer {
    FILE* files[2]{};
    std::mutex mutex;
    std::condition_variable ready;
    std::deque<Sample> queue;
    bool finished = false;
    std::string failure;
    std::thread runner;
    Stats stats[2];
    unsigned long long received[2]{}, overflow = 0;
    std::size_t high_water = 0;
    static constexpr std::size_t capacity = 65536; // >218 s at 300 Hz; bounded, no silent overflow.
    void run() {
        auto flushed = Clock::now();
        try {
            while (true) {
                std::deque<Sample> batch;
                bool done;
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    ready.wait_for(lock, std::chrono::milliseconds(200), [&] { return finished || !queue.empty(); });
                    batch.swap(queue); done = finished;
                }
                for (const auto& s : batch) {
                    if (std::fprintf(files[s.sensor], "%.17g,%s,%llu,%lld,%.17g,%.17g,%.17g\n",
                        s.stamp, rs2_timestamp_domain_to_string(s.domain), s.number, s.host,
                        double(s.xyz.x), double(s.xyz.y), double(s.xyz.z)) < 0)
                        throw std::runtime_error("CSV write failed (disk full or I/O error)");
                    { std::lock_guard<std::mutex> lock(mutex); stats[s.sensor].add(s); }
                    if (Clock::now() - flushed >= std::chrono::seconds(1)) {
                        flush(); flushed = Clock::now();
                    }
                }
                if (Clock::now() - flushed >= std::chrono::seconds(1) || done) {
                    flush(); flushed = Clock::now();
                }
                if (done) break;
            }
        } catch (const std::exception& e) { fail(e.what()); }
    }
    void flush() {
        for (auto f : files) if (std::fflush(f) != 0) throw std::runtime_error("CSV flush failed");
    }
public:
    explicit Writer(const std::filesystem::path& dir) {
        for (int i = 0; i < 2; ++i) {
            files[i] = std::fopen((dir / (i == 0 ? "gyro.csv" : "accel.csv")).c_str(), "wx");
            if (!files[i]) {
                for (auto f : files) if (f) std::fclose(f);
                throw std::runtime_error("cannot exclusively create CSV files");
            }
            std::fputs("sdk_ts_ms,domain,frame_number,host_steady_ns,x,y,z\n", files[i]);
        }
        runner = std::thread([this] { run(); });
    }
    ~Writer() { close(); for (auto f : files) if (f) std::fclose(f); }
    void fail(const std::string& s) { std::lock_guard<std::mutex> lock(mutex); if (failure.empty()) failure = s; }
    std::string error() { std::lock_guard<std::mutex> lock(mutex); return failure; }
    void push(const rs2::frame& f, long long host) {
        const auto type = f.get_profile().stream_type();
        if (type != RS2_STREAM_GYRO && type != RS2_STREAM_ACCEL) throw std::runtime_error("unexpected non-IMU frame");
        auto motion = f.as<rs2::motion_frame>();
        if (!motion) throw std::runtime_error("expected motion frame");
        Sample s{type == RS2_STREAM_GYRO ? 0 : 1, f.get_timestamp(), f.get_frame_timestamp_domain(),
                 f.get_frame_number(), host, motion.get_motion_data()};
        if (!std::isfinite(s.stamp) || !std::isfinite(s.xyz.x) || !std::isfinite(s.xyz.y) || !std::isfinite(s.xyz.z))
            throw std::runtime_error("nonfinite motion sample");
        std::lock_guard<std::mutex> lock(mutex);
        ++received[s.sensor];
        if (!failure.empty()) return;
        if (queue.size() >= capacity) { ++overflow; failure = "sample queue overflow; run incomplete"; return; }
        queue.push_back(s); high_water = std::max(high_water, queue.size()); ready.notify_one();
    }
    void close() {
        { std::lock_guard<std::mutex> lock(mutex); finished = true; ready.notify_one(); }
        if (runner.joinable()) runner.join();
    }
    void status(double elapsed) {
        std::lock_guard<std::mutex> lock(mutex);
        std::cout << "elapsed_s=" << elapsed << " gyro=" << stats[0].count << " rate=" << stats[0].count / elapsed
                  << " domain=" << stats[0].domain << " accel=" << stats[1].count << " rate=" << stats[1].count / elapsed
                  << " domain=" << stats[1].domain << std::endl;
    }
    std::string json() {
        std::lock_guard<std::mutex> lock(mutex);
        return "\"gyro\":" + stats[0].json() + ",\"accel\":" + stats[1].json() +
            ",\"callback_samples\":[" + std::to_string(received[0]) + ',' + std::to_string(received[1]) +
            "],\"queue_high_water\":" + std::to_string(high_water) + ",\"queue_overflow\":" + std::to_string(overflow);
    }
    bool missing() { std::lock_guard<std::mutex> lock(mutex); return !stats[0].count || !stats[1].count; }
};
} // namespace

#ifndef ALLAN_RECORDER_TEST
int main(int argc, char** argv) {
    if (argc != 3 && argc != 5) {
        std::cerr << "Usage: d435i_imu_record OUTPUT_DIR HOURS (0.01..30) --serial SERIAL|ANY\n"; return 2;
    }
    try {
        std::size_t used = 0;
        const double hours = std::stod(argv[2], &used);
        if (used != std::string(argv[2]).size() || !std::isfinite(hours) || hours < 0.01 || hours > 30)
            throw std::runtime_error("hours must be finite and in 0.01..30");
        if (argc != 5 || std::string(argv[3]) != "--serial")
            throw std::runtime_error("supply --serial with the intended camera serial or ANY");
        const std::string serial = argv[4];
        const std::filesystem::path dir(argv[1]);
        if (std::filesystem::exists(dir)) {
            if (!std::filesystem::is_directory(dir) || !std::filesystem::is_empty(dir))
                throw std::runtime_error("output must be a new or empty directory");
        } else if (!std::filesystem::create_directory(dir)) throw std::runtime_error("cannot create output directory");
        std::signal(SIGINT, on_signal); std::signal(SIGTERM, on_signal);
        Writer writer(dir);
        std::string sdk_serial = "UNKNOWN", firmware = "UNKNOWN", resolved = "[]", before = "[]", after = "[]";
        std::string start = utc(), stop, reason = "duration", error;
        const auto begin = Clock::now();
        rs2::pipeline pipe; bool started = false;
        try {
            rs2::log_to_file(RS2_LOG_SEVERITY_INFO, (dir / "sdk.log").c_str());
            rs2::config cfg; cfg.disable_all_streams();
            cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);
            cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 100);
            if (serial != "ANY") cfg.enable_device(serial);
            const auto profile = cfg.resolve(pipe);
            auto dev = profile.get_device();
            sdk_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            firmware = dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
            if (serial != "ANY" && sdk_serial != serial) throw std::runtime_error("wrong SDK device serial");
            cfg.enable_device(sdk_serial); // Pin the resolved unit even when ANY was requested.
            resolved = profiles(profile); before = options(dev);
            // Preserve global time, motion correction and firmware/calibration. No images or SDK recorder.
            start = utc();
            auto callback = [&](rs2::frame frame) {
                const auto host = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
                try {
                    if (auto fs = frame.as<rs2::frameset>()) { for (const auto& f : fs) writer.push(f, host); }
                    else writer.push(frame, host);
                } catch (const std::exception& e) { writer.fail(e.what()); }
            };
            const auto actual = pipe.start(cfg, callback); started = true;
            if (std::string(actual.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) != sdk_serial)
                throw std::runtime_error("started SDK device differs from resolved unit");
            resolved = profiles(actual);
            const auto recording_begin = Clock::now();
            auto next_status = recording_begin + std::chrono::seconds(60);
            while (!interrupted && Clock::now() - recording_begin < std::chrono::duration<double>(hours * 3600)) {
                if (!writer.error().empty()) throw std::runtime_error(writer.error());
                if (Clock::now() >= next_status) {
                    writer.status(std::chrono::duration<double>(Clock::now() - recording_begin).count());
                    next_status += std::chrono::seconds(60);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (interrupted) reason = interrupted == SIGINT ? "SIGINT" : "SIGTERM";
            pipe.stop(); started = false; after = options(dev);
        } catch (const std::exception& e) {
            error = e.what(); reason = "error";
            if (started) { try { pipe.stop(); } catch (const std::exception& stop_error) { error += "; stop: " + std::string(stop_error.what()); } }
        }
        stop = utc(); writer.close();
        if (!writer.error().empty()) { error += (error.empty() ? "" : "; ") + writer.error(); reason = "error"; }
        if (writer.missing() && error.empty()) { error = "missing motion stream"; reason = "error"; }
        std::ofstream meta(dir / "meta.json");
        meta << std::setprecision(17) << "{\"status\":\"OPEN\",\"frame\":\"imu_camera\",\"sensor\":\"D435i imu_sensor\","
             << "\"sdk_device_serial\":" << quote(sdk_serial) << ",\"serial_check_bypassed\":" << (serial == "ANY" ? "true" : "false")
             << ",\"asic_serial\":null,\"firmware_update_id\":null,\"usb_descriptor_serial\":null,\"firmware\":" << quote(firmware)
             << ",\"sdk_version\":" << quote(RS2_API_VERSION_STR)
             << ",\"profiles_requested\":[{\"stream\":\"Gyro\",\"format\":\"MOTION_XYZ32F\",\"fps\":200},"
                "{\"stream\":\"Accel\",\"format\":\"MOTION_XYZ32F\",\"fps\":100}],\"profiles_resolved\":" << resolved
             << ",\"global_time_before\":" << before << ",\"global_time_after\":" << after
             << ",\"temperature\":{\"imu\":null,\"note\":\"UNKNOWN; wrapper logs Pi SoC only\"}"
             << ",\"start_utc\":" << quote(start) << ",\"stop_utc\":" << quote(stop) << ",\"requested_hours\":" << hours
             << ",\"stop_reason\":" << quote(reason) << ",\"error\":" << quote(error) << ',' << writer.json() << "}\n";
        meta.flush(); if (!meta) throw std::runtime_error("metadata write failed");
        writer.status(std::max(0.001, std::chrono::duration<double>(Clock::now() - begin).count()));
        if (!error.empty()) throw std::runtime_error(error);
        std::cout << "OPEN: clean recorder stop; physical timing/noise/power qualification pending\n"; return 0;
    } catch (const std::exception& e) { std::cerr << "OPEN: " << e.what() << '\n'; return 1; }
}
#endif
