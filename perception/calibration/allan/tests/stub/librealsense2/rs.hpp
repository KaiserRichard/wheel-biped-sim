// SYNTHETIC, syntax-only API surface. No SDK ABI/API compatibility or hardware proof.
#pragma once
#include <stdexcept>
#include <vector>
#define RS2_API_VERSION_STR "STUB-NOT-LIBREALSENSE"
enum rs2_stream { RS2_STREAM_GYRO, RS2_STREAM_ACCEL };
enum rs2_format { RS2_FORMAT_MOTION_XYZ32F };
enum rs2_timestamp_domain { RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME,
                            RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME };
enum rs2_camera_info { RS2_CAMERA_INFO_NAME, RS2_CAMERA_INFO_SERIAL_NUMBER, RS2_CAMERA_INFO_FIRMWARE_VERSION };
enum rs2_option { RS2_OPTION_GLOBAL_TIME_ENABLED };
enum rs2_log_severity { RS2_LOG_SEVERITY_INFO };
struct rs2_vector { float x = 0, y = 0, z = 0; };
inline const char* rs2_timestamp_domain_to_string(rs2_timestamp_domain d) {
    switch (d) {
    case RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK: return "Hardware Clock";
    case RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME: return "System Time";
    case RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME: return "Global Time";
    }
    return "Unknown";
}
namespace rs2 {
class stream_profile {
public:
    rs2_stream stream_type() const { return RS2_STREAM_GYRO; }
    rs2_format format() const { return RS2_FORMAT_MOTION_XYZ32F; }
    const char* stream_name() const { return "Gyro"; }
    int stream_index() const { return 0; }
    int fps() const { return 200; }
};
class frame {
public:
    template<class T> T as() const { return T{}; }
    explicit operator bool() const { return true; }
    stream_profile get_profile() const { return {}; }
    double get_timestamp() const { return 0; }
    unsigned long long get_frame_number() const { return 0; }
    rs2_timestamp_domain get_frame_timestamp_domain() const { return RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK; }
};
class motion_frame : public frame {
public:
    rs2_vector get_motion_data() const { return {}; }
};
class frameset : public frame {
    std::vector<frame> frames;
public:
    auto begin() const { return frames.begin(); }
    auto end() const { return frames.end(); }
};
class sensor {
public:
    const char* get_info(rs2_camera_info) const { return "STUB"; }
    bool supports(rs2_option) const { return false; }
    float get_option(rs2_option) const { return 0; }
};
class device {
public:
    const char* get_info(rs2_camera_info) const { return "STUB"; }
    std::vector<sensor> query_sensors() const { return {}; }
};
class pipeline_profile {
public:
    device get_device() const { return {}; }
    std::vector<stream_profile> get_streams() const { return {}; }
};
class config;
class pipeline {
public:
    template<class Callback> pipeline_profile start(const config&, Callback) {
        throw std::runtime_error("syntax-only stub cannot stream");
    }
    void stop() {}
};
class config {
public:
    void disable_all_streams() {}
    void enable_stream(rs2_stream, rs2_format, int) {}
    void enable_device(const std::string&) {}
    pipeline_profile resolve(pipeline&) { throw std::runtime_error("syntax-only stub cannot resolve"); }
};
inline void log_to_file(rs2_log_severity, const char*) {}
}
