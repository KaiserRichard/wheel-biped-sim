#pragma once
// ORB-free implementation of the optional binding's float motion boundary.
#include "adapter.h"
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace wbr::vio {
inline std::array<float,6> orb_motion(const Combined& m) {
    const std::array<double,6> input{m.accel.x,m.accel.y,m.accel.z,
                                     m.gyro.x,m.gyro.y,m.gyro.z};
    std::array<float,6> output{};
    for (size_t i=0;i<input.size();++i) {
        const double v=input[i];
        // Check before casting: out-of-range floating conversion is not valid
        // boundary data. Preserve nonzero subnormals but reject loss to zero.
        if (!std::isfinite(v) || std::abs(v)>std::numeric_limits<float>::max())
            throw std::range_error("ORB IMU motion exceeds finite float range");
        output[i]=static_cast<float>(v);
        if (v!=0 && output[i]==0)
            throw std::range_error("ORB IMU motion underflows float storage");
    }
    return output;
}
}
