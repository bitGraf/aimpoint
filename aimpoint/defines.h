#pragma once
#include <cstdint>

#include <laml/laml.hpp>

#ifndef RH_TYPES
#define RH_TYPES

typedef std::uint8_t  uint8;
typedef std::uint16_t uint16;
typedef std::uint32_t uint32;
typedef std::uint64_t uint64;

typedef std::int8_t  int8;
typedef std::int16_t int16;
typedef std::int32_t int32;
typedef std::int64_t int64;

#endif

typedef laml::Vec3 vec3f;
typedef laml::Vec3_highp vec3d;
typedef laml::Vec4 vec4f;
typedef laml::Vec4_highp vec4d;

typedef laml::Mat3 mat3f;
typedef laml::Mat3_highp mat3d;
typedef laml::Mat4 mat4f;
typedef laml::Mat4_highp mat4d;

#define trig_tol (1e-9)

template<typename T, size_t num_points>
struct plot_signal {
    plot_signal() : length(num_points) {
        for (size_t n = 0; n < num_points; n++) {
            data[n] = static_cast<T>(0);
        }
    }

    void add_point(T value) {
        data[offset] = value;

        offset++;

        if (offset == num_points)
            offset = 0;
    }

    T get_avg() {
        T sum = 0;
        for (size_t n = 0; n < num_points; n++) {
            sum = sum + data[n];
        }
        return sum / static_cast<T>(num_points);
    }

    T data[num_points];

    size_t offset = 0;
    const size_t length;
};

// Disable by default
#ifndef USE_DTV
#define USE_DTV 0
#endif