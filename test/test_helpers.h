#pragma once

#include <doctest/doctest.h>
#include <flywheel/math.h>
#include <cmath>
#include <limits>

namespace flywheel {
namespace test {

// Floating point comparison epsilon
constexpr float EPSILON = 1e-5f;
constexpr float LOOSE_EPSILON = 1e-3f;

// Helper function for approximate float comparison
inline bool approx_equal(float a, float b, float epsilon = EPSILON) {
    return std::abs(a - b) < epsilon;
}

// Helper function for approximate double comparison
inline bool approx_equal(double a, double b, double epsilon = EPSILON) {
    return std::abs(a - b) < epsilon;
}

// Vector2 approximate equality
inline bool approx_equal(const Vec2& a, const Vec2& b, float epsilon = EPSILON) {
    return approx_equal(a.x, b.x, epsilon) && approx_equal(a.y, b.y, epsilon);
}

// Check if vector is approximately zero
inline bool is_zero(const Vec2& v, float epsilon = EPSILON) {
    return approx_equal(v.x, 0.0f, epsilon) && approx_equal(v.y, 0.0f, epsilon);
}

// Check if vector is normalized
inline bool is_normalized(const Vec2& v, float epsilon = EPSILON) {
    float length_sq = v.x * v.x + v.y * v.y;
    return approx_equal(length_sq, 1.0f, epsilon);
}

// Custom doctest assertion macros for vectors
#define CHECK_VEC2_APPROX(a, b) \
    do { \
        CHECK(flywheel::test::approx_equal((a).x, (b).x)); \
        CHECK(flywheel::test::approx_equal((a).y, (b).y)); \
    } while(0)

#define REQUIRE_VEC2_APPROX(a, b) \
    do { \
        REQUIRE(flywheel::test::approx_equal((a).x, (b).x)); \
        REQUIRE(flywheel::test::approx_equal((a).y, (b).y)); \
    } while(0)

#define CHECK_FLOAT_APPROX(a, b) \
    CHECK(flywheel::test::approx_equal((a), (b)))

#define REQUIRE_FLOAT_APPROX(a, b) \
    REQUIRE(flywheel::test::approx_equal((a), (b)))

#define CHECK_VEC2_ZERO(v) \
    CHECK(flywheel::test::is_zero(v))

#define CHECK_VEC2_NORMALIZED(v) \
    CHECK(flywheel::test::is_normalized(v))

// Helper to create test vectors
inline Vec2 make_vec2(float x, float y) {
    return Vec2{x, y};
}

// Helper to generate random float in range
inline float random_float(float min, float max) {
    return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

// Helper to generate random Vec2 in range
inline Vec2 random_vec2(float min, float max) {
    return make_vec2(random_float(min, max), random_float(min, max));
}

// Helper to generate random normalized Vec2
inline Vec2 random_unit_vec2() {
    float angle = random_float(0.0f, 2.0f * 3.14159265f);
    return make_vec2(std::cos(angle), std::sin(angle));
}

} // namespace test
} // namespace flywheel
