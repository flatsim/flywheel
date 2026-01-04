#include "test_helpers.h"
#include <flywheel/flywheel.hpp>

using namespace flywheel;
using namespace flywheel::test;

TEST_SUITE("Math - Vec2") {
    TEST_CASE("Vec2 construction") {
        Vec2 v1;
        Vec2 v2(5.0f);
        Vec2 v3(3.0f, 4.0f);
        
        CHECK(v2.x == 5.0f);
        CHECK(v2.y == 5.0f);
        CHECK(v3.x == 3.0f);
        CHECK(v3.y == 4.0f);
    }
    
    TEST_CASE("Vec2 zero") {
        Vec2 v = Vec2::zero;
        CHECK(v.x == 0.0f);
        CHECK(v.y == 0.0f);
        
        Vec2 v2(5.0f, 10.0f);
        v2.SetZero();
        CHECK_VEC2_ZERO(v2);
    }
    
    TEST_CASE("Vec2 set operations") {
        Vec2 v;
        v.Set(7.0f);
        CHECK(v.x == 7.0f);
        CHECK(v.y == 7.0f);
        
        v.Set(2.0f, 3.0f);
        CHECK(v.x == 2.0f);
        CHECK(v.y == 3.0f);
    }
    
    TEST_CASE("Vec2 indexing") {
        Vec2 v(3.0f, 4.0f);
        CHECK(v[0] == 3.0f);
        CHECK(v[1] == 4.0f);
        
        v[0] = 10.0f;
        v[1] = 20.0f;
        CHECK(v.x == 10.0f);
        CHECK(v.y == 20.0f);
    }
    
    TEST_CASE("Vec2 negation") {
        Vec2 v(3.0f, -4.0f);
        Vec2 neg = -v;
        CHECK(neg.x == -3.0f);
        CHECK(neg.y == 4.0f);
    }
    
    TEST_CASE("Vec2 addition") {
        Vec2 a(1.0f, 2.0f);
        Vec2 b(3.0f, 4.0f);
        Vec2 c = a + b;
        
        CHECK(c.x == 4.0f);
        CHECK(c.y == 6.0f);
        
        a += b;
        CHECK(a.x == 4.0f);
        CHECK(a.y == 6.0f);
        
        Vec2 d(5.0f, 6.0f);
        d += 2.0f;
        CHECK(d.x == 7.0f);
        CHECK(d.y == 8.0f);
    }
    
    TEST_CASE("Vec2 subtraction") {
        Vec2 a(5.0f, 7.0f);
        Vec2 b(2.0f, 3.0f);
        Vec2 c = a - b;
        
        CHECK(c.x == 3.0f);
        CHECK(c.y == 4.0f);
        
        a -= b;
        CHECK(a.x == 3.0f);
        CHECK(a.y == 4.0f);
        
        Vec2 d(10.0f, 12.0f);
        d -= 2.0f;
        CHECK(d.x == 8.0f);
        CHECK(d.y == 10.0f);
    }
    
    TEST_CASE("Vec2 scalar multiplication") {
        Vec2 v(3.0f, 4.0f);
        Vec2 scaled = v * 2.0f;
        
        CHECK(scaled.x == 6.0f);
        CHECK(scaled.y == 8.0f);
        
        Vec2 scaled2 = 3.0f * v;
        CHECK(scaled2.x == 9.0f);
        CHECK(scaled2.y == 12.0f);
        
        v *= 2.0f;
        CHECK(v.x == 6.0f);
        CHECK(v.y == 8.0f);
    }
    
    TEST_CASE("Vec2 scalar division") {
        Vec2 v(10.0f, 20.0f);
        Vec2 divided = v / 2.0f;
        
        CHECK(divided.x == 5.0f);
        CHECK(divided.y == 10.0f);
        
        v /= 2.0f;
        CHECK(v.x == 5.0f);
        CHECK(v.y == 10.0f);
    }
    
    TEST_CASE("Vec2 length") {
        Vec2 v(3.0f, 4.0f);
        CHECK_FLOAT_APPROX(v.Length(), 5.0f);
        CHECK_FLOAT_APPROX(v.Length2(), 25.0f);
        CHECK_FLOAT_APPROX(Length(v), 5.0f);
        CHECK_FLOAT_APPROX(Length2(v), 25.0f);
    }
    
    TEST_CASE("Vec2 normalize") {
        Vec2 v(3.0f, 4.0f);
        float len = v.Normalize();
        
        CHECK_FLOAT_APPROX(len, 5.0f);
        CHECK_VEC2_NORMALIZED(v);
        CHECK_FLOAT_APPROX(v.x, 0.6f);
        CHECK_FLOAT_APPROX(v.y, 0.8f);
    }
    
    TEST_CASE("Vec2 normalize safe") {
        Vec2 zero(0.0f, 0.0f);
        float len = zero.NormalizeSafe();
        CHECK(len == 0.0f);
        CHECK_VEC2_ZERO(zero);
        
        Vec2 v(3.0f, 4.0f);
        len = v.NormalizeSafe();
        CHECK_FLOAT_APPROX(len, 5.0f);
        CHECK_VEC2_NORMALIZED(v);
    }
    
    TEST_CASE("Vec2 skew") {
        Vec2 v(3.0f, 4.0f);
        Vec2 skew = v.Skew();
        CHECK(skew.x == -4.0f);
        CHECK(skew.y == 3.0f);
    }
    
    TEST_CASE("Vec2 dot product") {
        Vec2 a(2.0f, 3.0f);
        Vec2 b(4.0f, 5.0f);
        float dot = Dot(a, b);
        CHECK_FLOAT_APPROX(dot, 23.0f);
        
        // Perpendicular vectors
        Vec2 c(1.0f, 0.0f);
        Vec2 d(0.0f, 1.0f);
        CHECK_FLOAT_APPROX(Dot(c, d), 0.0f);
    }
    
    TEST_CASE("Vec2 cross product") {
        Vec2 a(2.0f, 3.0f);
        Vec2 b(4.0f, 5.0f);
        float cross = Cross(a, b);
        CHECK_FLOAT_APPROX(cross, -2.0f);
        
        // Scalar cross
        Vec2 c = Cross(2.0f, Vec2(3.0f, 4.0f));
        CHECK(c.x == -8.0f);
        CHECK(c.y == 6.0f);
        
        Vec2 d = Cross(Vec2(3.0f, 4.0f), 2.0f);
        CHECK(d.x == 8.0f);
        CHECK(d.y == -6.0f);
    }
    
    TEST_CASE("Vec2 distance") {
        Vec2 a(0.0f, 0.0f);
        Vec2 b(3.0f, 4.0f);
        CHECK_FLOAT_APPROX(Dist(a, b), 5.0f);
        CHECK_FLOAT_APPROX(Dist2(a, b), 25.0f);
    }
    
    TEST_CASE("Vec2 equality") {
        Vec2 a(1.0f, 2.0f);
        Vec2 b(1.0f, 2.0f);
        Vec2 c(1.0f, 3.0f);
        
        CHECK(a == b);
        CHECK(a != c);
    }
}

TEST_SUITE("Math - Vec3") {
    TEST_CASE("Vec3 construction") {
        Vec3 v1;
        Vec3 v2(5.0f);
        Vec3 v3(1.0f, 2.0f, 3.0f);
        Vec3 v4(Vec2(4.0f, 5.0f));
        
        CHECK(v2.x == 5.0f);
        CHECK(v2.y == 5.0f);
        CHECK(v2.z == 5.0f);
        CHECK(v3.x == 1.0f);
        CHECK(v3.y == 2.0f);
        CHECK(v3.z == 3.0f);
        CHECK(v4.x == 4.0f);
        CHECK(v4.y == 5.0f);
        CHECK(v4.z == 0.0f);
    }
    
    TEST_CASE("Vec3 operations") {
        Vec3 a(1.0f, 2.0f, 3.0f);
        Vec3 b(4.0f, 5.0f, 6.0f);
        
        Vec3 sum = a + b;
        CHECK(sum.x == 5.0f);
        CHECK(sum.y == 7.0f);
        CHECK(sum.z == 9.0f);
        
        Vec3 diff = b - a;
        CHECK(diff.x == 3.0f);
        CHECK(diff.y == 3.0f);
        CHECK(diff.z == 3.0f);
        
        Vec3 scaled = a * 2.0f;
        CHECK(scaled.x == 2.0f);
        CHECK(scaled.y == 4.0f);
        CHECK(scaled.z == 6.0f);
    }
    
    TEST_CASE("Vec3 dot product") {
        Vec3 a(1.0f, 2.0f, 3.0f);
        Vec3 b(4.0f, 5.0f, 6.0f);
        float dot = Dot(a, b);
        CHECK_FLOAT_APPROX(dot, 32.0f);
    }
    
    TEST_CASE("Vec3 cross product") {
        Vec3 a(1.0f, 0.0f, 0.0f);
        Vec3 b(0.0f, 1.0f, 0.0f);
        Vec3 c = Cross(a, b);
        
        CHECK_FLOAT_APPROX(c.x, 0.0f);
        CHECK_FLOAT_APPROX(c.y, 0.0f);
        CHECK_FLOAT_APPROX(c.z, 1.0f);
    }
    
    TEST_CASE("Vec3 length and normalize") {
        Vec3 v(2.0f, 3.0f, 6.0f);
        CHECK_FLOAT_APPROX(v.Length(), 7.0f);
        CHECK_FLOAT_APPROX(v.Length2(), 49.0f);
        
        float len = v.Normalize();
        CHECK_FLOAT_APPROX(len, 7.0f);
        CHECK_FLOAT_APPROX(v.Length(), 1.0f);
    }
}

TEST_SUITE("Math - Mat2") {
    TEST_CASE("Mat2 construction") {
        Mat2 m1(identity);
        CHECK(m1.ex.x == 1.0f);
        CHECK(m1.ex.y == 0.0f);
        CHECK(m1.ey.x == 0.0f);
        CHECK(m1.ey.y == 1.0f);
        
        Mat2 m2(2.0f);
        CHECK(m2.ex.x == 2.0f);
        CHECK(m2.ey.y == 2.0f);
        
        Mat2 m3(Vec2(1.0f, 2.0f), Vec2(3.0f, 4.0f));
        CHECK(m3.ex.x == 1.0f);
        CHECK(m3.ex.y == 2.0f);
        CHECK(m3.ey.x == 3.0f);
        CHECK(m3.ey.y == 4.0f);
    }
    
    TEST_CASE("Mat2 identity and zero") {
        Mat2 m;
        m.SetIdentity();
        CHECK(m.ex.x == 1.0f);
        CHECK(m.ey.y == 1.0f);
        CHECK(m.ex.y == 0.0f);
        CHECK(m.ey.x == 0.0f);
        
        m.SetZero();
        CHECK(m.ex.x == 0.0f);
        CHECK(m.ex.y == 0.0f);
        CHECK(m.ey.x == 0.0f);
        CHECK(m.ey.y == 0.0f);
    }
    
    TEST_CASE("Mat2 transpose") {
        Mat2 m(Vec2(1.0f, 2.0f), Vec2(3.0f, 4.0f));
        Mat2 t = m.GetTranspose();
        
        CHECK(t.ex.x == 1.0f);
        CHECK(t.ex.y == 3.0f);
        CHECK(t.ey.x == 2.0f);
        CHECK(t.ey.y == 4.0f);
    }
    
    TEST_CASE("Mat2 determinant") {
        Mat2 m(Vec2(1.0f, 2.0f), Vec2(3.0f, 4.0f));
        float det = m.GetDeterminant();
        CHECK_FLOAT_APPROX(det, -2.0f);
    }
    
    TEST_CASE("Mat2 inverse") {
        Mat2 m(Vec2(1.0f, 2.0f), Vec2(3.0f, 4.0f));
        Mat2 inv = m.GetInverse();
        
        // m * inv should be identity
        Mat2 result = m * inv;
        CHECK_FLOAT_APPROX(result.ex.x, 1.0f);
        CHECK_FLOAT_APPROX(result.ey.y, 1.0f);
        CHECK_FLOAT_APPROX(result.ex.y, 0.0f);
        CHECK_FLOAT_APPROX(result.ey.x, 0.0f);
    }
    
    TEST_CASE("Mat2 vector multiplication") {
        Mat2 m(Vec2(2.0f, 0.0f), Vec2(0.0f, 3.0f));
        Vec2 v(4.0f, 5.0f);
        Vec2 result = m * v;
        
        CHECK_FLOAT_APPROX(result.x, 8.0f);
        CHECK_FLOAT_APPROX(result.y, 15.0f);
    }
    
    TEST_CASE("Mat2 matrix multiplication") {
        // Column-major: ex is first column, ey is second column
        Mat2 a(Vec2(1.0f, 2.0f), Vec2(3.0f, 4.0f));
        Mat2 b(Vec2(5.0f, 6.0f), Vec2(7.0f, 8.0f));
        Mat2 c = a * b;
        
        // Result should be a * b in column-major form
        CHECK_FLOAT_APPROX(c.ex.x, 23.0f);
        CHECK_FLOAT_APPROX(c.ex.y, 34.0f);
        CHECK_FLOAT_APPROX(c.ey.x, 31.0f);
        CHECK_FLOAT_APPROX(c.ey.y, 46.0f);
    }
}

TEST_SUITE("Math - Rotation") {
    TEST_CASE("Rotation construction") {
        Rotation r1(identity);
        CHECK(r1.s == 0.0f);
        CHECK(r1.c == 1.0f);
        
        Rotation r2(pi / 2.0f);
        CHECK_FLOAT_APPROX(r2.s, 1.0f);
        CHECK_FLOAT_APPROX(r2.c, 0.0f);
    }
    
    TEST_CASE("Rotation get angle") {
        Rotation r(pi / 4.0f);
        float angle = r.GetAngle();
        CHECK_FLOAT_APPROX(angle, pi / 4.0f);
    }
    
    TEST_CASE("Rotation vector multiplication") {
        Rotation r(pi / 2.0f);
        Vec2 v(1.0f, 0.0f);
        Vec2 rotated = Mul(r, v);
        
        CHECK_FLOAT_APPROX(rotated.x, 0.0f);
        CHECK_FLOAT_APPROX(rotated.y, 1.0f);
    }
    
    TEST_CASE("Rotation inverse") {
        Rotation r(pi / 3.0f);
        Vec2 v(1.0f, 2.0f);
        Vec2 rotated = Mul(r, v);
        Vec2 back = MulT(r, rotated);
        
        CHECK_VEC2_APPROX(back, v);
    }
}

TEST_SUITE("Math - Transform") {
    TEST_CASE("Transform construction") {
        Transform t1(identity);
        CHECK_VEC2_ZERO(t1.position);
        CHECK(t1.rotation.s == 0.0f);
        CHECK(t1.rotation.c == 1.0f);
        
        Transform t2(Vec2(1.0f, 2.0f));
        CHECK(t2.position.x == 1.0f);
        CHECK(t2.position.y == 2.0f);
        
        Transform t3(Vec2(3.0f, 4.0f), pi / 2.0f);
        CHECK(t3.position.x == 3.0f);
        CHECK(t3.position.y == 4.0f);
        CHECK_FLOAT_APPROX(t3.rotation.GetAngle(), pi / 2.0f);
    }
    
    TEST_CASE("Transform vector multiplication") {
        Transform t(Vec2(10.0f, 20.0f), 0.0f);
        Vec2 v(1.0f, 2.0f);
        Vec2 result = Mul(t, v);
        
        CHECK_FLOAT_APPROX(result.x, 11.0f);
        CHECK_FLOAT_APPROX(result.y, 22.0f);
    }
    
    TEST_CASE("Transform with rotation") {
        Transform t(Vec2(0.0f, 0.0f), pi / 2.0f);
        Vec2 v(1.0f, 0.0f);
        Vec2 result = Mul(t, v);
        
        CHECK_FLOAT_APPROX(result.x, 0.0f);
        CHECK_FLOAT_APPROX(result.y, 1.0f);
    }
    
    TEST_CASE("Transform inverse") {
        Transform t(Vec2(5.0f, 10.0f), pi / 4.0f);
        Vec2 v(1.0f, 2.0f);
        Vec2 transformed = Mul(t, v);
        Vec2 back = MulT(t, transformed);
        
        CHECK_VEC2_APPROX(back, v);
    }
    
    TEST_CASE("Transform composition") {
        Transform t1(Vec2(1.0f, 2.0f), 0.0f);
        Transform t2(Vec2(3.0f, 4.0f), 0.0f);
        Transform t3 = Mul(t1, t2);
        
        CHECK_FLOAT_APPROX(t3.position.x, 4.0f);
        CHECK_FLOAT_APPROX(t3.position.y, 6.0f);
    }
}

TEST_SUITE("Math - Utility Functions") {
    TEST_CASE("Abs") {
        CHECK(Abs(-5.0f) == 5.0f);
        CHECK(Abs(3.0f) == 3.0f);
        
        Vec2 v(-1.0f, -2.0f);
        Vec2 absV = Abs(v);
        CHECK(absV.x == 1.0f);
        CHECK(absV.y == 2.0f);
    }
    
    TEST_CASE("Min and Max") {
        CHECK(Min(3.0f, 5.0f) == 3.0f);
        CHECK(Max(3.0f, 5.0f) == 5.0f);
        
        Vec2 a(1.0f, 5.0f);
        Vec2 b(3.0f, 2.0f);
        Vec2 minV = Min(a, b);
        Vec2 maxV = Max(a, b);
        
        CHECK(minV.x == 1.0f);
        CHECK(minV.y == 2.0f);
        CHECK(maxV.x == 3.0f);
        CHECK(maxV.y == 5.0f);
    }
    
    TEST_CASE("Clamp") {
        CHECK(Clamp(5.0f, 0.0f, 10.0f) == 5.0f);
        CHECK(Clamp(-5.0f, 0.0f, 10.0f) == 0.0f);
        CHECK(Clamp(15.0f, 0.0f, 10.0f) == 10.0f);
        
        Vec2 v(5.0f, 15.0f);
        Vec2 clamped = Clamp(v, Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        CHECK(clamped.x == 5.0f);
        CHECK(clamped.y == 10.0f);
    }
    
    TEST_CASE("Trigonometric functions") {
        CHECK_FLOAT_APPROX(Sin(0.0f), 0.0f);
        CHECK_FLOAT_APPROX(Cos(0.0f), 1.0f);
        CHECK_FLOAT_APPROX(Sin(pi / 2.0f), 1.0f);
        CHECK_FLOAT_APPROX(Cos(pi / 2.0f), 0.0f);
        CHECK_FLOAT_APPROX(Tan(pi / 4.0f), 1.0f);
    }
    
    TEST_CASE("Angle conversion") {
        float deg = 180.0f;
        float rad = DegToRad(deg);
        CHECK_FLOAT_APPROX(rad, pi);
        
        float backToDeg = RadToDeg(rad);
        CHECK_FLOAT_APPROX(backToDeg, 180.0f);
    }
    
    TEST_CASE("Lerp") {
        float a = 0.0f;
        float b = 10.0f;
        CHECK_FLOAT_APPROX(Lerp(a, b, 0.0f), 0.0f);
        CHECK_FLOAT_APPROX(Lerp(a, b, 0.5f), 5.0f);
        CHECK_FLOAT_APPROX(Lerp(a, b, 1.0f), 10.0f);
        
        Vec2 v1(0.0f, 0.0f);
        Vec2 v2(10.0f, 20.0f);
        Vec2 mid = Lerp(v1, v2, 0.5f);
        CHECK_FLOAT_APPROX(mid.x, 5.0f);
        CHECK_FLOAT_APPROX(mid.y, 10.0f);
    }
    
    TEST_CASE("AngleBetween") {
        Vec2 a(1.0f, 0.0f);
        Vec2 b(0.0f, 1.0f);
        float angle = AngleBetween(a, b);
        CHECK_FLOAT_APPROX(angle, pi / 2.0f);
    }
    
    TEST_CASE("PolarToCart") {
        Vec2 v = PolarToCart(0.0f, 1.0f);
        CHECK_FLOAT_APPROX(v.x, 1.0f);
        CHECK_FLOAT_APPROX(v.y, 0.0f);
        
        Vec2 v2 = PolarToCart(pi / 2.0f, 1.0f);
        CHECK_FLOAT_APPROX(v2.x, 0.0f);
        CHECK_FLOAT_APPROX(v2.y, 1.0f);
    }
    
    TEST_CASE("Reflect") {
        Vec2 v(1.0f, -1.0f);
        Vec2 n(0.0f, 1.0f);
        Vec2 reflected = Reflect(v, n);
        
        CHECK_FLOAT_APPROX(reflected.x, 1.0f);
        CHECK_FLOAT_APPROX(reflected.y, 1.0f);
    }
}

TEST_SUITE("Math - Edge Cases") {
    TEST_CASE("Zero vector operations") {
        Vec2 zero(0.0f, 0.0f);
        CHECK_FLOAT_APPROX(zero.Length(), 0.0f);
        CHECK_FLOAT_APPROX(zero.Length2(), 0.0f);
        
        float len = zero.NormalizeSafe();
        CHECK(len == 0.0f);
        CHECK_VEC2_ZERO(zero);
    }
    
    TEST_CASE("Very small vectors") {
        Vec2 tiny(1e-10f, 1e-10f);
        float len = tiny.NormalizeSafe();
        CHECK(len == 0.0f);
    }
    
    TEST_CASE("Large vectors") {
        Vec2 large(1e6f, 1e6f);
        float len = large.Normalize();
        CHECK(len > 0.0f);
        CHECK_VEC2_NORMALIZED(large);
    }
    
    TEST_CASE("Perpendicular vectors") {
        Vec2 a(1.0f, 0.0f);
        Vec2 b(0.0f, 1.0f);
        CHECK_FLOAT_APPROX(Dot(a, b), 0.0f);
    }
    
    TEST_CASE("Parallel vectors") {
        Vec2 a(2.0f, 3.0f);
        Vec2 b(4.0f, 6.0f);
        float cross = Cross(a, b);
        CHECK_FLOAT_APPROX(cross, 0.0f);
    }
}
