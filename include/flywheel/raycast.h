#pragma once

#include "collision.h"
#include "math.h"
#include "settings.h"
#include "simplex.h"

namespace flywheel
{

// Define a ray such that:
// Ray = from + maxFraction * (to - from) with search radius
struct RayCastInput
{
    Vec2 from;
    Vec2 to;
    float maxFraction;
    float radius;
};

struct RayCastOutput
{
    Vec2 normal;
    float fraction;
};

struct ShapeCastInput
{
    class Shape* shapeA;
    class Shape* shapeB;
    Transform* tfA;
    Transform* tfB;
    Vec2 translationA;
    Vec2 translationB;
};

struct ShapeCastOutput
{
    Vec2 point;
    Vec2 normal;
    float t; // time of impact
};

// clang-format off
// Raycast functions
bool RayCastCircle(
    const Vec2& center,
    float radius,
    const RayCastInput& input,
    RayCastOutput* output
);

bool RayCastLineSegment(
    const Vec2& vertex1,
    const Vec2& vertex2,
    const RayCastInput& input,
    RayCastOutput* output
);

bool RayCastCapsule(
    const Vec2& vertex1,
    const Vec2& vertex2,
    float radius,
    const RayCastInput& input,
    RayCastOutput* output
);

// GJK-raycast
// Algorithm by Gino van den Bergen.
bool ShapeCast(
    const Shape* a, const Transform& tfA,
    const Shape* b, const Transform& tfB,
    const Vec2& translationA, const Vec2& translationB,
    ShapeCastOutput* output
);

// clang-format on

struct AABBCastInput
{
    Vec2 from;
    Vec2 to;
    float maxFraction;
    Vec2 halfExtents;
};

// https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
inline bool RayCastCircle(const Vec2& p, float r, const RayCastInput& input, RayCastOutput* output)
{
    Vec2 d = input.to - input.from;
    Vec2 f = input.from - p;
    float radii = r + input.radius;

    float a = Dot(d, d);
    float b = 2.0f * Dot(f, d);
    float c = Dot(f, f) - radii * radii;

    // Quadratic equation discriminant
    float discriminant = b * b - 4.0f * a * c;

    if (discriminant < 0.0f)
    {
        return false;
    }

    discriminant = Sqrt(discriminant);

    float t = (-b - discriminant) / (2.0f * a);
    if (0.0f <= t && t <= input.maxFraction)
    {
        output->fraction = t;
        output->normal = Normalize(f + d * t);

        return true;
    }

    return false;
}

inline bool RayCastLineSegment(const Vec2& v1, const Vec2& v2, const RayCastInput& input, RayCastOutput* output)
{
    Vec2 d = input.to - input.from;
    Vec2 e = v2 - v1;
    Vec2 normal = Cross(e, 1.0f);

    float length = normal.NormalizeSafe();
    if (length == 0.0f)
    {
        return false;
    }

    float denominator = Dot(normal, d);

    // Parallel or collinear case
    if (denominator == 0.0f)
    {
        return false;
    }

    float numerator = Dot(normal, v1 - input.from);
    if (denominator < 0)
    {
        numerator += input.radius;
    }
    else
    {
        numerator -= input.radius;
    }

    float t = numerator / denominator;
    if (t < 0.0f || t > input.maxFraction)
    {
        return false;
    }

    // Point on the v1-v2 line
    Vec2 q = input.from + t * d;

    float u = Dot(q - v1, e);
    if (u < 0.0f || u > Dot(e, e))
    {
        return false;
    }

    output->fraction = t;
    if (denominator > 0.0f)
    {
        output->normal = -normal;
    }
    else
    {
        output->normal = normal;
    }

    return true;
}

inline bool RayCastCapsule(const Vec2& va, const Vec2& vb, float radius, const RayCastInput& input, RayCastOutput* output)
{
    Vec2 p1 = input.from;
    Vec2 p2 = input.to;

    Vec2 v1 = va;
    Vec2 v2 = vb;

    Vec2 d = p2 - p1;
    Vec2 e = v2 - v1;
    Vec2 n = Cross(1.0f, e);
    n.Normalize();

    Vec2 pv = p1 - v1;

    float radii = radius + input.radius;

    // Signed distance along normal
    float distance = Dot(pv, n);

    // Does the ray start within the capsule band?
    if (Abs(distance) <= radii)
    {
        float r = Dot(e, pv);

        // Raycast to va circle
        if (r < 0.0)
        {
            // Ray cast to va circle
            Vec2 f = p1 - va;

            float a = Dot(d, d);
            float b = 2.0f * Dot(f, d);
            float c = Dot(f, f) - radii * radii;

            // Quadratic equation discriminant
            float discriminant = b * b - 4.0f * a * c;

            if (discriminant < 0.0f)
            {
                return false;
            }

            discriminant = Sqrt(discriminant);

            float t = (-b - discriminant) / (2.0f * a);
            if (0.0f <= t && t <= input.maxFraction)
            {
                output->fraction = t;
                output->normal = Normalize(f + d * t);
                return true;
            }
            else
            {
                return false;
            }
        }

        // Raycast to vb circle
        if (r > Dot(e, e))
        {
            // Raycast to vb circle
            Vec2 f = p1 - vb;

            float a = Dot(d, d);
            float b = 2.0f * Dot(f, d);
            float c = Dot(f, f) - radii * radii;

            // Quadratic equation discriminant
            float discriminant = b * b - 4.0f * a * c;

            if (discriminant < 0.0f)
            {
                return false;
            }

            discriminant = Sqrt(discriminant);

            float t = (-b - discriminant) / (2.0f * a);
            if (0.0f <= t && t <= input.maxFraction)
            {
                output->fraction = t;
                output->normal = Normalize(f + d * t);
                return true;
            }
            else
            {
                return false;
            }
        }

        // Totally inside the capsule
        return false;
    }

    Vec2 rn = n * radii;

    // Translate edge along normal
    if (distance > 0.0f)
    {
        v1 += rn;
        v2 += rn;
    }
    else
    {
        v1 -= rn;
        v2 -= rn;
    }

    // Raycast to a line segment

    float denominator = Dot(n, d);

    // Parallel or collinear case
    if (denominator == 0.0f)
    {
        return false;
    }

    float numerator = Dot(n, v1 - p1);

    float t = numerator / denominator;
    if (t < 0.0f || input.maxFraction < t)
    {
        return false;
    }

    // Point on the v1-v2 line
    Vec2 q = p1 + t * d;

    float u = Dot(q - v1, e);
    if (u < 0.0f)
    {
        // Ray cast to va circle
        Vec2 f = p1 - va;

        float a = Dot(d, d);
        float b = 2.0f * Dot(f, d);
        float c = Dot(f, f) - radii * radii;

        // Quadratic equation discriminant
        float discriminant = b * b - 4.0f * a * c;

        if (discriminant < 0.0f)
        {
            return false;
        }

        discriminant = Sqrt(discriminant);

        t = (-b - discriminant) / (2.0f * a);
        if (0.0f <= t && t <= input.maxFraction)
        {
            output->fraction = t;
            output->normal = Normalize(f + d * t);
            return true;
        }
        else
        {
            return false;
        }
    }

    if (Dot(e, e) < u)
    {
        // Raycast to vb circle
        Vec2 f = p1 - vb;

        float a = Dot(d, d);
        float b = 2.0f * Dot(f, d);
        float c = Dot(f, f) - radii * radii;

        // Quadratic equation discriminant
        float discriminant = b * b - 4.0f * a * c;

        if (discriminant < 0.0f)
        {
            return false;
        }

        discriminant = Sqrt(discriminant);

        t = (-b - discriminant) / (2.0f * a);
        if (0.0f <= t && t <= input.maxFraction)
        {
            output->fraction = t;
            output->normal = Normalize(f + d * t);
            return true;
        }
        else
        {
            return false;
        }
    }

    // Inside the edge region
    output->fraction = t;
    if (numerator > 0.0f)
    {
        output->normal = -n;
    }
    else
    {
        output->normal = n;
    }

    return true;
}

} // namespace flywheel
