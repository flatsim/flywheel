#pragma once

#include "../math/math.h"
#include "../utils/format.h"

namespace flywheel
{

struct AABB
{
    AABB() = default;
    AABB(const Vec2& min, const Vec2& max);

    Vec2 GetCenter() const;
    Vec2 GetExtents() const;

    float GetArea() const;
    float GetPerimeter() const;

    bool Contains(const AABB& aabb) const;
    bool TestPoint(const Vec2& point) const;
    bool TestOverlap(const AABB& other) const;
    bool TestRay(const Vec2& from, const Vec2& to, float tMin, float tMax, Vec2 margin = Vec2::zero) const;
    float RayCast(const Vec2& from, const Vec2& to, float tMin, float tMax, Vec2 margin = Vec2::zero) const;

    std::string ToString() const;

    Vec2 min;
    Vec2 max;

    static AABB Union(const AABB& b, const Vec2& p);
    static AABB Union(const AABB& b1, const AABB& b2);
    static AABB Intersection(const AABB& b1, const AABB& b2);
};

inline AABB::AABB(const Vec2& min, const Vec2& max)
    : min{ min }
    , max{ max }
{
}

inline Vec2 AABB::GetCenter() const
{
    return (min + max) * 0.5f;
}

inline Vec2 AABB::GetExtents() const
{
    return (max - min);
}

inline float AABB::GetArea() const
{
    return (max.x - min.x) * (max.y - min.y);
}

inline float AABB::GetPerimeter() const
{
    Vec2 d = max - min;
    return 2.0f * (d.x + d.y);
}

inline bool AABB::Contains(const AABB& aabb) const
{
    return min.x <= aabb.min.x && min.y <= aabb.min.y && max.x >= aabb.max.x && max.y >= aabb.max.y;
}

inline bool AABB::TestPoint(const Vec2& point) const
{
    if (min.x > point.x || max.x < point.x) return false;
    if (min.y > point.y || max.y < point.y) return false;

    return true;
}

inline bool AABB::TestOverlap(const AABB& other) const
{
    if (min.x > other.max.x || max.x < other.min.x) return false;
    if (min.y > other.max.y || max.y < other.min.y) return false;

    return true;
}

// Slab method
inline bool AABB::TestRay(const Vec2& from, const Vec2& to, float tMin, float tMax, Vec2 margin) const
{
    Vec2 dir = to - from;

    for (int32 axis = 0; axis < 2; ++axis)
    {
        float invD = 1.0f / dir[axis];
        float origin = from[axis];

        float t0 = (min[axis] - margin[axis] - origin) * invD;
        float t1 = (max[axis] + margin[axis] - origin) * invD;

        if (invD < 0.0)
        {
            std::swap(t0, t1);
        }

        tMin = t0 > tMin ? t0 : tMin;
        tMax = t1 < tMax ? t1 : tMax;

        if (tMax <= tMin)
        {
            return false;
        }
    }

    return true;
}

inline float AABB::RayCast(const Vec2& from, const Vec2& to, float tMin, float tMax, Vec2 margin) const
{
    Vec2 dir = to - from;

    for (int32 axis = 0; axis < 2; ++axis)
    {
        float invD = 1.0f / dir[axis];
        float origin = from[axis];

        float t0 = (min[axis] - margin[axis] - origin) * invD;
        float t1 = (max[axis] + margin[axis] - origin) * invD;

        if (invD < 0.0)
        {
            std::swap(t0, t1);
        }

        tMin = t0 > tMin ? t0 : tMin;
        tMax = t1 < tMax ? t1 : tMax;

        if (tMax <= tMin)
        {
            return max_value;
        }
    }

    return tMin;
}

inline std::string AABB::ToString() const
{
    return FormatString("min: %s\nmax: %s", min.ToString().c_str(), max.ToString().c_str());
}

inline AABB AABB::Union(const AABB& b, const Vec2& p)
{
    Vec2 min = Min(b.min, p);
    Vec2 max = Max(b.max, p);

    return AABB{ min, max };
}

inline AABB AABB::Union(const AABB& b1, const AABB& b2)
{
    Vec2 min = Min(b1.min, b2.min);
    Vec2 max = Max(b1.max, b2.max);

    return AABB{ min, max };
}

inline AABB AABB::Intersection(const AABB& b1, const AABB& b2)
{
    Vec2 min = Max(b1.min, b2.min);
    Vec2 max = Min(b1.max, b2.max);

    return AABB{ min, max };
}

} // namespace flywheel
