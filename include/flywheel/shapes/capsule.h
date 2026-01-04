#pragma once

#include "shape.h"

namespace flywheel
{

class Capsule : public Shape
{
public:
    Capsule(float length, float radius, bool horizontal = false, const Transform& tf = identity);
    Capsule(const Vec2& p1, const Vec2& p2, float radius, bool resetPosition = false, const Transform& tf = identity);

    Capsule(const Capsule& other, const Transform& tf);

    virtual void ComputeMass(float density, MassData* outMassData) const override;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const override;

    virtual int32 GetVertexCount() const override;
    virtual Vec2 GetVertex(int32 id) const override;
    virtual int32 GetSupport(const Vec2& localDir) const override;
    virtual Edge GetFeaturedEdge(const Transform& transform, const Vec2& dir) const override;

    virtual bool TestPoint(const Transform& transform, const Vec2& q) const override;
    virtual Vec2 GetClosestPoint(const Transform& transform, const Vec2& q) const override;
    virtual bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const override;

    float GetLength() const;
    const Vec2& GetVertexA() const;
    const Vec2& GetVertexB() const;

protected:
    virtual Shape* Clone(Allocator* allocator, const Transform& tf) const override;

private:
    Vec2 va, vb;
};

inline Capsule::Capsule(const Capsule& other, const Transform& tf)
    : Capsule(other.va, other.vb, other.radius, false, tf)
{
}

inline Shape* Capsule::Clone(Allocator* allocator, const Transform& tf) const
{
    void* mem = allocator->Allocate(sizeof(Capsule));
    Capsule* shape = new (mem) Capsule(*this, tf);
    return shape;
}

inline Edge Capsule::GetFeaturedEdge(const Transform& transform, const Vec2& dir) const
{
    MuliNotUsed(dir);
    return Edge{ Mul(transform, va), Mul(transform, vb), 0, 1 };
}

inline Vec2 Capsule::GetVertex(int32 id) const
{
    MuliAssert(id == 0 || id == 1);
    return id == 0 ? va : vb;
}

inline int32 Capsule::GetVertexCount() const
{
    return 2;
}

inline int32 Capsule::GetSupport(const Vec2& localDir) const
{
    Vec2 e = vb - va;
    return Dot(e, localDir) > 0.0f ? 1 : 0;
}

inline float Capsule::GetLength() const
{
    return Dist(va, vb);
}

inline const Vec2& Capsule::GetVertexA() const
{
    return va;
}

inline const Vec2& Capsule::GetVertexB() const
{
    return vb;
}

inline Capsule::Capsule(float length, float radius, bool horizontal, const Transform& tf)
    : Shape(capsule, radius)
{
    area = length * radius * 2.0f + pi * radius * radius;

    if (horizontal)
    {
        va = Vec2{ -length / 2.0f, 0.0f };
        vb = Vec2{ length / 2.0f, 0.0f };
    }
    else
    {
        va = Vec2{ 0.0f, -length / 2.0f };
        vb = Vec2{ 0.0f, length / 2.0f };
    }

    center = tf.position;

    va = Mul(tf, va);
    vb = Mul(tf, vb);
}

inline Capsule::Capsule(const Vec2& p1, const Vec2& p2, float radius, bool resetPosition, const Transform& tf)
    : Shape(capsule, radius)
{
    Vec2 a2b = p2 - p1;

    float length = a2b.Length();
    area = length * radius * 2.0f + pi * radius * radius;

    va = p1;
    vb = p2;
    center = (p1 + p2) * 0.5f;

    if (resetPosition)
    {
        va -= center;
        vb -= center;
        center.SetZero();
    }

    va = Mul(tf, va);
    vb = Mul(tf, vb);
    center = Mul(tf, center);
}

inline void Capsule::ComputeMass(float density, MassData* outMassData) const
{
    outMassData->mass = density * area;

    float length = Dist(va, vb);
    float height = radius * 2.0f;
    float invArea = 1.0f / area;

    float inertia;

    float rectArea = length * height;
    float rectInertia = (length * length + height * height) / 12.0f;

    inertia = rectInertia * rectArea * invArea;

    float circleArea = pi * radius * radius;
    float halfCircleInertia = ((pi / 4.0f) - 8.0f / (9.0f * pi)) * radius * radius * radius * radius;
    float dist2 = length * 0.5f + (4.0f * radius) / (pi * 3.0f);
    dist2 *= dist2;

    inertia += (halfCircleInertia + (circleArea * 0.5f) * dist2) * 2.0f * invArea;

    outMassData->inertia = outMassData->mass * (inertia + Length2(center));
    outMassData->centerOfMass = center;
}

inline void Capsule::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    Vec2 v1 = Mul(transform, va);
    Vec2 v2 = Mul(transform, vb);

    outAABB->min = Min(v1, v2) - Vec2{ radius, radius };
    outAABB->max = Max(v1, v2) + Vec2{ radius, radius };
}

inline bool Capsule::TestPoint(const Transform& transform, const Vec2& q) const
{
    Vec2 localQ = MulT(transform, q);

    Vec2 d = localQ - va;
    Vec2 e = vb - va;

    float w = Clamp(Dot(d, e) / Dot(e, e), 0.0f, 1.0f);

    Vec2 closest = va + w * e;

    return Dist2(localQ, closest) < radius * radius;
}

inline Vec2 Capsule::GetClosestPoint(const Transform& transform, const Vec2& q) const
{
    Vec2 localQ = MulT(transform, q);

    float u = Dot(localQ - vb, va - vb);
    float v = Dot(localQ - va, vb - va);

    Vec2 closest;
    Vec2 normal;
    float distance;

    if (v <= 0.0f) // Region A
    {
        closest = va;
        normal = localQ - va;
        distance = normal.Normalize();
    }
    else if (u <= 0.0f) // Region B
    {
        closest = vb;
        normal = localQ - vb;
        distance = normal.Normalize();
    }
    else // Region AB
    {
        normal = Normalize(Cross(1.0f, vb - va));
        distance = Dot(localQ - va, normal);

        if (Dot(normal, localQ - va) < 0.0f)
        {
            normal = -normal;
            distance = -distance;
        }

        closest = localQ + normal * -distance;
    }

    if (distance <= radius)
    {
        return q;
    }
    else
    {
        closest += normal * radius;
        return Mul(transform, closest);
    }
}

inline bool Capsule::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    Vec2 p1 = MulT(transform, input.from);
    Vec2 p2 = MulT(transform, input.to);

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
                output->normal = Mul(transform.rotation, Normalize(f + d * t));
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
                output->normal = Mul(transform.rotation, Normalize(f + d * t));
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
            output->normal = Mul(transform.rotation, Normalize(f + d * t));
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
            output->normal = Mul(transform.rotation, Normalize(f + d * t));
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
        output->normal = Mul(transform.rotation, -n);
    }
    else
    {
        output->normal = Mul(transform.rotation, n);
    }

    return true;
}

} // namespace flywheel
