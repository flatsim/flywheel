#pragma once

#include "shape.h"

namespace flywheel
{

class Circle : public Shape
{
public:
    Circle(float radius, const Transform& tf = identity);

    Circle(const Circle& other, const Transform& tf);

    virtual void ComputeMass(float density, MassData* outMassData) const override;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const override;

    virtual int32 GetVertexCount() const override;
    virtual Vec2 GetVertex(int32 id) const override;
    virtual int32 GetSupport(const Vec2& localDir) const override;
    virtual Edge GetFeaturedEdge(const Transform& transform, const Vec2& dir) const override;

    virtual bool TestPoint(const Transform& transform, const Vec2& q) const override;
    virtual Vec2 GetClosestPoint(const Transform& transform, const Vec2& q) const override;
    virtual bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const override;

protected:
    virtual Shape* Clone(Allocator* allocator, const Transform& tf) const override;
};

inline Circle::Circle(const Circle& other, const Transform& tf)
    : Circle(other.radius, Mul(tf, other.center))
{
}

inline Shape* Circle::Clone(Allocator* allocator, const Transform& tf) const
{
    void* mem = allocator->Allocate(sizeof(Circle));
    Circle* shape = new (mem) Circle(*this, tf);
    return shape;
}

inline Edge Circle::GetFeaturedEdge(const Transform& transform, const Vec2& dir) const
{
    MuliNotUsed(dir);
    return Edge{ Mul(transform, center), Mul(transform, center) };
}

inline Vec2 Circle::GetVertex(int32 id) const
{
    MuliAssert(id == 0);
    MuliNotUsed(id);
    return center;
}

inline int32 Circle::GetVertexCount() const
{
    return 1;
}

inline int32 Circle::GetSupport(const Vec2& localDir) const
{
    MuliNotUsed(localDir);
    return 0;
}

inline Circle::Circle(float radius, const Transform& tf)
    : Shape(circle, radius)
{
    area = radius * radius * pi;
    center = tf.position;
}

inline void Circle::ComputeMass(float density, MassData* outMassData) const
{
    outMassData->mass = density * area;
    float inertia = 0.5f * radius * radius;
    outMassData->inertia = outMassData->mass * (inertia + Length2(center));
    outMassData->centerOfMass = center;
}

inline void Circle::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    Vec2 p = Mul(transform, center);

    outAABB->min = Vec2{ p.x - radius, p.y - radius };
    outAABB->max = Vec2{ p.x + radius, p.y + radius };
}

inline bool Circle::TestPoint(const Transform& transform, const Vec2& q) const
{
    Vec2 localQ = MulT(transform, q);
    Vec2 d = center - localQ;

    return Dot(d, d) <= radius * radius;
}

inline Vec2 Circle::GetClosestPoint(const Transform& transform, const Vec2& q) const
{
    Vec2 position = Mul(transform, center);
    Vec2 dir = (q - position);

    float distance = dir.Normalize();
    if (distance <= radius)
    {
        return q;
    }
    else
    {
        return position + dir * radius;
    }
}

inline bool Circle::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    Vec2 position = Mul(transform, center);

    Vec2 d = input.to - input.from;
    Vec2 f = input.from - position;

    float radii = radius + input.radius;

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

} // namespace flywheel
