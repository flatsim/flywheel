#pragma once

#include "collision.h"
#include "shape.h"
#include "simplex.h"

namespace flywheel
{

// Closest features in world space
struct ClosestFeatures
{
    Point featuresA[max_simplex_vertex_count - 1];
    Point featuresB[max_simplex_vertex_count - 1];
    int32 count;
};

// clang-format off
float GetClosestFeatures(
    const Shape* a, const Transform& tfA,
    const Shape* b, const Transform& tfB, 
    ClosestFeatures* features
);

float ComputeDistance(
    const Shape* a, const Transform& tfA,
    const Shape* b, const Transform& tfB, 
    Vec2* pointA, Vec2* pointB
);
// clang-format on

inline float GetClosestFeatures(
    const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ClosestFeatures* features
)
{
    GJKResult gjkResult;

    bool collide = GJK(a, tfA, b, tfB, &gjkResult);
    if (collide == true)
    {
        return 0.0f;
    }

    Simplex& simplex = gjkResult.simplex;
    MuliAssert(simplex.count < max_simplex_vertex_count);

    features->count = simplex.count;
    for (int32 i = 0; i < features->count; ++i)
    {
        features->featuresA[i] = simplex.vertices[i].pointA;
        features->featuresB[i] = simplex.vertices[i].pointB;
    }

    return gjkResult.distance;
}

inline float ComputeDistance(
    const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, Vec2* pointA, Vec2* pointB
)
{
    GJKResult gjkResult;

    bool collide = GJK(a, tfA, b, tfB, &gjkResult);
    if (collide == true)
    {
        return 0.0f;
    }

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    if (gjkResult.distance < radii)
    {
        return 0.0f;
    }

    Simplex& simplex = gjkResult.simplex;

    MuliAssert(simplex.count < max_simplex_vertex_count);

    simplex.GetWitnessPoint(pointA, pointB);

    // displace simplex vertices along normal
    *pointA += gjkResult.direction * ra;
    *pointB -= gjkResult.direction * rb;

    return gjkResult.distance - radii;
}

} // namespace flywheel
