#pragma once

#include "common.h"
#include "growable_array.h"
#include "math.h"
#include "simplex.h"

namespace flywheel
{

struct PolytopeEdge
{
    int32 index;
    float distance;
    Vec2 normal;
};

struct Polytope
{
    GrowableArray<Vec2, 8> vertices;

    Polytope(const Simplex& simplex);
    PolytopeEdge GetClosestEdge() const;
};

inline Polytope::Polytope(const Simplex& simplex)
{
    vertices.EmplaceBack(simplex.vertices[0].point);
    vertices.EmplaceBack(simplex.vertices[1].point);
    vertices.EmplaceBack(simplex.vertices[2].point);
}

// Returns the edge closest to the Origin (0, 0)
inline PolytopeEdge Polytope::GetClosestEdge() const
{
    int32 minIndex = -1;
    float minDistance = max_value;
    Vec2 minNormal{ 0.0f };

    for (int32 i0 = vertices.Count() - 1, i1 = 0; i1 < vertices.Count(); i0 = i1, ++i1)
    {
        Vec2& v0 = vertices[i0];
        Vec2& v1 = vertices[i1];
        Vec2 edge = v1 - v0;

        Vec2 normal = Normalize(Cross(edge, 1.0f));
        float distance = Dot(normal, v0);

        if (distance < 0)
        {
            normal = -normal;
            distance = -distance;
        }

        if (distance < minDistance)
        {
            minDistance = distance;
            minNormal = normal;
            minIndex = i0;
        }
    }

    return PolytopeEdge{ minIndex, minDistance, minNormal };
}

} // namespace flywheel