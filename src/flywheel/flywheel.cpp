#include "flywheel/world.h"

namespace flywheel
{

// Global variables that need to be defined in exactly one translation unit
const Vec2 origin = Vec2::zero;
const int32 toi_postion_iteration = 20;
const int32 toi_index_1 = 0;
const int32 toi_index_2 = 1;

ContactListener defaultListener;

bool block_solve = true;
bool detection_function_initialized = false;
CollideFunction* collide_function_map[Shape::Type::shape_count][Shape::Type::shape_count] = {};

void InitializeDetectionFunctionMap()
{
    if (detection_function_initialized)
    {
        return;
    }

    collide_function_map[Shape::Type::circle][Shape::Type::circle] = &CircleVsCircle;

    collide_function_map[Shape::Type::capsule][Shape::Type::circle] = &CapsuleVsCircle;
    collide_function_map[Shape::Type::capsule][Shape::Type::capsule] = &ConvexVsConvex;

    collide_function_map[Shape::Type::polygon][Shape::Type::circle] = &PolygonVsCircle;
    collide_function_map[Shape::Type::polygon][Shape::Type::capsule] = &ConvexVsConvex;
    collide_function_map[Shape::Type::polygon][Shape::Type::polygon] = &ConvexVsConvex;

    detection_function_initialized = true;
}

} // namespace flywheel
