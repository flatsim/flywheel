#include "flywheel/world.h"
#include <cstdio>

namespace flywheel
{

// Global variables that need to be defined in exactly one translation unit
const Vec2 origin = Vec2::zero;
const int32 toi_postion_iteration = 20;
const int32 toi_index_1 = 0;
const int32 toi_index_2 = 1;

ContactListener defaultListener;

bool detection_function_initialized = false;
CollideFunction* collide_function_map[Shape::Type::shape_count][Shape::Type::shape_count] = {};

void InitializeDetectionFunctionMap()
{
    fprintf(stderr, "[DEBUG flywheel.cpp] InitializeDetectionFunctionMap called\n");
    fprintf(stderr, "[DEBUG flywheel.cpp] &detection_function_initialized = %p\n", (void*)&detection_function_initialized);
    fprintf(stderr, "[DEBUG flywheel.cpp] detection_function_initialized = %d\n", detection_function_initialized);

    if (detection_function_initialized)
    {
        fprintf(stderr, "[DEBUG flywheel.cpp] Already initialized, returning\n");
        return;
    }

    fprintf(stderr, "[DEBUG flywheel.cpp] Initializing collision function map...\n");

    collide_function_map[Shape::Type::circle][Shape::Type::circle] = &CircleVsCircle;

    collide_function_map[Shape::Type::capsule][Shape::Type::circle] = &CapsuleVsCircle;
    collide_function_map[Shape::Type::capsule][Shape::Type::capsule] = &ConvexVsConvex;

    collide_function_map[Shape::Type::polygon][Shape::Type::circle] = &PolygonVsCircle;
    collide_function_map[Shape::Type::polygon][Shape::Type::capsule] = &ConvexVsConvex;
    collide_function_map[Shape::Type::polygon][Shape::Type::polygon] = &ConvexVsConvex;

    detection_function_initialized = true;
    fprintf(
        stderr, "[DEBUG flywheel.cpp] Initialization complete, detection_function_initialized=%d\n",
        detection_function_initialized
    );
}

} // namespace flywheel
