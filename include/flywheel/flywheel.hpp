/**
 * @file flywheel.hpp
 * @brief Main interface header for the Flywheel physics engine
 *
 * This is the primary include file for the Flywheel 2D physics engine.
 * Include this file to access all Flywheel functionality.
 *
 * @section structure Library Structure
 * - core/      - Core types, settings, and common utilities
 * - math/      - Mathematical primitives and operations
 * - memory/    - Memory allocation systems
 * - collision/ - Collision detection and broad phase
 * - shapes/    - Geometric shape primitives
 * - dynamics/  - Physics simulation (bodies, contacts, constraints)
 * - joints/    - Constraint joints
 * - utils/     - Utility functions (callbacks, formatting, etc.)
 */

#pragma once

// clang-format off

// ============================================================================
// Core
// ============================================================================
#include "core/types.h"
#include "core/common.h"
#include "core/settings.h"

// ============================================================================
// Math
// ============================================================================
#include "math/math.h"
#include "math/primitives.h"
#include "math/raycast.h"
#include "math/spline.h"

// ============================================================================
// Memory
// ============================================================================
#include "memory/allocator.h"
#include "memory/block_allocator.h"
#include "memory/linear_allocator.h"
#include "memory/stack_allocator.h"
#include "memory/fixed_block_allocator.h"
#include "memory/growable_array.h"

// ============================================================================
// Collision (partial - just AABB and filter for now)
// ============================================================================
#include "collision/aabb.h"
#include "collision/collision_filter.h"

// ============================================================================
// Shapes (need to come before full collision and geometry)
// ============================================================================
#include "shapes/material.h"
#include "shapes/shape.h"
#include "shapes/circle.h"
#include "shapes/capsule.h"
#include "shapes/polygon.h"

// ============================================================================
// Math (geometry, distance - depend on shapes)
// ============================================================================
#include "math/geometry.h"
#include "math/distance.h"

// ============================================================================
// Collision (rest of collision system)
// ============================================================================
#include "collision/aabb_tree.h"
#include "collision/simplex.h"
#include "collision/polytope.h"
#include "collision/collision.h"
#include "collision/time_of_impact.h"
#include "collision/broad_phase.h"

// ============================================================================
// Dynamics
// ============================================================================
#include "dynamics/collider.h"
#include "dynamics/rigidbody.h"
#include "dynamics/constraint.h"
#include "dynamics/contact.h"
#include "dynamics/contact_solver.h"
#include "dynamics/position_solver.h"
#include "dynamics/contact_graph.h"
#include "dynamics/island.h"
#include "dynamics/world.h"

// ============================================================================
// Joints
// ============================================================================
#include "joints/joint.h"
#include "joints/angle_joint.h"
#include "joints/distance_joint.h"
#include "joints/grab_joint.h"
#include "joints/line_joint.h"
#include "joints/motor_joint.h"
#include "joints/prismatic_joint.h"
#include "joints/pulley_joint.h"
#include "joints/revolute_joint.h"
#include "joints/weld_joint.h"

// ============================================================================
// Utils
// ============================================================================
#include "utils/callbacks.h"
#include "utils/format.h"
#include "utils/hash.h"
#include "utils/random.h"

// clang-format on

/**
 * @namespace flywheel
 * @brief Main namespace for the Flywheel physics engine
 *
 * All Flywheel classes, functions, and constants are contained within
 * this namespace to avoid naming conflicts.
 */
