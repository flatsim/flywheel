# Changelog

## [0.0.9] - 2026-01-02

### <!-- 0 -->⛰️  Features

- Refactor collision and geometry management

### <!-- 1 -->🐛 Bug Fixes

- Fix `Collider` initialization and destructor
- Consolidate global variable definitions
- Refactor `block_solve` and remove debugging

## [0.0.8] - 2026-01-02

### <!-- 0 -->⛰️  Features

- Create empty flywheel.cpp for header-only conversion

### <!-- 2 -->🚜 Refactor

- Convert weld_joint.cpp to header-only
- Convert revolute_joint.cpp to header-only
- Convert pulley_joint.cpp to header-only
- Convert prismatic_joint.cpp to header-only
- Convert motor_joint.cpp to header-only
- Convert line_joint.cpp to header-only
- Convert joint.cpp to header-only
- Convert grab_joint.cpp to header-only
- Convert distance_joint.cpp to header-only
- Convert angle_joint.cpp to header-only
- Convert contact.cpp to header-only
- Convert world.cpp to header-only (moved to flywheel.cpp)
- Convert rigidbody.cpp to header-only (moved to flywheel.cpp)
- Convert island.cpp to header-only (moved to flywheel.cpp)
- Convert position_solver.cpp to header-only
- Convert contact_solver.cpp to header-only
- Convert contact_graph.cpp to header-only (moved to flywheel.cpp)
- Convert collider.cpp to header-only (moved to flywheel.cpp)
- Convert block_solver.cpp to header-only
- Convert time_of_impact.cpp to header-only
- Convert simplex.cpp to header-only
- Convert polytope.cpp to header-only
- Convert distance.cpp to header-only
- Convert broad_phase.cpp to header-only
- Convert aabb_tree.cpp to header-only
- Convert constraint.cpp to header-only
- Convert geometry.cpp to mostly header-only (extern ComputeConvexHull remains)
- Convert raycast.cpp to header-only (except ShapeCast)
- Convert stack_allocator.cpp to header-only
- Convert math.cpp to header-only
- Convert polygon.cpp to header-only
- Convert circle.cpp to header-only
- Convert linear_allocator.cpp to header-only
- Convert capsule.cpp to header-only
- Convert block_allocator.cpp to header-only

## [0.0.7] - 2026-01-02

### <!-- 6 -->🧪 Testing

- Add comprehensive integration scenario tests (flywheel-dzs.11)
- Add comprehensive joint system tests (flywheel-dzs.10)
- Add comprehensive constraint system tests (flywheel-dzs.9)
- Add comprehensive contact solver tests (flywheel-dzs.8)
- Add rigidbody dynamics, contact solver, constraint, and joint tests
- Add comprehensive collision detection tests
- Add allocator and baseline integration tests
- Add comprehensive test suite with doctest

## [0.0.6] - 2026-01-02

### <!-- 1 -->🐛 Bug Fixes

- Fix library source file globbing
- Use INTERFACE for target_link_libraries when library is header-only

## [0.0.4] - 2026-01-02

### <!-- 0 -->⛰️  Features

- Rename Muli to Flywheel

### <!-- 3 -->📚 Documentation

- Add acknowledgments and original author details

## [0.0.3] - 2026-01-02

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Add initial CHANGELOG.md file

