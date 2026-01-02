#include "flywheel/constraint.h"
#include "flywheel/world.h"

namespace flywheel
{

Constraint::Constraint(RigidBody* bodyA, RigidBody* bodyB)
    : bodyA{ bodyA }
    , bodyB{ bodyB }
    , beta{ 0.0f }
    , gamma{ 0.0f }
{
    MuliAssert(bodyA->GetWorld() == bodyB->GetWorld());
}

} // namespace flywheel
