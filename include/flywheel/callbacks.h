#pragma once

#include "collider.h"
#include "common.h"
#include "contact.h"
#include "joint.h"
#include "math.h"

namespace flywheel
{

class ColliderDestroyCallback
{
public:
    virtual ~ColliderDestroyCallback() {}
    virtual void OnColliderDestroy(Collider* me) = 0;
};

class BodyDestroyCallback
{
public:
    virtual ~BodyDestroyCallback() {}
    virtual void OnBodyDestroy(RigidBody* me) = 0;
};

class JointDestroyCallback
{
public:
    virtual ~JointDestroyCallback() {}
    virtual void OnJointDestroy(Joint* me) = 0;
};

class ContactListener
{
public:
    virtual ~ContactListener() {}

    virtual void OnContactBegin(Collider* me, Collider* other, Contact* contact)
    {
        MuliNotUsed(me);
        MuliNotUsed(other);
        MuliNotUsed(contact);
    }

    virtual void OnContactTouching(Collider* me, Collider* other, Contact* contact)
    {
        MuliNotUsed(me);
        MuliNotUsed(other);
        MuliNotUsed(contact);
    }

    virtual void OnContactEnd(Collider* me, Collider* other, Contact* contact)
    {
        MuliNotUsed(me);
        MuliNotUsed(other);
        MuliNotUsed(contact);
    }

    virtual void OnPreSolve(Collider* me, Collider* other, Contact* contact)
    {
        MuliNotUsed(me);
        MuliNotUsed(other);
        MuliNotUsed(contact);
    }

    virtual void OnPostSolve(Collider* me, Collider* other, Contact* contact)
    {
        MuliNotUsed(me);
        MuliNotUsed(other);
        MuliNotUsed(contact);
    }
};

class WorldQueryCallback
{
public:
    virtual ~WorldQueryCallback() {}
    virtual bool OnQuery(Collider* collider) = 0;
};

class RayCastAnyCallback
{
public:
    virtual ~RayCastAnyCallback() {}
    virtual float OnHitAny(Collider* collider, Vec2 point, Vec2 normal, float fraction) = 0;
};

class RayCastClosestCallback
{
public:
    virtual ~RayCastClosestCallback() {}
    virtual void OnHitClosest(Collider* collider, Vec2 point, Vec2 normal, float fraction) = 0;
};

class ShapeCastAnyCallback
{
public:
    virtual ~ShapeCastAnyCallback() {}
    virtual float OnHitAny(Collider* collider, Vec2 point, Vec2 normal, float t) = 0;
};

class ShapeCastClosestCallback
{
public:
    virtual ~ShapeCastClosestCallback() {}
    virtual void OnHitClosest(Collider* collider, Vec2 point, Vec2 normal, float t) = 0;
};

} // namespace flywheel

// Include headers needed for Contact inline implementations
#include "settings.h"
#include "world.h"

namespace flywheel
{

// Inline global variables (C++17)
inline bool block_solve = true;
inline CollideFunction* collide_function_map[Shape::Type::shape_count][Shape::Type::shape_count] = {};

inline Contact::Contact(Collider* colliderA, Collider* colliderB)
    : Constraint(colliderA->body, colliderB->body)
    , colliderA{ colliderA }
    , colliderB{ colliderB }
    , flag{ 0 }
    , toiCount{ 0 }
    , toi{ 0.0f }
{
    MuliAssert(colliderA->GetType() >= colliderB->GetType());

    manifold.contactCount = 0;

    friction = MixFriction(colliderA->GetFriction(), colliderB->GetFriction());
    restitution = MixRestitution(colliderA->GetRestitution(), colliderB->GetRestitution());
    restitutionThreshold = MixRestitutionTreshold(colliderA->GetRestitutionTreshold(), colliderB->GetRestitutionTreshold());
    surfaceSpeed = colliderB->GetSurfaceSpeed() + colliderA->GetSurfaceSpeed();

    collideFunction = collide_function_map[colliderA->GetType()][colliderB->GetType()];
    MuliAssert(collideFunction != nullptr);
}

inline void Contact::Update()
{
    flag |= flag_enabled;

    ContactManifold oldManifold = manifold;
    for (int32 i = 0; i < max_contact_point_count; ++i)
    {
        normalSolvers[i].impulseSave = normalSolvers[i].impulse;
        tangentSolvers[i].impulseSave = tangentSolvers[i].impulse;
        normalSolvers[i].impulse = 0.0f;
        tangentSolvers[i].impulse = 0.0f;
    }

    // clang-format off
    bool wasTouching = (flag & flag_touching) == flag_touching;
    bool touching = collideFunction(colliderA->shape, bodyA->transform,
                                    colliderB->shape, bodyB->transform,
                                    &manifold);
    // clang-format on

    if (touching == true)
    {
        flag |= flag_touching;
    }
    else
    {
        flag &= ~flag_touching;
    }

    if (touching == false)
    {
        if (wasTouching == true)
        {
            if (colliderA->ContactListener) colliderA->ContactListener->OnContactEnd(colliderA, colliderB, this);
            if (colliderB->ContactListener) colliderB->ContactListener->OnContactEnd(colliderB, colliderA, this);
        }

        return;
    }

    if (manifold.featureFlipped)
    {
        b1 = bodyB;
        b2 = bodyA;
    }
    else
    {
        b1 = bodyA;
        b2 = bodyB;
    }

    // Restore the impulses to warm start the solver
    for (int32 n = 0; n < manifold.contactCount; ++n)
    {
        for (int32 o = 0; o < oldManifold.contactCount; ++o)
        {
            if (manifold.contactPoints[n].id == oldManifold.contactPoints[o].id)
            {
                normalSolvers[n].impulse = normalSolvers[o].impulseSave;
                tangentSolvers[n].impulse = tangentSolvers[o].impulseSave;
                break;
            }
        }
    }

    if (touching == true)
    {
        if (wasTouching == false)
        {
            if (colliderA->ContactListener) colliderA->ContactListener->OnContactBegin(colliderA, colliderB, this);
            if (colliderB->ContactListener) colliderB->ContactListener->OnContactBegin(colliderB, colliderA, this);
        }
        else
        {
            if (colliderA->ContactListener) colliderA->ContactListener->OnContactTouching(colliderA, colliderB, this);
            if (colliderB->ContactListener) colliderB->ContactListener->OnContactTouching(colliderB, colliderA, this);
        }

        if (colliderA->ContactListener) colliderA->ContactListener->OnPreSolve(colliderA, colliderB, this);
        if (colliderB->ContactListener) colliderB->ContactListener->OnPreSolve(colliderB, colliderA, this);
    }

    if (colliderA->IsEnabled() == false || colliderB->IsEnabled() == false)
    {
        flag &= ~flag_enabled;
    }
}

inline void Contact::Prepare(const Timestep& step)
{
    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        normalSolvers[i].Prepare(this, manifold.contactNormal, i, step);
        tangentSolvers[i].Prepare(this, manifold.contactTangent, i, step);
        positionSolvers[i].Prepare(this, i);
    }

    if (manifold.contactCount == 2 && block_solve == true)
    {
        blockSolver.Prepare(this);
    }
}

inline void Contact::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Solve tangential constraint first
    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        tangentSolvers[i].Solve(this, normalSolvers + i);
    }

    if (manifold.contactCount == 1 || block_solve == false || blockSolver.enabled == false)
    {
        for (int32 i = 0; i < manifold.contactCount; ++i)
        {
            normalSolvers[i].Solve(this);
        }
    }
    else
    {
        // Solve two contact constraints simultaneously (2-Contact LCP solver)
        blockSolver.Solve(this);
    }
}

inline bool Contact::SolvePositionConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    bool solved = true;

    cLinearImpulseA.SetZero();
    cLinearImpulseB.SetZero();
    cAngularImpulseA = 0.0f;
    cAngularImpulseB = 0.0f;

    // Solve position constraint
    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        solved &= positionSolvers[i].Solve();
    }

    b1->motion.c += b1->invMass * cLinearImpulseA;
    b1->motion.a += b1->invInertia * cAngularImpulseA;
    b2->motion.c += b2->invMass * cLinearImpulseB;
    b2->motion.a += b2->invInertia * cAngularImpulseB;

    return solved;
}

inline bool Contact::SolveTOIPositionConstraints()
{
    bool solved = true;

    cLinearImpulseA.SetZero();
    cLinearImpulseB.SetZero();
    cAngularImpulseA = 0.0f;
    cAngularImpulseB = 0.0f;

    // Solve position constraint
    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        solved &= positionSolvers[i].SolveTOI();
    }

    // Push the body only if it's involved in TOI contact
    // TOI index == 0 or 1
    if (b1->islandIndex < 2)
    {
        b1->motion.c += b1->invMass * cLinearImpulseA;
        b1->motion.a += b1->invInertia * cAngularImpulseA;
    }
    if (b2->islandIndex < 2)
    {
        b2->motion.c += b2->invMass * cLinearImpulseB;
        b2->motion.a += b2->invInertia * cAngularImpulseB;
    }

    return solved;
}

} // namespace flywheel
