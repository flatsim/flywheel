#pragma once

#include "../core/common.h"
#include "../math/math.h"
#include "joint.h"

namespace flywheel
{

class GrabJoint : public Joint
{
public:
    inline GrabJoint(
        RigidBody* body, const Vec2& anchor, const Vec2& target, float frequency, float dampingRatio, float jointMass
    );

    virtual inline void Prepare(const Timestep& step) override;
    virtual inline void SolveVelocityConstraints(const Timestep& step) override;

    const Vec2& GetLocalAnchor() const;

    const Vec2& GetTarget() const;
    void SetTarget(const Vec2& newTarget);

private:
    Vec2 localAnchor;
    Vec2 target;

    Vec2 r;
    Mat2 m;

    Vec2 bias;
    Vec2 impulseSum;

    inline void ApplyImpulse(const Vec2& lambda);
};

inline GrabJoint::GrabJoint(
    RigidBody* body,
    const Vec2& anchor,
    const Vec2& targetPosition,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
    : Joint(grab_joint, body, body, jointFrequency, jointDampingRatio, jointMass)
    , impulseSum{ 0.0f }
{
    localAnchor = MulT(body->GetTransform(), anchor);
    target = targetPosition;
}

inline void GrabJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass W
    // J = [I, skew(r)]
    // W = (J · M^-1 · J^t)^-1

    r = Mul(bodyA->GetRotation(), localAnchor - bodyA->GetLocalCenter());
    Vec2 p = bodyA->motion.c + r;

    Mat2 k;

    k[0][0] = bodyA->invMass + bodyA->invInertia * r.y * r.y;
    k[1][0] = -bodyA->invInertia * r.y * r.x;
    k[0][1] = -bodyA->invInertia * r.x * r.y;
    k[1][1] = bodyA->invMass + bodyA->invInertia * r.x * r.x;

    k[0][0] += gamma;
    k[1][1] += gamma;

    m = k.GetInverse();

    Vec2 error = p - target;
    bias = error * beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

inline void GrabJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Compute corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    Vec2 jv = bodyA->linearVelocity + Cross(bodyA->angularVelocity, r);

    Vec2 lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

inline void GrabJoint::ApplyImpulse(const Vec2& lambda)
{
    bodyA->linearVelocity += lambda * bodyA->invMass;
    bodyA->angularVelocity += bodyA->invInertia * Cross(r, lambda);
}

inline const Vec2& GrabJoint::GetLocalAnchor() const
{
    return localAnchor;
}

inline const Vec2& GrabJoint::GetTarget() const
{
    return target;
}

inline void GrabJoint::SetTarget(const Vec2& newTarget)
{
    target = newTarget;
}

} // namespace flywheel