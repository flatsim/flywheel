#pragma once

#include "common.h"
#include "joint.h"
#include "math.h"

namespace flywheel
{

class DistanceJoint : public Joint
{
public:
    inline DistanceJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec2& anchorA,
        const Vec2& anchorB,
        float minLength,
        float maxLength,
        float frequency,
        float dampingRatio,
        float jointMass
    );

    virtual inline void Prepare(const Timestep& step) override;
    virtual inline void SolveVelocityConstraints(const Timestep& step) override;

    const Vec2& GetLocalAnchorA() const;
    const Vec2& GetLocalAnchorB() const;

    float GetJointLength() const;
    void SetJointLength(float newLength);

    float GetJointMinLength() const;
    void SetJointMinLength(float newMinLength);
    float GetJointMaxLength() const;
    void SetJointMaxLength(float newMaxLength);

private:
    Vec2 localAnchorA;
    Vec2 localAnchorB;
    float minLength, maxLength;

    Vec2 ra;
    Vec2 rb;
    Vec2 d;
    float m;

    Vec2 bias;
    Vec2 impulseSum;

    inline void ApplyImpulse(float lambda);
};

inline DistanceJoint::DistanceJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchorA,
    const Vec2& anchorB,
    float jointMinLength,
    float jointMaxLength,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
    : Joint(distance_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
    , impulseSum{ 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), anchorA);
    localAnchorB = MulT(bodyB->GetTransform(), anchorB);
    minLength = jointMinLength < 0 ? Length(anchorB - anchorA) : jointMinLength;
    maxLength = jointMaxLength < 0 ? Length(anchorB - anchorA) : jointMaxLength;
    maxLength = Max(minLength, maxLength);
}

inline void DistanceJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass W
    // J = [-d, -d×ra, d, d×rb] ( d = (anchorB-anchorA) / ||anchorB-anchorA|| )
    // W = (J · M^-1 · J^t)^-1

    ra = Mul(bodyA->GetRotation(), localAnchorA - bodyA->GetLocalCenter());
    rb = Mul(bodyB->GetRotation(), localAnchorB - bodyB->GetLocalCenter());

    Vec2 pa = bodyA->motion.c + ra;
    Vec2 pb = bodyB->motion.c + rb;

    d = pb - pa;
    float currentLength = d.Normalize();

    // clang-format off
    float k = bodyA->invMass + bodyB->invMass
            + bodyA->invInertia * Cross(d, ra) * Cross(d, ra)
            + bodyB->invInertia * Cross(d, rb) * Cross(d, rb)
            + gamma;
    // clang-format on

    if (k != 0.0f)
    {
        m = 1.0f / k;
    }

    Vec2 error(currentLength - minLength, currentLength - maxLength);
    bias = error * beta * step.inv_dt;

    if (step.warm_starting)
    {
        if (minLength == maxLength)
        {
            ApplyImpulse(impulseSum[0]);
        }
        else
        {
            if (bias[0] < 0)
            {
                ApplyImpulse(impulseSum[0]);
            }
            if (bias[1] > 0)
            {
                ApplyImpulse(impulseSum[1]);
            }
        }
    }
}

inline void DistanceJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Compute corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    float jv =
        Dot((bodyB->linearVelocity + Cross(bodyB->angularVelocity, rb)) -
                (bodyA->linearVelocity + Cross(bodyA->angularVelocity, ra)),
            d);

    if (minLength == maxLength)
    {
        // You don't have to clamp the impulse because it's equality constraint!
        float lambda = m * -(jv + bias[0] + impulseSum[0] * gamma);
        ApplyImpulse(lambda);
        impulseSum[0] += lambda;
    }
    else
    {
        if (bias[0] < 0)
        {
            float lambda = m * -(jv + bias[0] + impulseSum[0] * gamma);
            ApplyImpulse(lambda);
            impulseSum[0] += lambda;
        }

        if (bias[1] > 0)
        {
            float lambda = m * -(jv + bias[1] + impulseSum[1] * gamma);
            ApplyImpulse(lambda);
            impulseSum[1] += lambda;
        }
    }
}

inline void DistanceJoint::ApplyImpulse(float lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    Vec2 p = d * lambda;

    bodyA->linearVelocity -= p * bodyA->invMass;
    bodyA->angularVelocity -= Dot(d, Cross(lambda, ra)) * bodyA->invInertia;
    bodyB->linearVelocity += p * bodyB->invMass;
    bodyB->angularVelocity += Dot(d, Cross(lambda, rb)) * bodyB->invInertia;
}

inline const Vec2& DistanceJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const Vec2& DistanceJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

inline float DistanceJoint::GetJointLength() const
{
    return minLength;
}

inline void DistanceJoint::SetJointLength(float newLength)
{
    minLength = Max(newLength, 0.0f);
    maxLength = minLength;
}

inline float DistanceJoint::GetJointMinLength() const
{
    return minLength;
}

inline void DistanceJoint::SetJointMinLength(float newMinLength)
{
    minLength = Max(newMinLength, 0.0f);
    maxLength = Max(minLength, maxLength);
}

inline float DistanceJoint::GetJointMaxLength() const
{
    return maxLength;
}

inline void DistanceJoint::SetJointMaxLength(float newMaxLength)
{
    maxLength = Max(newMaxLength, 0.0f);
    minLength = Min(minLength, maxLength);
}

} // namespace flywheel