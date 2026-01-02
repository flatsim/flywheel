#pragma once

#include "common.h"
#include "joint.h"
#include "math.h"

namespace flywheel
{

class AngleJoint : public Joint
{
public:
    inline AngleJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        float angleOffset,
        float minAngle,
        float maxAngle,
        float frequency,
        float dampingRatio,
        float jointMass
    );

    virtual inline void Prepare(const Timestep& step) override;
    virtual inline void SolveVelocityConstraints(const Timestep& step) override;

    float GetJointAngleOffset() const;

    float GetJointAngle() const;
    void SetJointAngle(float newAngle);

    float GetJointMinAngle() const;
    void SetJointMinAngle(float newMinAngle);
    float GetJointMaxAngle() const;
    void SetJointMaxAngle(float newMaxAngle);

private:
    float angleOffset;
    float minAngle, maxAngle;

    float m;

    Vec2 bias;
    Vec2 impulseSum;

    inline void ApplyImpulse(float lambda);
};

inline AngleJoint::AngleJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    float jointAngleOffset,
    float jointMinAngle,
    float jointMaxAngle,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
    : Joint(angle_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
    , angleOffset{ jointAngleOffset }
    , minAngle{ jointMinAngle }
    , maxAngle{ jointMaxAngle }
    , impulseSum{ 0.0f }
{
    maxAngle = Max(minAngle, maxAngle);
}

inline void AngleJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass W
    // J = [0 -1 0 1]
    // W = (J · M^-1 · J^t)^-1

    float k = bodyA->invInertia + bodyB->invInertia + gamma;

    if (k != 0.0f)
    {
        m = 1.0f / k;
    }

    Vec2 error(bodyB->motion.a - bodyA->motion.a - angleOffset);
    error[0] -= minAngle;
    error[1] -= maxAngle;

    bias = error * beta * step.inv_dt;

    if (step.warm_starting)
    {
        if (minAngle == maxAngle)
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

inline void AngleJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Compute corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    float jv = bodyB->angularVelocity - bodyA->angularVelocity;

    if (minAngle == maxAngle)
    {
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

inline void AngleJoint::ApplyImpulse(float lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    bodyA->angularVelocity -= lambda * bodyA->invInertia;
    bodyB->angularVelocity += lambda * bodyB->invInertia;
}

inline float AngleJoint::GetJointAngleOffset() const
{
    return angleOffset;
}

inline float AngleJoint::GetJointAngle() const
{
    return minAngle;
}

inline void AngleJoint::SetJointAngle(float newAngle)
{
    minAngle = newAngle;
    maxAngle = newAngle;
}

inline float AngleJoint::GetJointMinAngle() const
{
    return minAngle;
}

inline void AngleJoint::SetJointMinAngle(float newMinAngle)
{
    minAngle = newMinAngle;
    maxAngle = Max(minAngle, maxAngle);
}

inline float AngleJoint::GetJointMaxAngle() const
{
    return maxAngle;
}

inline void AngleJoint::SetJointMaxAngle(float newMaxAngle)
{
    maxAngle = newMaxAngle;
    minAngle = Min(minAngle, maxAngle);
}
} // namespace flywheel