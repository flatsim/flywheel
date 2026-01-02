#pragma once

#include "collision.h"
#include "constraint.h"
#include "contact_solver.h"
#include "position_solver.h"

namespace flywheel
{

class RigidBody;
class Contact;

struct ContactEdge
{
    RigidBody* other;
    Contact* contact;
    ContactEdge* prev;
    ContactEdge* next;
};

class Contact : Constraint
{
public:
    Contact(Collider* colliderA, Collider* colliderB);
    ~Contact() = default;

    Collider* GetColliderA() const;
    Collider* GetColliderB() const;
    RigidBody* GetReferenceBody() const;
    RigidBody* GetIncidentBody() const;

    const Contact* GetNext() const;
    const Contact* GetPrev() const;

    bool IsTouching() const;

    const ContactManifold& GetContactManifold() const;
    int32 GetContactCount() const;
    float GetNormalImpulse(int32 index) const;
    float GetTangentImpulse(int32 index) const;

    float GetFriction() const;
    float GetRestitution() const;
    float GetRestitutionTreshold() const;
    float GetSurfaceSpeed() const;

    void SetEnabled(bool enabled);
    bool IsEnabled() const;

private:
    friend class World;
    friend class Island;
    friend class ContactGraph;
    friend class BroadPhase;
    friend class ContactSolverNormal;
    friend class ContactSolverTangent;
    friend class BlockSolver;
    friend class PositionSolver;

    enum
    {
        flag_enabled = 1,
        flag_touching = 1 << 1,
        flag_island = 1 << 2,
        flag_toi = 1 << 3,
    };

    virtual void Prepare(const Timestep& step) override;
    virtual void SolveVelocityConstraints(const Timestep& step) override;
    virtual bool SolvePositionConstraints(const Timestep& step) override;
    bool SolveTOIPositionConstraints();

    void Update();

    void SaveImpulses();
    void RestoreImpulses();

    CollideFunction* collideFunction;

    RigidBody* b1; // Reference body
    RigidBody* b2; // Incident body

    Collider* colliderA;
    Collider* colliderB;

    Contact* prev;
    Contact* next;

    ContactEdge nodeA;
    ContactEdge nodeB;

    float friction;
    float restitution;
    float restitutionThreshold;
    float surfaceSpeed;

    ContactManifold manifold;

    // TODO: Integrate decoupled solvers into SolveVelocityConstraints() and SolvePositionConstraints() for optimization
    ContactSolverNormal normalSolvers[max_contact_point_count];
    ContactSolverTangent tangentSolvers[max_contact_point_count];
    PositionSolver positionSolvers[max_contact_point_count];
    BlockSolver blockSolver;

    // Impulse buffer for position correction
    // prefix 'c' stands for corrective
    Vec2 cLinearImpulseA, cLinearImpulseB;
    float cAngularImpulseA, cAngularImpulseB;

    uint16 flag;

    int32 toiCount;
    float toi;
};

inline Collider* Contact::GetColliderA() const
{
    return colliderA;
}

inline Collider* Contact::GetColliderB() const
{
    return colliderB;
}

inline RigidBody* Contact::GetReferenceBody() const
{
    return b1;
}

inline RigidBody* Contact::GetIncidentBody() const
{
    return b2;
}

inline const Contact* Contact::GetPrev() const
{
    return prev;
}

inline const Contact* Contact::GetNext() const
{
    return next;
}

inline bool Contact::IsTouching() const
{
    return (flag & flag_touching) == flag_touching;
}

inline const ContactManifold& Contact::GetContactManifold() const
{
    return manifold;
}

inline int32 Contact::GetContactCount() const
{
    return manifold.contactCount;
}

inline float Contact::GetNormalImpulse(int32 index) const
{
    MuliAssert(index == 0 || index == 1);
    return normalSolvers[index].impulse;
}

inline float Contact::GetTangentImpulse(int32 index) const
{
    MuliAssert(index == 0 || index == 1);
    return tangentSolvers[index].impulse;
}

inline float Contact::GetFriction() const
{
    return friction;
}

inline float Contact::GetRestitution() const
{
    return restitution;
}

inline float Contact::GetRestitutionTreshold() const
{
    return restitutionThreshold;
}

inline float Contact::GetSurfaceSpeed() const
{
    return surfaceSpeed;
}

inline void Contact::SetEnabled(bool enabled)
{
    if (enabled)
    {
        flag |= flag_enabled;
    }
    else
    {
        flag &= ~flag_enabled;
    }
}

inline bool Contact::IsEnabled() const
{
    return (flag & flag_enabled) == flag_enabled;
}

inline void Contact::SaveImpulses()
{
    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        normalSolvers[i].impulseSave = normalSolvers[i].impulse;
        tangentSolvers[i].impulseSave = tangentSolvers[i].impulse;
        normalSolvers[i].impulse = 0.0f;
        tangentSolvers[i].impulse = 0.0f;
    }
}

inline void Contact::RestoreImpulses()
{
    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        normalSolvers[i].impulse = normalSolvers[i].impulseSave;
        tangentSolvers[i].impulse = tangentSolvers[i].impulseSave;
    }
}

inline void BlockSolver::Prepare(const Contact* c)
{
    // Compute Jacobian J and effective mass W
    // J = [-n, -ra1 × n, n, rb1 × n
    //      -n, -ra2 × n, n, rb2 × n]
    // K = (J · M^-1 · J^t)
    // W = K^-1

    ContactJacobian j1 = c->normalSolvers[0].j;
    ContactJacobian j2 = c->normalSolvers[1].j;

    float imA = c->b1->invMass;
    float imB = c->b2->invMass;
    float iiA = c->b1->invInertia;
    float iiB = c->b2->invInertia;

    k[0][0] = imA + imB + j1.wa * iiA * j1.wa + j1.wb * iiB * j1.wb;
    k[1][1] = imA + imB + j2.wa * iiA * j2.wa + j2.wb * iiB * j2.wb;
    k[0][1] = imA + imB + j1.wa * iiA * j2.wa + j1.wb * iiB * j2.wb;
    k[1][0] = k[0][1];

    if (k.GetDeterminant() != 0.0f)
    {
        enabled = true;
        m = k.GetInverse();
    }
    else
    {
        enabled = false;
    }
}

// Solve two contact constraints simultaneously
// https://www.gdcvault.com/play/1020603/Physics-for-Game-Programmers-Understanding
inline void BlockSolver::Solve(Contact* c)
{
    /*
        The comments below are copied from Box2D::b2_contact_solver.cpp
        Check out Box2D: https://box2d.org

        Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
        Build the mini LCP for this contact patch

        vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2

        A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
        b = vn0 - velocityBias

        The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
        implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
        vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
        solution that satisfies the problem is chosen.

        In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only
        requires that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable
        (x_i).

        Substitute:

        x = a + d

        a := old total impulse
        x := new total impulse
        d := incremental impulse

        For the current iteration we extend the formula for the incremental impulse
        to compute the new total impulse:

        vn = A * d + b
            = A * (x - a) + b
            = A * x + b - A * a
            = A * x + b'
        b' = b - A * a;
    */

    ContactSolverNormal* nc1 = &c->normalSolvers[0];
    ContactSolverNormal* nc2 = &c->normalSolvers[1];

    ContactJacobian j1 = nc1->j;
    ContactJacobian j2 = nc2->j;

    Vec2 a{ nc1->impulse, nc2->impulse }; // old total impulse
    MuliAssert(a.x >= 0.0f && a.y >= 0.0f);

    // clang-format off
    // (Velocity constraint) Normal velocity: Jv = 0
    float vn1 = Dot(j1.va, c->b1->linearVelocity)
              + j1.wa * c->b1->angularVelocity
              + Dot(j1.vb, c->b2->linearVelocity)
              + j1.wb * c->b2->angularVelocity;

    float vn2 = Dot(j2.va, c->b1->linearVelocity)
              + j2.wa * c->b1->angularVelocity
              + Dot(j2.vb, c->b2->linearVelocity)
              + j2.wb * c->b2->angularVelocity;
    // clang-format on

    Vec2 b{ vn1 + nc1->bias, vn2 + nc2->bias };

    // b' = b - K * a
    b = b - (k * a);
    Vec2 x{ 0.0f }; // Lambda;

    //
    // Case 1: vn = 0
    // Both constraints are violated
    //
    // 0 = A * x + b'
    //
    // Solve for x:
    //
    // x = - inv(A) * b'
    //
    x = -(m * b);
    if (x.x >= 0.0f && x.y >= 0.0f)
    {
        goto solved;
    }

    //
    // Case 2: vn1 = 0 and x2 = 0
    // The first constraint is violated and the second constraint is satisfied
    //
    //   0 = a11 * x1 + a12 * 0 + b1'
    // vn2 = a21 * x1 + a22 * 0 + b2'
    //
    x.x = nc1->m * -b.x;
    x.y = 0.0f;
    vn1 = 0.0f;
    vn2 = k[0][1] * x.x + b.y;
    if (x.x >= 0.0f && vn2 >= 0.0f)
    {
        goto solved;
    }

    //
    // Case 3: vn2 = 0 and x1 = 0
    // The first constraint is satisfied and the second constraint is violated
    //
    // vn1 = a11 * 0 + a12 * x2 + b1'
    //   0 = a21 * 0 + a22 * x2 + b2'
    //
    x.x = 0.0f;
    x.y = nc2->m * -b.y;
    vn1 = k[1][0] * x.y + b.x;
    vn2 = 0.0f;
    if (x.y >= 0.0f && vn1 >= 0.0f)
    {
        goto solved;
    }

    //
    // Case 4: x1 = 0 and x2 = 0
    // Both constraints are satisfied
    //
    // vn1 = b1
    // vn2 = b2;
    //
    x.x = 0.0f;
    x.y = 0.0f;
    vn1 = b.x;
    vn2 = b.y;
    if (vn1 >= 0.0f && vn2 >= 0.0f)
    {
        goto solved;
    }

// How did you reach here?! something went wrong!
// You can sometimes reach here because of floating point errors :(
#if 0
        MuliAssert(false);
#endif

solved:
    // Get the incremental impulse
    Vec2 d = x - a;

    // Apply incremental impulse
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ
    c->b1->linearVelocity += j1.va * (c->b1->invMass * (d.x + d.y));
    c->b1->angularVelocity += c->b1->invInertia * (j1.wa * d.x + j2.wa * d.y);
    c->b2->linearVelocity += j1.vb * (c->b2->invMass * (d.x + d.y));
    c->b2->angularVelocity += c->b2->invInertia * (j1.wb * d.x + j2.wb * d.y);

    // Accumulate
    nc1->impulse = x.x;
    nc2->impulse = x.y;
}

inline void ContactSolverNormal::Prepare(const Contact* c, const Vec2& normal, int32 index, const Timestep& step)
{
    // Compute Jacobian J and effective mass W
    // J = [-n, -ra × n, n, rb × n]
    // W = (J · M^-1 · J^t)^-1

    Vec2 point = c->manifold.contactPoints[index].p;
    Vec2 ra = point - c->b1->motion.c;
    Vec2 rb = point - c->b2->motion.c;

    // Setup jacobian
    j.va = -normal;
    j.wa = -Cross(ra, normal);
    j.vb = normal;
    j.wb = Cross(rb, normal);

    bias = 0.0f;

    // Relative velocity at contact point
    Vec2 relativeVelocity =
        (c->b2->linearVelocity + Cross(c->b2->angularVelocity, rb)) - (c->b1->linearVelocity + Cross(c->b1->angularVelocity, ra));

    // Normal velocity == veclocity constraint: jv
    float normalVelocity = Dot(c->manifold.contactNormal, relativeVelocity);

#if 1
    if (-normalVelocity > c->restitutionThreshold)
    {
        bias = c->restitution * normalVelocity;
    }
#else
    bias = c->restitution * Min(normalVelocity + c->restitutionThreshold, 0.0f);
#endif

#if 0
    // Position correction by velocity steering
    bias += -position_correction * step.inv_dt * Max(c->manifold.penetrationDepth - linear_slop, 0.0f);
#endif

    // clang-format off
    float k = c->b1->invMass
            + j.wa * c->b1->invInertia * j.wa
            + c->b2->invMass
            + j.wb * c->b2->invInertia * j.wb;
    // clang-format on

    m = k > 0.0f ? 1.0f / k : 0.0f;

    if (step.warm_starting)
    {
        // Warm start
        c->b1->linearVelocity += j.va * (c->b1->invMass * impulse);
        c->b1->angularVelocity += c->b1->invInertia * j.wa * impulse;
        c->b2->linearVelocity += j.vb * (c->b2->invMass * impulse);
        c->b2->angularVelocity += c->b2->invInertia * j.wb * impulse;
    }
}

inline void ContactSolverNormal::Solve(Contact* c)
{
    // Compute corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    // clang-format off
    // Velocity constraint: C' = jv
    // Jacobian * velocity vector (Normal velocity)
    float jv = Dot(j.va, c->b1->linearVelocity)
             + j.wa * c->b1->angularVelocity
             + Dot(j.vb, c->b2->linearVelocity)
             + j.wb * c->b2->angularVelocity;
    // clang-format on

    float lambda = m * -(jv + bias);

    // Clamp impulse correctly and accumulate it
    float oldImpulse = impulse;
    impulse = Max(0.0f, impulse + lambda);
    lambda = impulse - oldImpulse;

    // Apply impulse
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    c->b1->linearVelocity += j.va * (c->b1->invMass * lambda);
    c->b1->angularVelocity += c->b1->invInertia * j.wa * lambda;
    c->b2->linearVelocity += j.vb * (c->b2->invMass * lambda);
    c->b2->angularVelocity += c->b2->invInertia * j.wb * lambda;
}

inline void ContactSolverTangent::Prepare(const Contact* c, const Vec2& tangent, int32 index, const Timestep& step)
{
    // Compute Jacobian J and effective mass W
    // J = [-n, -ra × n, n, rb × n]
    // W = (J · M^-1 · J^t)^-1

    Vec2 point = c->manifold.contactPoints[index].p;
    Vec2 ra = point - c->b1->motion.c;
    Vec2 rb = point - c->b2->motion.c;

    // Setup jacobian
    j.va = -tangent;
    j.wa = -Cross(ra, tangent);
    j.vb = tangent;
    j.wb = Cross(rb, tangent);

    bias = -c->surfaceSpeed;

    // clang-format off
    float k = c->b1->invMass
            + j.wa * c->b1->invInertia * j.wa
            + c->b2->invMass
            + j.wb * c->b2->invInertia * j.wb;
    // clang-format on

    m = k > 0.0f ? 1.0f / k : 0.0f;

    if (step.warm_starting)
    {
        // Warm start
        c->b1->linearVelocity += j.va * (c->b1->invMass * impulse);
        c->b1->angularVelocity += c->b1->invInertia * j.wa * impulse;
        c->b2->linearVelocity += j.vb * (c->b2->invMass * impulse);
        c->b2->angularVelocity += c->b2->invInertia * j.wb * impulse;
    }
}

inline void ContactSolverTangent::Solve(Contact* c, const ContactSolverNormal* normalContact)
{
    // Compute corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    // clang-format off
    // Velocity constraint: C' = jv
    // Jacobian * velocity vector (Normal velocity)
    float jv = Dot(j.va, c->b1->linearVelocity)
             + j.wa * c->b1->angularVelocity
             + Dot(j.vb, c->b2->linearVelocity)
             + j.wb * c->b2->angularVelocity;
    // clang-format on

    float lambda = m * -(jv + bias);

    // Clamp impulse correctly and accumulate it
    float oldImpulse = impulse;
    float maxFriction = c->friction * normalContact->impulse;
    impulse = Clamp(impulse + lambda, -maxFriction, maxFriction);
    lambda = impulse - oldImpulse;

    // Apply impulse
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    c->b1->linearVelocity += j.va * (c->b1->invMass * lambda);
    c->b1->angularVelocity += c->b1->invInertia * j.wa * lambda;
    c->b2->linearVelocity += j.vb * (c->b2->invMass * lambda);
    c->b2->angularVelocity += c->b2->invInertia * j.wb * lambda;
}

inline void PositionSolver::Prepare(Contact* inContact, int32 index)
{
    contact = inContact;

    Transform tfA{ contact->b1->motion.c, contact->b1->motion.a };
    Transform tfB{ contact->b2->motion.c, contact->b2->motion.a };

    localPlainPoint = MulT(tfA, contact->manifold.referencePoint.p);
    localClipPoint = MulT(tfB, contact->manifold.contactPoints[index].p);
    localNormal = MulT(tfA.rotation, contact->manifold.contactNormal);
}

inline bool PositionSolver::Solve()
{
    Transform tfA{ contact->b1->motion.c, contact->b1->motion.a };
    Transform tfB{ contact->b2->motion.c, contact->b2->motion.a };

    Vec2 planePoint = Mul(tfA, localPlainPoint);
    Vec2 clipPoint = Mul(tfB, localClipPoint); // penetration point
    Vec2 normal = Mul(tfA.rotation, localNormal);

    float separation = Dot(clipPoint - planePoint, normal);

    Vec2 ra = clipPoint - tfA.position;
    Vec2 rb = clipPoint - tfB.position;

    float ran = Cross(ra, normal);
    float rbn = Cross(rb, normal);

    // clang-format off
    // effective mass = 1 / k;
    float k = contact->b1->invMass
            + ran * contact->b1->invInertia * ran
            + contact->b2->invMass
            + rbn * contact->b2->invInertia * rbn;
    // clang-format on

    // Constraint (bias)
    float c = Clamp(position_correction * (separation + linear_slop), -max_position_correction, 0.0f);

    // Compute normal impulse
    float lambda = k > 0.0f ? -c / k : 0.0f;
    Vec2 impulse = normal * lambda;

    contact->cLinearImpulseA -= impulse;
    contact->cAngularImpulseA -= Cross(ra, impulse);
    contact->cLinearImpulseB += impulse;
    contact->cAngularImpulseB += Cross(rb, impulse);

    // We can't expect separation >= -linear_slop
    // because we don't push the separation above -linear_slop
    return -separation <= position_solver_threshold;
}

inline bool PositionSolver::SolveTOI()
{
    Transform tfA{ contact->b1->motion.c, contact->b1->motion.a };
    Transform tfB{ contact->b2->motion.c, contact->b2->motion.a };

    Vec2 planePoint = Mul(tfA, localPlainPoint);
    Vec2 clipPoint = Mul(tfB, localClipPoint); // penetration point
    Vec2 normal = Mul(tfA.rotation, localNormal);

    float separation = Dot(clipPoint - planePoint, normal);

    Vec2 ra = clipPoint - tfA.position;
    Vec2 rb = clipPoint - tfB.position;

    float ran = Cross(ra, normal);
    float rbn = Cross(rb, normal);

    // clang-format off
    // effective mass = 1 / k;
    float k = contact->b1->invMass
            + ran * contact->b1->invInertia * ran
            + contact->b2->invMass
            + rbn * contact->b2->invInertia * rbn;
    // clang-format on

    // Constraint (bias)
    float c = Clamp(toi_position_correction * (separation + linear_slop), -max_toi_position_correction, 0.0f);

    // Compute normal impulse
    float lambda = k > 0.0f ? -c / k : 0.0f;
    Vec2 impulse = normal * lambda;

    contact->cLinearImpulseA -= impulse;
    contact->cAngularImpulseA -= Cross(ra, impulse);
    contact->cLinearImpulseB += impulse;
    contact->cAngularImpulseB += Cross(rb, impulse);

    // TOI position solver must push further than the discrete position solver
    return -separation <= toi_position_solver_threshold;
}

} // namespace flywheel
