#include "flywheel/aabb_tree.h"
#include "flywheel/callbacks.h"
#include "flywheel/capsule.h"
#include "flywheel/circle.h"
#include "flywheel/collider.h"
#include "flywheel/contact_graph.h"
#include "flywheel/island.h"
#include "flywheel/polygon.h"
#include "flywheel/rigidbody.h"
#include "flywheel/time_of_impact.h"
#include "flywheel/world.h"

namespace flywheel
{

ContactListener defaultListener;

Collider::Collider()
    : OnDestroy{ nullptr }
    , ContactListener{ &defaultListener }
    , next{ nullptr }
    , node{ AABBTree::nullNode }
    , enabled{ true }
{
}

Collider::~Collider()
{
    if (OnDestroy)
    {
        OnDestroy->OnColliderDestroy(this);
    }

    body = nullptr;
    next = nullptr;
}

void Collider::Create(
    Allocator* allocator, RigidBody* inBody, Shape* inShape, const Transform& tf, float inDensity, const Material& inMaterial
)
{
    body = inBody;
    shape = inShape->Clone(allocator, tf);
    density = inDensity;
    material = inMaterial;
}

void Collider::Destroy(Allocator* allocator)
{
    shape->~Shape();

    switch (shape->GetType())
    {
    case Shape::Type::circle:
        allocator->Free(shape, sizeof(Circle));
        break;
    case Shape::Type::capsule:
        allocator->Free(shape, sizeof(Capsule));
        break;
    case Shape::Type::polygon:
        allocator->Free(shape, sizeof(Polygon));
        break;
    default:
        MuliAssert(false);
        break;
    }

    shape = nullptr;
}

} // namespace flywheel
// ============================================================================
// ContactGraph implementation
// ============================================================================

namespace flywheel
{

extern void InitializeDetectionFunctionMap();

ContactGraph::ContactGraph(World* world)
    : world{ world }
    , broadPhase{ this }
    , contactList{ nullptr }
    , contactCount{ 0 }
{
    InitializeDetectionFunctionMap();
}

ContactGraph::~ContactGraph()
{
    MuliAssert(contactList == nullptr);
}

void ContactGraph::EvaluateContacts()
{
    // Narrow phase
    // Evaluate contacts, prepare for solving step
    Contact* c = contactList;
    while (c)
    {
        Collider* colliderA = c->colliderA;
        Collider* colliderB = c->colliderB;

        RigidBody* bodyA = c->bodyA;
        RigidBody* bodyB = c->bodyB;

        bool activeA = bodyA->IsSleeping() == false && bodyA->GetType() != RigidBody::static_body;
        bool activeB = bodyB->IsSleeping() == false && bodyB->GetType() != RigidBody::static_body;

        if (activeA == false && activeB == false)
        {
            c = c->next;
            continue;
        }

        bool overlap = broadPhase.TestOverlap(colliderA, colliderB);

        // This potential contact that is configured by aabb overlap is no longer valid so destroy it
        if (overlap == false)
        {
            Contact* t = c;
            c = c->next;
            Destroy(t);
            continue;
        }

        // Evaluate the contact, prepare the solve step
        c->Update();
        c = c->next;
    }
}

void ContactGraph::OnNewContact(Collider* colliderA, Collider* colliderB)
{
    RigidBody* bodyA = colliderA->body;
    RigidBody* bodyB = colliderB->body;

    MuliAssert(bodyA != bodyB);
    MuliAssert(colliderA->GetType() >= colliderB->GetType());

    if (bodyA->GetType() != RigidBody::dynamic_body && bodyB->GetType() != RigidBody::dynamic_body)
    {
        return;
    }

    if (EvaluateFilter(colliderA->GetFilter(), colliderB->GetFilter()) == false)
    {
        return;
    }

    // TODO: Use hash set to remove potential bottleneck
    ContactEdge* e = bodyB->contactList;
    while (e)
    {
        if (e->other == bodyA)
        {
            Collider* ceA = e->contact->colliderA;
            Collider* ceB = e->contact->colliderB;

            // This contact already exists
            if ((colliderA == ceA && colliderB == ceB) || (colliderA == ceB && colliderB == ceA))
            {
                return;
            }
        }

        e = e->next;
    }

    // Create new contact
    void* mem = world->blockAllocator.Allocate(sizeof(Contact));
    Contact* c = new (mem) Contact(colliderA, colliderB);

    // Insert into the world
    c->prev = nullptr;
    c->next = contactList;
    if (contactList != nullptr)
    {
        contactList->prev = c;
    }
    contactList = c;

    // Connect to island graph

    // Connect contact edge to body A
    c->nodeA.contact = c;
    c->nodeA.other = bodyB;

    c->nodeA.prev = nullptr;
    c->nodeA.next = bodyA->contactList;
    if (bodyA->contactList != nullptr)
    {
        bodyA->contactList->prev = &c->nodeA;
    }
    bodyA->contactList = &c->nodeA;

    // Connect contact edge to body B
    c->nodeB.contact = c;
    c->nodeB.other = bodyA;

    c->nodeB.prev = nullptr;
    c->nodeB.next = bodyB->contactList;
    if (bodyB->contactList != nullptr)
    {
        bodyB->contactList->prev = &c->nodeB;
    }
    bodyB->contactList = &c->nodeB;

    ++contactCount;
}

void ContactGraph::Destroy(Contact* c)
{
    RigidBody* bodyA = c->bodyA;
    RigidBody* bodyB = c->bodyB;

    // Remove from the world
    if (c->prev) c->prev->next = c->next;
    if (c->next) c->next->prev = c->prev;
    if (c == contactList) contactList = c->next;

    // Remove from bodyA
    if (c->nodeA.prev) c->nodeA.prev->next = c->nodeA.next;
    if (c->nodeA.next) c->nodeA.next->prev = c->nodeA.prev;
    if (&c->nodeA == bodyA->contactList) bodyA->contactList = c->nodeA.next;

    // Remove from bodyB
    if (c->nodeB.prev) c->nodeB.prev->next = c->nodeB.next;
    if (c->nodeB.next) c->nodeB.next->prev = c->nodeB.prev;
    if (&c->nodeB == bodyB->contactList) bodyB->contactList = c->nodeB.next;

    c->~Contact();
    world->blockAllocator.Free(c, sizeof(Contact));
    --contactCount;
}

void ContactGraph::AddCollider(Collider* collider)
{
    broadPhase.Add(collider, collider->GetAABB());
}

void ContactGraph::RemoveCollider(Collider* collider)
{
    broadPhase.Remove(collider);
    collider->node = AABBTree::nullNode;

    RigidBody* body = collider->body;

    // Destroy any contacts associated with the collider
    ContactEdge* edge = body->contactList;
    while (edge)
    {
        Contact* contact = edge->contact;
        edge = edge->next;

        Collider* colliderA = contact->GetColliderA();
        Collider* colliderB = contact->GetColliderB();

        if (collider == colliderA || collider == colliderB)
        {
            Destroy(contact);

            colliderA->body->Awake();
            colliderB->body->Awake();
        }
    }
}

void ContactGraph::UpdateCollider(Collider* collider, const Transform& tf)
{
    const Shape* shape = collider->GetShape();
    AABB aabb;
    shape->ComputeAABB(tf, &aabb);

    broadPhase.Update(collider, aabb, Vec2::zero);
}

void ContactGraph::UpdateCollider(Collider* collider, const Transform& tf0, const Transform& tf1)
{
    const Shape* shape = collider->GetShape();

    AABB aabb0, aabb1;
    shape->ComputeAABB(tf0, &aabb0);
    shape->ComputeAABB(tf1, &aabb1);

    Vec2 prediction = aabb1.GetCenter() - aabb0.GetCenter();
#if 0
    aabb0.min += prediction;
    aabb0.max += prediction;
#endif
    aabb1.min += prediction;
    aabb1.max += prediction;

    broadPhase.Update(collider, AABB::Union(aabb0, aabb1), prediction);
}

} // namespace flywheel

// ============================================================================
// Island implementation
// ============================================================================

#define SOLVE_CONTACT_CONSTRAINT 1

namespace flywheel
{

Island::Island(World* world, int32 bodyCapacity, int32 contactCapacity, int32 jointCapacity)
    : world{ world }
    , bodyCapacity{ bodyCapacity }
    , contactCapacity{ contactCapacity }
    , jointCapacity{ jointCapacity }
    , bodyCount{ 0 }
    , contactCount{ 0 }
    , jointCount{ 0 }
    , sleeping{ false }
{
    bodies = (RigidBody**)world->linearAllocator.Allocate(bodyCapacity * sizeof(RigidBody*));
    contacts = (Contact**)world->linearAllocator.Allocate(contactCapacity * sizeof(Contact*));
    joints = (Joint**)world->linearAllocator.Allocate(jointCapacity * sizeof(Joint*));
}

Island::~Island()
{
    world->linearAllocator.Free(joints, jointCapacity * sizeof(Joint*));
    world->linearAllocator.Free(contacts, contactCapacity * sizeof(Contact*));
    world->linearAllocator.Free(bodies, bodyCapacity * sizeof(RigidBody*));
}

void Island::Solve()
{
    bool awakeIsland = false;

    const WorldSettings& settings = world->settings;
    const Timestep& step = settings.step;

    // Integrate velocities, yield tentative velocities that possibly violate the constraint
    for (int32 i = 0; i < bodyCount; ++i)
    {
        RigidBody* b = bodies[i];

        // Save positions for continuous collision
        b->motion.c0 = b->motion.c;
        b->motion.a0 = b->motion.a;

        // All bodies on this island are resting more than sleep time, but flags are not set
        if (sleeping)
        {
            b->islandID = 0;
            b->islandIndex = 0;
            b->linearVelocity.SetZero();
            b->angularVelocity = 0.0f;
            b->flag |= RigidBody::flag_sleeping;
        }
        else
        {
            b->flag &= ~RigidBody::flag_sleeping;
        }

        if ((b->angularVelocity * b->angularVelocity > settings.rest_angular_tolerance) ||
            (Dot(b->linearVelocity, b->linearVelocity) > settings.rest_linear_tolerance) || (b->torque * b->torque > 0.0f) ||
            (Dot(b->force, b->force) > 0.0f))
        {
            MuliAssert(sleeping == false);
            awakeIsland = true;
        }
        else
        {
            b->resting += step.dt;
        }

        if (b->type == RigidBody::dynamic_body)
        {
            // Integrate velocites
            b->linearVelocity += b->invMass * step.dt * (b->force + settings.apply_gravity * settings.gravity * b->mass);
            b->angularVelocity += b->invInertia * step.dt * b->torque;

            /*
               Apply damping (found in box2d)
               ODE: dv/dt + c * v = 0
               dv/dt = -c * v
               (1/v) dv = -c dt ; integrate both sides
               ln|v| = -c * t + C ; exponentiate both sides (C is integration constant)
               v = C * exp(-c * t)
               v(0) = C
               Solution: v(t) = v0 * exp(-c * t)
               Time step: v(t + dt) = v0 * exp(-c * (t + dt))
                                    = v0 * exp(-c * t) * exp(-c * dt)
                                    = v * exp(-c * dt)
               v2 = exp(-c * dt) * v1
               Pade approximation:
               v2 = v1 * 1 / (1 + c * dt)
            */
            b->linearVelocity *= 1.0f / (1.0f + b->linearDamping * step.dt);
            b->angularVelocity *= 1.0f / (1.0f + b->angularDamping * step.dt);
        }
    }

    // Prepare constraints for solving step
    for (int32 i = 0; i < contactCount; ++i)
    {
        contacts[i]->Prepare(step);
    }
    for (int32 i = 0; i < jointCount; ++i)
    {
        joints[i]->Prepare(step);
    }

    // Iteratively solve the violated velocity constraints
    // Solving contacts backward converge fast
    for (int32 i = 0; i < step.velocity_iterations; ++i)
    {
#if SOLVE_CONTACTS_BACKWARD
#if SOLVE_CONTACT_CONSTRAINT
        for (int32 j = contactCount; j > 0; j--)
        {
            contacts[j - 1]->SolveVelocityConstraints(step);
        }
#endif
        for (int32 j = jointCount; j > 0; j--)
        {
            joints[j - 1]->SolveVelocityConstraints(step);
        }
#else
#if SOLVE_CONTACT_CONSTRAINT
        for (int32 j = 0; j < contactCount; ++j)
        {
            contacts[j]->SolveVelocityConstraints(step);
        }
#endif
        for (int32 j = 0; j < jointCount; ++j)
        {
            joints[j]->SolveVelocityConstraints(step);
        }
#endif
    }

    // Update positions using corrected velocities (Semi-implicit euler integration)
    for (int32 i = 0; i < bodyCount; ++i)
    {
        RigidBody* b = bodies[i];

        if (awakeIsland)
        {
            b->Awake();
        }

        b->force.SetZero();
        b->torque = 0.0f;

        b->motion.c += b->linearVelocity * step.dt;
        b->motion.a += b->angularVelocity * step.dt;

        if (settings.world_bounds.TestPoint(b->GetPosition()) == false)
        {
            world->BufferDestroy(b);
        }
    }

    // Solve position constraints
    for (int32 i = 0; i < settings.step.position_iterations; ++i)
    {
        bool contactSolved = true;
        bool jointSolved = true;

#if SOLVE_CONTACTS_BACKWARD
#if SOLVE_CONTACT_CONSTRAINT
        for (int32 j = contactCount; j > 0; j--)
        {
            Contact* c = contacts[j - 1];

            bool solved = c->SolvePositionConstraints(step);
            if (solved == false)
            {
                c->b1->Awake();
                c->b2->Awake();
            }

            contactSolved &= solved;
        }
#endif
        for (int32 j = jointCount; j > 0; j--)
        {
            jointSolved &= joints[j - 1]->SolvePositionConstraints(step);
        }
#else
#if SOLVE_CONTACT_CONSTRAINT
        for (int32 j = 0; j < contactCount; ++j)
        {
            Contact* c = contacts[j];

            bool solved = c->SolvePositionConstraints(step);
            if (solved == false)
            {
                c->b1->Awake();
                c->b2->Awake();
            }

            contactSolved &= solved;
        }
#endif
        for (int32 j = 0; j < jointCount; ++j)
        {
            jointSolved &= joints[j]->SolvePositionConstraints(step);
        }
#endif
        if (contactSolved && jointSolved)
        {
            break;
        }
    }

    for (int32 i = 0; i < contactCount; ++i)
    {
        Contact* contact = contacts[i];

        Collider* colliderA = contact->colliderA;
        Collider* colliderB = contact->colliderB;

        if (colliderA->ContactListener) colliderA->ContactListener->OnPostSolve(colliderA, colliderB, contact);
        if (colliderB->ContactListener) colliderB->ContactListener->OnPostSolve(colliderB, colliderA, contact);
    }
}

static constexpr int32 toi_postion_iteration = 20;
static constexpr int32 toi_index_1 = 0;
static constexpr int32 toi_index_2 = 1;

void Island::SolveTOI(float dt)
{
    Timestep& step = world->settings.step;
    bool warmStartingEnabled = step.warm_starting;

    step.warm_starting = false;

    for (int32 i = 0; i < contactCount; ++i)
    {
        // Save the impulses computed by the discrete solver
        contacts[i]->SaveImpulses();
        contacts[i]->Prepare(step);
    }

    // Move the TOI contact to a safe position so that the next ComputeTimeOfImpact() returns the separated state
    for (int32 i = 0; i < toi_postion_iteration; ++i)
    {
        bool solved = true;

        for (int32 j = 0; j < contactCount; ++j)
        {
            solved &= contacts[j]->SolveTOIPositionConstraints();
        }

        if (solved)
        {
            break;
        }
    }

    bodies[toi_index_1]->motion.c0 = bodies[toi_index_1]->motion.c;
    bodies[toi_index_1]->motion.a0 = bodies[toi_index_1]->motion.a;
    bodies[toi_index_2]->motion.c0 = bodies[toi_index_2]->motion.c;
    bodies[toi_index_2]->motion.a0 = bodies[toi_index_2]->motion.a;

    for (int32 i = 0; i < step.velocity_iterations; ++i)
    {
#if SOLVE_CONTACTS_BACKWARD
#if SOLVE_CONTACT_CONSTRAINT
        for (int32 j = contactCount; j > 0; j--)
        {
            contacts[j - 1]->SolveVelocityConstraints(step);
        }
#endif
#else
#if SOLVE_CONTACT_CONSTRAINT
        for (int32 j = 0; j < contactCount; ++j)
        {
            contacts[j]->SolveVelocityConstraints(step);
        }
#endif
#endif
    }

    // We don't need position correction
    // Because we solved velocity constraints in a position that is already safe

    for (int32 i = 0; i < bodyCount; ++i)
    {
        RigidBody* b = bodies[i];

        b->motion.c += b->linearVelocity * dt;
        b->motion.a += b->angularVelocity * dt;
        b->SynchronizeTransform();
    }

    step.warm_starting = warmStartingEnabled;

    for (int32 i = 0; i < contactCount; ++i)
    {
        Contact* contact = contacts[i];

        Collider* colliderA = contact->colliderA;
        Collider* colliderB = contact->colliderB;

        if (colliderA->ContactListener) colliderA->ContactListener->OnPostSolve(colliderA, colliderB, contact);
        if (colliderB->ContactListener) colliderB->ContactListener->OnPostSolve(colliderB, colliderA, contact);

        // Restore the impulses
        contact->RestoreImpulses();
    }
}

} // namespace flywheel
// ============================================================================
// RigidBody implementation
// ============================================================================


namespace flywheel
{

RigidBody::RigidBody(const Transform& tf, RigidBody::Type type)
    : OnDestroy{ nullptr }
    , UserData{ nullptr }
    , type{ type }
    , transform{ tf }
    , motion{ tf }
    , force{ 0.0f }
    , torque{ 0.0f }
    , linearVelocity{ 0.0f }
    , angularVelocity{ 0.0f }
    , mass{ 0.0f }
    , invMass{ 0.0f }
    , inertia{ 0.0f }
    , invInertia{ 0.0f }
    , linearDamping{ 0.0f }
    , angularDamping{ 0.0f }
    , islandIndex{ 0 }
    , islandID{ 0 }
    , flag{ flag_enabled }
    , world{ nullptr }
    , prev{ nullptr }
    , next{ nullptr }
    , colliderList{ nullptr }
    , colliderCount{ 0 }
    , contactList{ nullptr }
    , jointList{ nullptr }
    , resting{ 0.0f }
{
}

RigidBody::~RigidBody()
{
    if (OnDestroy)
    {
        OnDestroy->OnBodyDestroy(this);
    }

    world = nullptr;
}

Collider* RigidBody::CreateCollider(Shape* shape, const Transform& tf, float density, const Material& material)
{
    MuliAssert(world != nullptr);
    if (world == nullptr)
    {
        return nullptr;
    }

    Allocator* allocator = &world->blockAllocator;
    void* mem = allocator->Allocate(sizeof(Collider));

#if 1
    // Shape radius(skin) must be greater than or equal to linear_slop * 2.0 for stable CCD
    MuliAssert(shape->radius >= linear_slop * 2.0f);
#endif

    Collider* collider = new (mem) Collider;
    collider->Create(allocator, this, shape, tf, density, material);

    collider->next = colliderList;
    colliderList = collider;
    ++colliderCount;

    world->contactGraph.AddCollider(collider);

    ResetMassData();

    return collider;
}

void RigidBody::DestroyCollider(Collider* collider)
{
    if (collider == nullptr)
    {
        return;
    }

    MuliAssert(collider->body == this);
    MuliAssert(colliderCount > 0);

    // Remove collider from collider list
    Collider** c = &colliderList;
    while (*c)
    {
        if (*c == collider)
        {
            *c = collider->next;
            break;
        }

        c = &((*c)->next);
    }

    // Remove collider from contact manager(broad phase)
    world->contactGraph.RemoveCollider(collider);

    Allocator* allocator = &world->blockAllocator;

    collider->~Collider();
    collider->Destroy(allocator);
    allocator->Free(collider, sizeof(Collider));

    --colliderCount;

    ResetMassData();
}

Collider* RigidBody::CreateCircleCollider(float radius, const Transform& tf, float density, const Material& material)
{
    Circle circle{ radius };
    return CreateCollider(&circle, tf, density, material);
}

Collider* RigidBody::CreateBoxCollider(
    float width, float height, float radius, const Transform& tf, float density, const Material& material
)
{
    Polygon box{ width, height, radius };
    return CreateCollider(&box, tf, density, material);
}

Collider* RigidBody::CreateCapsuleCollider(
    float length, float radius, bool horizontal, const Transform& tf, float density, const Material& material
)
{
    Capsule capsule{ length, radius, horizontal };
    return CreateCollider(&capsule, tf, density, material);
}

Collider* RigidBody::CreateCapsuleCollider(
    const Vec2& p1, const Vec2& p2, float radius, bool resetPosition, const Transform& tf, float density, const Material& material
)
{
    Capsule capsule{ p1, p2, radius, resetPosition };
    return CreateCollider(&capsule, tf, density, material);
}

bool RigidBody::TestPoint(const Vec2& p) const
{
    MuliAssert(colliderCount > 0);

    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        if (collider->TestPoint(p))
        {
            return true;
        }
    }

    return false;
}

Vec2 RigidBody::GetClosestPoint(const Vec2& p) const
{
    MuliAssert(colliderCount > 0);

    Vec2 cp0 = colliderList->GetClosestPoint(p);
    if (cp0 == p)
    {
        return cp0;
    }

    float d0 = Dist2(cp0, p);

    for (Collider* collider = colliderList->next; collider; collider = collider->next)
    {
        Vec2 cp1 = collider->GetClosestPoint(p);
        if (cp1 == p)
        {
            return cp1;
        }

        float d1 = Dist2(cp1, p);
        if (d1 < d0)
        {
            cp0 = cp1;
        }
    }

    return cp0;
}

void RigidBody::RayCastAny(const Vec2& from, const Vec2& to, float radius, RayCastAnyCallback* callback) const
{
    RayCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;
    input.radius = radius;

    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        RayCastOutput output;

        bool hit = collider->RayCast(input, &output);
        if (hit)
        {
            float fraction = output.fraction;
            Vec2 point = (1.0f - fraction) * input.from + fraction * input.to;

            input.maxFraction = callback->OnHitAny(collider, point, output.normal, fraction);
        }

        if (input.maxFraction <= 0)
        {
            return;
        }
    }
}

bool RigidBody::RayCastClosest(const Vec2& from, const Vec2& to, float radius, RayCastClosestCallback* callback) const
{
    struct TempCallback : public RayCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider = nullptr;
        Vec2 closestPoint;
        Vec2 closestNormal;
        float closestFraction = 1.0f;

        float OnHitAny(Collider* collider, Vec2 point, Vec2 normal, float fraction)
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestFraction = fraction;

            return fraction;
        }
    } tempCallback;

    RayCastAny(from, to, radius, &tempCallback);

    if (tempCallback.hit)
    {
        callback->OnHitClosest(
            tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal, tempCallback.closestFraction
        );
        return true;
    }

    return false;
}

void RigidBody::RayCastAny(
    const Vec2& from,
    const Vec2& to,
    float radius,
    std::function<float(Collider* collider, Vec2 point, Vec2 normal, float fraction)> callback
) const
{
    RayCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;
    input.radius = radius;

    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        RayCastOutput output;

        bool hit = collider->RayCast(input, &output);
        if (hit)
        {
            float fraction = output.fraction;
            Vec2 point = (1.0f - fraction) * input.from + fraction * input.to;

            input.maxFraction = callback(collider, point, output.normal, fraction);
        }

        if (input.maxFraction <= 0)
        {
            return;
        }
    }
}

bool RigidBody::RayCastClosest(
    const Vec2& from,
    const Vec2& to,
    float radius,
    std::function<void(Collider* collider, Vec2 point, Vec2 normal, float fraction)> callback
) const
{
    struct TempCallback : public RayCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider = nullptr;
        Vec2 closestPoint;
        Vec2 closestNormal;
        float closestFraction = 1.0f;

        float OnHitAny(Collider* collider, Vec2 point, Vec2 normal, float fraction)
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestFraction = fraction;

            return fraction;
        }
    } tempCallback;

    RayCastAny(from, to, radius, &tempCallback);

    if (tempCallback.hit)
    {
        callback(
            tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal, tempCallback.closestFraction
        );
        return true;
    }

    return false;
}

void RigidBody::SetType(RigidBody::Type newType)
{
    if (type == newType)
    {
        return;
    }

    type = newType;

    ResetMassData();

    force.SetZero();
    torque = 0.0f;
    if (type == Type::static_body)
    {
        linearVelocity.SetZero();
        angularVelocity = 0.0f;
        motion.c0 = motion.c;
        motion.a0 = motion.a;
        SynchronizeColliders();
    }

    Awake();

    // Refresh the broad phase contacts
    ContactEdge* ce = contactList;
    while (ce)
    {
        ContactEdge* ce0 = ce;
        ce = ce->next;
        world->contactGraph.Destroy(ce0->contact);
    }
    contactList = nullptr;

    for (Collider* c = colliderList; c; c = c->next)
    {
        world->contactGraph.broadPhase.Refresh(c);
    }

    islandID = 0;
    islandIndex = 0;
}

void RigidBody::SetEnabled(bool enabled)
{
    if (enabled == IsEnabled())
    {
        return;
    }

    if (enabled)
    {
        flag |= flag_enabled;

        for (Collider* c = colliderList; c; c = c->next)
        {
            world->contactGraph.broadPhase.Add(c, c->GetAABB());
        }
    }
    else
    {
        flag &= ~flag_enabled;

        ContactEdge* ce = contactList;
        while (ce)
        {
            ContactEdge* ce0 = ce;
            ce = ce->next;
            world->contactGraph.Destroy(ce0->contact);
        }
        contactList = nullptr;

        for (Collider* c = colliderList; c; c = c->next)
        {
            world->contactGraph.broadPhase.Remove(c);
        }

        islandID = 0;
        islandIndex = 0;
    }
}

void RigidBody::SetCollisionFilter(const CollisionFilter& filter) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetFilter(filter);
    }
}

void RigidBody::SetFriction(float friction) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetFriction(friction);
    }
}

void RigidBody::SetRestitution(float restitution) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetRestitution(restitution);
    }
}

void RigidBody::SetRestitutionThreshold(float threshold) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetRestitutionTreshold(threshold);
    }
}

void RigidBody::SetSurfaceSpeed(float surfaceSpeed) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetSurfaceSpeed(surfaceSpeed);
    }
}

void RigidBody::ResetMassData()
{
    mass = 0.0f;
    invMass = 0.0f;
    inertia = 0.0f;
    invInertia = 0.0f;

    if (type == static_body || type == kinematic_body)
    {
        return;
    }

    if (colliderCount <= 0)
    {
        return;
    }

    Vec2 localCenter{ 0.0f };

    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        MassData massData = collider->GetMassData();
        mass += massData.mass;
        localCenter += massData.mass * massData.centerOfMass;
        inertia += massData.inertia;
    }

    if (mass > 0.0f)
    {
        invMass = 1.0f / mass;
        localCenter *= invMass;
    }

    if (inertia > 0.0f && (flag & flag_fixed_rotation) == 0)
    {
        // Center the inertia about the center of mass
        inertia -= mass * Length2(localCenter);
        invInertia = 1.0f / inertia;
    }
    else
    {
        inertia = 0.0f;
        invInertia = 0.0f;
    }

    Vec2 oldPosition = motion.c;
    motion.localCenter = localCenter;
    motion.c = Mul(transform, motion.localCenter);
    motion.c0 = motion.c;

    linearVelocity += Cross(angularVelocity, motion.c - oldPosition);
}

void RigidBody::SynchronizeColliders()
{
    if (IsSleeping())
    {
        for (Collider* collider = colliderList; collider; collider = collider->next)
        {
            world->contactGraph.UpdateCollider(collider, transform);
        }
    }
    else
    {
        // Transform at previus step
        Transform tf0;
        motion.GetTransform(0.0f, &tf0);

        for (Collider* collider = colliderList; collider; collider = collider->next)
        {
            world->contactGraph.UpdateCollider(collider, tf0, transform);
        }
    }
}

} // namespace flywheel