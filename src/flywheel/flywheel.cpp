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
// ============================================================================
// World implementation
// ============================================================================


namespace flywheel
{

World::World(const WorldSettings& settings)
    : settings{ settings }
    , contactGraph{ this }
    , bodyList{ nullptr }
    , bodyListTail{ nullptr }
    , bodyCount{ 0 }
    , jointList{ nullptr }
    , jointCount{ 0 }
    , islandCount{ 0 }
    , sleepingBodyCount{ 0 }
    , stepComplete{ true }
{
    // Assertions for stable CCD
    MuliAssert(toi_position_solver_threshold < linear_slop * 2.0f);
    MuliAssert(default_radius >= toi_position_solver_threshold);
    MuliAssert(position_solver_threshold > toi_position_solver_threshold);
}

World::~World()
{
    Reset();
}

void World::Reset()
{
    RigidBody* b = bodyList;
    while (b)
    {
        RigidBody* b0 = b;
        b = b->next;
        Destroy(b0);
    }

    MuliAssert(bodyList == nullptr);
    MuliAssert(bodyListTail == nullptr);
    MuliAssert(jointList == nullptr);
    MuliAssert(bodyCount == 0);
    MuliAssert(jointCount == 0);
    MuliAssert(blockAllocator.GetBlockCount() == 0);

    destroyBodyBuffer.clear();
    destroyJointBuffer.clear();
}

void World::Solve()
{
    // Build the constraint island
    Island island{ this, bodyCount, contactGraph.contactCount, jointCount };

    int32 restingBodies = 0;
    int32 islandID = 0;
    sleepingBodyCount = 0;

    // Use arena allocator to avoid per-frame allocation
    RigidBody** stack = (RigidBody**)linearAllocator.Allocate(bodyCount * sizeof(RigidBody*));
    int32 stackPointer;

    // Perform a DFS(Depth First Search) on the constraint graph
    // After building island, each island can be solved in parallel because they are independent of each other
    for (RigidBody* b = bodyList; b; b = b->next)
    {
        if (b->flag & RigidBody::flag_island)
        {
            continue;
        }

        if (b->IsSleeping() == true)
        {
            ++sleepingBodyCount;
            continue;
        }

        if (b->type == RigidBody::static_body)
        {
            continue;
        }

        if (b->IsEnabled() == false)
        {
            continue;
        }

        stackPointer = 0;
        stack[stackPointer++] = b;
        b->flag |= RigidBody::flag_island;

        ++islandID;
        while (stackPointer > 0)
        {
            RigidBody* t = stack[--stackPointer];

            island.Add(t);
            t->islandID = islandID;

            for (ContactEdge* ce = t->contactList; ce; ce = ce->next)
            {
                Contact* c = ce->contact;

                if (c->flag & Contact::flag_island)
                {
                    continue;
                }

                if ((c->flag & Contact::flag_touching) == 0)
                {
                    continue;
                }

                if ((c->flag & Contact::flag_enabled) == 0)
                {
                    continue;
                }

                island.Add(c);
                c->flag |= Contact::flag_island;

                RigidBody* other = ce->other;

                if (other->flag & RigidBody::flag_island)
                {
                    continue;
                }

                if (other->type == RigidBody::static_body)
                {
                    continue;
                }

                MuliAssert(stackPointer < bodyCount);
                stack[stackPointer++] = other;
                other->flag |= RigidBody::flag_island;
            }

            for (JointEdge* je = t->jointList; je; je = je->next)
            {
                Joint* j = je->joint;

                if (j->flagIsland == true)
                {
                    continue;
                }

                RigidBody* other = je->other;

                if (other->IsEnabled() == false)
                {
                    continue;
                }

                island.Add(j);
                j->flagIsland = true;

                if (other->flag & RigidBody::flag_island)
                {
                    continue;
                }

                if (other->type == RigidBody::static_body)
                {
                    continue;
                }

                MuliAssert(stackPointer < bodyCount);
                stack[stackPointer++] = other;
                other->flag |= RigidBody::flag_island;
            }

            if (t->resting > settings.sleeping_time)
            {
                restingBodies++;
            }
        }

        island.sleeping = settings.sleeping && (restingBodies == island.bodyCount);
        island.Solve();
        island.Clear();
        restingBodies = 0;
    }

    linearAllocator.Free(stack, bodyCount * sizeof(RigidBody*));

    islandCount = islandID;

    for (RigidBody* body = bodyList; body; body = body->next)
    {
        MuliAssert(body->motion.alpha0 == 0.0f);

        if ((body->flag & RigidBody::flag_island) == 0)
        {
            continue;
        }

        MuliAssert(body->type != RigidBody::static_body);

        // Clear island flag
        body->flag &= ~RigidBody::flag_island;

        // Synchronize transform and broad-phase collider node
        body->SynchronizeTransform();
        body->SynchronizeColliders();
    }

    for (Contact* contact = contactGraph.contactList; contact; contact = contact->next)
    {
        contact->flag &= ~Contact::flag_island;
    }

    for (Joint* joint = jointList; joint; joint = joint->next)
    {
        joint->flagIsland = false;
    }
}

// Find TOI contacts and solve them
float World::SolveTOI()
{
    Island island{ this, 2 * max_toi_contacts, max_toi_contacts, 0 };

    while (true)
    {
        contactGraph.UpdateContactGraph();

        // Find the first TOI
        Contact* minContact = nullptr;
        float minAlpha = 1.0f;

        for (Contact* c = contactGraph.contactList; c; c = c->next)
        {
            if (c->IsEnabled() == false)
            {
                continue;
            }

            if (c->toiCount > max_sub_steps)
            {
                continue;
            }

            float alpha = 1.0f;

            if (c->flag & Contact::flag_toi)
            {
                // This contact has a cached TOI
                alpha = c->toi;
            }
            else
            {
                Collider* colliderA = c->colliderA;
                Collider* colliderB = c->colliderB;

                if (colliderA->IsEnabled() == false || colliderB->IsEnabled() == false)
                {
                    continue;
                }

                RigidBody* bodyA = colliderA->body;
                RigidBody* bodyB = colliderB->body;

                RigidBody::Type typeA = bodyA->type;
                RigidBody::Type typeB = bodyB->type;
                MuliAssert(typeA == RigidBody::dynamic_body || typeB == RigidBody::dynamic_body);

                bool activeA = bodyA->IsSleeping() == false && typeA != RigidBody::static_body;
                bool activeB = bodyB->IsSleeping() == false && typeB != RigidBody::static_body;

                // Is at least one body active (awake and dynamic or kinematic)?
                if (activeA == false && activeB == false)
                {
                    continue;
                }

                bool collideA = bodyA->IsContinuous() || typeA == RigidBody::static_body;
                bool collideB = bodyB->IsContinuous() || typeB == RigidBody::static_body;

                // Discard non-continuous dynamic|kinematic vs. non-continuous dynamic|kinematic case
                if (collideA == false && collideB == false)
                {
                    continue;
                }

                // Compute the TOI for this contact

                // Put the motions onto the same time interval
                float alpha0 = bodyA->motion.alpha0;
                if (bodyA->motion.alpha0 < bodyB->motion.alpha0)
                {
                    alpha0 = bodyB->motion.alpha0;
                    bodyA->motion.Advance(alpha0);
                }
                else if (bodyA->motion.alpha0 > bodyB->motion.alpha0)
                {
                    alpha0 = bodyA->motion.alpha0;
                    bodyB->motion.Advance(alpha0);
                }

                MuliAssert(alpha0 < 1.0f);

                TOIOutput output;
                ComputeTimeOfImpact(colliderA->shape, bodyA->motion, colliderB->shape, bodyB->motion, 1.0f, &output);

#if 0
                switch (output.state)
                {
                case TOIOutput::unknown:
                    std::cout << "unknown" << std::endl;
                    break;
                case TOIOutput::failed:
                    std::cout << "failed" << std::endl;
                    break;
                case TOIOutput::overlapped:
                    std::cout << "overlapped" << std::endl;
                    break;
                case TOIOutput::touching:
                    std::cout << "touching: " << output.t << std::endl;
                    break;
                case TOIOutput::separated:
                    std::cout << "separated" << std::endl;
                    break;

                default:
                    MuliAssert(false);
                    break;
                }
#endif

                if (output.state == TOIOutput::touching)
                {
                    // TOI is the fraction in [alpha0, 1.0]
                    alpha = Min(alpha0 + (1.0f - alpha0) * output.t, 1.0f);
                }
                else
                {
                    alpha = 1.0f;
                }

                // Save the TOI
                c->toi = alpha;
                c->flag |= Contact::flag_toi;
            }

            if (alpha < minAlpha)
            {
                // Update the minimum TOI
                minContact = c;
                minAlpha = alpha;
            }
        }

        if (minContact == nullptr || 1.0f - 10.0f * epsilon < minAlpha)
        {
            // Done! No more TOI events
            stepComplete = true;
            break;
        }

        // Advance the bodies to the TOI
        Collider* colliderA = minContact->colliderA;
        Collider* colliderB = minContact->colliderB;
        RigidBody* bodyA = colliderA->body;
        RigidBody* bodyB = colliderB->body;

        Motion save1 = bodyA->motion;
        Motion save2 = bodyB->motion;

        bodyA->Advance(minAlpha);
        bodyB->Advance(minAlpha);

        // Find the TOI contact points
        minContact->Update();
        minContact->flag &= ~Contact::flag_toi;
        ++minContact->toiCount;

        // Contact disabled by the user or no contact points found
        if (minContact->IsEnabled() == false || minContact->IsTouching() == false)
        {
            // Restore the motions
            minContact->SetEnabled(false); // Prevent duplicate
            bodyA->motion = save1;
            bodyB->motion = save2;
            bodyA->SynchronizeTransform();
            bodyB->SynchronizeTransform();
            continue;
        }

        bodyA->Awake();
        bodyB->Awake();

        // Build the island
        island.Clear();
        island.Add(bodyA);
        island.Add(bodyB);
        island.Add(minContact);

        bodyA->flag |= RigidBody::flag_island;
        bodyB->flag |= RigidBody::flag_island;
        minContact->flag |= Contact::flag_island;

        // Find contacts for TOI contact bodies
        RigidBody* bodies[2] = { bodyA, bodyB };
        for (int32 i = 0; i < 2; ++i)
        {
            RigidBody* body = bodies[i];

            if (body->type != RigidBody::dynamic_body)
            {
                continue;
            }

            for (ContactEdge* ce = body->contactList; ce; ce = ce->next)
            {
                if (island.bodyCount == island.bodyCapacity)
                {
                    break;
                }

                if (island.contactCount == island.contactCapacity)
                {
                    break;
                }

                Contact* contact = ce->contact;

                if (contact->flag & Contact::flag_island)
                {
                    continue;
                }

                RigidBody* other = ce->other;

                // Awake linked bodies
                other->Awake();

                // Discard non-continuous dynamic vs. non-continuous dynamic case
                if (body->IsContinuous() == false && other->IsContinuous() == false && other->type == RigidBody::dynamic_body)
                {
                    continue;
                }

                Motion save = other->motion;

                // Tentatively advance the body to the TOI
                if ((other->flag & RigidBody::flag_island) == 0)
                {
                    other->Advance(minAlpha);
                }

                // Find the contact points
                contact->Update();

                // Contact disabled by the user or no contact points found
                if (contact->IsEnabled() == false || contact->IsTouching() == false)
                {
                    other->motion = save;
                    other->SynchronizeTransform();
                    continue;
                }

                // Add the contact to the island
                contact->flag |= Contact::flag_island;
                island.Add(contact);

                // Has the other body already been added to the island?
                if (other->flag & RigidBody::flag_island)
                {
                    continue;
                }

                if (other->type == RigidBody::static_body)
                {
                    continue;
                }

                island.Add(other);

                // Awake linked bodies
                for (ContactEdge* oce = other->contactList; oce; oce = oce->next)
                {
                    oce->other->Awake();
                }
            }
        }

        // step the rest time
        float dt = (1.0f - minAlpha) * settings.step.dt;
        island.SolveTOI(dt);

        // Reset island flags and synchronize broad-phase collider node
        for (int32 i = 0; i < island.bodyCount; ++i)
        {
            RigidBody* body = island.bodies[i];
            body->flag &= ~RigidBody::flag_island;

            if (body->type != RigidBody::dynamic_body)
            {
                continue;
            }

            body->SynchronizeColliders();

            // Invalidate all contact TOIs on this displaced body
            for (ContactEdge* ce = body->contactList; ce; ce = ce->next)
            {
                ce->contact->flag &= ~(Contact::flag_toi | Contact::flag_island);
            }
        }

        if (settings.sub_stepping)
        {
            // Solve only one TOI event and passed the remaining computation to the next Step() call
            stepComplete = false;
            return minAlpha;
        }
    }

    MuliAssert(stepComplete == true);

    for (RigidBody* body = bodyList; body; body = body->next)
    {
        body->motion.alpha0 = 0.0f;
        body->flag &= ~RigidBody::flag_island;
    }

    for (Contact* contact = contactGraph.contactList; contact; contact = contact->next)
    {
        contact->flag &= ~(Contact::flag_toi | Contact::flag_island);
        contact->toiCount = 0;
        contact->toi = 1.0f;
    }

    return 1.0f;
}

float World::Step(float dt)
{
    settings.step.dt = dt;
    settings.step.inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

    if (settings.step.inv_dt == 0.0f)
    {
        return 0.0f;
    }

    // Grow the allocator buffer size if needed
    linearAllocator.GrowMemory();

    if (stepComplete)
    {
        // Update broad-phase contact graph
        contactGraph.UpdateContactGraph();

        // Narrow-phase
        contactGraph.EvaluateContacts();

        Solve();
    }

    float progress = 1.0f;
    if (settings.continuous)
    {
        progress = SolveTOI();
    }

    for (RigidBody* b : destroyBodyBuffer)
    {
        Destroy(b);
    }
    for (Joint* j : destroyJointBuffer)
    {
        Destroy(j);
    }

    destroyBodyBuffer.clear();
    destroyJointBuffer.clear();

    return progress;
}

void World::Destroy(RigidBody* body)
{
    MuliAssert(body->world == this);

    Collider* c = body->colliderList;
    while (c)
    {
        Collider* c0 = c;
        c = c->next;
        body->DestroyCollider(c0);
    }

    JointEdge* je = body->jointList;
    while (je)
    {
        JointEdge* je0 = je;
        je = je->next;
        je0->other->Awake();

        Destroy(je0->joint);
    }

    if (body->next) body->next->prev = body->prev;
    if (body->prev) body->prev->next = body->next;
    if (body == bodyList) bodyList = body->next;
    if (body == bodyListTail) bodyListTail = body->prev;

    FreeBody(body);
    --bodyCount;
}

void World::Destroy(std::span<RigidBody*> bodies)
{
    std::unordered_set<RigidBody*> destroyed;

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        RigidBody* b = bodies[i];

        if (destroyed.find(b) != destroyed.end())
        {
            destroyed.insert(b);
            Destroy(b);
        }
    }
}

void World::BufferDestroy(RigidBody* body)
{
    destroyBodyBuffer.push_back(body);
}

void World::BufferDestroy(std::span<RigidBody*> bodies)
{
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        BufferDestroy(bodies[i]);
    }
}

void World::Destroy(Joint* joint)
{
    RigidBody* bodyA = joint->bodyA;
    RigidBody* bodyB = joint->bodyB;

    // Remove from the world
    if (joint->prev) joint->prev->next = joint->next;
    if (joint->next) joint->next->prev = joint->prev;
    if (joint == jointList) jointList = joint->next;

    // Remove from bodyA
    if (joint->nodeA.prev) joint->nodeA.prev->next = joint->nodeA.next;
    if (joint->nodeA.next) joint->nodeA.next->prev = joint->nodeA.prev;
    if (&joint->nodeA == bodyA->jointList) bodyA->jointList = joint->nodeA.next;

    // Remove from bodyB
    if (joint->bodyA != joint->bodyB)
    {
        if (joint->nodeB.prev) joint->nodeB.prev->next = joint->nodeB.next;
        if (joint->nodeB.next) joint->nodeB.next->prev = joint->nodeB.prev;
        if (&joint->nodeB == bodyB->jointList) bodyB->jointList = joint->nodeB.next;
    }

    FreeJoint(joint);
    --jointCount;
}

void World::Destroy(std::span<Joint*> joints)
{
    std::unordered_set<Joint*> destroyed;

    for (size_t i = 0; i < joints.size(); ++i)
    {
        Joint* j = joints[i];

        if (destroyed.find(j) != destroyed.end())
        {
            destroyed.insert(j);
            Destroy(j);
        }
    }
}

void World::BufferDestroy(Joint* joint)
{
    destroyJointBuffer.push_back(joint);
}

void World::BufferDestroy(std::span<Joint*> joints)
{
    for (size_t i = 0; i < joints.size(); ++i)
    {
        BufferDestroy(joints[i]);
    }
}

void World::Query(const Vec2& point, std::function<bool(Collider* collider)> callback) const
{
    struct TempCallback
    {
        Vec2 point;
        decltype(callback)& callbackFcn;

        TempCallback(Vec2 point, decltype(callback)& callback)
            : point{ point }
            , callbackFcn{ callback }
        {
        }

        bool QueryCallback(NodeProxy node, Collider* collider)
        {
            MuliNotUsed(node);

            // Body was destroyed while querying
            if (collider->body == nullptr)
            {
                return true;
            }

            if (collider->TestPoint(point))
            {
                return callbackFcn(collider);
            }

            return true;
        }
    } tempCallback(point, callback);

    tempCallback.point = point;

    contactGraph.broadPhase.tree.Query(point, &tempCallback);
}

void World::Query(const AABB& aabb, std::function<bool(Collider* collider)> callback) const
{
    struct TempCallback
    {
        Polygon box;
        decltype(callback)& callbackFcn;

        TempCallback(const AABB& aabb, decltype(callback)& callback)
            : box{ { aabb.min, { aabb.max.x, aabb.min.y }, aabb.max, { aabb.min.x, aabb.max.y } }, false, 0.0f }
            , callbackFcn{ callback }
        {
        }

        bool QueryCallback(NodeProxy node, Collider* collider)
        {
            MuliNotUsed(node);

            // Body was destroyed while querying
            if (collider->body == nullptr)
            {
                return true;
            }

            if (Collide(collider->shape, collider->body->transform, &box, identity))
            {
                return callbackFcn(collider);
            }

            return true;
        }
    } tempCallback(aabb, callback);

    contactGraph.broadPhase.tree.Query(aabb, &tempCallback);
}

void World::Query(const Vec2& point, WorldQueryCallback* callback) const
{
    struct TempCallback
    {
        Vec2 point;
        WorldQueryCallback* callback;

        bool QueryCallback(NodeProxy node, Collider* collider)
        {
            MuliNotUsed(node);

            // Body was destroyed while querying
            if (collider->body == nullptr)
            {
                return true;
            }

            if (collider->TestPoint(point))
            {
                return callback->OnQuery(collider);
            }

            return true;
        }
    } tempCallback;

    tempCallback.point = point;
    tempCallback.callback = callback;

    contactGraph.broadPhase.tree.Query(point, &tempCallback);
}

void World::Query(const AABB& aabb, WorldQueryCallback* callback) const
{
    Vec2 vertices[4] = { aabb.min, { aabb.max.x, aabb.min.y }, aabb.max, { aabb.min.x, aabb.max.y } };
    Polygon box{ vertices, 4, false, 0.0f };

    struct TempCallback
    {
        Polygon region;
        WorldQueryCallback* callback;
        Transform t{ identity };

        TempCallback(const Polygon& box)
            : region{ box }
        {
        }

        bool QueryCallback(NodeProxy node, Collider* collider)
        {
            MuliNotUsed(node);

            // Body was destroyed while querying
            if (collider->body == nullptr)
            {
                return true;
            }

            if (Collide(collider->shape, collider->body->transform, &region, t))
            {
                return callback->OnQuery(collider);
            }

            return true;
        }
    } tempCallback(box);

    tempCallback.callback = callback;

    contactGraph.broadPhase.tree.Query(aabb, &tempCallback);
}

void World::RayCastAny(const Vec2& from, const Vec2& to, float radius, RayCastAnyCallback* callback) const
{
    AABBCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;
    input.halfExtents.Set(radius);

    struct TempCallback
    {
        RayCastAnyCallback* callback;

        float AABBCastCallback(const AABBCastInput& subInput, Collider* collider)
        {
            RayCastInput input;
            input.from = subInput.from;
            input.to = subInput.to;
            input.maxFraction = subInput.maxFraction;
            input.radius = subInput.halfExtents.x;

            RayCastOutput output;

            bool hit = collider->RayCast(input, &output);
            if (hit)
            {
                float fraction = output.fraction;
                Vec2 point = (1.0f - fraction) * input.from + fraction * input.to;

                return callback->OnHitAny(collider, point, output.normal, fraction);
            }

            return input.maxFraction;
        }
    } tempCallback;

    tempCallback.callback = callback;

    contactGraph.broadPhase.tree.AABBCast(input, &tempCallback);
}

bool World::RayCastClosest(const Vec2& from, const Vec2& to, float radius, RayCastClosestCallback* callback) const
{
    struct TempCallback : public RayCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider;
        Vec2 closestPoint;
        Vec2 closestNormal;
        float closestFraction;

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

void World::ShapeCastAny(const Shape* shape, const Transform& tf, const Vec2& translation, ShapeCastAnyCallback* callback) const
{
    AABB aabb;
    shape->ComputeAABB(tf, &aabb);

    AABBCastInput input;
    input.from = tf.position;
    input.to = tf.position + translation;
    input.maxFraction = 1.0f;
    input.halfExtents = 0.5f * aabb.GetExtents();

    struct TempCallback
    {
        ShapeCastAnyCallback* callback;
        const Shape* shape;
        Transform tf;
        Vec2 translation;

        float AABBCastCallback(const AABBCastInput& input, Collider* collider)
        {
            ShapeCastOutput output;

            bool hit = ShapeCast(
                shape, tf, collider->GetShape(), collider->GetBody()->GetTransform(), translation * input.maxFraction, Vec2::zero,
                &output
            );
            if (hit)
            {
                return callback->OnHitAny(collider, output.point, output.normal, output.t * input.maxFraction);
            }

            return input.maxFraction;
        }
    } tempCallback;

    tempCallback.callback = callback;
    tempCallback.shape = shape;
    tempCallback.tf = tf;
    tempCallback.translation = translation;

    contactGraph.broadPhase.tree.AABBCast(input, &tempCallback);
}

bool World::ShapeCastClosest(
    const Shape* shape, const Transform& tf, const Vec2& translation, ShapeCastClosestCallback* callback
) const
{
    struct TempCallback : ShapeCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider;
        Vec2 closestPoint;
        Vec2 closestNormal;
        float closestT = 1.0f;

        float OnHitAny(Collider* collider, Vec2 point, Vec2 normal, float t)
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestT = t;

            return t;
        }
    } tempCallback;

    ShapeCastAny(shape, tf, translation, &tempCallback);

    if (tempCallback.hit)
    {
        callback->OnHitClosest(
            tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal, tempCallback.closestT
        );
        return true;
    }

    return false;
}

void World::RayCastAny(
    const Vec2& from,
    const Vec2& to,
    float radius,
    std::function<float(Collider* collider, Vec2 point, Vec2 normal, float fraction)> callback
) const
{
    AABBCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;
    input.halfExtents.Set(radius);

    struct TempCallback
    {
        decltype(callback)& callbackFcn;

        TempCallback(decltype(callback)& callback)
            : callbackFcn{ callback }
        {
        }

        float AABBCastCallback(const AABBCastInput& subInput, Collider* collider)
        {
            RayCastInput input;
            input.from = subInput.from;
            input.to = subInput.to;
            input.maxFraction = subInput.maxFraction;
            input.radius = subInput.halfExtents.x;

            RayCastOutput output;

            bool hit = collider->RayCast(input, &output);
            if (hit)
            {
                float fraction = output.fraction;
                Vec2 point = (1.0f - fraction) * input.from + fraction * input.to;

                return callbackFcn(collider, point, output.normal, fraction);
            }

            return input.maxFraction;
        }
    } tempCallback(callback);

    contactGraph.broadPhase.tree.AABBCast(input, &tempCallback);
}

bool World::RayCastClosest(
    const Vec2& from,
    const Vec2& to,
    float radius,
    std::function<void(Collider* collider, Vec2 point, Vec2 normal, float fraction)> callback
) const
{
    struct TempCallback : public RayCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider;
        Vec2 closestPoint;
        Vec2 closestNormal;
        float closestFraction;

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

void World::ShapeCastAny(
    const Shape* shape,
    const Transform& tf,
    const Vec2& translation,
    std::function<float(Collider* collider, Vec2 point, Vec2 normal, float t)> callback
) const
{
    AABB aabb;
    shape->ComputeAABB(tf, &aabb);

    AABBCastInput input;
    input.from = tf.position;
    input.to = tf.position + translation;
    input.maxFraction = 1.0f;
    input.halfExtents = 0.5f * aabb.GetExtents();

    struct TempCallback
    {
        decltype(callback)& callbackFcn;
        const Shape* shape;
        Transform tf;
        Vec2 translation;

        TempCallback(decltype(callback)& callback, const Shape* shape, Transform tf, Vec2 translation)
            : callbackFcn{ callback }
            , shape{ shape }
            , tf{ tf }
            , translation{ translation }
        {
        }

        float AABBCastCallback(const AABBCastInput& input, Collider* collider)
        {
            ShapeCastOutput output;

            bool hit = ShapeCast(
                shape, tf, collider->GetShape(), collider->GetBody()->GetTransform(), translation * input.maxFraction, Vec2::zero,
                &output
            );
            if (hit)
            {
                return callbackFcn(collider, output.point, output.normal, output.t * input.maxFraction);
            }

            return input.maxFraction;
        }
    } tempCallback(callback, shape, tf, translation);

    contactGraph.broadPhase.tree.AABBCast(input, &tempCallback);
}

bool World::ShapeCastClosest(
    const Shape* shape,
    const Transform& tf,
    const Vec2& translation,
    std::function<void(Collider* collider, Vec2 point, Vec2 normal, float t)> callback
) const
{
    struct TempCallback : ShapeCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider;
        Vec2 closestPoint;
        Vec2 closestNormal;
        float closestT = 1.0f;

        float OnHitAny(Collider* collider, Vec2 point, Vec2 normal, float t)
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestT = t;

            return t;
        }
    } tempCallback;

    ShapeCastAny(shape, tf, translation, &tempCallback);

    if (tempCallback.hit)
    {
        callback(tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal, tempCallback.closestT);
        return true;
    }

    return false;
}

RigidBody* World::DuplicateBody(RigidBody* body, const Transform& tf)
{
    MuliAssert(body->world == this);
    if (body->world != this)
    {
        return nullptr;
    }

    RigidBody* b = CreateEmptyBody(identity, body->GetType());

    for (Collider* collider = body->colliderList; collider; collider = collider->next)
    {
        Collider* c = b->CreateCollider(collider->GetShape(), identity, collider->GetDensity(), collider->GetMaterial());

        c->SetFilter(collider->GetFilter());
        c->SetEnabled(collider->IsEnabled());

        c->OnDestroy = collider->OnDestroy;
        c->ContactListener = collider->ContactListener;
    }

    b->SetTransform(Mul(body->transform, tf));

    b->SetLinearVelocity(body->linearVelocity);
    b->SetAngularDamping(body->angularVelocity);

    b->SetForce(body->force);
    b->SetTorque(body->torque);

    b->SetLinearDamping(body->linearDamping);
    b->SetAngularDamping(body->angularDamping);

    b->SetFixedRotation(body->IsRotationFixed());
    b->SetContinuous(body->IsContinuous());
    b->SetSleeping(body->IsSleeping());
    b->resting = body->resting;

    b->SetEnabled(body->IsEnabled());

    b->OnDestroy = body->OnDestroy;
    b->UserData = body->UserData;

    return b;
}

RigidBody* World::CreateEmptyBody(const Transform& tf, RigidBody::Type type)
{
    void* mem = blockAllocator.Allocate(sizeof(RigidBody));
    RigidBody* body = new (mem) RigidBody(tf, type);

    body->world = this;

    if (bodyList == nullptr && bodyListTail == nullptr)
    {
        bodyList = body;
        bodyListTail = body;
    }
    else
    {
        bodyListTail->next = body;
        body->prev = bodyListTail;
        bodyListTail = body;
    }

    ++bodyCount;

    return body;
}

RigidBody* World::CreateCircle(float radius, const Transform& tf, RigidBody::Type type, float density)
{
    RigidBody* b = CreateEmptyBody(tf, type);

    Circle circle{ radius };
    b->CreateCollider(&circle, identity, density);

    return b;
}

RigidBody* World::CreateCapsule(
    float length, float radius, bool horizontal, const Transform& tf, RigidBody::Type type, float density
)
{
    RigidBody* b = CreateEmptyBody(tf, type);

    Capsule capsule{ length, radius, horizontal };
    b->CreateCollider(&capsule, identity, density);

    return b;
}

RigidBody* World::CreateCapsule(
    const Vec2& point1,
    const Vec2& point2,
    float radius,
    const Transform& tf,
    RigidBody::Type type,
    bool resetPosition,
    float density
)
{
    RigidBody* b = CreateEmptyBody(tf, type);

    Vec2 center = (point1 + point2) * 0.5f;
    Capsule capsule{ point1, point2, radius, true };
    b->CreateCollider(&capsule, identity, density);

    if (resetPosition == false)
    {
        b->Translate(center);
    }

    return b;
}

RigidBody* World::CreatePolygon(
    std::span<Vec2> vertices, const Transform& tf, RigidBody::Type type, bool resetPosition, float radius, float density
)
{
    RigidBody* b = CreateEmptyBody(tf, type);

    Polygon polygon(vertices.data(), int32(vertices.size()), true, radius);
    b->CreateCollider(&polygon, identity, density);

    Vec2 center{ 0.0f };
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        center += vertices[i];
    }
    center *= 1.0f / vertices.size();

    if (resetPosition == false)
    {
        b->Translate(center);
    }

    return b;
}

RigidBody* World::CreateBox(float width, float height, const Transform& tf, RigidBody::Type type, float radius, float density)
{
    RigidBody* b = CreateEmptyBody(tf, type);

    Vec2 vertices[4] = { Vec2{ 0, 0 }, Vec2{ width, 0 }, Vec2{ width, height }, Vec2{ 0, height } };
    Polygon box{ vertices, 4, true, radius };
    b->CreateCollider(&box, identity, density);

    return b;
}

RigidBody* World::CreateBox(float size, const Transform& tf, RigidBody::Type type, float radius, float density)
{
    return CreateBox(size, size, tf, type, radius, density);
}

GrabJoint* World::CreateGrabJoint(
    RigidBody* body, const Vec2& anchor, const Vec2& target, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    if (body->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(GrabJoint));
    GrabJoint* gj = new (mem) GrabJoint(body, anchor, target, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(gj);
    return gj;
}

RevoluteJoint* World::CreateRevoluteJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec2& anchor, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(RevoluteJoint));
    RevoluteJoint* rj = new (mem) RevoluteJoint(bodyA, bodyB, anchor, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(rj);
    return rj;
}

DistanceJoint* World::CreateDistanceJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchorA,
    const Vec2& anchorB,
    float length,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(DistanceJoint));
    DistanceJoint* dj =
        new (mem) DistanceJoint(bodyA, bodyB, anchorA, anchorB, length, length, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(dj);
    return dj;
}

DistanceJoint* World::CreateDistanceJoint(
    RigidBody* bodyA, RigidBody* bodyB, float length, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    return CreateDistanceJoint(
        bodyA, bodyB, bodyA->GetPosition(), bodyB->GetPosition(), length, jointFrequency, jointDampingRatio, jointMass
    );
}

DistanceJoint* World::CreateLimitedDistanceJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchorA,
    const Vec2& anchorB,
    float minLength,
    float maxLength,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(DistanceJoint));
    DistanceJoint* dj = new (mem)
        DistanceJoint(bodyA, bodyB, anchorA, anchorB, minLength, maxLength, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(dj);
    return dj;
}

DistanceJoint* World::CreateLimitedDistanceJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    float minLength,
    float maxLength,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    return CreateLimitedDistanceJoint(
        bodyA, bodyB, bodyA->GetPosition(), bodyB->GetPosition(), minLength, maxLength, jointFrequency, jointDampingRatio,
        jointMass
    );
}

AngleJoint* World::CreateAngleJoint(
    RigidBody* bodyA, RigidBody* bodyB, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(AngleJoint));
    AngleJoint* aj = new (mem)
        AngleJoint(bodyA, bodyB, bodyB->GetAngle() - bodyA->GetAngle(), 0.0f, 0.0f, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(aj);
    return aj;
}

AngleJoint* World::CreateLimitedAngleJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    float minAngle,
    float maxAngle,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(AngleJoint));
    AngleJoint* aj = new (mem) AngleJoint(
        bodyA, bodyB, bodyB->GetAngle() - bodyA->GetAngle(), minAngle, maxAngle, jointFrequency, jointDampingRatio, jointMass
    );

    AddJoint(aj);
    return aj;
}

WeldJoint* World::CreateWeldJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec2& anchor, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(WeldJoint));
    WeldJoint* wj = new (mem) WeldJoint(bodyA, bodyB, anchor, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(wj);
    return wj;
}

LineJoint* World::CreateLineJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchor,
    const Vec2& dir,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(LineJoint));
    LineJoint* lj = new (mem) LineJoint(bodyA, bodyB, anchor, dir, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(lj);
    return lj;
}

LineJoint* World::CreateLineJoint(
    RigidBody* bodyA, RigidBody* bodyB, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    return CreateLineJoint(
        bodyA, bodyB, bodyA->GetPosition(), Normalize(bodyB->GetPosition() - bodyA->GetPosition()), jointFrequency,
        jointDampingRatio, jointMass
    );
}

PrismaticJoint* World::CreatePrismaticJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchor,
    const Vec2& dir,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(PrismaticJoint));
    PrismaticJoint* pj = new (mem) PrismaticJoint(bodyA, bodyB, anchor, dir, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(pj);
    return pj;
}

PrismaticJoint* World::CreatePrismaticJoint(
    RigidBody* bodyA, RigidBody* bodyB, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    return CreatePrismaticJoint(
        bodyA, bodyB, bodyB->GetPosition(), Normalize(bodyB->GetPosition() - bodyA->GetPosition()), jointFrequency,
        jointDampingRatio, jointMass
    );
}

PulleyJoint* World::CreatePulleyJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchorA,
    const Vec2& anchorB,
    const Vec2& groundAnchorA,
    const Vec2& groundAnchorB,
    float ratio,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(PulleyJoint));
    PulleyJoint* pj = new (mem) PulleyJoint(
        bodyA, bodyB, anchorA, anchorB, groundAnchorA, groundAnchorB, ratio, jointFrequency, jointDampingRatio, jointMass
    );

    AddJoint(pj);
    return pj;
}

MotorJoint* World::CreateMotorJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchor,
    float maxForce,
    float maxTorque,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(MotorJoint));
    MotorJoint* mj =
        new (mem) MotorJoint(bodyA, bodyB, anchor, maxForce, maxTorque, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(mj);
    return mj;
}

void World::AddJoint(Joint* joint)
{
    // Insert into the world
    joint->prev = nullptr;
    joint->next = jointList;
    if (jointList != nullptr)
    {
        jointList->prev = joint;
    }
    jointList = joint;

    // Connect to island graph

    // Connect joint edge to body A
    joint->nodeA.joint = joint;
    joint->nodeA.other = joint->bodyB;

    joint->nodeA.prev = nullptr;
    joint->nodeA.next = joint->bodyA->jointList;
    if (joint->bodyA->jointList != nullptr)
    {
        joint->bodyA->jointList->prev = &joint->nodeA;
    }
    joint->bodyA->jointList = &joint->nodeA;

    // Connect joint edge to body B
    if (joint->bodyA != joint->bodyB)
    {
        joint->nodeB.joint = joint;
        joint->nodeB.other = joint->bodyA;

        joint->nodeB.prev = nullptr;
        joint->nodeB.next = joint->bodyB->jointList;
        if (joint->bodyB->jointList != nullptr)
        {
            joint->bodyB->jointList->prev = &joint->nodeB;
        }
        joint->bodyB->jointList = &joint->nodeB;
    }

    ++jointCount;
}

void World::FreeBody(RigidBody* body)
{
    body->~RigidBody();
    blockAllocator.Free(body, sizeof(RigidBody));
}

void World::FreeJoint(Joint* joint)
{
    joint->~Joint();

    switch (joint->type)
    {
    case Joint::Type::grab_joint:
        blockAllocator.Free(joint, sizeof(GrabJoint));
        break;
    case Joint::Type::revolute_joint:
        blockAllocator.Free(joint, sizeof(RevoluteJoint));
        break;
    case Joint::Type::distance_joint:
        blockAllocator.Free(joint, sizeof(DistanceJoint));
        break;
    case Joint::Type::angle_joint:
        blockAllocator.Free(joint, sizeof(AngleJoint));
        break;
    case Joint::Type::weld_joint:
        blockAllocator.Free(joint, sizeof(WeldJoint));
        break;
    case Joint::Type::line_joint:
        blockAllocator.Free(joint, sizeof(LineJoint));
        break;
    case Joint::Type::prismatic_joint:
        blockAllocator.Free(joint, sizeof(PrismaticJoint));
        break;
    case Joint::Type::pulley_joint:
        blockAllocator.Free(joint, sizeof(PulleyJoint));
        break;
    case Joint::Type::motor_joint:
        blockAllocator.Free(joint, sizeof(MotorJoint));
        break;
    default:
        MuliAssert(false);
        break;
    }
}

} // namespace flywheel