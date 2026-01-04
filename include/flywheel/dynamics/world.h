#pragma once

// Island solver configuration
#define SOLVE_CONTACTS_BACKWARD 1
#define SOLVE_CONTACT_CONSTRAINT 1

#include "../core/common.h"
#include "../math/math.h"

#include "../math/raycast.h"
#include "collider.h"
#include "rigidbody.h"

#include "../math/geometry.h"
#include "../shapes/capsule.h"
#include "../shapes/circle.h"
#include "../shapes/polygon.h"

#include "../joints/angle_joint.h"
#include "../joints/distance_joint.h"
#include "../joints/grab_joint.h"
#include "../joints/joint.h"
#include "../joints/line_joint.h"
#include "../joints/motor_joint.h"
#include "../joints/prismatic_joint.h"
#include "../joints/pulley_joint.h"
#include "../joints/revolute_joint.h"
#include "../joints/weld_joint.h"

#include "../collision/collision.h"
#include "../collision/time_of_impact.h"
#include "../utils/callbacks.h"
#include "contact_graph.h"
#include "island.h"

#include "../memory/block_allocator.h"
#include "../memory/linear_allocator.h"

namespace flywheel
{

// Joint destructor implementation (placed here after callbacks.h is included to avoid circular dependency)
inline Joint::~Joint()
{
    if (OnDestroy)
    {
        OnDestroy->OnJointDestroy(this);
    }
}

class World
{
public:
    World(const WorldSettings& settings);
    ~World();

    World(const World&) = delete;
    World& operator=(const World&) = delete;

    float Step(float dt);
    void Reset();

    void Destroy(RigidBody* body);
    void Destroy(std::span<RigidBody*> bodies);
    void Destroy(Joint* joint);
    void Destroy(std::span<Joint*> joints);

    // Buffered body will be destroy at the end of the step
    void BufferDestroy(RigidBody* body);
    void BufferDestroy(std::span<RigidBody*> bodies);
    void BufferDestroy(Joint* joint);
    void BufferDestroy(std::span<Joint*> joints);

    // clang-format off
    // Factory functions for bodies
    RigidBody* DuplicateBody(
        RigidBody* body,
        const Transform& tf = identity
    );
    RigidBody* CreateEmptyBody(
        const Transform& tf = identity,
        RigidBody::Type type = RigidBody::dynamic_body
    );

    RigidBody* CreateCircle(
        float radius,
        const Transform& tf = identity,
        RigidBody::Type type = RigidBody::dynamic_body,
        float density = default_density
    );
    RigidBody* CreateCapsule(
        float length,
        float radius,
        bool horizontal = false,
        const Transform& tf = identity,
        RigidBody::Type type = RigidBody::dynamic_body,
        float density = default_density
    );
    RigidBody* CreateCapsule(
        const Vec2& point1,
        const Vec2& point2,
        float radius,
        const Transform& tf = identity,
        RigidBody::Type type = RigidBody::dynamic_body,
        bool resetPosition = false,
        float density = default_density
    );
    RigidBody* CreatePolygon(
        std::span<Vec2> vertices,
        const Transform& tf = identity,
        RigidBody::Type type = RigidBody::dynamic_body,
        bool resetCenter = true,
        float radius = default_radius,
        float density = default_density
    );
    RigidBody* CreateBox(
        float size,
        const Transform& tf = identity,
        RigidBody::Type type = RigidBody::dynamic_body,
        float radius = default_radius,
        float density = default_density
    );
    RigidBody* CreateBox(
        float width,
        float height,
        const Transform& tf = identity,
        RigidBody::Type type = RigidBody::dynamic_body,
        float radius = default_radius,
        float density = default_density
    );

    // Factory functions for joints
    // You should register the bodies to the world before registering the joints
    // Otherwise the functions will return nullptr
    GrabJoint* CreateGrabJoint(
        RigidBody* body,
        const Vec2& anchor,
        const Vec2& target,
        float frequency = 1.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    RevoluteJoint* CreateRevoluteJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec2& anchor,
        float frequency = 10.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    DistanceJoint* CreateDistanceJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec2& anchorA,
        const Vec2& anchorB,
        float length = -1.0f,
        float frequency = 10.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    DistanceJoint* CreateDistanceJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        float length = -1.0f,
        float frequency = 10.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    DistanceJoint* CreateLimitedDistanceJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec2& anchorA,
        const Vec2& anchorB,
        float minLength = -1.0f,
        float maxLength = -1.0f,
        float frequency = 10.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    DistanceJoint* CreateLimitedDistanceJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        float minLength = -1.0f,
        float maxLength = -1.0f,
        float frequency = 10.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    AngleJoint* CreateAngleJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        float frequency = 10.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    AngleJoint* CreateLimitedAngleJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        float minAngle,
        float maxAngle,
        float frequency = 10.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    WeldJoint* CreateWeldJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec2& anchor,
        float frequency = -1.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    LineJoint* CreateLineJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec2& anchorA,
        const Vec2& dir,
        float frequency = 10.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    LineJoint* CreateLineJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        float frequency = 10.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    PrismaticJoint* CreatePrismaticJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec2& anchor,
        const Vec2& dir,
        float frequency = -1.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    PrismaticJoint* CreatePrismaticJoint(
        RigidBody* bodyA, 
        RigidBody* bodyB, 
        float frequency = -1.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    PulleyJoint* CreatePulleyJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec2& anchorA,
        const Vec2& anchorB,
        const Vec2& groundAnchorA,
        const Vec2& groundAnchorB,
        float ratio = 1.0f,
        float frequency = -1.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    MotorJoint* CreateMotorJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec2& anchor,
        float maxForce = 1000.0f,
        float maxTorque = 1000.0f,
        float frequency = -1.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    // clang-format on

    // clang-format off
    void Query(const Vec2& point, WorldQueryCallback* callback) const;
    void Query(const AABB& aabb, WorldQueryCallback* callback) const;

    void RayCastAny(
        const Vec2& from,
        const Vec2& to,
        float radius,
        RayCastAnyCallback* callback
    ) const;
    bool RayCastClosest(
        const Vec2& from,
        const Vec2& to,
        float radius,
        RayCastClosestCallback* callback
    ) const;
    void ShapeCastAny(
        const Shape* shape,
        const Transform& tf,
        const Vec2& translation,
        ShapeCastAnyCallback* callback
    ) const;
    bool ShapeCastClosest(
        const Shape* shape,
        const Transform& tf,
        const Vec2& translation,
        ShapeCastClosestCallback* callback
    ) const;

    void Query(const Vec2& point, std::function<bool(Collider* collider)> callback) const;
    void Query(const AABB& aabb, std::function<bool(Collider* collider)> callback) const;
    
    void RayCastAny(
        const Vec2& from,
        const Vec2& to,
        float radius,
        std::function<float(Collider* collider, Vec2 point, Vec2 normal, float fraction)> callback
    ) const;
    bool RayCastClosest(
        const Vec2& from,
        const Vec2& to,
        float radius,
        std::function<void(Collider* collider, Vec2 point, Vec2 normal, float fraction)> callback
    ) const;
    void ShapeCastAny(
        const Shape* shape,
        const Transform& tf,
        const Vec2& translation,
        std::function<float(Collider* collider, Vec2 point, Vec2 normal, float t)> callback
    ) const;
    bool ShapeCastClosest(
        const Shape* shape,
        const Transform& tf,
        const Vec2& translation,
        std::function<void(Collider* collider, Vec2 point, Vec2 normal, float t)> callback
    ) const;
    // clang-format on

    RigidBody* GetBodyList() const;
    RigidBody* GetBodyListTail() const;
    int32 GetBodyCount() const;
    Joint* GetJoints() const;
    int32 GetJointCount() const;

    const Contact* GetContacts() const;
    int32 GetContactCount() const;

    int32 GetSleepingBodyCount() const;
    int32 GetAwakeIslandCount() const;

    const AABBTree& GetDynamicTree() const;
    void RebuildDynamicTree();

    const WorldSettings& GetWorldSettings() const;

    void Awake();

private:
    friend class RigidBody;
    friend class Island;
    friend class ContactGraph;
    friend class BroadPhase;

    void Solve();
    float SolveTOI();

    void FreeBody(RigidBody* body);
    void AddJoint(Joint* joint);
    void FreeJoint(Joint* joint);

    const WorldSettings& settings;
    ContactGraph contactGraph;

    // Doubly linked list of all registered rigid bodies
    RigidBody* bodyList;
    RigidBody* bodyListTail;
    int32 bodyCount;

    Joint* jointList;
    int32 jointCount;

    int32 islandCount;
    int32 sleepingBodyCount;

    bool stepComplete;

    std::vector<RigidBody*> destroyBodyBuffer;
    std::vector<Joint*> destroyJointBuffer;

    LinearAllocator linearAllocator;
    BlockAllocator blockAllocator;
};

inline void World::Awake()
{
    for (RigidBody* b = bodyList; b; b = b->next)
    {
        b->Awake();
    }
}

inline RigidBody* World::GetBodyList() const
{
    return bodyList;
}

inline RigidBody* World::GetBodyListTail() const
{
    return bodyListTail;
}

inline int32 World::GetBodyCount() const
{
    return bodyCount;
}

inline int32 World::GetSleepingBodyCount() const
{
    return sleepingBodyCount;
}

inline int32 World::GetAwakeIslandCount() const
{
    return islandCount;
}

inline const Contact* World::GetContacts() const
{
    return contactGraph.contactList;
}

inline int32 World::GetContactCount() const
{
    return contactGraph.contactCount;
}

inline Joint* World::GetJoints() const
{
    return jointList;
}

inline int32 World::GetJointCount() const
{
    return jointCount;
}

inline const AABBTree& World::GetDynamicTree() const
{
    return contactGraph.broadPhase.tree;
}

inline void World::RebuildDynamicTree()
{
    contactGraph.broadPhase.tree.Rebuild();
}

inline const WorldSettings& World::GetWorldSettings() const
{
    return settings;
}

// BroadPhase inline implementations (defined here after World is complete)

inline BroadPhase::BroadPhase(ContactGraph* contactGraph)
    : contactGraph{ contactGraph }
    , moveCapacity{ 16 }
    , moveCount{ 0 }
{
    moveBuffer = (NodeProxy*)flywheel::Alloc(moveCapacity * sizeof(NodeProxy));
}

inline BroadPhase::~BroadPhase()
{
    flywheel::Free(moveBuffer);
}

inline void BroadPhase::BufferMove(NodeProxy node)
{
    // Grow the buffer as needed
    if (moveCount == moveCapacity)
    {
        NodeProxy* old = moveBuffer;
        moveCapacity *= 2;
        moveBuffer = (NodeProxy*)flywheel::Alloc(moveCapacity * sizeof(NodeProxy));
        memcpy(moveBuffer, old, moveCount * sizeof(NodeProxy));
        flywheel::Free(old);
    }

    moveBuffer[moveCount] = node;
    ++moveCount;
}

inline void BroadPhase::UnBufferMove(NodeProxy node)
{
    for (int32 i = 0; i < moveCount; ++i)
    {
        if (moveBuffer[i] == node)
        {
            moveBuffer[i] = AABBTree::nullNode;
        }
    }
}

inline void BroadPhase::FindNewContacts()
{
    for (int32 i = 0; i < moveCount; ++i)
    {
        nodeA = moveBuffer[i];
        if (nodeA == AABBTree::nullNode)
        {
            continue;
        }

        colliderA = tree.GetData(nodeA);
        bodyA = colliderA->body;
        typeA = colliderA->GetType();

        const AABB& treeAABB = tree.GetAABB(colliderA->node);

        // This will callback our BroadPhase::QueryCallback(NodeProxy, Collider*)
        tree.Query(treeAABB, this);
    }

    // Clear move buffer for next step
    for (int32 i = 0; i < moveCount; ++i)
    {
        NodeProxy node = moveBuffer[i];
        if (node != AABBTree::nullNode)
        {
            tree.ClearMoved(node);
        }
    }

    moveCount = 0;
}

inline void BroadPhase::Add(Collider* collider, const AABB& aabb)
{
    NodeProxy node = tree.CreateNode(collider, aabb);
    collider->node = node;

    BufferMove(node);
}

inline void BroadPhase::Remove(Collider* collider)
{
    NodeProxy node = collider->node;
    tree.RemoveNode(node);

    UnBufferMove(node);
}

inline void BroadPhase::Update(Collider* collider, const AABB& aabb, const Vec2& displacement)
{
    NodeProxy node = collider->node;
    bool rested = collider->body->resting > contactGraph->world->settings.sleeping_time;

    bool nodeMoved = tree.MoveNode(node, aabb, displacement, rested);
    if (nodeMoved)
    {
        BufferMove(node);
    }
}

inline void BroadPhase::Refresh(Collider* collider)
{
    NodeProxy node = collider->node;
    AABB aabb = collider->GetAABB();

    tree.MoveNode(node, aabb, Vec2::zero, true);
    BufferMove(node);
}

inline bool BroadPhase::QueryCallback(NodeProxy nodeB, Collider* colliderB)
{
    if (nodeA == nodeB)
    {
        return true;
    }

    RigidBody* bodyB = colliderB->body;
    if (bodyA == bodyB)
    {
        return true;
    }

    // Avoid duplicate contact
    if (tree.WasMoved(nodeB) && nodeA < nodeB)
    {
        return true;
    }

    Shape::Type typeB = colliderB->GetType();
    if (typeA <= typeB)
    {
        contactGraph->OnNewContact(colliderB, colliderA);
    }
    else
    {
        contactGraph->OnNewContact(colliderA, colliderB);
    }

    return true;
}

// ============================================================================
// Collider inline implementations (placed here to avoid circular dependencies)
// ============================================================================

// Inline global variable (C++17)
inline ContactListener defaultListener;

inline Collider::Collider()
    : OnDestroy{ nullptr }
    , ContactListener{ &defaultListener }
    , body{ nullptr }
    , next{ nullptr }
    , shape{ nullptr }
    , density{ 0.0f }
    , node{ AABBTree::nullNode }
    , enabled{ true }
{
}

inline Collider::~Collider()
{
    if (OnDestroy)
    {
        OnDestroy->OnColliderDestroy(this);
    }

    body = nullptr;
    next = nullptr;
}

inline void Collider::Create(
    Allocator* allocator, RigidBody* inBody, Shape* inShape, const Transform& tf, float inDensity, const Material& inMaterial
)
{
    body = inBody;
    shape = inShape->Clone(allocator, tf);
    density = inDensity;
    material = inMaterial;
}

inline void Collider::Destroy(Allocator* allocator)
{
    if (shape == nullptr)
    {
        return;
    }

    Shape::Type type = shape->GetType();
    shape->~Shape();

    switch (type)
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

// ============================================================================
// ContactGraph inline implementations
void InitializeDetectionFunctionMap();
// ============================================================================

inline ContactGraph::ContactGraph(World* world)
    : world{ world }
    , broadPhase{ this }
    , contactList{ nullptr }
    , contactCount{ 0 }
{
    InitializeDetectionFunctionMap();
}

inline ContactGraph::~ContactGraph()
{
    MuliAssert(contactList == nullptr);
}

inline void ContactGraph::EvaluateContacts()
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

inline void ContactGraph::OnNewContact(Collider* colliderA, Collider* colliderB)
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

inline void ContactGraph::Destroy(Contact* c)
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

inline void ContactGraph::AddCollider(Collider* collider)
{
    broadPhase.Add(collider, collider->GetAABB());
}

inline void ContactGraph::RemoveCollider(Collider* collider)
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

inline void ContactGraph::UpdateCollider(Collider* collider, const Transform& tf)
{
    const Shape* shape = collider->GetShape();
    AABB aabb;
    shape->ComputeAABB(tf, &aabb);

    broadPhase.Update(collider, aabb, Vec2::zero);
}

inline void ContactGraph::UpdateCollider(Collider* collider, const Transform& tf0, const Transform& tf1)
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

inline Island::Island(World* world, int32 bodyCapacity, int32 contactCapacity, int32 jointCapacity)
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

inline Island::~Island()
{
    world->linearAllocator.Free(joints, jointCapacity * sizeof(Joint*));
    world->linearAllocator.Free(contacts, contactCapacity * sizeof(Contact*));
    world->linearAllocator.Free(bodies, bodyCapacity * sizeof(RigidBody*));
}

inline void Island::Solve()
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

// Inline global constants (C++17)
inline constexpr int32 toi_postion_iteration = 20;
inline constexpr int32 toi_index_1 = 0;
inline constexpr int32 toi_index_2 = 1;

inline void Island::SolveTOI(float dt)
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

inline RigidBody::RigidBody(const Transform& tf, RigidBody::Type type)
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

inline RigidBody::~RigidBody()
{
    if (OnDestroy)
    {
        OnDestroy->OnBodyDestroy(this);
    }

    world = nullptr;
}

inline Collider* RigidBody::CreateCollider(Shape* shape, const Transform& tf, float density, const Material& material)
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

inline void RigidBody::DestroyCollider(Collider* collider)
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

inline Collider* RigidBody::CreateCircleCollider(float radius, const Transform& tf, float density, const Material& material)
{
    Circle circle{ radius };
    return CreateCollider(&circle, tf, density, material);
}

inline Collider* RigidBody::CreateBoxCollider(
    float width, float height, float radius, const Transform& tf, float density, const Material& material
)
{
    Polygon box{ width, height, radius };
    return CreateCollider(&box, tf, density, material);
}

inline Collider* RigidBody::CreateCapsuleCollider(
    float length, float radius, bool horizontal, const Transform& tf, float density, const Material& material
)
{
    Capsule capsule{ length, radius, horizontal };
    return CreateCollider(&capsule, tf, density, material);
}

inline Collider* RigidBody::CreateCapsuleCollider(
    const Vec2& p1, const Vec2& p2, float radius, bool resetPosition, const Transform& tf, float density, const Material& material
)
{
    Capsule capsule{ p1, p2, radius, resetPosition };
    return CreateCollider(&capsule, tf, density, material);
}

inline bool RigidBody::TestPoint(const Vec2& p) const
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

inline Vec2 RigidBody::GetClosestPoint(const Vec2& p) const
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

inline void RigidBody::RayCastAny(const Vec2& from, const Vec2& to, float radius, RayCastAnyCallback* callback) const
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

inline bool RigidBody::RayCastClosest(const Vec2& from, const Vec2& to, float radius, RayCastClosestCallback* callback) const
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

inline void RigidBody::RayCastAny(
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

inline bool RigidBody::RayCastClosest(
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

inline void RigidBody::SetType(RigidBody::Type newType)
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

inline void RigidBody::SetEnabled(bool enabled)
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

inline void RigidBody::SetCollisionFilter(const CollisionFilter& filter) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetFilter(filter);
    }
}

inline void RigidBody::SetFriction(float friction) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetFriction(friction);
    }
}

inline void RigidBody::SetRestitution(float restitution) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetRestitution(restitution);
    }
}

inline void RigidBody::SetRestitutionThreshold(float threshold) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetRestitutionTreshold(threshold);
    }
}

inline void RigidBody::SetSurfaceSpeed(float surfaceSpeed) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetSurfaceSpeed(surfaceSpeed);
    }
}

inline void RigidBody::ResetMassData()
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

inline void RigidBody::SynchronizeColliders()
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

inline World::World(const WorldSettings& settings)
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

inline World::~World()
{
    Reset();
}

inline void World::Reset()
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

inline void World::Solve()
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
inline float World::SolveTOI()
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

inline float World::Step(float dt)
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

inline void World::Destroy(RigidBody* body)
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

inline void World::Destroy(std::span<RigidBody*> bodies)
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

inline void World::BufferDestroy(RigidBody* body)
{
    destroyBodyBuffer.push_back(body);
}

inline void World::BufferDestroy(std::span<RigidBody*> bodies)
{
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        BufferDestroy(bodies[i]);
    }
}

inline void World::Destroy(Joint* joint)
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

inline void World::Destroy(std::span<Joint*> joints)
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

inline void World::BufferDestroy(Joint* joint)
{
    destroyJointBuffer.push_back(joint);
}

inline void World::BufferDestroy(std::span<Joint*> joints)
{
    for (size_t i = 0; i < joints.size(); ++i)
    {
        BufferDestroy(joints[i]);
    }
}

inline void World::Query(const Vec2& point, std::function<bool(Collider* collider)> callback) const
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

inline void World::Query(const AABB& aabb, std::function<bool(Collider* collider)> callback) const
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

inline void World::Query(const Vec2& point, WorldQueryCallback* callback) const
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

inline void World::Query(const AABB& aabb, WorldQueryCallback* callback) const
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

inline void World::RayCastAny(const Vec2& from, const Vec2& to, float radius, RayCastAnyCallback* callback) const
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

inline bool World::RayCastClosest(const Vec2& from, const Vec2& to, float radius, RayCastClosestCallback* callback) const
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

inline void World::ShapeCastAny(
    const Shape* shape, const Transform& tf, const Vec2& translation, ShapeCastAnyCallback* callback
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

inline bool World::ShapeCastClosest(
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

inline void World::RayCastAny(
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

inline bool World::RayCastClosest(
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

inline void World::ShapeCastAny(
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

inline bool World::ShapeCastClosest(
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

inline RigidBody* World::DuplicateBody(RigidBody* body, const Transform& tf)
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

inline RigidBody* World::CreateEmptyBody(const Transform& tf, RigidBody::Type type)
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

inline RigidBody* World::CreateCircle(float radius, const Transform& tf, RigidBody::Type type, float density)
{
    RigidBody* b = CreateEmptyBody(tf, type);

    Circle circle{ radius };
    b->CreateCollider(&circle, identity, density);

    return b;
}

inline RigidBody* World::CreateCapsule(
    float length, float radius, bool horizontal, const Transform& tf, RigidBody::Type type, float density
)
{
    RigidBody* b = CreateEmptyBody(tf, type);

    Capsule capsule{ length, radius, horizontal };
    b->CreateCollider(&capsule, identity, density);

    return b;
}

inline RigidBody* World::CreateCapsule(
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

inline RigidBody* World::CreatePolygon(
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

inline RigidBody* World::CreateBox(
    float width, float height, const Transform& tf, RigidBody::Type type, float radius, float density
)
{
    RigidBody* b = CreateEmptyBody(tf, type);

    Vec2 vertices[4] = { Vec2{ 0, 0 }, Vec2{ width, 0 }, Vec2{ width, height }, Vec2{ 0, height } };
    Polygon box{ vertices, 4, true, radius };
    b->CreateCollider(&box, identity, density);

    return b;
}

inline RigidBody* World::CreateBox(float size, const Transform& tf, RigidBody::Type type, float radius, float density)
{
    return CreateBox(size, size, tf, type, radius, density);
}

inline GrabJoint* World::CreateGrabJoint(
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

inline RevoluteJoint* World::CreateRevoluteJoint(
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

inline DistanceJoint* World::CreateDistanceJoint(
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

inline DistanceJoint* World::CreateDistanceJoint(
    RigidBody* bodyA, RigidBody* bodyB, float length, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    return CreateDistanceJoint(
        bodyA, bodyB, bodyA->GetPosition(), bodyB->GetPosition(), length, jointFrequency, jointDampingRatio, jointMass
    );
}

inline DistanceJoint* World::CreateLimitedDistanceJoint(
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

inline DistanceJoint* World::CreateLimitedDistanceJoint(
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

inline AngleJoint* World::CreateAngleJoint(
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

inline AngleJoint* World::CreateLimitedAngleJoint(
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

inline WeldJoint* World::CreateWeldJoint(
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

inline LineJoint* World::CreateLineJoint(
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

inline LineJoint* World::CreateLineJoint(
    RigidBody* bodyA, RigidBody* bodyB, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    return CreateLineJoint(
        bodyA, bodyB, bodyA->GetPosition(), Normalize(bodyB->GetPosition() - bodyA->GetPosition()), jointFrequency,
        jointDampingRatio, jointMass
    );
}

inline PrismaticJoint* World::CreatePrismaticJoint(
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

inline PrismaticJoint* World::CreatePrismaticJoint(
    RigidBody* bodyA, RigidBody* bodyB, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    return CreatePrismaticJoint(
        bodyA, bodyB, bodyB->GetPosition(), Normalize(bodyB->GetPosition() - bodyA->GetPosition()), jointFrequency,
        jointDampingRatio, jointMass
    );
}

inline PulleyJoint* World::CreatePulleyJoint(
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

inline MotorJoint* World::CreateMotorJoint(
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

inline void World::AddJoint(Joint* joint)
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

inline void World::FreeBody(RigidBody* body)
{
    body->~RigidBody();
    blockAllocator.Free(body, sizeof(RigidBody));
}

inline void World::FreeJoint(Joint* joint)
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

// ============================================================================
// Collision inline implementations
// ============================================================================

// Inline global variables (C++17)
inline const Vec2 origin = Vec2::zero;
inline bool detection_function_initialized = false;

// Forward declaration - implementation after collision functions are defined
inline void InitializeDetectionFunctionMap();

/*
 * Returns support point in 'Minkowski Difference' set
 * Minkowski Sum: A ⊕ B = {Pa + Pb| Pa ∈ A, Pb ∈ B}
 * Minkowski Difference : A ⊖ B = {Pa - Pb| Pa ∈ A, Pb ∈ B}
 * CSO stands for Configuration Space Object
 *
 * 'dir' should be normalized
 */
inline SupportPoint CSOSupport(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, const Vec2& dir)
{
    SupportPoint supportPoint;
    supportPoint.pointA.id = a->GetSupport(MulT(tfA.rotation, dir));
    supportPoint.pointB.id = b->GetSupport(MulT(tfB.rotation, -dir));
    supportPoint.pointA.p = Mul(tfA, a->GetVertex(supportPoint.pointA.id));
    supportPoint.pointB.p = Mul(tfB, b->GetVertex(supportPoint.pointB.id));
    supportPoint.point = supportPoint.pointA.p - supportPoint.pointB.p;

    return supportPoint;
}

inline bool GJK(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, GJKResult* result)
{
    Simplex simplex;

    // Random initial search direction
    Vec2 direction = tfB.position - tfA.position;
    SupportPoint support = CSOSupport(a, tfA, b, tfB, direction);
    simplex.AddVertex(support);

    Vec2 save[max_simplex_vertex_count];
    int32 saveCount;

    for (int32 k = 0; k < gjk_max_iteration; ++k)
    {
        simplex.Save(save, &saveCount);
        simplex.Advance(origin);

        if (simplex.count == 3)
        {
            break;
        }

        direction = simplex.GetSearchDirection();

        // Simplex contains origin
        if (Dot(direction, direction) == 0.0f)
        {
            break;
        }

        support = CSOSupport(a, tfA, b, tfB, direction);

        // Check duplicate vertices
        for (int32 i = 0; i < saveCount; ++i)
        {
            if (save[i] == support.point)
            {
                goto end;
            }
        }

        simplex.AddVertex(support);
    }

end:
    Vec2 closest = simplex.GetClosestPoint();
    float distance = Length(closest);

    result->simplex = simplex;
    result->direction = Normalize(direction);
    result->distance = distance;

    return distance < gjk_tolerance;
}

inline void EPA(
    const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, const Simplex& simplex, EPAResult* result
)
{
    Polytope polytope{ simplex };
    PolytopeEdge edge{ 0, max_value, Vec2::zero };

    for (int32 k = 0; k < epa_max_iteration; ++k)
    {
        edge = polytope.GetClosestEdge();
        Vec2 supportPoint = CSOSupport(a, tfA, b, tfB, edge.normal).point;
        float newDistance = Dot(edge.normal, supportPoint);

        if (Abs(edge.distance - newDistance) > epa_tolerance)
        {
            // Insert the support vertex so that it expands our polytope
            polytope.vertices.Insert(edge.index + 1, supportPoint);
        }
        else
        {
            // We finally reached the closest outer edge!
            break;
        }
    }

    result->contactNormal = edge.normal;
    result->penetrationDepth = edge.distance;
}

static void ClipEdge(Edge* e, const Vec2& p, const Vec2& dir, bool removeClippedPoint)
{
    float d1 = Dot(e->p1.p - p, dir);
    float d2 = Dot(e->p2.p - p, dir);

    if (d1 >= 0 && d2 >= 0)
    {
        return;
    }

    if (d1 < 0)
    {
        if (removeClippedPoint)
        {
            e->p1 = e->p2;
        }
        else
        {
            e->p1.p = e->p1.p + (e->p2.p - e->p1.p) * (-d1 / (Abs(d1) + Abs(d2)));
        }
    }
    else if (d2 < 0)
    {
        if (removeClippedPoint)
        {
            e->p2 = e->p1;
        }
        else
        {
            e->p2.p = e->p2.p + (e->p1.p - e->p2.p) * (-d2 / (Abs(d1) + Abs(d2)));
        }
    }
}

static void FindContactPoints(
    const Vec2& n, const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold
)
{
    Edge edgeA = a->GetFeaturedEdge(tfA, n);
    Edge edgeB = b->GetFeaturedEdge(tfB, -n);

    edgeA.Translate(n * a->GetRadius());
    edgeB.Translate(-n * b->GetRadius());

    Edge* ref = &edgeA; // Reference edge
    Edge* inc = &edgeB; // Incident edge
    manifold->contactNormal = n;
    manifold->featureFlipped = false;

    float aPerpendicularness = Abs(Dot(edgeA.tangent, n));
    float bPerpendicularness = Abs(Dot(edgeB.tangent, n));

    if (bPerpendicularness < aPerpendicularness)
    {
        ref = &edgeB;
        inc = &edgeA;
        manifold->contactNormal = -n;
        manifold->featureFlipped = true;
    }

    ClipEdge(inc, ref->p1.p, ref->tangent, false);
    ClipEdge(inc, ref->p2.p, -ref->tangent, false);
    ClipEdge(inc, ref->p1.p, -manifold->contactNormal, true);

    // To ensure consistent warm starting, the contact point id is always set based on Shape A
    if (inc->GetLength2() <= contact_merge_threshold)
    {
        // If two points are closer than the threshold, merge them into one point
        manifold->contactPoints[0].id = edgeA.p1.id;
        manifold->contactPoints[0].p = inc->p1.p;
        manifold->contactCount = 1;
    }
    else
    {
        manifold->contactPoints[0].id = edgeA.p1.id;
        manifold->contactPoints[0].p = inc->p1.p;
        manifold->contactPoints[1].id = edgeA.p2.id;
        manifold->contactPoints[1].p = inc->p2.p;
        manifold->contactCount = 2;
    }

    manifold->referencePoint = ref->p1;
}

inline bool CircleVsCircle(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    Vec2 pa = Mul(tfA, a->GetCenter());
    Vec2 pb = Mul(tfB, b->GetCenter());
    Vec2 d = pb - pa;

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    float distance2 = d.Length2();
    if (distance2 > radii * radii || distance2 == 0.0f)
    {
        return false;
    }

    float distance = Sqrt(distance2);
    Vec2 normal = d / distance;

    manifold->contactNormal = normal;
    manifold->contactTangent.Set(-normal.y, normal.x);
    manifold->contactPoints[0].id = 0;
    manifold->contactPoints[0].p = pb - normal * rb;
    manifold->referencePoint.id = 0;
    manifold->referencePoint.p = pa + normal * ra;
    manifold->contactCount = 1;
    manifold->penetrationDepth = radii - distance;
    manifold->featureFlipped = false;

    return true;
}

inline bool CapsuleVsCircle(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const Capsule* c = (const Capsule*)a;
    Vec2 va = c->GetVertexA();
    Vec2 vb = c->GetVertexB();
    Vec2 l = vb - va;

    Vec2 pb = Mul(tfB, b->GetCenter());
    Vec2 localP = MulT(tfA, pb);

    Vec2 bp = localP - vb;
    Vec2 ap = localP - va;

    float u = Dot(bp, -l);
    float v = Dot(ap, l);

    // Find closest point depending on the Voronoi region
    Vec2 normal;
    float distance;
    int32 index;

    if (v <= 0.0f) // Region A
    {
        normal = ap;
        distance = normal.Normalize();
        index = 0;
    }
    else if (u <= 0.0f) // Region B
    {
        normal = bp;
        distance = normal.Normalize();
        index = 1;
    }
    else // Region AB
    {
        normal = Normalize(Cross(1.0f, l));
        distance = Dot(ap, normal);
        if (distance < 0.0f)
        {
            normal = -normal;
            distance = -distance;
        }
        index = 0;
    }

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    if (distance > radii)
    {
        return false;
    }

    normal = Mul(tfA.rotation, normal);
    Vec2 point = Mul(tfA, (index ? vb : va));

    manifold->contactNormal = normal;
    manifold->contactTangent.Set(-normal.y, normal.x);
    manifold->penetrationDepth = radii - distance;
    manifold->contactPoints[0].id = 0;
    manifold->contactPoints[0].p = pb - normal * rb;
    manifold->referencePoint.id = index;
    manifold->referencePoint.p = point + normal * ra;
    manifold->contactCount = 1;
    manifold->featureFlipped = false;

    return true;
}

inline bool PolygonVsCircle(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const Polygon* p = (const Polygon*)a;
    const Vec2* vertices = p->GetVertices();
    const Vec2* normals = p->GetNormals();
    int32 vertexCount = p->GetVertexCount();

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    Vec2 pb = Mul(tfB, b->GetCenter());
    Vec2 localP = MulT(tfA, pb);

    int32 index = 0;
    float minSeparation = Dot(normals[index], localP - vertices[index]);

    for (int32 i = 1; i < vertexCount; ++i)
    {
        float separation = Dot(normals[i], localP - vertices[i]);
        if (separation > radii)
        {
            return false;
        }

        if (separation > minSeparation)
        {
            minSeparation = separation;
            index = i;
        }
    }

    // Circle center is inside the polygon
    if (minSeparation < 0.0f)
    {
        Vec2 normal = Mul(tfA.rotation, normals[index]);
        Vec2 point = Mul(tfA, vertices[index]);

        manifold->contactNormal = normal;
        manifold->contactTangent.Set(-normal.y, normal.x);
        manifold->penetrationDepth = radii - minSeparation;
        manifold->contactPoints[0].id = 0;
        manifold->contactPoints[0].p = pb - normal * rb;
        manifold->referencePoint.id = index;
        manifold->referencePoint.p = point + normal * ra;
        manifold->contactCount = 1;
        manifold->featureFlipped = false;

        return true;
    }

    Vec2 v0 = vertices[index];
    Vec2 v1 = vertices[(index + 1) % vertexCount];

    Vec2 v0p = localP - v0;
    Vec2 v1p = localP - v1;

    float u = Dot(v1p, v0 - v1);
    float v = Dot(v0p, v1 - v0);

    Vec2 normal;
    float distance;

    if (v <= 0.0f) // Region v0
    {
        normal = v0p;
        distance = normal.Normalize();
    }
    else if (u <= 0.0f) // Region v1
    {
        normal = v1p;
        distance = normal.Normalize();
        index = (index + 1) % vertexCount;
    }
    else // Inside the region
    {
        normal = normals[index];
        distance = Dot(normal, v0p);
    }

    if (distance > radii)
    {
        return false;
    }

    normal = Mul(tfA.rotation, normal);
    Vec2 point = Mul(tfA, vertices[index]);

    manifold->contactNormal = normal;
    manifold->contactTangent.Set(-normal.y, normal.x);
    manifold->penetrationDepth = radii - distance;
    manifold->contactPoints[0].id = 0;
    manifold->contactPoints[0].p = pb - normal * rb;
    manifold->referencePoint.id = index;
    manifold->referencePoint.p = point + normal * ra;
    manifold->contactCount = 1;
    manifold->featureFlipped = false;

    return true;
}

// This works for all possible shape pairs
inline bool ConvexVsConvex(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    GJKResult gjkResult;
    bool collide = GJK(a, tfA, b, tfB, &gjkResult);

    Simplex& simplex = gjkResult.simplex;

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    if (collide == false)
    {
        switch (simplex.count)
        {
        case 1: // vertex vs. vertex collision
            if (gjkResult.distance < radii)
            {
                Vec2 normal = Normalize(origin - simplex.vertices[0].point);

                Point supportA = simplex.vertices[0].pointA;
                Point supportB = simplex.vertices[0].pointB;
                supportA.p += normal * ra;
                supportB.p -= normal * rb;

                manifold->contactNormal = normal;
                manifold->contactTangent.Set(-normal.y, normal.x);
                manifold->contactPoints[0] = supportB;
                manifold->contactCount = 1;
                manifold->referencePoint = supportA;
                manifold->penetrationDepth = radii - gjkResult.distance;
                manifold->featureFlipped = false;

                return true;
            }
            else
            {
                return false;
            }
        case 2: // vertex vs. edge collision
            if (gjkResult.distance < radii)
            {
                Vec2 normal = Normalize(Cross(1.0f, simplex.vertices[1].point - simplex.vertices[0].point));
                Vec2 k = origin - simplex.vertices[0].point;
                if (Dot(normal, k) < 0)
                {
                    normal = -normal;
                }

                manifold->contactNormal = normal;
                manifold->penetrationDepth = radii - gjkResult.distance;
            }
            else
            {
                return false;
            }
        }
    }
    else
    {
        // Expand to a full simplex if the gjk termination simplex has vertices less than 3
        // We need a full n-simplex to start EPA (actually it's rare case)
        switch (simplex.count)
        {
        case 1:
        {
            SupportPoint support = CSOSupport(a, tfA, b, tfB, Vec2{ 1.0f, 0.0f });
            if (support.point == simplex.vertices[0].point)
            {
                support = CSOSupport(a, tfA, b, tfB, Vec2{ -1.0f, 0.0f });
            }

            simplex.AddVertex(support);
        }

            [[fallthrough]];

        case 2:
        {
            Vec2 normal = Normalize(Cross(1.0f, simplex.vertices[1].point - simplex.vertices[0].point));
            SupportPoint support = CSOSupport(a, tfA, b, tfB, normal);

            if (simplex.vertices[0].point == support.point || simplex.vertices[1].point == support.point)
            {
                simplex.AddVertex(CSOSupport(a, tfA, b, tfB, -normal));
            }
            else
            {
                simplex.AddVertex(support);
            }
        }
        }

        EPAResult epaResult;
        EPA(a, tfA, b, tfB, simplex, &epaResult);

        manifold->contactNormal = epaResult.contactNormal;
        manifold->penetrationDepth = epaResult.penetrationDepth;
    }

    FindContactPoints(manifold->contactNormal, a, tfA, b, tfB, manifold);
    manifold->contactTangent.Set(-manifold->contactNormal.y, manifold->contactNormal.x);

    return true;
}

// Initialize the collision detection function map
inline void InitializeDetectionFunctionMap()
{
    if (detection_function_initialized)
    {
        return;
    }

    collide_function_map[Shape::Type::circle][Shape::Type::circle] = &CircleVsCircle;

    collide_function_map[Shape::Type::capsule][Shape::Type::circle] = &CapsuleVsCircle;
    collide_function_map[Shape::Type::capsule][Shape::Type::capsule] = &ConvexVsConvex;

    collide_function_map[Shape::Type::polygon][Shape::Type::circle] = &PolygonVsCircle;
    collide_function_map[Shape::Type::polygon][Shape::Type::capsule] = &ConvexVsConvex;
    collide_function_map[Shape::Type::polygon][Shape::Type::polygon] = &ConvexVsConvex;

    detection_function_initialized = true;
}

inline bool Collide(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    if (detection_function_initialized == false)
    {
        InitializeDetectionFunctionMap();
    }

    static ContactManifold default_manifold;
    if (manifold == nullptr)
    {
        manifold = &default_manifold;
    }

    Shape::Type shapeA = a->GetType();
    Shape::Type shapeB = b->GetType();

    if (shapeB > shapeA)
    {
        MuliAssert(collide_function_map[shapeB][shapeA] != nullptr);

        bool collide = collide_function_map[shapeB][shapeA](b, tfB, a, tfA, manifold);
        manifold->featureFlipped = !manifold->featureFlipped;

        return collide;
    }
    else
    {
        MuliAssert(collide_function_map[shapeA][shapeB] != nullptr);

        return collide_function_map[shapeA][shapeB](a, tfA, b, tfB, manifold);
    }
}

// ============================================================================
// Raycast inline implementations
// ============================================================================

inline bool ShapeCast(
    const Shape* a,
    const Transform& tfA,
    const Shape* b,
    const Transform& tfB,
    const Vec2& translationA,
    const Vec2& translationB,
    ShapeCastOutput* output
)
{
    output->point.SetZero();
    output->normal.SetZero();
    output->t = 1.0f;

    float t = 0.0f;
    Vec2 n = Vec2::zero;

    const float radii = a->GetRadius() + b->GetRadius();
    const Vec2 r = translationB - translationA; // Ray vector

    Simplex simplex;

    // Get CSO support point in inverse ray direction
    int32 idA = a->GetSupport(MulT(tfA.rotation, -r));
    Vec2 pointA = Mul(tfA, a->GetVertex(idA));
    int32 idB = b->GetSupport(MulT(tfB.rotation, r));
    Vec2 pointB = Mul(tfB, b->GetVertex(idB));
    Vec2 v = pointA - pointB;

    const float target = Max(default_radius, radii - toi_position_solver_threshold);
    const float tolerance = linear_slop * 0.1f;

    const int32 maxIterations = 20;
    int32 iteration = 0;

    while (iteration < maxIterations && v.Length() - target > tolerance)
    {
        MuliAssert(simplex.count < 3);

        // Get CSO support point in search direction(-v)
        idA = a->GetSupport(MulT(tfA.rotation, -v));
        pointA = Mul(tfA, a->GetVertex(idA));
        idB = b->GetSupport(MulT(tfB.rotation, v));
        pointB = Mul(tfB, b->GetVertex(idB));
        Vec2 p = pointA - pointB; // Outer vertex of CSO

        // -v is the plane normal at p
        v.Normalize();

        // Find intersection with support plane
        float vp = Dot(v, p);
        float vr = Dot(v, r);

        // March ray by (vp - target) / vr if the new t is greater
        if (vp - target > t * vr)
        {
            if (vr <= 0.0f)
            {
                return false;
            }

            t = (vp - target) / vr;
            if (t > 1.0f)
            {
                return false;
            }

            n = -v;
            simplex.count = 0;
        }

        SupportPoint* vertex = simplex.vertices + simplex.count;
        vertex->pointA.id = idA;
        vertex->pointA.p = pointA;
        vertex->pointB.id = idB;
        vertex->pointB.p = pointB + t * r; // This effectively shifts the ray origin to the new clip plane
        vertex->point = vertex->pointA.p - vertex->pointB.p;
        vertex->weight = 1.0f;
        simplex.count += 1;

        simplex.Advance(Vec2::zero);

        if (simplex.count == 3)
        {
            // Initial overlap
            return false;
        }

        // Update search direciton
        v = simplex.GetClosestPoint();

        ++iteration;
    }

    if (iteration == 0 || t == 0.0f)
    {
        // Initial overlap
        return false;
    }

    simplex.GetWitnessPoint(&pointA, &pointB);

    if (v.Length2() > 0.0f)
    {
        n = -v;
        n.Normalize();
    }

    output->point = pointA + a->GetRadius() * n + translationA * t;
    output->normal = n;
    output->t = t;
    return true;
}

} // namespace flywheel
