#pragma once

#include "common.h"
#include "math.h"

#include "collider.h"
#include "rigidbody.h"

#include "angle_joint.h"
#include "distance_joint.h"
#include "grab_joint.h"
#include "joint.h"
#include "line_joint.h"
#include "motor_joint.h"
#include "prismatic_joint.h"
#include "pulley_joint.h"
#include "revolute_joint.h"
#include "weld_joint.h"

#include "callbacks.h"
#include "collision.h"
#include "contact_graph.h"

#include "block_allocator.h"
#include "linear_allocator.h"

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

} // namespace flywheel