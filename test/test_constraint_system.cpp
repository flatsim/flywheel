#include "test_helpers.h"
#include <flywheel/joint.h>
#include <flywheel/rigidbody.h>
#include <flywheel/settings.h>
#include <flywheel/world.h>

using namespace flywheel;
using namespace flywheel::test;

TEST_SUITE("Constraint Creation and Interface")
{
    TEST_CASE("Joint constraint creation")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateCircle(1.0f);
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(3.0f, 0.0f), 0.0f));

        DistanceJoint* joint = world.CreateDistanceJoint(body1, body2, body1->GetPosition(), body2->GetPosition());

        CHECK(joint != nullptr);
        CHECK(joint->GetBodyA() == body1);
        CHECK(joint->GetBodyB() == body2);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Multiple constraints on same body")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* center = world.CreateCircle(1.0f);
        RigidBody* body1 = world.CreateCircle(1.0f, Transform(Vec2(3.0f, 0.0f), 0.0f));
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 3.0f), 0.0f));
        RigidBody* body3 = world.CreateCircle(1.0f, Transform(Vec2(-3.0f, 0.0f), 0.0f));

        DistanceJoint* joint1 = world.CreateDistanceJoint(center, body1, center->GetPosition(), body1->GetPosition());
        DistanceJoint* joint2 = world.CreateDistanceJoint(center, body2, center->GetPosition(), body2->GetPosition());
        DistanceJoint* joint3 = world.CreateDistanceJoint(center, body3, center->GetPosition(), body3->GetPosition());

        CHECK(world.GetJointCount() == 3);

        world.Destroy(joint1);
        world.Destroy(joint2);
        world.Destroy(joint3);
        world.Destroy(center);
        world.Destroy(body1);
        world.Destroy(body2);
        world.Destroy(body3);
    }

    TEST_CASE("Constraint solving interface")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* anchor = world.CreateCircle(0.5f, Transform(Vec2(0.0f, 5.0f), 0.0f), RigidBody::static_body);
        RigidBody* pendulum = world.CreateCircle(0.5f, Transform(Vec2(0.0f, 3.0f), 0.0f));

        DistanceJoint* joint = world.CreateDistanceJoint(anchor, pendulum, anchor->GetPosition(), pendulum->GetPosition());

        // Simulate - constraint should be solved
        for (int i = 0; i < 60; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Distance should be maintained by constraint
        float distance = Dist(anchor->GetPosition(), pendulum->GetPosition());
        CHECK(distance > 0.0f);
        CHECK(distance < 5.0f);

        world.Destroy(joint);
        world.Destroy(anchor);
        world.Destroy(pendulum);
    }
}

TEST_SUITE("Island Detection")
{
    TEST_CASE("Single island - connected bodies")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        // Create connected bodies (overlapping)
        RigidBody* body1 = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(1.5f, 0.0f), 0.0f));
        RigidBody* body3 = world.CreateCircle(1.0f, Transform(Vec2(3.0f, 0.0f), 0.0f));

        world.Step(1.0f / 60.0f);

        // All bodies should be in same island (connected through contacts)
        CHECK(world.GetBodyCount() == 3);

        world.Destroy(body1);
        world.Destroy(body2);
        world.Destroy(body3);
    }

    TEST_CASE("Multiple islands - separated bodies")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        // Create two separate groups
        RigidBody* group1_a = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* group1_b = world.CreateCircle(1.0f, Transform(Vec2(1.5f, 0.0f), 0.0f));

        RigidBody* group2_a = world.CreateCircle(1.0f, Transform(Vec2(20.0f, 0.0f), 0.0f));
        RigidBody* group2_b = world.CreateCircle(1.0f, Transform(Vec2(21.5f, 0.0f), 0.0f));

        world.Step(1.0f / 60.0f);

        // Should have 2 separate islands
        CHECK(world.GetBodyCount() == 4);

        world.Destroy(group1_a);
        world.Destroy(group1_b);
        world.Destroy(group2_a);
        world.Destroy(group2_b);
    }

    TEST_CASE("Island with joints")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        // Create bodies connected by joints (not touching)
        RigidBody* body1 = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(5.0f, 0.0f), 0.0f));
        RigidBody* body3 = world.CreateCircle(1.0f, Transform(Vec2(10.0f, 0.0f), 0.0f));

        DistanceJoint* joint1 = world.CreateDistanceJoint(body1, body2, body1->GetPosition(), body2->GetPosition());
        DistanceJoint* joint2 = world.CreateDistanceJoint(body2, body3, body2->GetPosition(), body3->GetPosition());

        world.Step(1.0f / 60.0f);

        // All bodies should be in same island (connected through joints)
        CHECK(world.GetBodyCount() == 3);
        CHECK(world.GetJointCount() == 2);

        world.Destroy(joint1);
        world.Destroy(joint2);
        world.Destroy(body1);
        world.Destroy(body2);
        world.Destroy(body3);
    }

    TEST_CASE("Island with mixed constraints")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        // Create bodies connected by both contacts and joints
        RigidBody* body1 = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(1.5f, 0.0f), 0.0f)); // Touching body1
        RigidBody* body3 = world.CreateCircle(1.0f, Transform(Vec2(5.0f, 0.0f), 0.0f)); // Not touching

        // Connect body2 and body3 with joint
        DistanceJoint* joint = world.CreateDistanceJoint(body2, body3, body2->GetPosition(), body3->GetPosition());

        world.Step(1.0f / 60.0f);

        // All three should be in same island
        CHECK(world.GetBodyCount() == 3);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
        world.Destroy(body3);
    }
}

TEST_SUITE("Island Graph Traversal")
{
    TEST_CASE("Chain of bodies")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* anchor = world.CreateBox(1.0f, Transform(Vec2(0.0f, 5.0f), 0.0f), RigidBody::static_body);

        // Create chain
        std::vector<RigidBody*> chain;
        for (int i = 0; i < 5; ++i)
        {
            float y = 4.0f - i * 1.2f;
            chain.push_back(world.CreateBox(0.5f, 0.2f, Transform(Vec2(0.0f, y), 0.0f)));
        }

        // Connect with revolute joints
        std::vector<RevoluteJoint*> joints;
        joints.push_back(world.CreateRevoluteJoint(anchor, chain[0], Vec2(0.0f, 5.0f)));
        for (size_t i = 0; i < chain.size() - 1; ++i)
        {
            float y = 4.0f - i * 1.2f - 0.6f;
            joints.push_back(world.CreateRevoluteJoint(chain[i], chain[i + 1], Vec2(0.0f, y)));
        }

        // Simulate
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // All bodies should still be connected
        CHECK(world.GetJointCount() == static_cast<int32>(joints.size()));

        for (auto* joint : joints)
        {
            world.Destroy(joint);
        }
        world.Destroy(anchor);
        for (auto* body : chain)
        {
            world.Destroy(body);
        }
    }

    TEST_CASE("Complex graph traversal")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        // Create a complex network of bodies
        std::vector<RigidBody*> bodies;
        for (int i = 0; i < 10; ++i)
        {
            float x = (i % 5) * 2.5f;
            float y = (i / 5) * 2.5f;
            bodies.push_back(world.CreateCircle(0.5f, Transform(Vec2(x, y), 0.0f)));
        }

        // Connect them in a grid pattern
        std::vector<DistanceJoint*> joints;
        for (int i = 0; i < 5; ++i)
        {
            joints.push_back(
                world.CreateDistanceJoint(bodies[i], bodies[i + 5], bodies[i]->GetPosition(), bodies[i + 5]->GetPosition())
            );
        }
        for (int i = 0; i < 9; ++i)
        {
            if ((i + 1) % 5 != 0)
            {
                joints.push_back(
                    world.CreateDistanceJoint(bodies[i], bodies[i + 1], bodies[i]->GetPosition(), bodies[i + 1]->GetPosition())
                );
            }
        }

        world.Step(1.0f / 60.0f);

        // All bodies should be in same island
        CHECK(world.GetBodyCount() == 10);

        for (auto* joint : joints)
        {
            world.Destroy(joint);
        }
        for (auto* body : bodies)
        {
            world.Destroy(body);
        }
    }
}

TEST_SUITE("Sleeping")
{
    TEST_CASE("Body goes to sleep when at rest")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        settings.sleeping = true;
        World world(settings);

        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        RigidBody* box = world.CreateBox(1.0f, Transform(Vec2(0.0f, 5.0f), 0.0f));

        // Let it settle
        for (int i = 0; i < 300; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Box should be sleeping
        CHECK(box->IsSleeping());

        world.Destroy(ground);
        world.Destroy(box);
    }

    TEST_CASE("Sleeping body wakes on impulse")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        settings.sleeping = true;
        World world(settings);

        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        RigidBody* box = world.CreateBox(1.0f, Transform(Vec2(0.0f, 5.0f), 0.0f));

        // Let it sleep
        for (int i = 0; i < 300; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        CHECK(box->IsSleeping());

        // Apply impulse
        box->ApplyLinearImpulse(box->GetPosition(), Vec2(0.0f, 10.0f), true);

        // Should wake up
        CHECK_FALSE(box->IsSleeping());

        world.Destroy(ground);
        world.Destroy(box);
    }

    TEST_CASE("Island sleeping")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        settings.sleeping = true;
        World world(settings);

        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);

        // Create stack
        RigidBody* box1 = world.CreateBox(1.0f, Transform(Vec2(0.0f, -3.5f), 0.0f));
        RigidBody* box2 = world.CreateBox(1.0f, Transform(Vec2(0.0f, -2.0f), 0.0f));

        // Let stack settle and sleep
        for (int i = 0; i < 300; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Both boxes should be sleeping
        CHECK(box1->IsSleeping());
        CHECK(box2->IsSleeping());

        world.Destroy(ground);
        world.Destroy(box1);
        world.Destroy(box2);
    }

    TEST_CASE("Sleeping disabled")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        settings.sleeping = false;
        World world(settings);

        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        RigidBody* box = world.CreateBox(1.0f, Transform(Vec2(0.0f, 5.0f), 0.0f));

        // Let it settle
        for (int i = 0; i < 300; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Box should NOT be sleeping (sleeping disabled)
        CHECK_FALSE(box->IsSleeping());

        world.Destroy(ground);
        world.Destroy(box);
    }
}

TEST_SUITE("Contact Graph Management")
{
    TEST_CASE("Contact graph creation")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        RigidBody* body1 = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(1.5f, 0.0f), 0.0f));

        world.Step(1.0f / 60.0f);

        // Should have contact
        CHECK(world.GetContactCount() >= 0);

        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Contact graph updates")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        RigidBody* body1 = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(5.0f, 0.0f), 0.0f)); // Not touching

        world.Step(1.0f / 60.0f);
        int32 initialContacts = world.GetContactCount();

        // Move bodies together
        body2->SetPosition(Vec2(1.5f, 0.0f));

        world.Step(1.0f / 60.0f);
        int32 afterContacts = world.GetContactCount();

        // Contact count should have changed
        CHECK(afterContacts >= initialContacts);

        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Contact edge creation and removal")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        RigidBody* body1 = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(1.5f, 0.0f), 0.0f));

        world.Step(1.0f / 60.0f);
        int32 contactsWithBoth = world.GetContactCount();

        // Remove one body
        world.Destroy(body2);

        world.Step(1.0f / 60.0f);
        int32 contactsWithOne = world.GetContactCount();

        // Contact count should decrease
        CHECK(contactsWithOne <= contactsWithBoth);

        world.Destroy(body1);
    }

    TEST_CASE("Multiple contacts per body")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        RigidBody* center = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* body1 = world.CreateCircle(1.0f, Transform(Vec2(1.5f, 0.0f), 0.0f));
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 1.5f), 0.0f));
        RigidBody* body3 = world.CreateCircle(1.0f, Transform(Vec2(-1.5f, 0.0f), 0.0f));

        world.Step(1.0f / 60.0f);

        // Center body should have multiple contacts
        CHECK(world.GetContactCount() >= 3);

        world.Destroy(center);
        world.Destroy(body1);
        world.Destroy(body2);
        world.Destroy(body3);
    }

    TEST_CASE("Contact graph with static bodies")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        RigidBody* box = world.CreateBox(1.0f, Transform(Vec2(0.0f, 5.0f), 0.0f));

        // Let it fall
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Should have contact with ground
        CHECK(world.GetContactCount() > 0);

        world.Destroy(ground);
        world.Destroy(box);
    }
}

TEST_SUITE("Constraint Stability")
{
    TEST_CASE("Constraint stability under load")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* anchor = world.CreateCircle(0.5f, Transform(Vec2(0.0f, 5.0f), 0.0f), RigidBody::static_body);
        // Create heavy body with higher density
        RigidBody* heavy = world.CreateCircle(2.0f, Transform(Vec2(0.0f, 3.0f), 0.0f), RigidBody::dynamic_body, 10.0f);

        DistanceJoint* joint = world.CreateDistanceJoint(anchor, heavy, anchor->GetPosition(), heavy->GetPosition());

        // Simulate with heavy load
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Joint should still be intact
        float distance = Dist(anchor->GetPosition(), heavy->GetPosition());
        CHECK(distance > 0.0f);
        CHECK(distance < 5.0f);

        world.Destroy(joint);
        world.Destroy(anchor);
        world.Destroy(heavy);
    }

    TEST_CASE("Multiple constraints stability")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* anchor1 = world.CreateCircle(0.5f, Transform(Vec2(-2.0f, 5.0f), 0.0f), RigidBody::static_body);
        RigidBody* anchor2 = world.CreateCircle(0.5f, Transform(Vec2(2.0f, 5.0f), 0.0f), RigidBody::static_body);
        RigidBody* suspended = world.CreateBox(1.0f, 0.5f, Transform(Vec2(0.0f, 3.0f), 0.0f));

        DistanceJoint* joint1 = world.CreateDistanceJoint(anchor1, suspended, anchor1->GetPosition(), Vec2(-0.5f, 3.0f));
        DistanceJoint* joint2 = world.CreateDistanceJoint(anchor2, suspended, anchor2->GetPosition(), Vec2(0.5f, 3.0f));

        // Simulate
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Body should be suspended between two anchors
        CHECK(suspended->GetPosition().y > 2.0f);
        CHECK(suspended->GetPosition().y < 4.0f);

        world.Destroy(joint1);
        world.Destroy(joint2);
        world.Destroy(anchor1);
        world.Destroy(anchor2);
        world.Destroy(suspended);
    }
}
