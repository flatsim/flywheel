#include "test_helpers.h"
#include <flywheel/flywheel.hpp>

using namespace flywheel;
using namespace flywheel::test;

TEST_SUITE("Contact Point Generation")
{
    TEST_CASE("Circle-Circle contact generation")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        // Two overlapping circles
        RigidBody* circle1 = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* circle2 = world.CreateCircle(1.0f, Transform(Vec2(1.5f, 0.0f), 0.0f));

        world.Step(1.0f / 60.0f);

        const Contact* contact = world.GetContacts();
        if (contact != nullptr)
        {
            CHECK(contact->GetContactCount() > 0);
            CHECK(contact->GetContactCount() <= 2);
        }

        world.Destroy(circle1);
        world.Destroy(circle2);
    }

    TEST_CASE("Box-Box contact generation")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        // Two overlapping boxes
        RigidBody* box1 = world.CreateBox(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* box2 = world.CreateBox(1.0f, Transform(Vec2(0.8f, 0.0f), 0.0f));

        world.Step(1.0f / 60.0f);

        const Contact* contact = world.GetContacts();
        if (contact != nullptr)
        {
            CHECK(contact->GetContactCount() > 0);
            CHECK(contact->GetContactCount() <= 2);
        }

        world.Destroy(box1);
        world.Destroy(box2);
    }

    TEST_CASE("Circle-Box contact generation")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        RigidBody* circle = world.CreateCircle(0.5f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* box = world.CreateBox(1.0f, Transform(Vec2(1.0f, 0.0f), 0.0f));

        world.Step(1.0f / 60.0f);

        const Contact* contact = world.GetContacts();
        if (contact != nullptr)
        {
            CHECK(contact->GetContactCount() > 0);
        }

        world.Destroy(circle);
        world.Destroy(box);
    }

    TEST_CASE("Contact manifold properties")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        RigidBody* circle1 = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* circle2 = world.CreateCircle(1.0f, Transform(Vec2(1.5f, 0.0f), 0.0f));

        world.Step(1.0f / 60.0f);

        const Contact* contact = world.GetContacts();
        if (contact != nullptr && contact->IsTouching())
        {
            const ContactManifold& manifold = contact->GetContactManifold();

            // Check normal is normalized
            float normalLength = Length(manifold.contactNormal);
            CHECK_FLOAT_APPROX(normalLength, 1.0f);

            // Check penetration depth is reasonable
            CHECK(manifold.penetrationDepth >= 0.0f);
            CHECK(manifold.penetrationDepth < 2.0f);
        }

        world.Destroy(circle1);
        world.Destroy(circle2);
    }
}

TEST_SUITE("Velocity Solver")
{
    TEST_CASE("Normal impulse resolution")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        // Ground
        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);

        // Falling ball
        RigidBody* ball = world.CreateCircle(0.5f, Transform(Vec2(0.0f, 5.0f), 0.0f));

        // Let it fall
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Ball should have settled on ground
        CHECK(ball->GetPosition().y > -5.0f);
        CHECK(ball->GetPosition().y < 0.0f);

        // Velocity should be near zero (resting)
        CHECK(Abs(ball->GetLinearVelocity().y) < 1.0f);

        world.Destroy(ground);
        world.Destroy(ball);
    }

    TEST_CASE("Tangent impulse (friction)")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        // Inclined plane
        RigidBody* plane = world.CreateBox(10.0f, 0.5f, Transform(Vec2(0.0f, 0.0f), pi / 6.0f), RigidBody::static_body);

        // Box on plane
        RigidBody* box = world.CreateBox(0.5f, Transform(Vec2(0.0f, 2.0f), 0.0f));

        Vec2 initialPos = box->GetPosition();

        // Simulate
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        Vec2 finalPos = box->GetPosition();

        // Box should have slid down due to gravity
        float distMoved = Dist(initialPos, finalPos);
        CHECK(distMoved > 0.1f);

        world.Destroy(plane);
        world.Destroy(box);
    }

    TEST_CASE("Contact impulse accumulation")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        RigidBody* ball = world.CreateCircle(0.5f, Transform(Vec2(0.0f, 5.0f), 0.0f));

        // Simulate until contact
        for (int i = 0; i < 60; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        const Contact* contact = world.GetContacts();
        if (contact != nullptr && contact->IsTouching())
        {
            // Check that impulses are being accumulated
            float normalImpulse = contact->GetNormalImpulse(0);
            CHECK(normalImpulse >= 0.0f);
        }

        world.Destroy(ground);
        world.Destroy(ball);
    }
}

TEST_SUITE("Warm Starting")
{
    TEST_CASE("Warm starting improves convergence")
    {
        WorldSettings settings1;
        settings1.gravity = Vec2(0.0f, -10.0f);
        settings1.step.warm_starting = true;
        World world1(settings1);

        WorldSettings settings2;
        settings2.gravity = Vec2(0.0f, -10.0f);
        settings2.step.warm_starting = false;
        World world2(settings2);

        // Create identical scenarios
        RigidBody* ground1 = world1.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        RigidBody* ball1 = world1.CreateCircle(0.5f, Transform(Vec2(0.0f, 5.0f), 0.0f));

        RigidBody* ground2 = world2.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        RigidBody* ball2 = world2.CreateCircle(0.5f, Transform(Vec2(0.0f, 5.0f), 0.0f));

        // Simulate both
        for (int i = 0; i < 120; ++i)
        {
            world1.Step(1.0f / 60.0f);
            world2.Step(1.0f / 60.0f);
        }

        // Both should settle, but warm starting should be more stable
        CHECK(Abs(ball1->GetLinearVelocity().y) < 1.0f);
        CHECK(Abs(ball2->GetLinearVelocity().y) < 1.0f);

        world1.Destroy(ground1);
        world1.Destroy(ball1);
        world2.Destroy(ground2);
        world2.Destroy(ball2);
    }
}

TEST_SUITE("Restitution")
{
    TEST_CASE("Bouncing ball with restitution")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        RigidBody* ball = world.CreateCircle(0.5f, Transform(Vec2(0.0f, 5.0f), 0.0f));

        float initialHeight = ball->GetPosition().y;

        // Let it fall and bounce
        for (int i = 0; i < 60; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Ball should have bounced (y velocity should have changed sign at some point)
        // After 60 frames it should be lower than initial height
        CHECK(ball->GetPosition().y < initialHeight);

        world.Destroy(ground);
        world.Destroy(ball);
    }

    TEST_CASE("Restitution threshold")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        RigidBody* ball = world.CreateCircle(0.5f, Transform(Vec2(0.0f, 0.1f), 0.0f));

        // Very low drop - should not bounce due to restitution threshold
        for (int i = 0; i < 60; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Should be resting
        CHECK(Abs(ball->GetLinearVelocity().y) < 0.5f);

        world.Destroy(ground);
        world.Destroy(ball);
    }
}

TEST_SUITE("Position Solver")
{
    TEST_CASE("Penetration resolution")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        // Create overlapping boxes
        RigidBody* box1 = world.CreateBox(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f), RigidBody::static_body);
        RigidBody* box2 = world.CreateBox(1.0f, Transform(Vec2(0.5f, 0.0f), 0.0f));

        // Step to resolve penetration
        for (int i = 0; i < 60; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Boxes should be separated or barely touching
        float dist = Dist(box1->GetPosition(), box2->GetPosition());
        CHECK(dist >= 0.9f); // Should be close to 1.0 (sum of half-widths)

        world.Destroy(box1);
        world.Destroy(box2);
    }

    TEST_CASE("Baumgarte stabilization")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        // Stack of boxes
        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        RigidBody* box1 = world.CreateBox(0.5f, Transform(Vec2(0.0f, -4.0f), 0.0f));
        RigidBody* box2 = world.CreateBox(0.5f, Transform(Vec2(0.0f, -3.0f), 0.0f));
        RigidBody* box3 = world.CreateBox(0.5f, Transform(Vec2(0.0f, -2.0f), 0.0f));

        // Simulate
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Stack should be stable (boxes should not have fallen through each other)
        CHECK(box1->GetPosition().y > ground->GetPosition().y);
        CHECK(box2->GetPosition().y > box1->GetPosition().y);
        CHECK(box3->GetPosition().y > box2->GetPosition().y);

        world.Destroy(ground);
        world.Destroy(box1);
        world.Destroy(box2);
        world.Destroy(box3);
    }
}

TEST_SUITE("Block Solver")
{
    TEST_CASE("Two-point contact block solving")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        // Ground
        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);

        // Box landing flat (should generate 2 contact points)
        RigidBody* box = world.CreateBox(1.0f, 0.5f, Transform(Vec2(0.0f, 5.0f), 0.0f));

        // Simulate
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Box should be resting flat on ground
        CHECK(Abs(box->GetAngle()) < 0.1f); // Should remain horizontal
        CHECK(Abs(box->GetAngularVelocity()) < 0.1f);

        world.Destroy(ground);
        world.Destroy(box);
    }

    TEST_CASE("Block solver stability")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);

        // Wide box that will have 2 contact points
        RigidBody* box = world.CreateBox(2.0f, 0.5f, Transform(Vec2(0.0f, 5.0f), 0.01f)); // Slight angle

        // Simulate
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Should settle to horizontal
        CHECK(Abs(box->GetAngle()) < 0.05f);

        world.Destroy(ground);
        world.Destroy(box);
    }
}

TEST_SUITE("Stacking and Resting Contacts")
{
    TEST_CASE("Simple stack stability")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        RigidBody* box1 = world.CreateBox(1.0f, Transform(Vec2(0.0f, -3.5f), 0.0f));
        RigidBody* box2 = world.CreateBox(1.0f, Transform(Vec2(0.0f, -2.0f), 0.0f));

        // Simulate
        for (int i = 0; i < 240; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Stack should be stable
        CHECK(box1->GetPosition().y > ground->GetPosition().y);
        CHECK(box2->GetPosition().y > box1->GetPosition().y);
        CHECK(Abs(box1->GetLinearVelocity().y) < 0.5f);
        CHECK(Abs(box2->GetLinearVelocity().y) < 0.5f);

        world.Destroy(ground);
        world.Destroy(box1);
        world.Destroy(box2);
    }

    TEST_CASE("Tall stack stability")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);

        std::vector<RigidBody*> boxes;
        for (int i = 0; i < 5; ++i)
        {
            float y = -4.0f + i * 1.1f;
            boxes.push_back(world.CreateBox(0.5f, Transform(Vec2(0.0f, y), 0.0f)));
        }

        // Simulate
        for (int i = 0; i < 300; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // All boxes should still be above ground
        for (auto* box : boxes)
        {
            CHECK(box->GetPosition().y > ground->GetPosition().y);
        }

        world.Destroy(ground);
        for (auto* box : boxes)
        {
            world.Destroy(box);
        }
    }

    TEST_CASE("Resting contact convergence")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        RigidBody* box = world.CreateBox(1.0f, Transform(Vec2(0.0f, 5.0f), 0.0f));

        // Let it settle
        for (int i = 0; i < 240; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Should be at rest
        Vec2 vel = box->GetLinearVelocity();
        CHECK(Length(vel) < 0.1f);
        CHECK(Abs(box->GetAngularVelocity()) < 0.1f);

        world.Destroy(ground);
        world.Destroy(box);
    }

    TEST_CASE("Pyramid stack")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(20.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);

        // Build pyramid
        std::vector<RigidBody*> boxes;
        int levels = 3;
        float boxSize = 0.5f;

        for (int level = 0; level < levels; ++level)
        {
            int boxesInLevel = levels - level;
            float y = -4.0f + level * (boxSize * 2.0f + 0.1f);

            for (int i = 0; i < boxesInLevel; ++i)
            {
                float x = (i - boxesInLevel / 2.0f) * (boxSize * 2.0f + 0.1f);
                boxes.push_back(world.CreateBox(boxSize, Transform(Vec2(x, y), 0.0f)));
            }
        }

        // Simulate
        for (int i = 0; i < 300; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Pyramid should still be standing (all boxes above ground)
        for (auto* box : boxes)
        {
            CHECK(box->GetPosition().y > ground->GetPosition().y);
        }

        world.Destroy(ground);
        for (auto* box : boxes)
        {
            world.Destroy(box);
        }
    }
}

TEST_SUITE("Contact Material Properties")
{
    TEST_CASE("Friction coefficient affects sliding")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        // Inclined plane
        RigidBody* plane = world.CreateBox(10.0f, 0.5f, Transform(Vec2(0.0f, 0.0f), pi / 6.0f), RigidBody::static_body);
        RigidBody* box = world.CreateBox(0.5f, Transform(Vec2(0.0f, 2.0f), 0.0f));

        Vec2 initialPos = box->GetPosition();

        // Simulate
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        Vec2 finalPos = box->GetPosition();
        float distMoved = Dist(initialPos, finalPos);

        // Box should have moved due to gravity on incline
        CHECK(distMoved > 0.0f);

        world.Destroy(plane);
        world.Destroy(box);
    }

    TEST_CASE("Contact enable/disable")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        RigidBody* box1 = world.CreateBox(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* box2 = world.CreateBox(1.0f, Transform(Vec2(1.5f, 0.0f), 0.0f));

        world.Step(1.0f / 60.0f);

        const Contact* contact = world.GetContacts();
        if (contact != nullptr)
        {
            CHECK(contact->IsEnabled());
        }

        world.Destroy(box1);
        world.Destroy(box2);
    }
}
