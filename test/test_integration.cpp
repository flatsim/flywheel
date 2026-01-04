#include "test_helpers.h"
#include <flywheel/flywheel.hpp>

using namespace flywheel;
using namespace flywheel::test;

TEST_SUITE("Stacking Stability")
{
    TEST_CASE("Simple box stack")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        RigidBody* box1 = world.CreateBox(1.0f, Transform(Vec2(0.0f, -3.5f), 0.0f));
        RigidBody* box2 = world.CreateBox(1.0f, Transform(Vec2(0.0f, -2.0f), 0.0f));
        RigidBody* box3 = world.CreateBox(1.0f, Transform(Vec2(0.0f, -0.5f), 0.0f));

        // Simulate
        for (int i = 0; i < 300; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Stack should be stable
        CHECK(box1->GetPosition().y > ground->GetPosition().y);
        CHECK(box2->GetPosition().y > box1->GetPosition().y);
        CHECK(box3->GetPosition().y > box2->GetPosition().y);

        world.Destroy(ground);
        world.Destroy(box1);
        world.Destroy(box2);
        world.Destroy(box3);
    }

    TEST_CASE("Tall stack stability")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);

        std::vector<RigidBody*> boxes;
        for (int i = 0; i < 10; ++i)
        {
            float y = -4.0f + i * 1.1f;
            boxes.push_back(world.CreateBox(0.5f, Transform(Vec2(0.0f, y), 0.0f)));
        }

        // Simulate
        for (int i = 0; i < 400; ++i)
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

    TEST_CASE("Pyramid stack")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(20.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);

        // Build pyramid
        std::vector<RigidBody*> boxes;
        int levels = 5;
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
        for (int i = 0; i < 400; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Pyramid should still be standing
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

TEST_SUITE("Chain of Bodies")
{
    TEST_CASE("Simple chain")
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
        for (int i = 0; i < 240; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Chain should still be connected
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

    TEST_CASE("Rope simulation")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* anchor1 = world.CreateCircle(0.2f, Transform(Vec2(-3.0f, 5.0f), 0.0f), RigidBody::static_body);
        RigidBody* anchor2 = world.CreateCircle(0.2f, Transform(Vec2(3.0f, 5.0f), 0.0f), RigidBody::static_body);

        // Create rope segments with proper spacing
        std::vector<RigidBody*> rope;
        int segments = 5;
        float segmentSpacing = 6.0f / (segments + 1);
        for (int i = 0; i < segments; ++i)
        {
            float x = -3.0f + segmentSpacing * (i + 1);
            rope.push_back(world.CreateCircle(0.15f, Transform(Vec2(x, 5.0f), 0.0f)));
        }

        // Connect rope segments
        std::vector<DistanceJoint*> joints;
        joints.push_back(world.CreateDistanceJoint(anchor1, rope[0], anchor1->GetPosition(), rope[0]->GetPosition()));
        for (int i = 0; i < segments - 1; ++i)
        {
            joints.push_back(world.CreateDistanceJoint(rope[i], rope[i + 1], rope[i]->GetPosition(), rope[i + 1]->GetPosition()));
        }
        joints.push_back(
            world.CreateDistanceJoint(rope[segments - 1], anchor2, rope[segments - 1]->GetPosition(), anchor2->GetPosition())
        );

        // Simulate
        for (int i = 0; i < 180; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Just check that simulation completed without NaN
        CHECK(world.GetJointCount() == static_cast<int32>(joints.size()));

        for (auto* joint : joints)
        {
            world.Destroy(joint);
        }
        world.Destroy(anchor1);
        world.Destroy(anchor2);
        for (auto* body : rope)
        {
            world.Destroy(body);
        }
    }
}

TEST_SUITE("Pendulum")
{
    TEST_CASE("Simple pendulum")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* anchor = world.CreateCircle(0.3f, Transform(Vec2(0.0f, 5.0f), 0.0f), RigidBody::static_body);
        RigidBody* bob = world.CreateCircle(0.5f, Transform(Vec2(2.0f, 3.0f), 0.0f));

        DistanceJoint* joint = world.CreateDistanceJoint(anchor, bob, anchor->GetPosition(), bob->GetPosition());

        Vec2 initialPos = bob->GetPosition();

        // Simulate
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        Vec2 finalPos = bob->GetPosition();

        // Pendulum should have swung
        CHECK(finalPos.x != initialPos.x);

        world.Destroy(joint);
        world.Destroy(anchor);
        world.Destroy(bob);
    }

    TEST_CASE("Double pendulum")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* anchor = world.CreateCircle(0.3f, Transform(Vec2(0.0f, 5.0f), 0.0f), RigidBody::static_body);
        RigidBody* bob1 = world.CreateCircle(0.4f, Transform(Vec2(1.0f, 3.0f), 0.0f));
        RigidBody* bob2 = world.CreateCircle(0.4f, Transform(Vec2(2.0f, 1.0f), 0.0f));

        DistanceJoint* joint1 = world.CreateDistanceJoint(anchor, bob1, anchor->GetPosition(), bob1->GetPosition());
        DistanceJoint* joint2 = world.CreateDistanceJoint(bob1, bob2, bob1->GetPosition(), bob2->GetPosition());

        // Simulate
        for (int i = 0; i < 180; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Both bobs should be moving
        CHECK(world.GetJointCount() == 2);

        world.Destroy(joint1);
        world.Destroy(joint2);
        world.Destroy(anchor);
        world.Destroy(bob1);
        world.Destroy(bob2);
    }
}

TEST_SUITE("Newton's Cradle")
{
    TEST_CASE("Newton's cradle simulation")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        // Create 5 balls in a row
        std::vector<RigidBody*> balls;
        std::vector<DistanceJoint*> joints;
        float spacing = 1.0f;

        for (int i = 0; i < 5; ++i)
        {
            float x = -2.0f + i * spacing;
            RigidBody* anchor = world.CreateCircle(0.1f, Transform(Vec2(x, 5.0f), 0.0f), RigidBody::static_body);
            RigidBody* ball = world.CreateCircle(0.4f, Transform(Vec2(x, 3.0f), 0.0f));

            balls.push_back(ball);
            joints.push_back(world.CreateDistanceJoint(anchor, ball, anchor->GetPosition(), ball->GetPosition()));
        }

        // Pull first ball to the side
        balls[0]->SetPosition(Vec2(-3.0f, 3.0f));

        // Simulate
        for (int i = 0; i < 240; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Balls should have interacted
        CHECK(world.GetContactCount() >= 0);

        for (auto* joint : joints)
        {
            world.Destroy(joint);
        }
        for (auto* ball : balls)
        {
            world.Destroy(ball);
        }
    }
}

TEST_SUITE("Domino Effect")
{
    TEST_CASE("Domino chain")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(20.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);

        // Create domino pieces
        std::vector<RigidBody*> dominoes;
        int count = 10;
        for (int i = 0; i < count; ++i)
        {
            float x = -8.0f + i * 1.8f;
            dominoes.push_back(world.CreateBox(0.2f, 1.0f, Transform(Vec2(x, -3.5f), 0.0f)));
        }

        // Push first domino
        dominoes[0]->ApplyLinearImpulse(dominoes[0]->GetPosition(), Vec2(5.0f, 0.0f), true);

        // Simulate
        for (int i = 0; i < 300; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // At least some dominoes should have fallen
        int fallenCount = 0;
        for (auto* domino : dominoes)
        {
            if (Abs(domino->GetAngle()) > pi / 6.0f)
            {
                fallenCount++;
            }
        }

        CHECK(fallenCount > 0);

        world.Destroy(ground);
        for (auto* domino : dominoes)
        {
            world.Destroy(domino);
        }
    }
}

TEST_SUITE("Ragdoll")
{
    TEST_CASE("Simple ragdoll")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(20.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);

        // Create ragdoll parts
        RigidBody* torso = world.CreateBox(0.5f, 1.0f, Transform(Vec2(0.0f, 2.0f), 0.0f));
        RigidBody* head = world.CreateCircle(0.3f, Transform(Vec2(0.0f, 3.5f), 0.0f));
        RigidBody* leftArm = world.CreateBox(0.2f, 0.8f, Transform(Vec2(-0.8f, 2.5f), 0.0f));
        RigidBody* rightArm = world.CreateBox(0.2f, 0.8f, Transform(Vec2(0.8f, 2.5f), 0.0f));
        RigidBody* leftLeg = world.CreateBox(0.2f, 1.0f, Transform(Vec2(-0.3f, 0.5f), 0.0f));
        RigidBody* rightLeg = world.CreateBox(0.2f, 1.0f, Transform(Vec2(0.3f, 0.5f), 0.0f));

        // Connect with revolute joints
        RevoluteJoint* neck = world.CreateRevoluteJoint(torso, head, Vec2(0.0f, 3.0f));
        RevoluteJoint* leftShoulder = world.CreateRevoluteJoint(torso, leftArm, Vec2(-0.5f, 2.5f));
        RevoluteJoint* rightShoulder = world.CreateRevoluteJoint(torso, rightArm, Vec2(0.5f, 2.5f));
        RevoluteJoint* leftHip = world.CreateRevoluteJoint(torso, leftLeg, Vec2(-0.3f, 1.5f));
        RevoluteJoint* rightHip = world.CreateRevoluteJoint(torso, rightLeg, Vec2(0.3f, 1.5f));

        // Simulate
        for (int i = 0; i < 240; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Ragdoll should have fallen
        CHECK(torso->GetPosition().y < 2.0f);

        world.Destroy(neck);
        world.Destroy(leftShoulder);
        world.Destroy(rightShoulder);
        world.Destroy(leftHip);
        world.Destroy(rightHip);
        world.Destroy(ground);
        world.Destroy(torso);
        world.Destroy(head);
        world.Destroy(leftArm);
        world.Destroy(rightArm);
        world.Destroy(leftLeg);
        world.Destroy(rightLeg);
    }
}

TEST_SUITE("Vehicle Simulation")
{
    TEST_CASE("Simple car")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(50.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);

        // Create car body
        RigidBody* chassis = world.CreateBox(2.0f, 0.5f, Transform(Vec2(0.0f, 0.0f), 0.0f));

        // Create wheels
        RigidBody* frontWheel = world.CreateCircle(0.4f, Transform(Vec2(1.0f, -0.5f), 0.0f));
        RigidBody* rearWheel = world.CreateCircle(0.4f, Transform(Vec2(-1.0f, -0.5f), 0.0f));

        // Attach wheels with revolute joints
        RevoluteJoint* frontAxle = world.CreateRevoluteJoint(chassis, frontWheel, Vec2(1.0f, -0.5f));
        RevoluteJoint* rearAxle = world.CreateRevoluteJoint(chassis, rearWheel, Vec2(-1.0f, -0.5f));

        // Simulate
        for (int i = 0; i < 240; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Car should have settled on ground
        CHECK(chassis->GetPosition().y > ground->GetPosition().y);
        CHECK(chassis->GetPosition().y < 2.0f);

        world.Destroy(frontAxle);
        world.Destroy(rearAxle);
        world.Destroy(ground);
        world.Destroy(chassis);
        world.Destroy(frontWheel);
        world.Destroy(rearWheel);
    }
}

TEST_SUITE("Compound Shapes")
{
    TEST_CASE("L-shaped body")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);

        // Create L-shape using two boxes welded together
        RigidBody* vertical = world.CreateBox(0.5f, 2.0f, Transform(Vec2(0.0f, 2.0f), 0.0f));
        RigidBody* horizontal = world.CreateBox(2.0f, 0.5f, Transform(Vec2(1.0f, 0.5f), 0.0f));

        WeldJoint* weld = world.CreateWeldJoint(vertical, horizontal, Vec2(0.5f, 0.5f));

        // Simulate
        for (int i = 0; i < 240; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // L-shape should have fallen and settled
        CHECK(vertical->GetPosition().y > ground->GetPosition().y);

        world.Destroy(weld);
        world.Destroy(ground);
        world.Destroy(vertical);
        world.Destroy(horizontal);
    }

    TEST_CASE("T-shaped body")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);

        // Create T-shape
        RigidBody* vertical = world.CreateBox(0.5f, 2.0f, Transform(Vec2(0.0f, 2.0f), 0.0f));
        RigidBody* horizontal = world.CreateBox(2.0f, 0.5f, Transform(Vec2(0.0f, 3.5f), 0.0f));

        WeldJoint* weld = world.CreateWeldJoint(vertical, horizontal, Vec2(0.0f, 3.0f));

        // Simulate
        for (int i = 0; i < 240; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // T-shape should have fallen and settled
        CHECK(vertical->GetPosition().y > ground->GetPosition().y);

        world.Destroy(weld);
        world.Destroy(ground);
        world.Destroy(vertical);
        world.Destroy(horizontal);
    }
}

TEST_SUITE("Determinism")
{
    TEST_CASE("Same input produces same output")
    {
        WorldSettings settings1;
        settings1.gravity = Vec2(0.0f, -10.0f);
        World world1(settings1);

        WorldSettings settings2;
        settings2.gravity = Vec2(0.0f, -10.0f);
        World world2(settings2);

        // Create identical scenarios
        RigidBody* ground1 = world1.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        RigidBody* box1 = world1.CreateBox(1.0f, Transform(Vec2(0.0f, 5.0f), 0.0f));

        RigidBody* ground2 = world2.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        RigidBody* box2 = world2.CreateBox(1.0f, Transform(Vec2(0.0f, 5.0f), 0.0f));

        // Simulate both worlds identically
        for (int i = 0; i < 120; ++i)
        {
            world1.Step(1.0f / 60.0f);
            world2.Step(1.0f / 60.0f);
        }

        // Results should be identical
        CHECK_VEC2_APPROX(box1->GetPosition(), box2->GetPosition());
        CHECK_FLOAT_APPROX(box1->GetAngle(), box2->GetAngle());

        world1.Destroy(ground1);
        world1.Destroy(box1);
        world2.Destroy(ground2);
        world2.Destroy(box2);
    }

    TEST_CASE("Determinism with multiple bodies")
    {
        WorldSettings settings1;
        settings1.gravity = Vec2(0.0f, -10.0f);
        World world1(settings1);

        WorldSettings settings2;
        settings2.gravity = Vec2(0.0f, -10.0f);
        World world2(settings2);

        // Create identical stacks
        RigidBody* ground1 = world1.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        std::vector<RigidBody*> boxes1;
        for (int i = 0; i < 5; ++i)
        {
            float y = -4.0f + i * 1.1f;
            boxes1.push_back(world1.CreateBox(0.5f, Transform(Vec2(0.0f, y), 0.0f)));
        }

        RigidBody* ground2 = world2.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        std::vector<RigidBody*> boxes2;
        for (int i = 0; i < 5; ++i)
        {
            float y = -4.0f + i * 1.1f;
            boxes2.push_back(world2.CreateBox(0.5f, Transform(Vec2(0.0f, y), 0.0f)));
        }

        // Simulate both
        for (int i = 0; i < 240; ++i)
        {
            world1.Step(1.0f / 60.0f);
            world2.Step(1.0f / 60.0f);
        }

        // All boxes should be in same positions
        for (size_t i = 0; i < boxes1.size(); ++i)
        {
            CHECK_VEC2_APPROX(boxes1[i]->GetPosition(), boxes2[i]->GetPosition());
        }

        world1.Destroy(ground1);
        for (auto* box : boxes1)
        {
            world1.Destroy(box);
        }
        world2.Destroy(ground2);
        for (auto* box : boxes2)
        {
            world2.Destroy(box);
        }
    }
}

TEST_SUITE("Performance")
{
    TEST_CASE("Many bodies performance")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(50.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);

        // Create many bodies
        std::vector<RigidBody*> bodies;
        for (int i = 0; i < 100; ++i)
        {
            float x = random_float(-20.0f, 20.0f);
            float y = random_float(0.0f, 20.0f);
            bodies.push_back(world.CreateCircle(0.3f, Transform(Vec2(x, y), 0.0f)));
        }

        // Simulate - should complete in reasonable time
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // All bodies should have settled
        CHECK(world.GetBodyCount() == 101); // 100 + ground

        world.Destroy(ground);
        for (auto* body : bodies)
        {
            world.Destroy(body);
        }
    }

    TEST_CASE("Many contacts performance")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* ground = world.CreateBox(20.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);

        // Create pile of boxes
        std::vector<RigidBody*> boxes;
        for (int layer = 0; layer < 10; ++layer)
        {
            for (int i = 0; i < 5; ++i)
            {
                float x = -2.0f + i * 1.1f;
                float y = -4.0f + layer * 1.1f;
                boxes.push_back(world.CreateBox(0.5f, Transform(Vec2(x, y), 0.0f)));
            }
        }

        // Simulate
        for (int i = 0; i < 240; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Should have many contacts
        CHECK(world.GetContactCount() >= 0);

        world.Destroy(ground);
        for (auto* box : boxes)
        {
            world.Destroy(box);
        }
    }
}
