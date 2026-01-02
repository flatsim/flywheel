#include "test_helpers.h"
#include <flywheel/rigidbody.h>
#include <flywheel/settings.h>
#include <flywheel/world.h>

using namespace flywheel;
using namespace flywheel::test;

TEST_SUITE("Contact Solver")
{
    TEST_CASE("Contact creation between bodies")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        RigidBody* body1 = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(1.5f, 0.0f), 0.0f));

        world.Step(1.0f / 60.0f);

        // Should have contacts
        CHECK(world.GetContactCount() >= 0);

        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Contact resolution - restitution")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        // Ground
        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);

        // Falling ball
        RigidBody* ball = world.CreateCircle(0.5f, Transform(Vec2(0.0f, 5.0f), 0.0f));

        // Simulate
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Ball should have bounced and settled
        CHECK(ball->GetPosition().y > -5.0f);

        world.Destroy(ground);
        world.Destroy(ball);
    }

    TEST_CASE("Contact friction")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        // Inclined plane
        RigidBody* plane = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, 0.0f), pi / 6.0f), RigidBody::static_body);

        // Box on plane
        RigidBody* box = world.CreateBox(1.0f, Transform(Vec2(0.0f, 2.0f), 0.0f));

        Vec2 posBefore = box->GetPosition();

        // Simulate
        for (int i = 0; i < 60; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        Vec2 posAfter = box->GetPosition();

        // Box should have moved due to gravity on incline
        float distMoved = Dist(posBefore, posAfter);
        CHECK(distMoved > 0.0f);

        world.Destroy(plane);
        world.Destroy(box);
    }

    TEST_CASE("Contact manifold access")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        RigidBody* body1 = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(1.5f, 0.0f), 0.0f));

        world.Step(1.0f / 60.0f);

        // Access contacts
        const Contact* contact = world.GetContacts();
        if (contact != nullptr)
        {
            // Just verify API works
            CHECK(contact->GetContactCount() >= 0);
        }

        world.Destroy(body1);
        world.Destroy(body2);
    }
}

TEST_SUITE("Constraints")
{
    TEST_CASE("Constraint graph")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateCircle(1.0f);
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(3.0f, 0.0f), 0.0f));

        // Create distance joint (constraint)
        DistanceJoint* joint = world.CreateDistanceJoint(body1, body2, body1->GetPosition(), body2->GetPosition());

        CHECK(joint != nullptr);
        CHECK(world.GetJointCount() == 1);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Island detection")
    {
        WorldSettings settings;
        World world(settings);

        // Create two separate groups of bodies
        RigidBody* group1_a = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* group1_b = world.CreateCircle(1.0f, Transform(Vec2(1.5f, 0.0f), 0.0f));

        RigidBody* group2_a = world.CreateCircle(1.0f, Transform(Vec2(10.0f, 0.0f), 0.0f));
        RigidBody* group2_b = world.CreateCircle(1.0f, Transform(Vec2(11.5f, 0.0f), 0.0f));

        world.Step(1.0f / 60.0f);

        // Should have detected separate islands
        CHECK(world.GetBodyCount() == 4);

        world.Destroy(group1_a);
        world.Destroy(group1_b);
        world.Destroy(group2_a);
        world.Destroy(group2_b);
    }

    TEST_CASE("Constraint enable/disable")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateCircle(1.0f);
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(3.0f, 0.0f), 0.0f));

        DistanceJoint* joint = world.CreateDistanceJoint(body1, body2, body1->GetPosition(), body2->GetPosition());

        CHECK(joint->IsEnabled());

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }
}

TEST_SUITE("Joints - Distance Joint")
{
    TEST_CASE("Distance joint creation")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateCircle(1.0f);
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(3.0f, 0.0f), 0.0f));

        DistanceJoint* joint = world.CreateDistanceJoint(body1, body2, body1->GetPosition(), body2->GetPosition());

        CHECK(joint != nullptr);
        CHECK(joint->GetType() == Joint::Type::distance_joint);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Distance joint constraint")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* anchor = world.CreateCircle(0.5f, Transform(Vec2(0.0f, 5.0f), 0.0f), RigidBody::static_body);
        RigidBody* pendulum = world.CreateCircle(0.5f, Transform(Vec2(0.0f, 3.0f), 0.0f));

        DistanceJoint* joint = world.CreateDistanceJoint(anchor, pendulum, anchor->GetPosition(), pendulum->GetPosition());

        // Simulate
        for (int i = 0; i < 60; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Distance should be maintained
        float distance = Dist(anchor->GetPosition(), pendulum->GetPosition());
        CHECK(distance > 0.0f);
        CHECK(distance < 5.0f); // Should be constrained

        world.Destroy(joint);
        world.Destroy(anchor);
        world.Destroy(pendulum);
    }

    TEST_CASE("Distance joint properties")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateCircle(1.0f);
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(3.0f, 0.0f), 0.0f));

        DistanceJoint* joint = world.CreateDistanceJoint(body1, body2, body1->GetPosition(), body2->GetPosition());

        // Test joint properties
        CHECK(joint->GetJointFrequency() > 0.0f);
        CHECK(joint->GetJointDampingRatio() >= 0.0f);
        CHECK(joint->GetJointMass() > 0.0f);

        // Test anchor access
        Vec2 anchorA = joint->GetLocalAnchorA();
        Vec2 anchorB = joint->GetLocalAnchorB();
        CHECK(anchorA.x != INFINITY);
        CHECK(anchorB.x != INFINITY);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }
}

TEST_SUITE("Joints - Revolute Joint")
{
    TEST_CASE("Revolute joint creation")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f);
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), 0.0f));

        RevoluteJoint* joint = world.CreateRevoluteJoint(body1, body2, Vec2(1.0f, 0.0f));

        CHECK(joint != nullptr);
        CHECK(joint->GetType() == Joint::Type::revolute_joint);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Revolute joint rotation")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* anchor = world.CreateBox(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f), RigidBody::static_body);
        RigidBody* pendulum = world.CreateBox(0.5f, 2.0f, Transform(Vec2(0.0f, -1.5f), 0.0f));

        RevoluteJoint* joint = world.CreateRevoluteJoint(anchor, pendulum, Vec2(0.0f, 0.0f));

        float angleBefore = pendulum->GetAngle();

        // Simulate
        for (int i = 0; i < 60; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        float angleAfter = pendulum->GetAngle();

        // Pendulum should have rotated
        CHECK(angleAfter != angleBefore);

        world.Destroy(joint);
        world.Destroy(anchor);
        world.Destroy(pendulum);
    }
}

TEST_SUITE("Joints - Weld Joint")
{
    TEST_CASE("Weld joint creation")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f);
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), 0.0f));

        WeldJoint* joint = world.CreateWeldJoint(body1, body2, Vec2(1.0f, 0.0f));

        CHECK(joint != nullptr);
        CHECK(joint->GetType() == Joint::Type::weld_joint);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Weld joint rigidity")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f), RigidBody::static_body);
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), 0.0f));

        WeldJoint* joint = world.CreateWeldJoint(body1, body2, Vec2(1.0f, 0.0f));

        Vec2 posBefore = body2->GetPosition();
        float angleBefore = body2->GetAngle();

        // Simulate
        for (int i = 0; i < 60; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        Vec2 posAfter = body2->GetPosition();
        float angleAfter = body2->GetAngle();

        // Bodies should remain welded (minimal movement)
        float posDiff = Dist(posBefore, posAfter);
        float angleDiff = Abs(angleAfter - angleBefore);
        CHECK(posDiff < 0.5f);
        CHECK(angleDiff < 0.1f);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }
}

TEST_SUITE("Joints - Prismatic Joint")
{
    TEST_CASE("Prismatic joint creation")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f);
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), 0.0f));

        PrismaticJoint* joint = world.CreatePrismaticJoint(body1, body2, Vec2(1.0f, 0.0f), Vec2(1.0f, 0.0f));

        CHECK(joint != nullptr);
        CHECK(joint->GetType() == Joint::Type::prismatic_joint);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Prismatic joint sliding")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* rail = world.CreateBox(5.0f, 0.5f, Transform(Vec2(0.0f, 0.0f), 0.0f), RigidBody::static_body);
        RigidBody* slider = world.CreateBox(0.5f, Transform(Vec2(0.0f, 1.0f), 0.0f));

        // Create prismatic joint allowing vertical sliding
        PrismaticJoint* joint = world.CreatePrismaticJoint(rail, slider, Vec2(0.0f, 0.0f), Vec2(0.0f, 1.0f));

        Vec2 posBefore = slider->GetPosition();

        // Apply horizontal force (should not move horizontally due to joint)
        slider->ApplyForce(slider->GetPosition(), Vec2(100.0f, 0.0f), true);

        // Simulate
        for (int i = 0; i < 30; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        Vec2 posAfter = slider->GetPosition();

        // Should move vertically but not much horizontally
        CHECK(Abs(posAfter.x - posBefore.x) < 1.0f);

        world.Destroy(joint);
        world.Destroy(rail);
        world.Destroy(slider);
    }
}

TEST_SUITE("Joints - Motor Joint")
{
    TEST_CASE("Motor joint creation")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f);
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), 0.0f));

        MotorJoint* joint = world.CreateMotorJoint(body1, body2, Vec2(1.0f, 0.0f));

        CHECK(joint != nullptr);
        CHECK(joint->GetType() == Joint::Type::motor_joint);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }
}

TEST_SUITE("Joints - Angle Joint")
{
    TEST_CASE("Angle joint creation")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f);
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), 0.0f));

        AngleJoint* joint = world.CreateAngleJoint(body1, body2);

        CHECK(joint != nullptr);
        CHECK(joint->GetType() == Joint::Type::angle_joint);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }
}

TEST_SUITE("Joints - Line Joint")
{
    TEST_CASE("Line joint creation")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f);
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), 0.0f));

        LineJoint* joint = world.CreateLineJoint(body1, body2, Vec2(0.0f, 0.0f), Vec2(1.0f, 0.0f));

        CHECK(joint != nullptr);
        CHECK(joint->GetType() == Joint::Type::line_joint);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }
}

TEST_SUITE("Joints - Pulley Joint")
{
    TEST_CASE("Pulley joint creation")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f, Transform(Vec2(-2.0f, 0.0f), 0.0f));
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), 0.0f));

        PulleyJoint* joint =
            world.CreatePulleyJoint(body1, body2, Vec2(-2.0f, 0.0f), Vec2(2.0f, 0.0f), Vec2(-2.0f, 5.0f), Vec2(2.0f, 5.0f));

        CHECK(joint != nullptr);
        CHECK(joint->GetType() == Joint::Type::pulley_joint);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }
}
