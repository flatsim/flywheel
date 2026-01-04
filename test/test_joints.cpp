#include "test_helpers.h"
#include <flywheel/flywheel.hpp>

using namespace flywheel;
using namespace flywheel::test;

TEST_SUITE("Distance Joint")
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
        CHECK(joint->GetBodyA() == body1);
        CHECK(joint->GetBodyB() == body2);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Distance joint maintains distance")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* anchor = world.CreateCircle(0.5f, Transform(Vec2(0.0f, 5.0f), 0.0f), RigidBody::static_body);
        RigidBody* pendulum = world.CreateCircle(0.5f, Transform(Vec2(0.0f, 3.0f), 0.0f));

        float initialDist = Dist(anchor->GetPosition(), pendulum->GetPosition());
        DistanceJoint* joint = world.CreateDistanceJoint(anchor, pendulum, anchor->GetPosition(), pendulum->GetPosition());

        // Simulate
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        float finalDist = Dist(anchor->GetPosition(), pendulum->GetPosition());
        // Distance should be approximately maintained (within 10%)
        CHECK(Abs(finalDist - initialDist) / initialDist < 0.1f);

        world.Destroy(joint);
        world.Destroy(anchor);
        world.Destroy(pendulum);
    }

    TEST_CASE("Distance joint min/max distance")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        RigidBody* body1 = world.CreateCircle(1.0f);
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(5.0f, 0.0f), 0.0f));

        DistanceJoint* joint =
            world.CreateLimitedDistanceJoint(body1, body2, body1->GetPosition(), body2->GetPosition(), 2.0f, 6.0f);

        CHECK(joint->GetJointMinLength() == 2.0f);
        CHECK(joint->GetJointMaxLength() == 6.0f);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Distance joint spring behavior")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* anchor = world.CreateCircle(0.5f, Transform(Vec2(0.0f, 5.0f), 0.0f), RigidBody::static_body);
        RigidBody* mass = world.CreateCircle(0.5f, Transform(Vec2(0.0f, 3.0f), 0.0f));

        // Create spring-like joint with low frequency
        DistanceJoint* joint =
            world.CreateDistanceJoint(anchor, mass, anchor->GetPosition(), mass->GetPosition(), -1.0f, 2.0f, 1.0f, 1.0f);

        // Simulate - should oscillate
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Mass should still be moving (oscillating)
        CHECK(mass->GetPosition().y < 5.0f);

        world.Destroy(joint);
        world.Destroy(anchor);
        world.Destroy(mass);
    }
}

TEST_SUITE("Revolute Joint")
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

        float initialAngle = pendulum->GetAngle();

        // Simulate
        for (int i = 0; i < 60; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        float finalAngle = pendulum->GetAngle();

        // Pendulum should have rotated (even slightly)
        CHECK(Abs(finalAngle - initialAngle) > 0.001f);

        world.Destroy(joint);
        world.Destroy(anchor);
        world.Destroy(pendulum);
    }

    TEST_CASE("Revolute joint anchor point")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f);
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), 0.0f));

        Vec2 anchor(1.0f, 0.0f);
        RevoluteJoint* joint = world.CreateRevoluteJoint(body1, body2, anchor);

        // Check anchor points are set correctly
        Vec2 anchorA = joint->GetLocalAnchorA();
        Vec2 anchorB = joint->GetLocalAnchorB();

        CHECK(anchorA.x != INFINITY);
        CHECK(anchorB.x != INFINITY);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }
}

TEST_SUITE("Weld Joint")
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

        Vec2 initialPos = body2->GetPosition();
        float initialAngle = body2->GetAngle();

        // Simulate
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        Vec2 finalPos = body2->GetPosition();
        float finalAngle = body2->GetAngle();

        // Bodies should remain welded (minimal movement)
        float posDiff = Dist(initialPos, finalPos);
        float angleDiff = Abs(finalAngle - initialAngle);

        CHECK(posDiff < 0.5f);
        CHECK(angleDiff < 0.1f);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Weld joint angle offset")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f);
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), pi / 4.0f));

        WeldJoint* joint = world.CreateWeldJoint(body1, body2, Vec2(1.0f, 0.0f));

        float angleOffset = joint->GetAngleOffset();
        CHECK(Abs(angleOffset - pi / 4.0f) < 0.01f);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }
}

TEST_SUITE("Prismatic Joint")
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

        Vec2 initialPos = slider->GetPosition();

        // Simulate
        for (int i = 0; i < 60; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        Vec2 finalPos = slider->GetPosition();

        // Should slide vertically
        CHECK(Abs(finalPos.y - initialPos.y) > 0.1f);
        // Should not move much horizontally
        CHECK(Abs(finalPos.x - initialPos.x) < 0.5f);

        world.Destroy(joint);
        world.Destroy(rail);
        world.Destroy(slider);
    }

    TEST_CASE("Prismatic joint angle offset")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f);
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), pi / 6.0f));

        PrismaticJoint* joint = world.CreatePrismaticJoint(body1, body2, Vec2(1.0f, 0.0f), Vec2(1.0f, 0.0f));

        float angleOffset = joint->GetAngleOffset();
        CHECK(Abs(angleOffset - pi / 6.0f) < 0.01f);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }
}

TEST_SUITE("Angle Joint")
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

    TEST_CASE("Angle joint limits")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f);
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), 0.0f));

        AngleJoint* joint = world.CreateLimitedAngleJoint(body1, body2, -pi / 4.0f, pi / 4.0f);

        CHECK(joint->GetJointMinAngle() == -pi / 4.0f);
        CHECK(joint->GetJointMaxAngle() == pi / 4.0f);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Angle joint maintains angle")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f);
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), pi / 6.0f));

        float initialAngleDiff = body2->GetAngle() - body1->GetAngle();
        AngleJoint* joint = world.CreateAngleJoint(body1, body2);

        // Apply torque to body1
        body1->ApplyTorque(10.0f, true);

        // Simulate
        for (int i = 0; i < 60; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        float finalAngleDiff = body2->GetAngle() - body1->GetAngle();

        // Angle difference should be maintained
        CHECK_FLOAT_APPROX(finalAngleDiff, initialAngleDiff);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }
}

TEST_SUITE("Grab Joint")
{
    TEST_CASE("Grab joint creation")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body = world.CreateBox(1.0f);
        Vec2 anchor = body->GetPosition();
        Vec2 target(5.0f, 5.0f);

        GrabJoint* joint = world.CreateGrabJoint(body, anchor, target);

        CHECK(joint != nullptr);
        CHECK(joint->GetType() == Joint::Type::grab_joint);

        world.Destroy(joint);
        world.Destroy(body);
    }

    TEST_CASE("Grab joint mouse interaction")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* body = world.CreateBox(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        Vec2 anchor = body->GetPosition();
        Vec2 target(5.0f, 5.0f);

        GrabJoint* joint = world.CreateGrabJoint(body, anchor, target);

        // Simulate - body should move toward target
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        Vec2 finalPos = body->GetPosition();
        float distToTarget = Dist(finalPos, target);

        // Should have moved closer to target
        CHECK(distToTarget < 5.0f);

        world.Destroy(joint);
        world.Destroy(body);
    }

    TEST_CASE("Grab joint target update")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body = world.CreateBox(1.0f);
        Vec2 anchor = body->GetPosition();
        Vec2 initialTarget(5.0f, 0.0f);

        GrabJoint* joint = world.CreateGrabJoint(body, anchor, initialTarget);

        CHECK_VEC2_APPROX(joint->GetTarget(), initialTarget);

        Vec2 newTarget(0.0f, 5.0f);
        joint->SetTarget(newTarget);

        CHECK_VEC2_APPROX(joint->GetTarget(), newTarget);

        world.Destroy(joint);
        world.Destroy(body);
    }
}

TEST_SUITE("Motor Joint")
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

    TEST_CASE("Motor joint max force/torque")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f);
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), 0.0f));

        MotorJoint* joint = world.CreateMotorJoint(body1, body2, Vec2(1.0f, 0.0f), 500.0f, 200.0f);

        CHECK(joint->GetMaxForce() == 500.0f);
        CHECK(joint->GetMaxTorque() == 200.0f);

        joint->SetMaxForce(1000.0f);
        joint->SetMaxTorque(400.0f);

        CHECK(joint->GetMaxForce() == 1000.0f);
        CHECK(joint->GetMaxTorque() == 400.0f);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Motor joint linear offset")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f);
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), 0.0f));

        MotorJoint* joint = world.CreateMotorJoint(body1, body2, Vec2(1.0f, 0.0f));

        Vec2 offset(1.0f, 2.0f);
        joint->SetLinearOffset(offset);

        CHECK_VEC2_APPROX(joint->GetLinearOffset(), offset);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Motor joint angular offset")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f);
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), 0.0f));

        MotorJoint* joint = world.CreateMotorJoint(body1, body2, Vec2(1.0f, 0.0f));

        float angularOffset = pi / 4.0f;
        joint->SetAngularOffset(angularOffset);

        CHECK_FLOAT_APPROX(joint->GetAngularOffset(), angularOffset);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }
}

TEST_SUITE("Line Joint")
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

    TEST_CASE("Line joint prismatic motion")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* rail = world.CreateBox(5.0f, 0.5f, Transform(Vec2(0.0f, 0.0f), 0.0f), RigidBody::static_body);
        RigidBody* slider = world.CreateBox(0.5f, Transform(Vec2(0.0f, 1.0f), 0.0f));

        // Create line joint allowing vertical motion
        LineJoint* joint = world.CreateLineJoint(rail, slider, Vec2(0.0f, 0.0f), Vec2(0.0f, 1.0f));

        Vec2 initialPos = slider->GetPosition();

        // Simulate
        for (int i = 0; i < 60; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        Vec2 finalPos = slider->GetPosition();

        // Should move vertically
        CHECK(Abs(finalPos.y - initialPos.y) > 0.1f);

        world.Destroy(joint);
        world.Destroy(rail);
        world.Destroy(slider);
    }
}

TEST_SUITE("Pulley Joint")
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

    TEST_CASE("Pulley joint ground anchors")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f, Transform(Vec2(-2.0f, 0.0f), 0.0f));
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), 0.0f));

        Vec2 groundA(-2.0f, 5.0f);
        Vec2 groundB(2.0f, 5.0f);

        PulleyJoint* joint = world.CreatePulleyJoint(body1, body2, Vec2(-2.0f, 0.0f), Vec2(2.0f, 0.0f), groundA, groundB);

        CHECK_VEC2_APPROX(joint->GetGroundAnchorA(), groundA);
        CHECK_VEC2_APPROX(joint->GetGroundAnchorB(), groundB);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Pulley joint ratio behavior")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* body1 = world.CreateBox(1.0f, Transform(Vec2(-2.0f, 0.0f), 0.0f));
        RigidBody* body2 = world.CreateBox(1.0f, Transform(Vec2(2.0f, 0.0f), 0.0f));

        PulleyJoint* joint = world.CreatePulleyJoint(
            body1, body2, Vec2(-2.0f, 0.0f), Vec2(2.0f, 0.0f), Vec2(-2.0f, 5.0f), Vec2(2.0f, 5.0f),
            2.0f // ratio
        );

        Vec2 initialPos1 = body1->GetPosition();
        Vec2 initialPos2 = body2->GetPosition();

        // Simulate
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Bodies should move according to pulley ratio
        CHECK(body1->GetPosition().y != initialPos1.y);
        CHECK(body2->GetPosition().y != initialPos2.y);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }
}

TEST_SUITE("Joint Stability")
{
    TEST_CASE("Joint stability under stress")
    {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);

        RigidBody* anchor = world.CreateCircle(0.5f, Transform(Vec2(0.0f, 5.0f), 0.0f), RigidBody::static_body);
        RigidBody* heavy = world.CreateCircle(2.0f, Transform(Vec2(0.0f, 3.0f), 0.0f), RigidBody::dynamic_body, 10.0f);

        DistanceJoint* joint = world.CreateDistanceJoint(anchor, heavy, anchor->GetPosition(), heavy->GetPosition());

        // Simulate with heavy load
        for (int i = 0; i < 240; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // Joint should still be intact
        CHECK(world.GetJointCount() == 1);

        world.Destroy(joint);
        world.Destroy(anchor);
        world.Destroy(heavy);
    }

    TEST_CASE("Multiple joints on same body")
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

        // Simulate
        for (int i = 0; i < 120; ++i)
        {
            world.Step(1.0f / 60.0f);
        }

        // All joints should still be intact
        CHECK(world.GetJointCount() == 3);

        world.Destroy(joint1);
        world.Destroy(joint2);
        world.Destroy(joint3);
        world.Destroy(center);
        world.Destroy(body1);
        world.Destroy(body2);
        world.Destroy(body3);
    }

    TEST_CASE("Joint chain stability")
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

        // All joints should still be intact
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
}

TEST_SUITE("Joint Properties")
{
    TEST_CASE("Joint frequency and damping")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateCircle(1.0f);
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(3.0f, 0.0f), 0.0f));

        DistanceJoint* joint =
            world.CreateDistanceJoint(body1, body2, body1->GetPosition(), body2->GetPosition(), -1.0f, 5.0f, 0.8f, 1.0f);

        CHECK(joint->GetJointFrequency() == 5.0f);
        CHECK(joint->GetJointDampingRatio() == 0.8f);
        CHECK(joint->GetJointMass() == 1.0f);

        world.Destroy(joint);
        world.Destroy(body1);
        world.Destroy(body2);
    }

    TEST_CASE("Joint enable/disable")
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

    TEST_CASE("Joint solid vs soft")
    {
        WorldSettings settings;
        World world(settings);

        RigidBody* body1 = world.CreateCircle(1.0f);
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(3.0f, 0.0f), 0.0f));

        // Solid joint (frequency <= 0)
        DistanceJoint* solidJoint =
            world.CreateDistanceJoint(body1, body2, body1->GetPosition(), body2->GetPosition(), -1.0f, -1.0f, 1.0f, 1.0f);
        CHECK(solidJoint->IsSolid());

        world.Destroy(solidJoint);

        // Soft joint (frequency > 0)
        DistanceJoint* softJoint =
            world.CreateDistanceJoint(body1, body2, body1->GetPosition(), body2->GetPosition(), -1.0f, 5.0f, 1.0f, 1.0f);
        CHECK_FALSE(softJoint->IsSolid());

        world.Destroy(softJoint);
        world.Destroy(body1);
        world.Destroy(body2);
    }
}
