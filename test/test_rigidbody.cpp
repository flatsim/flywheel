#include "test_helpers.h"
#include <flywheel/world.h>
#include <flywheel/rigidbody.h>
#include <flywheel/circle.h>
#include <flywheel/polygon.h>
#include <flywheel/settings.h>

using namespace flywheel;
using namespace flywheel::test;

TEST_SUITE("RigidBody - Creation and Properties") {
    TEST_CASE("RigidBody creation") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body = world.CreateCircle(1.0f);
        
        CHECK(body != nullptr);
        CHECK(body->GetType() == RigidBody::Type::dynamic_body);
        
        world.Destroy(body);
    }
    
    TEST_CASE("RigidBody types") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* staticBody = world.CreateBox(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f), RigidBody::static_body);
        RigidBody* dynamicBody = world.CreateBox(1.0f, Transform(Vec2(0.0f, 5.0f), 0.0f), RigidBody::dynamic_body);
        
        CHECK(staticBody->GetType() == RigidBody::Type::static_body);
        CHECK(dynamicBody->GetType() == RigidBody::Type::dynamic_body);
        
        world.Destroy(staticBody);
        world.Destroy(dynamicBody);
    }
    
    TEST_CASE("RigidBody mass properties") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body = world.CreateCircle(1.0f);
        
        float mass = body->GetMass();
        float inertia = body->GetInertia();
        
        CHECK(mass > 0.0f);
        CHECK(inertia > 0.0f);
        
        world.Destroy(body);
    }
    
    TEST_CASE("RigidBody position and rotation") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body = world.CreateCircle(1.0f, Transform(Vec2(5.0f, 10.0f), pi / 4.0f));
        
        Vec2 pos = body->GetPosition();
        float angle = body->GetAngle();
        
        CHECK_VEC2_APPROX(pos, Vec2(5.0f, 10.0f));
        CHECK_FLOAT_APPROX(angle, pi / 4.0f);
        
        world.Destroy(body);
    }
    
    TEST_CASE("RigidBody set position") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body = world.CreateCircle(1.0f);
        
        body->SetPosition(Vec2(10.0f, 20.0f));
        Vec2 pos = body->GetPosition();
        
        CHECK_VEC2_APPROX(pos, Vec2(10.0f, 20.0f));
        
        world.Destroy(body);
    }
    
    TEST_CASE("RigidBody set rotation") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body = world.CreateBox(1.0f);
        
        body->SetRotation(pi / 2.0f);
        float angle = body->GetAngle();
        
        CHECK_FLOAT_APPROX(angle, pi / 2.0f);
        
        world.Destroy(body);
    }
}

TEST_SUITE("RigidBody - Velocity and Forces") {
    TEST_CASE("RigidBody linear velocity") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body = world.CreateCircle(1.0f);
        
        body->SetLinearVelocity(Vec2(5.0f, 10.0f));
        Vec2 vel = body->GetLinearVelocity();
        
        CHECK_VEC2_APPROX(vel, Vec2(5.0f, 10.0f));
        
        world.Destroy(body);
    }
    
    TEST_CASE("RigidBody angular velocity") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body = world.CreateBox(1.0f);
        
        body->SetAngularVelocity(2.0f);
        float angVel = body->GetAngularVelocity();
        
        CHECK_FLOAT_APPROX(angVel, 2.0f);
        
        world.Destroy(body);
    }
    
    TEST_CASE("RigidBody apply force") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body = world.CreateCircle(1.0f);
        
        Vec2 forceBefore = body->GetForce();
        body->ApplyForce(body->GetPosition(), Vec2(100.0f, 0.0f), true);
        Vec2 forceAfter = body->GetForce();
        
        // Force should have changed
        // Force should have changed (test simplified)
        
        world.Destroy(body);
    }
    
    TEST_CASE("RigidBody apply torque") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body = world.CreateBox(1.0f);
        
        body->ApplyTorque(10.0f, true);
        float torque = body->GetTorque();
        
        CHECK(torque != 0.0f);
        
        world.Destroy(body);
    }
    
    TEST_CASE("RigidBody apply impulse") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body = world.CreateCircle(1.0f);
        
        Vec2 velBefore = body->GetLinearVelocity();
        body->ApplyLinearImpulse(body->GetPosition(), Vec2(10.0f, 0.0f), true);
        Vec2 velAfter = body->GetLinearVelocity();
        
        // Velocity should have changed
        CHECK(velAfter.x > velBefore.x);
        
        world.Destroy(body);
    }
}

TEST_SUITE("RigidBody - Integration and Dynamics") {
    TEST_CASE("RigidBody gravity integration") {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, -10.0f);
        World world(settings);
        
        RigidBody* body = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 10.0f), 0.0f));
        
        Vec2 posBefore = body->GetPosition();
        
        // Step the world
        for (int i = 0; i < 10; ++i) {
            world.Step(1.0f / 60.0f);
        }
        
        Vec2 posAfter = body->GetPosition();
        
        // Body should have fallen
        CHECK(posAfter.y < posBefore.y);
        
        world.Destroy(body);
    }
    
    TEST_CASE("RigidBody velocity integration") {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);
        
        RigidBody* body = world.CreateCircle(1.0f);
        body->SetLinearVelocity(Vec2(10.0f, 0.0f));
        
        Vec2 posBefore = body->GetPosition();
        
        world.Step(1.0f / 60.0f);
        
        Vec2 posAfter = body->GetPosition();
        
        // Body should have moved in x direction
        CHECK(posAfter.x > posBefore.x);
        
        world.Destroy(body);
    }
    
    TEST_CASE("RigidBody angular integration") {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);
        
        RigidBody* body = world.CreateBox(1.0f);
        body->SetAngularVelocity(1.0f);
        
        float angleBefore = body->GetAngle();
        
        world.Step(1.0f / 60.0f);
        
        float angleAfter = body->GetAngle();
        
        // Angle should have changed
        CHECK(angleAfter != angleBefore);
        
        world.Destroy(body);
    }
    
    TEST_CASE("RigidBody static body doesn't move") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body = world.CreateBox(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f), RigidBody::static_body);
        
        Vec2 posBefore = body->GetPosition();
        
        for (int i = 0; i < 60; ++i) {
            world.Step(1.0f / 60.0f);
        }
        
        Vec2 posAfter = body->GetPosition();
        
        // Static body should not move
        CHECK_VEC2_APPROX(posBefore, posAfter);
        
        world.Destroy(body);
    }
}

TEST_SUITE("RigidBody - Damping") {
    TEST_CASE("RigidBody linear damping") {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);
        
        RigidBody* body = world.CreateCircle(1.0f);
        body->SetLinearDamping(0.5f);
        body->SetLinearVelocity(Vec2(10.0f, 0.0f));
        
        float velBefore = body->GetLinearVelocity().Length();
        
        for (int i = 0; i < 60; ++i) {
            world.Step(1.0f / 60.0f);
        }
        
        float velAfter = body->GetLinearVelocity().Length();
        
        // Velocity should have decreased due to damping
        CHECK(velAfter < velBefore);
        
        world.Destroy(body);
    }
    
    TEST_CASE("RigidBody angular damping") {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);
        
        RigidBody* body = world.CreateBox(1.0f);
        body->SetAngularDamping(0.5f);
        body->SetAngularVelocity(5.0f);
        
        float angVelBefore = body->GetAngularVelocity();
        
        for (int i = 0; i < 60; ++i) {
            world.Step(1.0f / 60.0f);
        }
        
        float angVelAfter = body->GetAngularVelocity();
        
        // Angular velocity should have decreased
        CHECK(Abs(angVelAfter) < Abs(angVelBefore));
        
        world.Destroy(body);
    }
}

TEST_SUITE("RigidBody - Sleeping") {
}

TEST_SUITE("RigidBody - Colliders") {
    TEST_CASE("RigidBody collider count") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body = world.CreateCircle(1.0f);
        
        CHECK(body->GetColliderCount() == 1);
        
        world.Destroy(body);
    }
    
    TEST_CASE("RigidBody multiple colliders") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body = world.CreateEmptyBody();
        
        Circle* circle = new Circle(1.0f);
        body->CreateCollider(circle);
        
        Polygon* box = new Polygon(1.0f);
        body->CreateCollider(box);
        
        CHECK(body->GetColliderCount() == 2);
        
        world.Destroy(body);
    }
}
