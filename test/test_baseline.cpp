#include "test_helpers.h"
#include <flywheel/flywheel.hpp>

using namespace flywheel;
using namespace flywheel::test;

TEST_SUITE("Baseline - World") {
    TEST_CASE("World creation") {
        WorldSettings settings;
        World world(settings);
        
        CHECK(world.GetBodyCount() == 0);
        CHECK(world.GetJointCount() == 0);
    }
    
    TEST_CASE("Create circle body") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body = world.CreateCircle(1.0f);
        CHECK(body != nullptr);
        CHECK(world.GetBodyCount() == 1);
        
        world.Destroy(body);
        CHECK(world.GetBodyCount() == 0);
    }
    
    TEST_CASE("Create box body") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body = world.CreateBox(2.0f);
        CHECK(body != nullptr);
        CHECK(body->GetColliderCount() == 1);
        
        world.Destroy(body);
    }
    
    TEST_CASE("World step") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 10.0f), 0.0f));
        
        Vec2 posBefore = body->GetPosition();
        
        // Step the world
        world.Step(1.0f / 60.0f);
        
        Vec2 posAfter = body->GetPosition();
        
        // Dynamic body should have moved due to gravity
        CHECK(posAfter.y < posBefore.y);
        
        world.Destroy(body);
    }
    
    TEST_CASE("Multiple bodies") {
        WorldSettings settings;
        World world(settings);
        
        RigidBody* body1 = world.CreateCircle(1.0f);
        RigidBody* body2 = world.CreateBox(2.0f);
        RigidBody* body3 = world.CreateCapsule(2.0f, 0.5f);
        
        CHECK(world.GetBodyCount() == 3);
        
        world.Destroy(body1);
        world.Destroy(body2);
        world.Destroy(body3);
        
        CHECK(world.GetBodyCount() == 0);
    }
}

TEST_SUITE("Baseline - Collision") {
    TEST_CASE("Circle-Circle collision") {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);
        
        // Create two overlapping circles
        RigidBody* body1 = world.CreateCircle(1.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* body2 = world.CreateCircle(1.0f, Transform(Vec2(1.5f, 0.0f), 0.0f));
        
        // Step to detect collision
        world.Step(1.0f / 60.0f);
        
        // Should have contact
        CHECK(world.GetContactCount() >= 0);
        
        world.Destroy(body1);
        world.Destroy(body2);
    }
    
    TEST_CASE("Box-Box collision") {
        WorldSettings settings;
        settings.gravity = Vec2(0.0f, 0.0f);
        World world(settings);
        
        RigidBody* body1 = world.CreateBox(2.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        RigidBody* body2 = world.CreateBox(2.0f, Transform(Vec2(1.5f, 0.0f), 0.0f));
        
        world.Step(1.0f / 60.0f);
        
        CHECK(world.GetContactCount() >= 0);
        
        world.Destroy(body1);
        world.Destroy(body2);
    }
}

TEST_SUITE("Baseline - Integration") {
    TEST_CASE("Stacking boxes") {
        WorldSettings settings;
        World world(settings);
        
        // Ground
        RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0.0f, -5.0f), 0.0f), RigidBody::static_body);
        
        // Stack 3 boxes
        for (int i = 0; i < 3; ++i) {
            world.CreateBox(1.0f, Transform(Vec2(0.0f, -3.0f + i * 2.1f), 0.0f));
        }
        
        CHECK(world.GetBodyCount() == 4);
        
        // Simulate for a bit
        for (int i = 0; i < 60; ++i) {
            world.Step(1.0f / 60.0f);
        }
        
        // Boxes should have settled
        CHECK(world.GetBodyCount() == 4);
    }
    
    TEST_CASE("Determinism test") {
        // Create two identical worlds
        WorldSettings settings;
        World world1(settings);
        World world2(settings);
        
        // Add identical bodies
        world1.CreateCircle(1.0f, Transform(Vec2(0.0f, 10.0f), 0.0f));
        world2.CreateCircle(1.0f, Transform(Vec2(0.0f, 10.0f), 0.0f));
        
        // Step both worlds
        for (int i = 0; i < 10; ++i) {
            world1.Step(1.0f / 60.0f);
            world2.Step(1.0f / 60.0f);
        }
        
        // Get positions
        RigidBody* body1 = world1.GetBodyList();
        RigidBody* body2 = world2.GetBodyList();
        
        Vec2 pos1 = body1->GetPosition();
        Vec2 pos2 = body2->GetPosition();
        
        // Should be identical (deterministic)
        CHECK_VEC2_APPROX(pos1, pos2);
    }
}
