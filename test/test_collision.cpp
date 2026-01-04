#include "test_helpers.h"
#include <flywheel/flywheel.hpp>

using namespace flywheel;
using namespace flywheel::test;

TEST_SUITE("Collision - Circle vs Circle") {
    TEST_CASE("Circles overlapping") {
        Circle circle1(1.0f);
        Circle circle2(1.0f);
        
        Transform tf1(Vec2(0.0f, 0.0f), 0.0f);
        Transform tf2(Vec2(1.5f, 0.0f), 0.0f);
        
        ContactManifold manifold;
        bool colliding = Collide(&circle1, tf1, &circle2, tf2, &manifold);
        
        CHECK(colliding);
        CHECK(manifold.contactCount > 0);
        CHECK(manifold.penetrationDepth > 0.0f);
    }
    
    TEST_CASE("Circles touching") {
        Circle circle1(1.0f);
        Circle circle2(1.0f);
        
        Transform tf1(Vec2(0.0f, 0.0f), 0.0f);
        Transform tf2(Vec2(2.0f, 0.0f), 0.0f);
        
        ContactManifold manifold;
        bool colliding = Collide(&circle1, tf1, &circle2, tf2, &manifold);
        
        // Touching should be detected
        CHECK(colliding);
    }
    
    TEST_CASE("Circles separated") {
        Circle circle1(1.0f);
        Circle circle2(1.0f);
        
        Transform tf1(Vec2(0.0f, 0.0f), 0.0f);
        Transform tf2(Vec2(5.0f, 0.0f), 0.0f);
        
        ContactManifold manifold;
        bool colliding = Collide(&circle1, tf1, &circle2, tf2, &manifold);
        
        CHECK_FALSE(colliding);
    }
    
}

TEST_SUITE("Collision - Box vs Box") {
    TEST_CASE("Boxes overlapping") {
        Polygon box1(2.0f, 2.0f, default_radius);
        Polygon box2(2.0f, 2.0f, default_radius);
        
        Transform tf1(Vec2(0.0f, 0.0f), 0.0f);
        Transform tf2(Vec2(1.5f, 0.0f), 0.0f);
        
        ContactManifold manifold;
        bool colliding = Collide(&box1, tf1, &box2, tf2, &manifold);
        
        CHECK(colliding);
        CHECK(manifold.contactCount > 0);
    }
    
    TEST_CASE("Boxes separated") {
        Polygon box1(2.0f, 2.0f, default_radius);
        Polygon box2(2.0f, 2.0f, default_radius);
        
        Transform tf1(Vec2(0.0f, 0.0f), 0.0f);
        Transform tf2(Vec2(5.0f, 0.0f), 0.0f);
        
        ContactManifold manifold;
        bool colliding = Collide(&box1, tf1, &box2, tf2, &manifold);
        
        CHECK_FALSE(colliding);
    }
    
    TEST_CASE("Boxes rotated") {
        Polygon box1(2.0f, 2.0f, default_radius);
        Polygon box2(2.0f, 2.0f, default_radius);
        
        Transform tf1(Vec2(0.0f, 0.0f), 0.0f);
        Transform tf2(Vec2(1.5f, 0.0f), pi / 4.0f);
        
        ContactManifold manifold;
        bool colliding = Collide(&box1, tf1, &box2, tf2, &manifold);
        
        CHECK(colliding);
    }
}

TEST_SUITE("Collision - Circle vs Box") {
    TEST_CASE("Circle and box overlapping") {
        Circle circle(1.0f);
        Polygon box(2.0f, 2.0f, default_radius);
        
        Transform tf1(Vec2(0.0f, 0.0f), 0.0f);
        Transform tf2(Vec2(1.5f, 0.0f), 0.0f);
        
        ContactManifold manifold;
        bool colliding = Collide(&circle, tf1, &box, tf2, &manifold);
        
        CHECK(colliding);
    }
    
    TEST_CASE("Circle and box separated") {
        Circle circle(1.0f);
        Polygon box(2.0f, 2.0f, default_radius);
        
        Transform tf1(Vec2(0.0f, 0.0f), 0.0f);
        Transform tf2(Vec2(5.0f, 0.0f), 0.0f);
        
        ContactManifold manifold;
        bool colliding = Collide(&circle, tf1, &box, tf2, &manifold);
        
        CHECK_FALSE(colliding);
    }
}

TEST_SUITE("Collision - GJK Algorithm") {
    
    TEST_CASE("GJK detects separation") {
        Circle circle1(1.0f);
        Circle circle2(1.0f);
        
        Transform tf1(Vec2(0.0f, 0.0f), 0.0f);
        Transform tf2(Vec2(5.0f, 0.0f), 0.0f);
        
        GJKResult result;
        bool overlapping = GJK(&circle1, tf1, &circle2, tf2, &result);
        
        CHECK_FALSE(overlapping);
        CHECK(result.distance > 0.0f);
    }
    
}

TEST_SUITE("Collision - Distance Queries") {
    TEST_CASE("Distance between separated circles") {
        Circle circle1(1.0f);
        Circle circle2(1.0f);
        
        Transform tf1(Vec2(0.0f, 0.0f), 0.0f);
        Transform tf2(Vec2(5.0f, 0.0f), 0.0f);
        
        Vec2 pointA, pointB;
        float distance = ComputeDistance(&circle1, tf1, &circle2, tf2, &pointA, &pointB);
        
        CHECK(distance > 0.0f);
        CHECK_FLOAT_APPROX(distance, 3.0f); // 5 - 1 - 1
    }
    
    TEST_CASE("Distance between overlapping shapes") {
        Circle circle1(1.0f);
        Circle circle2(1.0f);
        
        Transform tf1(Vec2(0.0f, 0.0f), 0.0f);
        Transform tf2(Vec2(1.5f, 0.0f), 0.0f);
        
        Vec2 pointA, pointB;
        float distance = ComputeDistance(&circle1, tf1, &circle2, tf2, &pointA, &pointB);
        
        // Overlapping shapes have negative or zero distance
        CHECK(distance <= 0.0f);
    }
    
    TEST_CASE("Closest features") {
        Circle circle1(1.0f);
        Circle circle2(1.0f);
        
        Transform tf1(Vec2(0.0f, 0.0f), 0.0f);
        Transform tf2(Vec2(5.0f, 0.0f), 0.0f);
        
        ClosestFeatures features;
        float distance = GetClosestFeatures(&circle1, tf1, &circle2, tf2, &features);
        
        CHECK(distance > 0.0f);
        CHECK(features.count > 0);
    }
}

TEST_SUITE("Collision - Contact Manifold") {
    TEST_CASE("Contact normal direction") {
        Circle circle1(1.0f);
        Circle circle2(1.0f);
        
        Transform tf1(Vec2(0.0f, 0.0f), 0.0f);
        Transform tf2(Vec2(1.5f, 0.0f), 0.0f);
        
        ContactManifold manifold;
        Collide(&circle1, tf1, &circle2, tf2, &manifold);
        
        // Normal should be normalized
        float normalLen = manifold.contactNormal.Length();
        CHECK_FLOAT_APPROX(normalLen, 1.0f);
    }
    
    TEST_CASE("Contact point count") {
        Polygon box1(2.0f, 2.0f, default_radius);
        Polygon box2(2.0f, 2.0f, default_radius);
        
        Transform tf1(Vec2(0.0f, 0.0f), 0.0f);
        Transform tf2(Vec2(1.5f, 0.0f), 0.0f);
        
        ContactManifold manifold;
        Collide(&box1, tf1, &box2, tf2, &manifold);
        
        // Should have at least one contact point
        CHECK(manifold.contactCount > 0);
        CHECK(manifold.contactCount <= max_contact_point_count);
    }
}
