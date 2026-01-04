#include "test_helpers.h"
#include <flywheel/flywheel.hpp>

using namespace flywheel;
using namespace flywheel::test;

TEST_SUITE("Geometry - Circle") {
    TEST_CASE("Circle construction") {
        Circle circle(5.0f);
        
        CHECK_FLOAT_APPROX(circle.GetRadius(), 5.0f);
        CHECK(circle.GetType() == Shape::Type::circle);
    }
    
    TEST_CASE("Circle mass computation") {
        Circle circle(2.0f);
        MassData mass;
        circle.ComputeMass(1.0f, &mass);
        
        // Area = pi * r^2 = pi * 4
        float expectedArea = pi * 4.0f;
        CHECK_FLOAT_APPROX(mass.mass, expectedArea);
        CHECK(mass.inertia > 0.0f);
    }
    
    TEST_CASE("Circle AABB computation") {
        Circle circle(5.0f, Transform(Vec2(10.0f, 20.0f), 0.0f));
        AABB aabb;
        circle.ComputeAABB(identity, &aabb);
        
        // AABB should contain the circle
        CHECK(aabb.min.x <= 5.0f);
        CHECK(aabb.min.y <= 15.0f);
        CHECK(aabb.max.x >= 15.0f);
        CHECK(aabb.max.y >= 25.0f);
    }
    
    TEST_CASE("Circle point test - inside") {
        Circle circle(5.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        
        CHECK(circle.TestPoint(identity, Vec2(0.0f, 0.0f)));
        CHECK(circle.TestPoint(identity, Vec2(3.0f, 0.0f)));
        CHECK(circle.TestPoint(identity, Vec2(0.0f, 3.0f)));
    }
    
    TEST_CASE("Circle point test - outside") {
        Circle circle(5.0f, Transform(Vec2(0.0f, 0.0f), 0.0f));
        
        CHECK_FALSE(circle.TestPoint(identity, Vec2(10.0f, 0.0f)));
        CHECK_FALSE(circle.TestPoint(identity, Vec2(0.0f, 10.0f)));
    }
    
    TEST_CASE("Circle vertex count") {
        Circle circle(5.0f);
        CHECK(circle.GetVertexCount() == 1);
    }
}

TEST_SUITE("Geometry - Polygon") {
    TEST_CASE("Polygon box construction") {
        Polygon box(10.0f, 5.0f, default_radius);
        
        CHECK(box.GetType() == Shape::Type::polygon);
        CHECK(box.GetVertexCount() == 4);
        CHECK(box.GetArea() > 0.0f);
    }
    
    TEST_CASE("Polygon square construction") {
        Polygon square(10.0f);
        
        CHECK(square.GetVertexCount() == 4);
        float area = square.GetArea();
        CHECK_FLOAT_APPROX(area, 100.0f);
    }
    
    TEST_CASE("Polygon from vertices") {
        Vec2 vertices[] = {
            Vec2(0.0f, 0.0f),
            Vec2(10.0f, 0.0f),
            Vec2(10.0f, 10.0f),
            Vec2(0.0f, 10.0f)
        };
        
        Polygon poly(vertices, 4);
        
        CHECK(poly.GetVertexCount() == 4);
        CHECK(poly.GetArea() > 0.0f);
    }
    
    TEST_CASE("Polygon initializer list") {
        Polygon poly({
            Vec2(0.0f, 0.0f),
            Vec2(5.0f, 0.0f),
            Vec2(5.0f, 5.0f),
            Vec2(0.0f, 5.0f)
        });
        
        CHECK(poly.GetVertexCount() == 4);
        CHECK(poly.GetArea() > 20.0f); // Approximately 25, but may vary with radius
    }
    
    TEST_CASE("Polygon mass computation") {
        Polygon box(10.0f, 10.0f, default_radius);
        MassData mass;
        box.ComputeMass(1.0f, &mass);
        
        CHECK_FLOAT_APPROX(mass.mass, 100.0f);
        CHECK(mass.inertia > 0.0f);
    }
    
    TEST_CASE("Polygon AABB computation") {
        Polygon box(10.0f, 5.0f, default_radius);
        AABB aabb;
        box.ComputeAABB(identity, &aabb);
        
        Vec2 extents = aabb.GetExtents();
        CHECK(extents.x >= 10.0f);
        CHECK(extents.y >= 5.0f);
    }
    
    TEST_CASE("Polygon normals") {
        Polygon box(10.0f, 10.0f, default_radius);
        const Vec2* normals = box.GetNormals();
        
        // Check that normals are normalized
        for (int i = 0; i < box.GetVertexCount(); ++i) {
            float len = normals[i].Length();
            CHECK_FLOAT_APPROX(len, 1.0f);
        }
    }
    
    TEST_CASE("Polygon triangle") {
        Polygon triangle({
            Vec2(0.0f, 0.0f),
            Vec2(10.0f, 0.0f),
            Vec2(5.0f, 10.0f)
        });
        
        CHECK(triangle.GetVertexCount() == 3);
        CHECK(triangle.GetArea() > 0.0f);
    }
    
    TEST_CASE("Polygon vertices access") {
        Polygon box(10.0f, 10.0f, default_radius);
        const Vec2* vertices = box.GetVertices();
        
        // Should have 4 vertices
        CHECK(box.GetVertexCount() == 4);
        
        // All vertices should be accessible
        for (int i = 0; i < box.GetVertexCount(); ++i) {
            Vec2 v = box.GetVertex(i);
            CHECK_VEC2_APPROX(v, vertices[i]);
        }
    }
}

TEST_SUITE("Geometry - Capsule") {
    TEST_CASE("Capsule construction from length") {
        Capsule capsule(10.0f, 2.0f);
        
        CHECK(capsule.GetType() == Shape::Type::capsule);
        CHECK_FLOAT_APPROX(capsule.GetRadius(), 2.0f);
        CHECK(capsule.GetVertexCount() == 2);
    }
    
    TEST_CASE("Capsule construction from points") {
        Vec2 p1(0.0f, 0.0f);
        Vec2 p2(10.0f, 0.0f);
        Capsule capsule(p1, p2, 2.0f);
        
        CHECK_FLOAT_APPROX(capsule.GetLength(), 10.0f);
        CHECK_FLOAT_APPROX(capsule.GetRadius(), 2.0f);
    }
    
    TEST_CASE("Capsule horizontal") {
        Capsule capsule(10.0f, 2.0f, true);
        
        Vec2 va = capsule.GetVertexA();
        Vec2 vb = capsule.GetVertexB();
        
        // Should be horizontal
        CHECK_FLOAT_APPROX(va.y, vb.y);
    }
    
    TEST_CASE("Capsule vertical") {
        Capsule capsule(10.0f, 2.0f, false);
        
        Vec2 va = capsule.GetVertexA();
        Vec2 vb = capsule.GetVertexB();
        
        // Should be vertical
        CHECK_FLOAT_APPROX(va.x, vb.x);
    }
    
    TEST_CASE("Capsule mass computation") {
        Capsule capsule(10.0f, 2.0f);
        MassData mass;
        capsule.ComputeMass(1.0f, &mass);
        
        CHECK(mass.mass > 0.0f);
        CHECK(mass.inertia > 0.0f);
    }
    
    TEST_CASE("Capsule AABB computation") {
        Capsule capsule(10.0f, 2.0f);
        AABB aabb;
        capsule.ComputeAABB(identity, &aabb);
        
        // AABB should contain the capsule
        Vec2 extents = aabb.GetExtents();
        CHECK(extents.x > 0.0f);
        CHECK(extents.y > 0.0f);
    }
    
    TEST_CASE("Capsule support") {
        Capsule capsule(10.0f, 2.0f, true);
        
        // Support in positive x direction should be vertex B
        int32 support = capsule.GetSupport(Vec2(1.0f, 0.0f));
        CHECK(support == 1);
        
        // Support in negative x direction should be vertex A
        support = capsule.GetSupport(Vec2(-1.0f, 0.0f));
        CHECK(support == 0);
    }
    
    TEST_CASE("Capsule vertices") {
        Capsule capsule(10.0f, 2.0f, true);
        
        Vec2 v0 = capsule.GetVertex(0);
        Vec2 v1 = capsule.GetVertex(1);
        
        CHECK_VEC2_APPROX(v0, capsule.GetVertexA());
        CHECK_VEC2_APPROX(v1, capsule.GetVertexB());
    }
}

TEST_SUITE("Geometry - Edge Cases") {
    TEST_CASE("Very small circle") {
        Circle circle(1e-6f);
        
        CHECK(circle.GetRadius() > 0.0f);
        MassData mass;
        circle.ComputeMass(1.0f, &mass);
        CHECK(mass.mass > 0.0f);
    }
    
    TEST_CASE("Very large circle") {
        Circle circle(1e6f);
        
        MassData mass;
        circle.ComputeMass(1.0f, &mass);
        CHECK(mass.mass > 0.0f);
        CHECK(mass.inertia > 0.0f);
    }
    
    TEST_CASE("Very small polygon") {
        Polygon box(1e-6f, 1e-6f, default_radius);
        
        CHECK(box.GetVertexCount() == 4);
        CHECK(box.GetArea() > 0.0f);
    }
    
    TEST_CASE("Very large polygon") {
        Polygon box(1e6f, 1e6f, default_radius);
        
        MassData mass;
        box.ComputeMass(1.0f, &mass);
        CHECK(mass.mass > 0.0f);
    }
    
    TEST_CASE("Zero radius capsule") {
        Capsule capsule(10.0f, 0.0f);
        
        // Should still be valid
        CHECK_FLOAT_APPROX(capsule.GetLength(), 10.0f);
    }
}
