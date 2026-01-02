#include "test_helpers.h"
#include <flywheel/aabb.h>

using namespace flywheel;
using namespace flywheel::test;

TEST_SUITE("AABB - Construction and Properties") {
    TEST_CASE("AABB construction") {
        AABB aabb1;
        AABB aabb2(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        
        CHECK(aabb2.min.x == 0.0f);
        CHECK(aabb2.min.y == 0.0f);
        CHECK(aabb2.max.x == 10.0f);
        CHECK(aabb2.max.y == 10.0f);
    }
    
    TEST_CASE("AABB center") {
        AABB aabb(Vec2(0.0f, 0.0f), Vec2(10.0f, 20.0f));
        Vec2 center = aabb.GetCenter();
        
        CHECK_FLOAT_APPROX(center.x, 5.0f);
        CHECK_FLOAT_APPROX(center.y, 10.0f);
    }
    
    TEST_CASE("AABB extents") {
        AABB aabb(Vec2(2.0f, 3.0f), Vec2(12.0f, 13.0f));
        Vec2 extents = aabb.GetExtents();
        
        CHECK_FLOAT_APPROX(extents.x, 10.0f);
        CHECK_FLOAT_APPROX(extents.y, 10.0f);
    }
    
    TEST_CASE("AABB area") {
        AABB aabb(Vec2(0.0f, 0.0f), Vec2(5.0f, 4.0f));
        float area = aabb.GetArea();
        
        CHECK_FLOAT_APPROX(area, 20.0f);
    }
    
    TEST_CASE("AABB perimeter") {
        AABB aabb(Vec2(0.0f, 0.0f), Vec2(5.0f, 4.0f));
        float perimeter = aabb.GetPerimeter();
        
        CHECK_FLOAT_APPROX(perimeter, 18.0f); // 2 * (5 + 4)
    }
    
    TEST_CASE("AABB zero size") {
        AABB aabb(Vec2(5.0f, 5.0f), Vec2(5.0f, 5.0f));
        
        CHECK_FLOAT_APPROX(aabb.GetArea(), 0.0f);
        CHECK_FLOAT_APPROX(aabb.GetPerimeter(), 0.0f);
    }
}

TEST_SUITE("AABB - Point and Overlap Tests") {
    TEST_CASE("AABB point containment - inside") {
        AABB aabb(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        
        CHECK(aabb.TestPoint(Vec2(5.0f, 5.0f)));
        CHECK(aabb.TestPoint(Vec2(0.0f, 0.0f)));
        CHECK(aabb.TestPoint(Vec2(10.0f, 10.0f)));
        CHECK(aabb.TestPoint(Vec2(0.0f, 10.0f)));
        CHECK(aabb.TestPoint(Vec2(10.0f, 0.0f)));
    }
    
    TEST_CASE("AABB point containment - outside") {
        AABB aabb(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        
        CHECK_FALSE(aabb.TestPoint(Vec2(-1.0f, 5.0f)));
        CHECK_FALSE(aabb.TestPoint(Vec2(11.0f, 5.0f)));
        CHECK_FALSE(aabb.TestPoint(Vec2(5.0f, -1.0f)));
        CHECK_FALSE(aabb.TestPoint(Vec2(5.0f, 11.0f)));
        CHECK_FALSE(aabb.TestPoint(Vec2(-1.0f, -1.0f)));
        CHECK_FALSE(aabb.TestPoint(Vec2(11.0f, 11.0f)));
    }
    
    TEST_CASE("AABB overlap - overlapping") {
        AABB aabb1(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        AABB aabb2(Vec2(5.0f, 5.0f), Vec2(15.0f, 15.0f));
        
        CHECK(aabb1.TestOverlap(aabb2));
        CHECK(aabb2.TestOverlap(aabb1));
    }
    
    TEST_CASE("AABB overlap - contained") {
        AABB aabb1(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        AABB aabb2(Vec2(2.0f, 2.0f), Vec2(8.0f, 8.0f));
        
        CHECK(aabb1.TestOverlap(aabb2));
        CHECK(aabb2.TestOverlap(aabb1));
    }
    
    TEST_CASE("AABB overlap - touching edges") {
        AABB aabb1(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        AABB aabb2(Vec2(10.0f, 0.0f), Vec2(20.0f, 10.0f));
        
        CHECK(aabb1.TestOverlap(aabb2));
        CHECK(aabb2.TestOverlap(aabb1));
    }
    
    TEST_CASE("AABB overlap - separated") {
        AABB aabb1(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        AABB aabb2(Vec2(11.0f, 0.0f), Vec2(20.0f, 10.0f));
        
        CHECK_FALSE(aabb1.TestOverlap(aabb2));
        CHECK_FALSE(aabb2.TestOverlap(aabb1));
    }
    
    TEST_CASE("AABB overlap - separated vertically") {
        AABB aabb1(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        AABB aabb2(Vec2(0.0f, 11.0f), Vec2(10.0f, 20.0f));
        
        CHECK_FALSE(aabb1.TestOverlap(aabb2));
        CHECK_FALSE(aabb2.TestOverlap(aabb1));
    }
    
    TEST_CASE("AABB contains - fully contained") {
        AABB aabb1(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        AABB aabb2(Vec2(2.0f, 2.0f), Vec2(8.0f, 8.0f));
        
        CHECK(aabb1.Contains(aabb2));
        CHECK_FALSE(aabb2.Contains(aabb1));
    }
    
    TEST_CASE("AABB contains - same size") {
        AABB aabb1(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        AABB aabb2(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        
        CHECK(aabb1.Contains(aabb2));
        CHECK(aabb2.Contains(aabb1));
    }
    
    TEST_CASE("AABB contains - partially overlapping") {
        AABB aabb1(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        AABB aabb2(Vec2(5.0f, 5.0f), Vec2(15.0f, 15.0f));
        
        CHECK_FALSE(aabb1.Contains(aabb2));
        CHECK_FALSE(aabb2.Contains(aabb1));
    }
}

TEST_SUITE("AABB - Union and Intersection") {
    TEST_CASE("AABB union with point - inside") {
        AABB aabb(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        Vec2 point(5.0f, 5.0f);
        
        AABB result = AABB::Union(aabb, point);
        
        CHECK_VEC2_APPROX(result.min, aabb.min);
        CHECK_VEC2_APPROX(result.max, aabb.max);
    }
    
    TEST_CASE("AABB union with point - outside") {
        AABB aabb(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        Vec2 point(15.0f, 15.0f);
        
        AABB result = AABB::Union(aabb, point);
        
        CHECK_VEC2_APPROX(result.min, Vec2(0.0f, 0.0f));
        CHECK_VEC2_APPROX(result.max, Vec2(15.0f, 15.0f));
    }
    
    TEST_CASE("AABB union with point - negative") {
        AABB aabb(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        Vec2 point(-5.0f, -5.0f);
        
        AABB result = AABB::Union(aabb, point);
        
        CHECK_VEC2_APPROX(result.min, Vec2(-5.0f, -5.0f));
        CHECK_VEC2_APPROX(result.max, Vec2(10.0f, 10.0f));
    }
    
    TEST_CASE("AABB union with AABB - overlapping") {
        AABB aabb1(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        AABB aabb2(Vec2(5.0f, 5.0f), Vec2(15.0f, 15.0f));
        
        AABB result = AABB::Union(aabb1, aabb2);
        
        CHECK_VEC2_APPROX(result.min, Vec2(0.0f, 0.0f));
        CHECK_VEC2_APPROX(result.max, Vec2(15.0f, 15.0f));
    }
    
    TEST_CASE("AABB union with AABB - separated") {
        AABB aabb1(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        AABB aabb2(Vec2(20.0f, 20.0f), Vec2(30.0f, 30.0f));
        
        AABB result = AABB::Union(aabb1, aabb2);
        
        CHECK_VEC2_APPROX(result.min, Vec2(0.0f, 0.0f));
        CHECK_VEC2_APPROX(result.max, Vec2(30.0f, 30.0f));
    }
    
    TEST_CASE("AABB union with AABB - contained") {
        AABB aabb1(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        AABB aabb2(Vec2(2.0f, 2.0f), Vec2(8.0f, 8.0f));
        
        AABB result = AABB::Union(aabb1, aabb2);
        
        CHECK_VEC2_APPROX(result.min, aabb1.min);
        CHECK_VEC2_APPROX(result.max, aabb1.max);
    }
    
    TEST_CASE("AABB intersection - overlapping") {
        AABB aabb1(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        AABB aabb2(Vec2(5.0f, 5.0f), Vec2(15.0f, 15.0f));
        
        AABB result = AABB::Intersection(aabb1, aabb2);
        
        CHECK_VEC2_APPROX(result.min, Vec2(5.0f, 5.0f));
        CHECK_VEC2_APPROX(result.max, Vec2(10.0f, 10.0f));
    }
    
    TEST_CASE("AABB intersection - contained") {
        AABB aabb1(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        AABB aabb2(Vec2(2.0f, 2.0f), Vec2(8.0f, 8.0f));
        
        AABB result = AABB::Intersection(aabb1, aabb2);
        
        CHECK_VEC2_APPROX(result.min, aabb2.min);
        CHECK_VEC2_APPROX(result.max, aabb2.max);
    }
    
    TEST_CASE("AABB intersection - no overlap") {
        AABB aabb1(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
        AABB aabb2(Vec2(20.0f, 20.0f), Vec2(30.0f, 30.0f));
        
        AABB result = AABB::Intersection(aabb1, aabb2);
        
        // Result should have min > max (invalid AABB)
        CHECK(result.min.x > result.max.x);
        CHECK(result.min.y > result.max.y);
    }
}

TEST_SUITE("AABB - Edge Cases") {
    TEST_CASE("AABB with negative coordinates") {
        AABB aabb(Vec2(-10.0f, -10.0f), Vec2(-5.0f, -5.0f));
        
        CHECK_FLOAT_APPROX(aabb.GetArea(), 25.0f);
        CHECK(aabb.TestPoint(Vec2(-7.0f, -7.0f)));
        CHECK_FALSE(aabb.TestPoint(Vec2(0.0f, 0.0f)));
    }
    
    TEST_CASE("AABB spanning origin") {
        AABB aabb(Vec2(-5.0f, -5.0f), Vec2(5.0f, 5.0f));
        
        CHECK(aabb.TestPoint(Vec2(0.0f, 0.0f)));
        CHECK_VEC2_APPROX(aabb.GetCenter(), Vec2(0.0f, 0.0f));
        CHECK_FLOAT_APPROX(aabb.GetArea(), 100.0f);
    }
    
    TEST_CASE("AABB very large") {
        AABB aabb(Vec2(-1e6f, -1e6f), Vec2(1e6f, 1e6f));
        
        CHECK(aabb.TestPoint(Vec2(0.0f, 0.0f)));
        CHECK(aabb.TestPoint(Vec2(5e5f, 5e5f)));
        CHECK_VEC2_APPROX(aabb.GetCenter(), Vec2(0.0f, 0.0f));
    }
    
    TEST_CASE("AABB very small") {
        AABB aabb(Vec2(0.0f, 0.0f), Vec2(1e-6f, 1e-6f));
        
        CHECK(aabb.TestPoint(Vec2(5e-7f, 5e-7f)));
        CHECK(aabb.GetArea() > 0.0f);
    }
    
    TEST_CASE("AABB line (zero height)") {
        AABB aabb(Vec2(0.0f, 5.0f), Vec2(10.0f, 5.0f));
        
        CHECK_FLOAT_APPROX(aabb.GetArea(), 0.0f);
        CHECK(aabb.TestPoint(Vec2(5.0f, 5.0f)));
        CHECK_FALSE(aabb.TestPoint(Vec2(5.0f, 6.0f)));
    }
    
    TEST_CASE("AABB line (zero width)") {
        AABB aabb(Vec2(5.0f, 0.0f), Vec2(5.0f, 10.0f));
        
        CHECK_FLOAT_APPROX(aabb.GetArea(), 0.0f);
        CHECK(aabb.TestPoint(Vec2(5.0f, 5.0f)));
        CHECK_FALSE(aabb.TestPoint(Vec2(6.0f, 5.0f)));
    }
}

TEST_SUITE("AABB - Stress Tests") {
    TEST_CASE("AABB many unions") {
        AABB aabb(Vec2(0.0f, 0.0f), Vec2(1.0f, 1.0f));
        
        // Union with many points
        for (int i = 0; i < 100; ++i) {
            Vec2 point(random_float(-100.0f, 100.0f), random_float(-100.0f, 100.0f));
            aabb = AABB::Union(aabb, point);
        }
        
        // AABB should be valid
        CHECK(aabb.min.x <= aabb.max.x);
        CHECK(aabb.min.y <= aabb.max.y);
        CHECK(aabb.GetArea() >= 0.0f);
    }
    
    TEST_CASE("AABB many overlaps") {
        AABB center(Vec2(-10.0f, -10.0f), Vec2(10.0f, 10.0f));
        
        int overlapCount = 0;
        int noOverlapCount = 0;
        
        for (int i = 0; i < 100; ++i) {
            Vec2 min(random_float(-50.0f, 50.0f), random_float(-50.0f, 50.0f));
            Vec2 max = min + Vec2(random_float(1.0f, 20.0f), random_float(1.0f, 20.0f));
            AABB test(min, max);
            
            if (center.TestOverlap(test)) {
                overlapCount++;
            } else {
                noOverlapCount++;
            }
        }
        
        // Should have some of each
        CHECK(overlapCount > 0);
        CHECK(noOverlapCount > 0);
    }
}
