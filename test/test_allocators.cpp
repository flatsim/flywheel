#include "test_helpers.h"
#include <flywheel/flywheel.hpp>

using namespace flywheel;
using namespace flywheel::test;

TEST_SUITE("Allocators - BlockAllocator") {
    TEST_CASE("BlockAllocator construction") {
        BlockAllocator allocator;
        
        CHECK(allocator.GetBlockCount() >= 0);
        CHECK(allocator.GetChunkCount() >= 0);
    }
    
    TEST_CASE("BlockAllocator simple allocation") {
        BlockAllocator allocator;
        
        void* p1 = allocator.Allocate(64);
        CHECK(p1 != nullptr);
        
        allocator.Free(p1, 64);
    }
    
    TEST_CASE("BlockAllocator multiple allocations") {
        BlockAllocator allocator;
        
        void* p1 = allocator.Allocate(32);
        void* p2 = allocator.Allocate(64);
        void* p3 = allocator.Allocate(128);
        
        CHECK(p1 != nullptr);
        CHECK(p2 != nullptr);
        CHECK(p3 != nullptr);
        CHECK(p1 != p2);
        CHECK(p2 != p3);
        
        allocator.Free(p1, 32);
        allocator.Free(p2, 64);
        allocator.Free(p3, 128);
    }
    
    TEST_CASE("BlockAllocator reuse") {
        BlockAllocator allocator;
        
        void* p1 = allocator.Allocate(64);
        allocator.Free(p1, 64);
        
        void* p2 = allocator.Allocate(64);
        CHECK(p2 != nullptr);
        // Should reuse the freed block
        CHECK(p1 == p2);
        
        allocator.Free(p2, 64);
    }
    
    TEST_CASE("BlockAllocator clear") {
        BlockAllocator allocator;
        
        void* p1 = allocator.Allocate(64);
        void* p2 = allocator.Allocate(128);
        
        allocator.Clear();
        
        // After clear, should be able to allocate again
        void* p3 = allocator.Allocate(64);
        CHECK(p3 != nullptr);
        
        allocator.Free(p3, 64);
    }
    
    TEST_CASE("BlockAllocator stress test") {
        BlockAllocator allocator;
        const int count = 100;
        void* ptrs[count];
        
        // Allocate many blocks
        for (int i = 0; i < count; ++i) {
            int size = 8 + (i % 10) * 8;
            ptrs[i] = allocator.Allocate(size);
            CHECK(ptrs[i] != nullptr);
        }
        
        // Free all blocks
        for (int i = 0; i < count; ++i) {
            int size = 8 + (i % 10) * 8;
            allocator.Free(ptrs[i], size);
        }
    }
}

TEST_SUITE("Allocators - LinearAllocator") {
    TEST_CASE("LinearAllocator construction") {
        LinearAllocator allocator;
        
        CHECK(allocator.GetCapacity() > 0);
        CHECK(allocator.GetAllocation() == 0);
        CHECK(allocator.GetMaxAllocation() == 0);
    }
    
    TEST_CASE("LinearAllocator simple allocation") {
        LinearAllocator allocator;
        
        void* p1 = allocator.Allocate(64);
        CHECK(p1 != nullptr);
        CHECK(allocator.GetAllocation() >= 64);
        
        allocator.Free(p1, 64);
    }
    
    TEST_CASE("LinearAllocator sequential allocations") {
        LinearAllocator allocator;
        
        void* p1 = allocator.Allocate(32);
        void* p2 = allocator.Allocate(64);
        void* p3 = allocator.Allocate(128);
        
        CHECK(p1 != nullptr);
        CHECK(p2 != nullptr);
        CHECK(p3 != nullptr);
        
        int32 totalAlloc = allocator.GetAllocation();
        CHECK(totalAlloc >= 32 + 64 + 128);
        
        // Must free in reverse order (LIFO)
        allocator.Free(p3, 128);
        allocator.Free(p2, 64);
        allocator.Free(p1, 32);
    }
    
    TEST_CASE("LinearAllocator clear") {
        LinearAllocator allocator;
        
        void* p1 = allocator.Allocate(64);
        void* p2 = allocator.Allocate(128);
        
        int32 allocBefore = allocator.GetAllocation();
        CHECK(allocBefore > 0);
        
        allocator.Clear();
        
        CHECK(allocator.GetAllocation() == 0);
        
        // Should be able to allocate after clear
        void* p3 = allocator.Allocate(64);
        CHECK(p3 != nullptr);
        
        allocator.Free(p3, 64);
    }
    
    TEST_CASE("LinearAllocator max allocation tracking") {
        LinearAllocator allocator;
        
        void* p1 = allocator.Allocate(100);
        int32 max1 = allocator.GetMaxAllocation();
        
        void* p2 = allocator.Allocate(200);
        int32 max2 = allocator.GetMaxAllocation();
        
        CHECK(max2 >= max1);
        CHECK(max2 >= 300);
        
        allocator.Free(p2, 200);
        allocator.Free(p1, 100);
    }
}

TEST_SUITE("Allocators - StackAllocator") {
    TEST_CASE("StackAllocator construction") {
        StackAllocator allocator;
        
        CHECK(allocator.GetAllocation() == 0);
        CHECK(allocator.GetMaxAllocation() == 0);
    }
    
    TEST_CASE("StackAllocator simple allocation") {
        StackAllocator allocator;
        
        void* p1 = allocator.Allocate(64);
        CHECK(p1 != nullptr);
        CHECK(allocator.GetAllocation() >= 64);
        
        allocator.Free(p1, 64);
        CHECK(allocator.GetAllocation() == 0);
    }
    
    TEST_CASE("StackAllocator LIFO order") {
        StackAllocator allocator;
        
        void* p1 = allocator.Allocate(32);
        void* p2 = allocator.Allocate(64);
        void* p3 = allocator.Allocate(128);
        
        CHECK(p1 != nullptr);
        CHECK(p2 != nullptr);
        CHECK(p3 != nullptr);
        
        // Must free in reverse order (LIFO)
        allocator.Free(p3, 128);
        allocator.Free(p2, 64);
        allocator.Free(p1, 32);
        
        CHECK(allocator.GetAllocation() == 0);
    }
    
    TEST_CASE("StackAllocator nesting") {
        StackAllocator allocator;
        
        void* p1 = allocator.Allocate(100);
        int32 alloc1 = allocator.GetAllocation();
        
        void* p2 = allocator.Allocate(200);
        int32 alloc2 = allocator.GetAllocation();
        
        void* p3 = allocator.Allocate(300);
        int32 alloc3 = allocator.GetAllocation();
        
        CHECK(alloc3 > alloc2);
        CHECK(alloc2 > alloc1);
        
        allocator.Free(p3, 300);
        CHECK(allocator.GetAllocation() == alloc2);
        
        allocator.Free(p2, 200);
        CHECK(allocator.GetAllocation() == alloc1);
        
        allocator.Free(p1, 100);
        CHECK(allocator.GetAllocation() == 0);
    }
    
    TEST_CASE("StackAllocator clear") {
        StackAllocator allocator;
        
        void* p1 = allocator.Allocate(64);
        void* p2 = allocator.Allocate(128);
        
        CHECK(allocator.GetAllocation() > 0);
        
        allocator.Clear();
        
        CHECK(allocator.GetAllocation() == 0);
        
        // Should be able to allocate after clear
        void* p3 = allocator.Allocate(64);
        CHECK(p3 != nullptr);
        
        allocator.Free(p3, 64);
    }
    
    TEST_CASE("StackAllocator max allocation tracking") {
        StackAllocator allocator;
        
        void* p1 = allocator.Allocate(100);
        int32 max1 = allocator.GetMaxAllocation();
        
        void* p2 = allocator.Allocate(200);
        int32 max2 = allocator.GetMaxAllocation();
        
        CHECK(max2 >= max1);
        
        allocator.Free(p2, 200);
        allocator.Free(p1, 100);
        
        // Max should persist even after freeing
        CHECK(allocator.GetMaxAllocation() == max2);
    }
}

TEST_SUITE("Allocators - Edge Cases") {
    TEST_CASE("BlockAllocator zero size") {
        BlockAllocator allocator;
        
        void* p = allocator.Allocate(0);
        // Implementation may return nullptr or valid pointer
        if (p != nullptr) {
            allocator.Free(p, 0);
        }
    }
    
    TEST_CASE("LinearAllocator large allocation") {
        LinearAllocator allocator(1024);
        
        // Allocate larger than initial capacity
        void* p = allocator.Allocate(2048);
        CHECK(p != nullptr);
        
        allocator.Free(p, 2048);
    }
    
    TEST_CASE("StackAllocator within limits") {
        StackAllocator allocator;
        
        // Allocate within stack size
        void* p = allocator.Allocate(1024);
        CHECK(p != nullptr);
        
        allocator.Free(p, 1024);
    }
}
