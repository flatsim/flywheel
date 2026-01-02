#pragma once

#include "allocator.h"

namespace flywheel
{

// Stack allocator used for transient, predictable allocations.
// You muse nest allocate/free pairs
class StackAllocator : public Allocator
{
    static constexpr inline int32 stack_size = 100 * 1024;
    static constexpr inline int32 max_stack_entries = 32;

public:
    StackAllocator();
    ~StackAllocator();

    virtual void* Allocate(int32 size) override;
    virtual void Free(void* p, int32 size) override;
    virtual void Clear() override;

    int32 GetAllocation() const;
    int32 GetMaxAllocation() const;

private:
    struct StackEntry
    {
        int8* data;
        int32 size;
        bool mallocUsed;
    };

    int8 stack[stack_size];
    int32 index;

    int32 allocation;
    int32 maxAllocation;

    StackEntry entries[max_stack_entries];
    int32 entryCount;
};

inline int32 StackAllocator::GetAllocation() const
{
    return allocation;
}

inline int32 StackAllocator::GetMaxAllocation() const
{
    return maxAllocation;
}

inline StackAllocator::StackAllocator()
    : index{ 0 }
    , allocation{ 0 }
    , maxAllocation{ 0 }
    , entryCount{ 0 }
{
}

inline StackAllocator::~StackAllocator()
{
    MuliAssert(index == 0 && entryCount == 0);
}

inline void* StackAllocator::Allocate(int32 size)
{
    MuliAssert(entryCount < max_stack_entries && "Increase the maxStackEntries");

    StackEntry* entry = entries + entryCount;
    entry->size = size;

    if (index + size > stack_size)
    {
        entry->data = (int8*)flywheel::Alloc(size);
        entry->mallocUsed = true;
    }
    else
    {
        entry->data = stack + index;
        entry->mallocUsed = false;
        index += size;
    }

    allocation += size;
    if (allocation > maxAllocation)
    {
        maxAllocation = allocation;
    }

    ++entryCount;

    return entry->data;
}

inline void StackAllocator::Free(void* p, int32 size)
{
    MuliNotUsed(size);
    MuliAssert(entryCount > 0);

    StackEntry* entry = entries + entryCount - 1;
    MuliAssert(entry->data == p);
    MuliAssert(entry->size == size);

    if (entry->mallocUsed)
    {
        flywheel::Free(p);
    }
    else
    {
        index -= entry->size;
    }

    allocation -= entry->size;
    --entryCount;
}

inline void StackAllocator::Clear()
{
    index = 0;
    allocation = 0;
    maxAllocation = 0;
    entryCount = 0;
}

} // namespace flywheel
