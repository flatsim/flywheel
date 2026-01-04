#pragma once

#include "allocator.h"

namespace flywheel
{

// You muse nest allocate/free pairs
class LinearAllocator : public Allocator
{
public:
    LinearAllocator(int32 initialCapacity = 16 * 1024);
    ~LinearAllocator();

    virtual void* Allocate(int32 size) override;
    virtual void Free(void* p, int32 size) override;
    virtual void Clear() override;

    bool GrowMemory();

    int32 GetCapacity() const;
    int32 GetAllocation() const;
    int32 GetMaxAllocation() const;

private:
    struct MemoryEntry
    {
        int8* data;
        int32 size;
        bool mallocUsed;
    };

    MemoryEntry* entries;
    int32 entryCount;
    int32 entryCapacity;

    int8* mem;
    int32 capacity;
    int32 index;

    int32 allocation;
    int32 maxAllocation;
};

inline int32 LinearAllocator::GetCapacity() const
{
    return capacity;
}

inline int32 LinearAllocator::GetAllocation() const
{
    return allocation;
}

inline int32 LinearAllocator::GetMaxAllocation() const
{
    return maxAllocation;
}

inline LinearAllocator::LinearAllocator(int32 initialCapacity)
    : entryCount{ 0 }
    , entryCapacity{ 32 }
    , capacity{ initialCapacity }
    , index{ 0 }
    , allocation{ 0 }
    , maxAllocation{ 0 }
{
    mem = (int8*)malloc(capacity);
    memset(mem, 0, capacity);
    entries = (MemoryEntry*)malloc(entryCapacity * sizeof(MemoryEntry));
}

inline LinearAllocator::~LinearAllocator()
{
    assert(index == 0 && entryCount == 0);

    flywheel::Free(entries);
    flywheel::Free(mem);
}

inline void* LinearAllocator::Allocate(int32 size)
{
    if (entryCount == entryCapacity)
    {
        // Grow entry array by half
        MemoryEntry* old = entries;
        entryCapacity += entryCapacity / 2;
        entries = (MemoryEntry*)malloc(entryCapacity * sizeof(MemoryEntry));
        memcpy(entries, old, entryCount * sizeof(MemoryEntry));
        memset(entries + entryCount, 0, (entryCapacity - entryCount) * sizeof(MemoryEntry));
        flywheel::Free(old);
    }

    MemoryEntry* entry = entries + entryCount;
    entry->size = size;

    if (index + size > capacity)
    {
        entry->data = (int8*)malloc(size);
        entry->mallocUsed = true;
    }
    else
    {
        entry->data = mem + index;
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

inline void LinearAllocator::Free(void* p, int32 size)
{
    MuliNotUsed(size);
    assert(entryCount > 0);

    MemoryEntry* entry = entries + (entryCount - 1);
    assert(entry->data == p);
    assert(entry->size == size);

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

    p = nullptr;
}

inline bool LinearAllocator::GrowMemory()
{
    assert(index == 0);

    if (maxAllocation < capacity)
    {
        return false;
    }

    // Grow memory by half
    flywheel::Free(mem);
    capacity += capacity / 2;
    mem = (int8*)malloc(capacity);
    memset(mem, 0, capacity);

    return true;
}

inline void LinearAllocator::Clear()
{
    entryCount = 0;
    index = 0;
    allocation = 0;
    maxAllocation = 0;
}

} // namespace flywheel
