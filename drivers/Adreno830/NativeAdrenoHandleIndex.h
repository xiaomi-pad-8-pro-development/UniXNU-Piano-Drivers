/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_HANDLE_INDEX_H
#define NATIVE_ADRENO_HANDLE_INDEX_H
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <IOKit/IOLib.h>
/* Handle -> slot of a fixed table whose handles are never 0: open addressing over twice the slots with linear probing, deletion by
 * backward shift (no tombstones), so a lookup is a few probes instead of a walk over every slot. i567: the submit path looked each IB's
 * handle up three times by walking a 4096-entry table, and the table had to grow to 16384 entries. */
template<size_t Slots> class NativeAdrenoHandleIndex {
    static_assert(Slots && !(Slots&(Slots-1)) && Slots<=(size_t(1)<<23),"a power of two the 24-bit hash covers twice");
    struct Cell {uint64_t key;uint32_t value;};
    Cell *cells_=nullptr;
    /* Twice the slots of the last allocate() (Slots is only the bound): a connection of a small Metal client indexes 4096 objects in
     * 128 KiB, not the render server's 16384 in 512 KiB (2026-10-30, 32 connection slots). */
    size_t size_=0,mask_=0;
    size_t home(uint64_t key) const {return (size_t)((key*UINT64_C(0x9e3779b97f4a7c15))>>40)&mask_;}
public:
    /* slots: a power of two up to Slots, the most keys present at a time. */
    bool allocate(size_t slots=Slots)
    {
        if(!slots || slots>Slots || (slots&(slots-1)))return false;
        if(cells_ && size_!=slots*2)release();
        if(!cells_) {
            cells_=(Cell*)IOMalloc(sizeof(Cell)*(slots*2));   /* no IONewData in a template: its typed-allocation macro trips clang's parser here */
            if(!cells_)return false;
            size_=slots*2;mask_=size_-1;
        }
        memset(cells_,0,sizeof(Cell)*size_);
        return true;
    }
    void release() {if(cells_)IOFree(cells_,sizeof(Cell)*size_);cells_=nullptr;size_=mask_=0;}
    /* The slots of the last allocate() (0 while released). */
    size_t capacity() const {return size_/2;}
    /* UINT32_MAX: not present. */
    uint32_t find(uint64_t key) const
    {
        if(!cells_ || !key)return UINT32_MAX;
        for(size_t i=home(key),n=0;n<size_;i=(i+1)&mask_,++n) {
            if(!cells_[i].key)return UINT32_MAX;
            if(cells_[i].key==key)return cells_[i].value;
        }
        return UINT32_MAX;
    }
    /* At most capacity() keys are ever present, so a free cell always exists. */
    void insert(uint64_t key,uint32_t value)
    {
        if(!cells_ || !key)return;
        size_t i=home(key);
        while(cells_[i].key && cells_[i].key!=key)i=(i+1)&mask_;
        cells_[i]={key,value};
    }
    void erase(uint64_t key)
    {
        if(!cells_ || !key)return;
        size_t i=home(key);
        for(size_t n=0;;i=(i+1)&mask_,++n) {
            if(n==size_ || !cells_[i].key)return;
            if(cells_[i].key==key)break;
        }
        /* Pull back each later cell of the run whose home is not cyclically inside (i, j]. */
        for(size_t j=i;;) {
            j=(j+1)&mask_;
            if(!cells_[j].key)break;
            const size_t k=home(cells_[j].key);
            if(i<=j?(i<k && k<=j):(i<k || k<=j))continue;
            cells_[i]=cells_[j];i=j;
        }
        cells_[i]={0,0};
    }
};
/* M10 (ARCHITECTURE 6.5 step 2): user-client handle slots recycled first in, first out and quarantined. A freed slot is handed out
 * again only after at least Quarantine further allocations, so a handle a client thread still holds in a sparse array it has not
 * cleared yet cannot name a new buffer soon after (the Turnip darwin_bo_finish race of 6.5). The handle encodes slot + 1 in the
 * low 32 bits (Turnip indexes its sparse array with them) and the slot's generation in the high 32 bits. Slots never used before
 * come first, in order. Serialized by the owner's lock. */
template<size_t Slots,size_t Quarantine=4096> class NativeAdrenoHandleSlots {
    static_assert(Slots && Slots<UINT32_MAX,"slot numbers fit 32 bits");
    struct Free {uint32_t slot;uint64_t freed_at;};
    Free *queue_=nullptr;              /* ring of free slots, oldest first */
    uint32_t *generation_=nullptr;
    size_t head_=0,count_=0;
    uint64_t allocations_=0;
public:
    bool allocate()
    {
        if(!queue_)queue_=(Free*)IOMalloc(sizeof(Free)*Slots);
        if(!generation_)generation_=(uint32_t*)IOMalloc(sizeof(uint32_t)*Slots);
        if(!queue_ || !generation_) {release();return false;}
        for(size_t i=0;i<Slots;++i) {queue_[i]={uint32_t(i),0};generation_[i]=0;}
        head_=0;count_=Slots;allocations_=Quarantine;   /* never-used slots are past any quarantine */
        return true;
    }
    void release()
    {
        if(queue_)IOFree(queue_,sizeof(Free)*Slots);
        if(generation_)IOFree(generation_,sizeof(uint32_t)*Slots);
        queue_=nullptr;generation_=nullptr;head_=count_=0;
    }
    /* The oldest free slot past its quarantine, or UINT32_MAX (none free, or the oldest freed too recently). */
    uint32_t take()
    {
        if(!queue_ || !count_)return UINT32_MAX;
        const Free &front=queue_[head_];
        if(allocations_-front.freed_at<Quarantine)return UINT32_MAX;
        const uint32_t slot=front.slot;
        head_=(head_+1)%Slots;--count_;++allocations_;
        return slot;
    }
    /* Back to the queue; its next handle has the next generation. */
    bool give(uint32_t slot)
    {
        if(!queue_ || slot>=Slots || count_>=Slots)return false;
        ++generation_[slot];
        queue_[(head_+count_)%Slots]={slot,allocations_};++count_;
        return true;
    }
    uint64_t handleFor(uint32_t slot) const {return slot<Slots && generation_?encode(slot,generation_[slot]):0;}
    static uint64_t encode(uint32_t slot,uint32_t generation) {return uint64_t(generation)<<32|(uint64_t(slot)+1);}
    /* UINT32_MAX for a handle no slot of this table can have issued. */
    static uint32_t slotOf(uint64_t handle)
    {
        const uint32_t low=uint32_t(handle);
        return low && low<=Slots?low-1:UINT32_MAX;
    }
    /* Whether handle is the current one of its slot (a stale generation is not). */
    bool current(uint64_t handle) const
    {
        const uint32_t slot=slotOf(handle);
        return slot!=UINT32_MAX && generation_ && generation_[slot]==uint32_t(handle>>32);
    }
    size_t freeSlots() const {return count_;}
    uint64_t allocations() const {return allocations_;}
};
#endif
