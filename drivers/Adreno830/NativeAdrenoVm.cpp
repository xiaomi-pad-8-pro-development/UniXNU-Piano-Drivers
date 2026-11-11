/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoVm.h"
#include "NativeAdrenoVmAlias.h"
#include "NativeAdrenoMeter.h"
extern "C" volatile uint32_t native_adreno_trace_stage;
extern "C" {int (*native_adreno_dram_read32)(uint64_t,uint32_t *);int (*native_adreno_cached_read32)(uint64_t,uint32_t *);
    int (*native_adreno_line_audit)(uint64_t,size_t,uint32_t *,uint32_t *);}
#include <string.h>
#include <IOKit/IOLib.h>
#ifdef ADRENO830_HOST_TEST
#include <mutex>
#else
#include <IOKit/IOLocks.h>
#endif
/* The effective coherence level of ARCHITECTURE 4.14.1 (NativeAdrenoVm::setCoherence): 0 none, 1 GPU-read-only imports
 * write-back inner-shareable, 2 also GPU-written imports and own BOs mapped write-back. Counters: maintenance passes skipped for
 * coherent mappings at map, unmap, submission and completion. */
extern "C" {volatile uint32_t native_adreno_io_coherent;volatile uint32_t native_adreno_coherent_skips[4];}
static bool coherentMapping(unsigned writeback) {return writeback && native_adreno_io_coherent;}
static bool overlap(uint64_t a,uint64_t ab,uint64_t b,uint64_t bb)
{return a<b+bb && b<a+ab;}
/* Mappings of one cache/shareability/privilege class may share pages; only pages shared across classes refuse. */
static unsigned aliasClass(unsigned permissions,unsigned writeback,unsigned shareability)
{return writeback|(shareability<<1)|((permissions&ArmSmmu4KPrivileged)?8u:0u);}
static void reverse(ArmSmmu4KRange *first,ArmSmmu4KRange *last)
{while(first<last && first<--last) {const ArmSmmu4KRange t=*first;*first++=*last;*last=t;}}
static bool validBinding(const NativeAdrenoVmRegion &r,unsigned upper)
{
    return r.tables && r.arena && r.state && !r.tables->quarantined() &&
        r.tables->deviceReferences() && r.tables->spanCount()==1 &&
        r.tables->spans()[0].bytes==r.tables->bytes() && !r.tables->spans()[0].offset &&
        r.arena->mapping==r.tables->mapping() && r.arena->upper_region==upper &&
        r.arena->config.table_physical==r.tables->spans()[0].physical &&
        r.arena->capacity_pages==r.tables->bytes()/4096 && r.state->prepared &&
        !r.state->poisoned && !r.state->tables.leaf_pages;
}
/* The VM lock (ARCHITECTURE 5.4, M10): page-table updates and their TLB synchronization. A wait for it goes to the call meter on the
 * VM's own clock: a submission holding the Ring lock waits here for another process's table transaction. */
#ifdef ADRENO830_HOST_TEST
static void *lockAlloc() {return new std::mutex;}
static void lockFree(void *lock) {delete static_cast<std::mutex *>(lock);}
static void lockTake(void *lock)
{
    std::mutex *mutex=static_cast<std::mutex *>(lock);
    NativeAdrenoMeterLock(Adreno830WaitVm,native_adreno_vm_now_us,[&] {return mutex->try_lock();},[&] {mutex->lock();});
}
static void lockDrop(void *lock) {static_cast<std::mutex *>(lock)->unlock();}
#else
static void *lockAlloc() {return IOLockAlloc();}
static void lockFree(void *lock) {IOLockFree(static_cast<IOLock *>(lock));}
static void lockTake(void *lock)
{
    IOLock *held=static_cast<IOLock *>(lock);
    NativeAdrenoMeterLock(Adreno830WaitVm,native_adreno_vm_now_us,[&] {return IOLockTryLock(held)!=0;},[&] {IOLockLock(held);});
}
static void lockDrop(void *lock) {IOLockUnlock(static_cast<IOLock *>(lock));}
#endif
void NativeAdrenoVm::lock() const {if(lock_)lockTake(lock_);}
void NativeAdrenoVm::unlock() const {if(lock_)lockDrop(lock_);}
namespace {
struct Locked {
    const NativeAdrenoVm *vm;void (NativeAdrenoVm::*drop)() const;
    ~Locked() {(vm->*drop)();}
};
}
#define VM_LOCKED Locked locked_scope{this,&NativeAdrenoVm::unlock};lock();(void)locked_scope

ArmSmmu4KLiveRegion NativeAdrenoVm::region(unsigned upper) const
{
    return {regions_[upper].arena,regions_[upper].state,
        ranges_+(upper?lower_count_:0),upper?upper_count_:lower_count_};
}
/* The clock (the publish shell sets it, pagani-gpu-kopt bit 5; null: no time in any figure below) and per-phase time of the two table
 * transactions: [0] map, [1] unmap; each row is count, the phases in order, the slowest call and the bytes mapped (int-134: BoMap
 * averaged 135 ms and the desktop crawled). */
extern "C" {uint64_t (*native_adreno_vm_now_us)(void)=nullptr;volatile uint64_t native_adreno_vm_phase_us[2][8];}
/* {count, us, spans} per kind: 0 map of driver memory, 1 import map without the alias pass, 2 import map with it, 3 unmap dropping its
 * pages from the TLB one by one, 4 unmap invalidating the ASID. */
extern "C" {volatile uint64_t native_adreno_vm_kind_us[NativeAdrenoVmKinds][3];}
extern "C" {volatile uint32_t native_adreno_vm_reserve_refused[2];}
namespace {
struct PhaseClock {
    uint64_t last=native_adreno_vm_now_us?native_adreno_vm_now_us():0,began=last;
    void lap(unsigned row,unsigned slot) {
        if(!native_adreno_vm_now_us)return;
        const uint64_t now=native_adreno_vm_now_us();native_adreno_vm_phase_us[row][slot]+=now-last;last=now;
    }
    uint64_t elapsed() const {return native_adreno_vm_now_us?native_adreno_vm_now_us()-began:0;}
    void done(unsigned row,uint64_t bytes) {
        if(!native_adreno_vm_now_us)return;
        const uint64_t total=native_adreno_vm_now_us()-began;
        native_adreno_vm_phase_us[row][0]+=1;native_adreno_vm_phase_us[row][7]+=bytes;
        if(total>native_adreno_vm_phase_us[row][6])native_adreno_vm_phase_us[row][6]=total;
    }
};
}
/* A transaction that cannot be proved: the VM refuses everything from now on, every BO and table stays referenced, and the
 * owned streams FAULT (their SMRs stay valid, so the SIDs stay blocked). Only this path poisons; no fault status does. */
IOReturn NativeAdrenoVm::fail(ArmSmmu4KStatus status)
{
    poisoned_=true;result_.operation=status;result_.complete=false;
    for(size_t i=0;mappings_ && i<high_;++i)if(mappings_[i].memory)mappings_[i].memory->quarantine();
    for(auto &r:regions_)if(r.arena) {r.tables->quarantine();r.state->poisoned=1;}
    if(has_programmer_)
        result_.quarantine=programmer_.fault(programmer_.cookie,bank_,streams_,stream_count_)==kIOReturnSuccess?
            ArmSmmu4KSuccess:ArmSmmu4KIoError;
    else if(kit_streams_) {
        ArmSmmu4KStreamResult routes;
        result_.quarantine=ArmSmmu4KFaultStreams(&io_,kit_streams_,&routes);
    }
    return kIOReturnIOError;
}
IOReturn NativeAdrenoVm::attachCommon(const ArmSmmu4KIo *io,ArmSmmu4KContextConfig *context,const NativeAdrenoVmRegion *lower,
    const NativeAdrenoVmRegion *upper,size_t capacity,unsigned budget)
{
    if(attached_ || ranges_ || poisoned_)return kIOReturnBusy;
    if(!io || !io->read32 || !io->write32 || !io->write64 || !io->barrier || !context || (!lower && !upper) || !capacity ||
        capacity>262144 || !budget || budget>1048576 || (lower && !validBinding(*lower,0)) || (upper && !validBinding(*upper,1)))
        return kIOReturnBadArgument;
    if(lower && (context->lower!=&lower->arena->config || context->lower_tables!=&lower->state->tables))return kIOReturnBadArgument;
    if(upper && (context->upper!=&upper->arena->config || context->upper_tables!=&upper->state->tables))return kIOReturnBadArgument;
    if(!lock_ && !(lock_=lockAlloc()))return kIOReturnNoMemory;
    ranges_=IONewData(ArmSmmu4KRange,capacity);
    mappings_=IONewData(Mapping,NativeAdrenoVmMaxObjects);
    if(!ranges_ || !mappings_ || !index_.allocate()) {
        if(ranges_)IODeleteData(ranges_,ArmSmmu4KRange,capacity);
        if(mappings_)IODeleteData(mappings_,Mapping,NativeAdrenoVmMaxObjects);
        index_.release();lockFree(lock_);lock_=nullptr;ranges_=nullptr;mappings_=nullptr;
        return kIOReturnNoMemory;
    }
    for(size_t i=0;i<NativeAdrenoVmMaxObjects;++i)mappings_[i]=Mapping{};
    free_hint_=0;high_=0;memset(class_spans_,0,sizeof(class_spans_));memset(import_spans_,0,sizeof(import_spans_));
    maintained_head_=UINT32_MAX;maintained_count_=0;window_count_=0;
    capacity_=capacity;io_=*io;context_=context;budget_=budget;
    if(lower)regions_[0]=*lower;
    if(upper)regions_[1]=*upper;
    for(unsigned i=0;i<2;++i)if(regions_[i].arena) {
        if(NativeAdrenoBindSmmuVisibility(regions_[i].tables,&visibility_[i])!=kIOReturnSuccess) {
            IODeleteData(ranges_,ArmSmmu4KRange,capacity_);IODeleteData(mappings_,Mapping,NativeAdrenoVmMaxObjects);index_.release();
            regions_[0]={};regions_[1]={};ranges_=nullptr;mappings_=nullptr;capacity_=0;lockFree(lock_);lock_=nullptr;
            return kIOReturnBadArgument;
        }
    }
    attached_=true;coherent_walk_=false;
    return kIOReturnSuccess;
}
/* The whole-region proof the kit's first transaction of a region would make, done once here: every descriptor of both regions
 * (ArmSmmu4KLiveAuditContext, no SMMU access) and the routes and context through the programmer, under the board's own rules
 * (A830Smmu: TTBR1 guard table, CB2 TTBR1 0, FSR record only). The regions are then recorded as proved at the bank's register
 * offset, so the kit's transactions read no SMMU register: a map only writes and publishes leaves, an unmap invalidates its
 * pages (ARCHITECTURE 4.6.4). */
IOReturn NativeAdrenoVm::verifyAndSeed()
{
    ArmSmmu4KLiveRegion lower=region(0),upper=region(1);
    ArmSmmu4KLiveResult audit;
    ArmSmmu4KStatus status=ArmSmmu4KLiveAuditContext(context_,lower.arena?&lower:nullptr,upper.arena?&upper:nullptr,&audit);
    if(status)return fail(status);
    if(has_programmer_) {
        if(programmer_.verify(programmer_.cookie,bank_,streams_,stream_count_)!=kIOReturnSuccess)return fail(ArmSmmu4KStateMismatch);
    } else {
        ArmSmmu4KStreamResult routes;
        status=ArmSmmu4KObserveStreams(&io_,kit_streams_,context_,&routes);
        if(status)return fail(status);
        context_offset_=routes.context.context_offset;
    }
    if(!context_offset_ || context_offset_>io_.register_bytes || io_.register_bytes-context_offset_<0x7f8)
        return fail(ArmSmmu4KBadRange);
    for(auto &r:regions_)if(r.state) {
        if(r.state->generation==UINT64_MAX)return fail(ArmSmmu4KStateMismatch);
        if(!r.state->generation)r.state->generation=1;
        r.state->context_offset=context_offset_;
    }
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::attach(const NativeAdrenoSmmuBinding &smmu,const NativeAdrenoVmRegion *lower,
    const NativeAdrenoVmRegion *upper,size_t capacity,unsigned budget)
{
    const NativeAdrenoSmmuProgrammer *p=smmu.programmer;
    if(attached_ || ranges_ || poisoned_)return kIOReturnBusy;
    if(!p || !p->io || !p->verify || !p->fault || !smmu.streams || !smmu.stream_count ||
        smmu.stream_count>NativeAdrenoSmmuContext::MaxStreams || !smmu.context_offset || smmu.bank>255)return kIOReturnBadArgument;
    IOReturn status=attachCommon(p->io,smmu.context,lower,upper,capacity,budget);
    if(status!=kIOReturnSuccess)return status;
    VM_LOCKED;
    programmer_=*p;has_programmer_=true;kit_streams_=nullptr;sync_after_map_=p->sync_after_map;
    for(size_t i=0;i<smmu.stream_count;++i)streams_[i]=smmu.streams[i];
    stream_count_=smmu.stream_count;bank_=smmu.bank;context_offset_=smmu.context_offset;
    return verifyAndSeed();
}
IOReturn NativeAdrenoVm::attach(const ArmSmmu4KIo *io,ArmSmmu4KContextConfig *context,
    ArmSmmu4KStreamState *streams,const NativeAdrenoVmRegion *lower,const NativeAdrenoVmRegion *upper,
    size_t capacity,unsigned budget)
{
    if(attached_ || ranges_ || poisoned_)return kIOReturnBusy;
    if(!streams || !streams->attached || streams->poisoned || !streams->count || streams->count>8)return kIOReturnBadArgument;
    IOReturn status=attachCommon(io,context,lower,upper,capacity,budget);
    if(status!=kIOReturnSuccess)return status;
    VM_LOCKED;
    has_programmer_=false;sync_after_map_=false;kit_streams_=streams;bank_=streams->bank;stream_count_=streams->count;
    for(size_t i=0;i<stream_count_;++i)streams_[i]=streams->streams[i];
    return verifyAndSeed();
}
IOReturn NativeAdrenoVm::detach()
{
    if(!attached_)return kIOReturnNotReady;
    {
        VM_LOCKED;
        if(poisoned_ || objects_ || spanCount())return kIOReturnBusy;
        IODeleteData(ranges_,ArmSmmu4KRange,capacity_);IODeleteData(mappings_,Mapping,NativeAdrenoVmMaxObjects);index_.release();
        attached_=false;regions_[0]={};regions_[1]={};context_=nullptr;kit_streams_=nullptr;has_programmer_=false;
        sync_after_map_=false;
        capacity_=0;ranges_=nullptr;mappings_=nullptr;stream_count_=0;window_count_=0;fence_=nullptr;
    }
    lockFree(lock_);lock_=nullptr;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::reserveWindow(uint64_t iova,uint64_t bytes)
{
    if(!attached_)return kIOReturnNotReady;
    VM_LOCKED;
    if(poisoned_)return kIOReturnNotReady;
    if(!bytes || ((iova|bytes)&4095) || iova>UINT64_MAX-bytes+1)return kIOReturnBadArgument;
    if(window_count_>=NativeAdrenoVmMaxWindows)return kIOReturnNoResources;
    for(unsigned u=0;u<2;++u) {
        const ArmSmmu4KLiveArena *a=regions_[u].arena;
        if(!a || iova<a->iova || iova-a->iova>=a->bytes)continue;
        /* Only a window nothing is mapped in yet. */
        const size_t i=rangeAt(iova);
        if(i<spanCount() && ranges_[i].iova<=iova+(bytes-1))return kIOReturnBusy;
        windows_[window_count_++]={iova,bytes,u};
        return kIOReturnSuccess;
    }
    return kIOReturnBadArgument;
}
bool NativeAdrenoVm::forbidden(uint64_t iova,uint64_t bytes) const
{
    for(size_t i=0;i<window_count_;++i)
        if(iova<=windows_[i].iova+(windows_[i].bytes-1) && windows_[i].iova<=iova+(bytes-1))return true;
    return false;
}
IOReturn NativeAdrenoVm::findAddress(unsigned upper,size_t bytes,uint64_t alignment,uint64_t *out) const
{
    if(!out)return kIOReturnBadArgument;*out=0;
    if(!attached_)return kIOReturnNotReady;
    VM_LOCKED;
    if(poisoned_)return kIOReturnNotReady;
    if(upper>1 || !regions_[upper].arena || !bytes || (bytes&4095) || alignment<4096 ||
        (alignment&(alignment-1)))return kIOReturnBadArgument;
    const ArmSmmu4KLiveArena *a=regions_[upper].arena;
    const size_t end=upper?spanCount():lower_count_;
    size_t i=upper?lower_count_:0;
    uint64_t offset=0;   /* relative to the aperture: native addresses of the upper region may end at 2^64 */
    for(;;) {
        const uint64_t rem=(a->iova+offset)&(alignment-1),padding=rem?alignment-rem:0;
        if(padding>a->bytes-offset)return kIOReturnNoResources;
        offset+=padding;
        if(bytes>a->bytes-offset)return kIOReturnNoResources;
        bool moved=false;
        for(size_t w=0;w<window_count_;++w) {
            if(windows_[w].upper!=upper)continue;
            const uint64_t ws=windows_[w].iova-a->iova,we=ws+windows_[w].bytes;
            if(offset<we && ws<offset+bytes) {offset=we;moved=true;}
        }
        if(moved)continue;
        while(i<end && ranges_[i].iova-a->iova+ranges_[i].bytes<=offset)++i;
        if(i==end || ranges_[i].iova-a->iova>=offset+bytes) {*out=a->iova+offset;return kIOReturnSuccess;}
        offset=ranges_[i].iova-a->iova+ranges_[i].bytes;
    }
}
IOReturn NativeAdrenoVm::preflight(const NativeAdrenoMemory *memory,unsigned upper,uint64_t iova,
    unsigned permissions,unsigned writeback,unsigned shareability,bool exclusive) const
{
    if(!memory || upper>1 || !regions_[upper].arena || !memory->backed() || memory->quarantined() ||
        !memory->bytes() || !memory->spanCount() || !(permissions&ArmSmmu4KRead) || (permissions&~15u) ||
        writeback>1 || (shareability!=2 && shareability!=3))return kIOReturnBadArgument;
    const ArmSmmu4KLiveArena *a=regions_[upper].arena;
    if((iova&4095) || iova<a->iova || iova-a->iova>=a->bytes || memory->bytes()>a->bytes-(iova-a->iova))
        return kIOReturnBadArgument;
    /* The null guard and the GMEM window are never mapped (ARCHITECTURE 4.6.2). */
    if(forbidden(iova,memory->bytes()))return kIOReturnBadArgument;
    /* Memory the GPU may only read is never mapped writable; the scanout is never mapped write-back. */
    if((memory->readOnly() && (permissions&ArmSmmu4KWrite)) || (memory->scanout() && writeback))return kIOReturnBadArgument;
    if(objects_>=NativeAdrenoVmMaxObjects || memory->spanCount()>capacity_-spanCount())return kIOReturnNoResources;
    {
        /* ranges_ is sorted by address: only the first range not wholly below iova can overlap (native addresses may end at 2^64,
         * so ends are compared as last bytes). A walk over every range cost each map a pass over 52609 of them (i567). */
        const size_t i=rangeAt(iova);
        if(i<spanCount() && ranges_[i].iova<=iova+(memory->bytes()-1))return kIOReturnBusy;
    }
    if(a->config.output_bits<32 || a->config.output_bits>48)return kIOReturnBadArgument;
    uint64_t offset=0,limit=UINT64_C(1)<<a->config.output_bits;
    for(size_t j=0;j<memory->spanCount();++j) {
        const auto &span=memory->spans()[j];
        if(span.offset!=offset || !span.physical || !span.bytes || ((span.physical|span.bytes)&4095) ||
            span.bytes>memory->bytes()-offset || span.physical>=limit || span.bytes>limit-span.physical)
            return kIOReturnBadArgument;
        for(const auto &r:regions_)if(r.arena && overlap(span.physical,span.bytes,
            r.arena->config.table_physical,r.arena->capacity_pages*UINT64_C(4096)))return kIOReturnBadArgument;
        offset+=span.bytes;
    }
    if(offset!=memory->bytes())return kIOReturnBadArgument;
    /* Only another cache/shareability/privilege class can conflict. Reuse the
     * arena's caller-owned temporary alias scratch before the table transaction
     * uses it. Index the smaller side and query the other in O(log n); neither
     * the resource spans nor the actual page tables are modified here. */
    /* exclusive (exclusivePages): no mapped range can alias these pages in conflict, so the pass over every other class's ranges is
     * not needed. i568: this pass and the table transaction's walked every one of about 50000 ranges on each map, 0.6 ms of its 0.66. */
    if(exclusive)return kIOReturnSuccess;
    const unsigned mine=aliasClass(permissions,writeback,shareability);
    const size_t other=spanCount()-class_spans_[mine&15];
    if(!other)return kIOReturnSuccess;
    const bool indexNew=memory->spanCount()<=other;
    const size_t smaller=indexNew?memory->spanCount():other;
    uint32_t *order=a->alias_scratch;
    if(order && smaller<=a->alias_scratch_entries) {
        if(indexNew) {
            for(size_t i=0;i<smaller;++i)order[i]=(uint32_t)i;
            NativeAdrenoPhysicalIndex index(memory->spans(),order,smaller);
            for(size_t i=0;i<spanCount();++i) {
                const auto &r=ranges_[i];
                if(aliasClass(r.permissions,r.writeback,r.shareability)!=mine && index.overlaps(r.physical,r.bytes))return kIOReturnBadArgument;
            }
        } else {
            size_t n=0;
            for(size_t i=0;i<spanCount();++i)
                if(aliasClass(ranges_[i].permissions,ranges_[i].writeback,ranges_[i].shareability)!=mine)order[n++]=(uint32_t)i;
            NativeAdrenoPhysicalIndex index(ranges_,order,n);
            for(size_t j=0;j<memory->spanCount();++j)
                if(index.overlaps(memory->spans()[j].physical,memory->spans()[j].bytes))return kIOReturnBadArgument;
        }
        return kIOReturnSuccess;
    }
    /* Small/unconfigured callers retain the conservative allocation-free path. */
    for(size_t i=0;i<spanCount();++i) {
        const ArmSmmu4KRange &r=ranges_[i];
        if(aliasClass(r.permissions,r.writeback,r.shareability)==mine)continue;
        for(size_t j=0;j<memory->spanCount();++j)
            if(overlap(memory->spans()[j].physical,memory->spans()[j].bytes,r.physical,r.bytes))return kIOReturnBadArgument;
    }
    return kIOReturnSuccess;
}
/* Whether no mapped range can alias memory's pages with another cache/shareability/privilege class. A buffer this driver allocated
 * owns its pages until its unmap has completed (the free waits for it). An import (a user range, usually an IOSurface, which any
 * process may import again) can alias only another import or a range of driver memory it covers: it is exclusive when every mapped
 * import has its class and none of its pages lies in the BO pool or in a mapped scattered buffer. i568: about 100 surface imports of
 * 5000-6000 spans each, every one checked against every other range. */
bool NativeAdrenoVm::exclusivePages(const NativeAdrenoMemory *memory,unsigned cls) const
{
    if(!memory->imported() && !memory->scanout())return true;
    size_t imports=0;for(size_t k=0;k<16;++k)imports+=import_spans_[k];
    if(imports!=import_spans_[cls&15])return false;
    const uint64_t pool=NativeAdrenoMemory::poolPhysical();const size_t poolBytes=NativeAdrenoMemory::poolBytes();
    const auto *spans=memory->spans();const size_t n=memory->spanCount();
    if(pool)for(size_t j=0;j<n;++j)if(overlap(spans[j].physical,spans[j].bytes,pool,poolBytes))return false;
    size_t pairs=0;
    for(size_t i=0;i<high_;++i) {
        const Mapping &m=mappings_[i];
        if(!m.memory || m.memory->imported() || m.memory->scanout() || m.memory->pooled())continue;
        const size_t mn=m.memory->spanCount();
        if(mn>(size_t)65536/(n?n:1) || (pairs+=mn*n)>(size_t)1<<20)return false;   /* too many to compare: the full check */
        for(size_t a=0;a<mn;++a)for(size_t b=0;b<n;++b)
            if(overlap(m.memory->spans()[a].physical,m.memory->spans()[a].bytes,spans[b].physical,spans[b].bytes))return false;
    }
    return true;
}
NativeAdrenoVm::Mapping *NativeAdrenoVm::find(uint64_t handle)
{return const_cast<Mapping *>(static_cast<const NativeAdrenoVm *>(this)->find(handle));}
const NativeAdrenoVm::Mapping *NativeAdrenoVm::find(uint64_t handle) const
{
    const uint32_t slot=mappings_?index_.find(handle):UINT32_MAX;
    if(slot>=NativeAdrenoVmMaxObjects)return nullptr;
    const Mapping *m=&mappings_[slot];
    return m->handle==handle && m->memory?m:nullptr;
}
/* The first range whose last byte is at or past iova (spanCount() when none). */
size_t NativeAdrenoVm::rangeAt(uint64_t iova) const
{
    size_t low=0,high=spanCount();
    while(low<high) {
        const size_t mid=low+(high-low)/2;
        if(ranges_[mid].iova+(ranges_[mid].bytes-1)<iova)low=mid+1;else high=mid;
    }
    return low;
}
/* The n ranges just past the table's end (one mapping's spans, consecutive in IOVA and all in one gap) take their sorted place. */
void NativeAdrenoVm::insert(size_t n,unsigned upper)
{
    const size_t count=spanCount();
    for(size_t i=0;i<n;++i)++class_spans_[aliasClass(ranges_[count+i].permissions,ranges_[count+i].writeback,ranges_[count+i].shareability)&15];
    const size_t at=rangeAt(ranges_[count].iova);
    /* With room for a copy of the new ranges past them, one memmove opens the gap (the three reversals moved every later range twice). */
    if(capacity_-count-n>=n) {
        memcpy(ranges_+count+n,ranges_+count,n*sizeof(*ranges_));
        memmove(ranges_+at+n,ranges_+at,(count-at)*sizeof(*ranges_));
        memcpy(ranges_+at,ranges_+count+n,n*sizeof(*ranges_));
    } else {
        reverse(ranges_+at,ranges_+count);reverse(ranges_+count,ranges_+count+n);reverse(ranges_+at,ranges_+count+n);
    }
    if(upper)upper_count_+=n;else lower_count_+=n;
}
void NativeAdrenoVm::erase(size_t at,size_t n,unsigned upper)
{
    const size_t count=spanCount();
    for(size_t i=0;i<n;++i)--class_spans_[aliasClass(ranges_[at+i].permissions,ranges_[at+i].writeback,ranges_[at+i].shareability)&15];
    memmove(ranges_+at,ranges_+at+n,(count-at-n)*sizeof(*ranges_));
    if(upper)upper_count_-=n;else lower_count_-=n;
}
void NativeAdrenoVm::linkMaintained(Mapping &m)
{
    const uint32_t slot=uint32_t(&m-mappings_);
    m.prev=UINT32_MAX;m.next=maintained_head_;
    if(maintained_head_!=UINT32_MAX)mappings_[maintained_head_].prev=slot;
    maintained_head_=slot;m.maintained=true;++maintained_count_;
}
void NativeAdrenoVm::unlinkMaintained(Mapping &m)
{
    if(!m.maintained)return;
    if(m.prev!=UINT32_MAX)mappings_[m.prev].next=m.next;else maintained_head_=m.next;
    if(m.next!=UINT32_MAX)mappings_[m.next].prev=m.prev;
    m.prev=m.next=UINT32_MAX;m.maintained=false;--maintained_count_;
}
/* Per-submission maintenance (ARCHITECTURE 4.14.1): a GPU-read-only import mapped non-cacheable is cleaned before each submission
 * (empty from coherence level 1); so is an own BO whose CPU view is cacheable (not write-combined: never with the default
 * write-combined user BOs). */
static bool needsSubmitMaintenance(const NativeAdrenoMemory *memory,unsigned permissions,unsigned writeback)
{
    if(coherentMapping(writeback) || memory->scanout())return false;
    if(memory->imported())return !(permissions&ArmSmmu4KWrite);
    return !memory->writeCombined() && memory->mapping();
}
IOReturn NativeAdrenoVm::mapLocked(NativeAdrenoMemory *memory,unsigned upper,uint64_t iova,
    unsigned permissions,unsigned writeback,unsigned shareability,uint64_t *handle)
{
    if(!handle)return kIOReturnBadArgument;*handle=0;result_={};
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    PhaseClock clock;
    if(!memory)return kIOReturnBadArgument;
    const unsigned cls=aliasClass(permissions,writeback,shareability)&15;
    const bool exclusive=exclusivePages(memory,cls);
    IOReturn checked=preflight(memory,upper,iova,permissions,writeback,shareability,exclusive);
    if(checked)return checked;
    if(next_handle_==UINT64_MAX)return kIOReturnNoResources;
    clock.lap(0,5);
    checked=memory->holdForDevice();if(checked)return checked;
    clock.lap(0,1);
    Mapping *record=nullptr;
    for(size_t i=free_hint_;i<NativeAdrenoVmMaxObjects;++i)if(!mappings_[i].memory) {
        record=&mappings_[i];free_hint_=i+1;if(high_<i+1)high_=i+1;break;
    }
    if(!record) {memory->quarantine();return fail(ArmSmmu4KStateMismatch);}
    *record=Mapping{};
    record->memory=memory;record->iova=iova;record->spans=memory->spanCount();record->permissions=permissions;
    record->writeback=writeback;record->shareability=shareability;
    ++objects_;
    __atomic_store_n(&bytes_,bytes_+memory->bytes(),__ATOMIC_RELAXED);
    /* Once, before the GPU can see the pages: an import is cleaned (and invalidated when the GPU writes it) through its physical
     * pages; a write-combined own BO and the scanout need nothing; a coherent mapping needs nothing. */
    if(coherentMapping(writeback))native_adreno_coherent_skips[0]=native_adreno_coherent_skips[0]+1;
    else if(!memory->scanout()) {
        checked=memory->publishForDevice(0,memory->bytes(),permissions&ArmSmmu4KWrite,distrust_write_combined_);
        if(checked)return fail(ArmSmmu4KIoError);
    }
    clock.lap(0,2);
    /* Every span in one table transaction, staged in the table's free tail (preflight checked the room). */
    const size_t spans=record->spans;
    ArmSmmu4KRange *changes=ranges_+spanCount();
    for(size_t i=0;i<spans;++i) {
        const auto &span=memory->spans()[i];
        changes[i]={iova+span.offset,span.physical,span.bytes,permissions,writeback,shareability};
    }
    ArmSmmu4KLiveRegion lower=region(0),high=region(1);ArmSmmu4KLiveResult update;
    /* preflight checked a map that is not exclusive against every other class's range of both regions, the alias rule of the table
     * transaction's own pass (same classes, same overlap), so that pass would sort them all again for the same answer, under this
     * lock, which every submission waits for (upstream Adreno740 a087fca: 3.56 ms a map, 79% of all map time). pagani-gpu-kopt bit 2. */
    const ArmSmmu4KStatus status=exclusive || NativeAdrenoKoptOn(NativeAdrenoKoptAliasOnce)?
        ArmSmmu4KLiveMapContextBatchExclusive(&io_,bank_,context_,lower.arena?&lower:nullptr,
            high.arena?&high:nullptr,upper,&visibility_[upper],changes,spans,budget_,&update):
        ArmSmmu4KLiveUpdateContextBatch(&io_,bank_,context_,lower.arena?&lower:nullptr,
            high.arena?&high:nullptr,upper,&visibility_[upper],changes,spans,1,budget_,&update);
    if(status)return fail(status);
    /* The programmer's sync_after_map (A830Smmu, annibale profile): once the leaves are published, one invalidation of the
     * bank's ASID and its TLBSYNC, which A830Smmu's I/O sends as TLBIASID + TLBSYNC = 1. A sync that does not complete within
     * the budget fails the transaction like any other: the VM is poisoned and keeps every reference. */
    if(sync_after_map_) {
        ArmSmmu4KProgramResult synced;
        const ArmSmmu4KStatus sync=ArmSmmu4KSyncContextTlb(&io_,context_offset_,context_->asid,0,budget_,&synced);
        if(sync)return fail(sync);
    }
    clock.lap(0,3);
    insert(spans,upper);record->mapped_spans=spans;result_.completed_spans=spans;
    if(memory->imported() || memory->scanout())import_spans_[cls]+=spans;
    if(needsSubmitMaintenance(memory,permissions,writeback))linkMaintained(*record);
    clock.lap(0,4);clock.done(0,memory->bytes());
    {
        const unsigned kind=!memory->imported()?0:exclusive?1:2;
        native_adreno_vm_kind_us[kind][0]=native_adreno_vm_kind_us[kind][0]+1;
        native_adreno_vm_kind_us[kind][1]=native_adreno_vm_kind_us[kind][1]+clock.elapsed();
        native_adreno_vm_kind_us[kind][2]=native_adreno_vm_kind_us[kind][2]+spans;
    }
    record->handle=next_handle_++;*handle=record->handle;index_.insert(record->handle,(uint32_t)(record-mappings_));
    result_.complete=true;return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::map(NativeAdrenoMemory *memory,unsigned upper,uint64_t iova,
    unsigned permissions,unsigned writeback,unsigned shareability,uint64_t *handle)
{
    if(!handle)return kIOReturnBadArgument;
    *handle=0;
    if(!attached_)return kIOReturnNotReady;
    VM_LOCKED;
    return mapLocked(memory,upper,iova,permissions,writeback,shareability,handle);
}
void NativeAdrenoVm::setCoherence(unsigned level)
{
    __atomic_store_n(&native_adreno_io_coherent,level>NativeAdrenoCoherenceCached?uint32_t(NativeAdrenoCoherenceCached):level,
                     __ATOMIC_RELEASE);
}
unsigned NativeAdrenoVm::coherence() {return __atomic_load_n(&native_adreno_io_coherent,__ATOMIC_ACQUIRE);}
void NativeAdrenoVm::attributes(const NativeAdrenoMemory &memory,unsigned permissions,unsigned requested_writeback,
    unsigned requested_shareability,unsigned *writeback,unsigned *shareability)
{
    (void)requested_shareability;
    const unsigned level=coherence();
    bool cached=false;
    if(memory.scanout())cached=false;                                       /* the scanout: non-cacheable, the GPU writes it */
    else if(memory.imported())
        cached=(permissions&ArmSmmu4KWrite)?level>=NativeAdrenoCoherenceCached:level>=NativeAdrenoCoherenceReadImports;
    else cached=level>=NativeAdrenoCoherenceCached && requested_writeback;   /* own BOs: write-combined for the CPU below 2 */
    if(writeback)*writeback=cached?1u:0u;
    if(shareability)*shareability=cached?3u:2u;
}
IOReturn NativeAdrenoVm::mapClient(NativeAdrenoMemory *memory,uint64_t iova,unsigned permissions,unsigned writeback,
    unsigned shareability,uint64_t *handle,size_t keep_objects,size_t keep_spans)
{
    if(!handle)return kIOReturnBadArgument;
    *handle=0;
    if(!memory || writeback>1 || (shareability!=2 && shareability!=3) || (permissions&ArmSmmu4KPrivileged))
        return kIOReturnBadArgument;
    /* Memory the owner maps read-only could only be pinned for reading: the GPU gets the read half of what was asked. */
    if(memory->readOnly())permissions&=~unsigned(ArmSmmu4KWrite);
    unsigned wb=0,sh=2;
    attributes(*memory,permissions,writeback,shareability,&wb,&sh);
    if(!attached_)return kIOReturnNotReady;
    VM_LOCKED;
    /* The render server's reserve (NativeAdrenoVmRenderReserve*): another client's mapping must leave this much of both tables. */
    if(keep_objects && objects_+1+keep_objects>NativeAdrenoVmMaxObjects) {
        native_adreno_vm_reserve_refused[0]=native_adreno_vm_reserve_refused[0]+1;
        return kIOReturnNoResources;
    }
    if(keep_spans && memory->spanCount()+keep_spans>capacity_-spanCount()) {
        native_adreno_vm_reserve_refused[1]=native_adreno_vm_reserve_refused[1]+1;
        return kIOReturnNoResources;
    }
    return mapLocked(memory,0,iova,permissions,wb,sh,handle);
}
bool NativeAdrenoVm::completed(uint64_t fence) const
{
    if(!fence)return true;
    if(!fence_)return false;
    const uint32_t done=__atomic_load_n(fence_,__ATOMIC_ACQUIRE);
    return int32_t(done-uint32_t(fence))>=0;
}
IOReturn NativeAdrenoVm::unmap(uint64_t handle)
{
    native_adreno_trace_stage=10;
    result_={};
    if(!attached_)return kIOReturnNotReady;
    VM_LOCKED;
    if(poisoned_)return kIOReturnNotReady;
    Mapping *record=find(handle);if(!record)return kIOReturnBadArgument;
    /* Only this buffer's own last use decides (ARCHITECTURE 4.6.4): the GPU is never idled globally for an unmap. */
    if(record->jobs || !completed(record->last_use))return kIOReturnBusy;
    PhaseClock clock;
    native_adreno_trace_stage=11;
    clock.lap(1,1);
    unsigned upper=regions_[1].arena && record->iova>=regions_[1].arena->iova;
    /* Imports were cleaned and invalidated once when mapped and the scanout and write-combined BOs never need it; only an own BO
     * the CPU sees cacheable (not write-combined) drops the lines the GPU's writes made stale. */
    if(coherentMapping(record->writeback))native_adreno_coherent_skips[1]=native_adreno_coherent_skips[1]+1;
    else if((record->permissions&ArmSmmu4KWrite) && !record->memory->imported() && !record->memory->scanout()) {
        if(record->memory->acquireDeviceWrites(0,record->memory->bytes())!=kIOReturnSuccess)
            return fail(ArmSmmu4KIoError);
    }
    clock.lap(1,2);
    /* The mapping's spans are consecutive in the table; unmap them in one transaction. */
    const size_t spans=record->spans;
    const size_t at=rangeAt(record->iova);
    if(at>=spanCount() || ranges_[at].iova!=record->iova || spans>spanCount()-at)return fail(ArmSmmu4KStateMismatch);
    for(size_t i=0;i<spans;++i) {
        const auto &span=record->memory->spans()[i];
        const ArmSmmu4KRange &r=ranges_[at+i];
        if(r.iova!=record->iova+span.offset || r.physical!=span.physical || r.bytes!=span.bytes ||
            r.permissions!=record->permissions || r.writeback!=record->writeback || r.shareability!=record->shareability)
            return fail(ArmSmmu4KStateMismatch);
    }
    ArmSmmu4KLiveRegion lower=region(0),high=region(1);ArmSmmu4KLiveResult update;
    native_adreno_trace_stage=12;
    /* Leaves cleared and published, then <= 512 pages by TLBIVAL (one 64-bit store each) or TLBIASID, one TLBSYNC: the pages may
     * be freed only once that sync completed (the kit returns after it). */
    const ArmSmmu4KStatus status=ArmSmmu4KLiveUpdateContextBatch(&io_,bank_,context_,lower.arena?&lower:nullptr,
        high.arena?&high:nullptr,upper,&visibility_[upper],ranges_+at,spans,0,budget_,&update);
    native_adreno_trace_stage=13;
    if(status)return fail(status);
    clock.lap(1,3);
    erase(at,spans,upper);record->mapped_spans=0;result_.completed_spans=spans;
    native_adreno_trace_stage=14;
    clock.lap(1,4);
    if(record->memory->retireAfterQuiescence()!=kIOReturnSuccess)return fail(ArmSmmu4KStateMismatch);
    clock.lap(1,5);clock.done(1,record->memory->bytes());
    if(record->memory->imported() || record->memory->scanout())
        import_spans_[aliasClass(record->permissions,record->writeback,record->shareability)&15]-=spans;
    {
        const unsigned kind=record->memory->bytes()<=(size_t)ArmSmmu4KTlbPageLimit*4096?3:4;   /* page TLBIs, else the ASID */
        native_adreno_vm_kind_us[kind][0]=native_adreno_vm_kind_us[kind][0]+1;
        native_adreno_vm_kind_us[kind][1]=native_adreno_vm_kind_us[kind][1]+clock.elapsed();
        native_adreno_vm_kind_us[kind][2]=native_adreno_vm_kind_us[kind][2]+spans;
    }
    unlinkMaintained(*record);
    index_.erase(record->handle);
    const size_t slot=(size_t)(record-mappings_);if(slot<free_hint_)free_hint_=slot;
    __atomic_store_n(&bytes_,bytes_-record->memory->bytes(),__ATOMIC_RELAXED);
    *record=Mapping{};--objects_;
    while(high_ && !mappings_[high_-1].memory)--high_;result_.complete=true;return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::describe(uint64_t handle,uint64_t *iova,size_t *bytes,
    unsigned *permissions) const
{
    if(!iova || !bytes || !permissions)return kIOReturnBadArgument;
    *iova=0;*bytes=0;*permissions=0;
    if(!attached_)return kIOReturnNotReady;
    VM_LOCKED;
    if(poisoned_)return kIOReturnNotReady;
    const Mapping *mapping=find(handle);if(!mapping)return kIOReturnBadArgument;
    *iova=mapping->iova;*bytes=mapping->memory->bytes();*permissions=mapping->permissions;
    return mapping->mapped_spans==mapping->spans?kIOReturnSuccess:kIOReturnNotReady;
}
/* TLBIALL of the bank and TLBSYNC at the proved register offset: no image or fault observation. */
IOReturn NativeAdrenoVm::invalidateTlb()
{
    if(!attached_)return kIOReturnNotReady;
    VM_LOCKED;
    if(poisoned_)return kIOReturnNotReady;
    ArmSmmu4KProgramResult r;
    const ArmSmmu4KStatus status=ArmSmmu4KSyncContextTlb(&io_,context_offset_,context_->asid,1,budget_,&r);
    return status?fail(status):kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::setTableWalkCoherent(bool coherent)
{
    if(!attached_)return kIOReturnNotReady;
    VM_LOCKED;
    if(poisoned_)return kIOReturnNotReady;
    ArmSmmu4KLiveVisibility next[2]={};
    for(unsigned i=0;i<2;++i)if(regions_[i].arena) {
        const IOReturn status=coherent?NativeAdrenoBindSmmuVisibilityCoherent(regions_[i].tables,&next[i]):
            NativeAdrenoBindSmmuVisibility(regions_[i].tables,&next[i]);
        if(status!=kIOReturnSuccess)return status;
    }
    for(unsigned i=0;i<2;++i)if(regions_[i].arena)visibility_[i]=next[i];
    coherent_walk_=coherent;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::cleanForSubmission()
{
    if(!attached_)return kIOReturnNotReady;
    VM_LOCKED;
    if(poisoned_)return kIOReturnNotReady;
    for(uint32_t slot=maintained_head_;slot!=UINT32_MAX;slot=mappings_[slot].next) {
        const Mapping &m=mappings_[slot];
        if(m.memory->publishForDevice(0,m.memory->bytes(),false,distrust_write_combined_))return fail(ArmSmmu4KIoError);
    }
    native_adreno_coherent_skips[2]=native_adreno_coherent_skips[2]+(maintained_count_?0:1);
    return kIOReturnSuccess;
}
size_t NativeAdrenoVm::maintainedCount() const
{
    if(!attached_)return 0;
    VM_LOCKED;
    return maintained_count_;
}
IOReturn NativeAdrenoVm::publishMapped() {return cleanForSubmission();}
/* After a completion: nothing for imports (cleaned and invalidated once when mapped, 4.14.1) and write-combined BOs; an own BO
 * the CPU sees cacheable (never with the default write-combined user BOs) drops the lines the GPU's writes made stale. */
IOReturn NativeAdrenoVm::acquireImported()
{
    if(!attached_)return kIOReturnNotReady;
    VM_LOCKED;
    if(poisoned_)return kIOReturnNotReady;
    for(uint32_t slot=maintained_head_;slot!=UINT32_MAX;slot=mappings_[slot].next) {
        const Mapping &m=mappings_[slot];
        if(m.memory->imported() || !(m.permissions&ArmSmmu4KWrite))continue;
        if(m.memory->acquireDeviceWrites(0,m.memory->bytes()))return fail(ArmSmmu4KIoError);
    }
    native_adreno_coherent_skips[3]=native_adreno_coherent_skips[3]+1;
    return kIOReturnSuccess;
}
void NativeAdrenoVm::setFenceSource(const volatile uint32_t *completed)
{
    lock();fence_=completed;unlock();
}
IOReturn NativeAdrenoVm::noteUse(uint64_t handle,uint64_t fence)
{
    if(!attached_)return kIOReturnNotReady;
    VM_LOCKED;
    if(poisoned_)return kIOReturnNotReady;
    Mapping *record=find(handle);if(!record || !fence)return kIOReturnBadArgument;
    if(!record->last_use || int32_t(uint32_t(fence)-uint32_t(record->last_use))>0)record->last_use=fence;
    return kIOReturnSuccess;
}
uint64_t NativeAdrenoVm::lastUse(uint64_t handle) const
{
    if(!attached_)return 0;
    VM_LOCKED;
    const Mapping *record=find(handle);
    return record?record->last_use:0;
}
bool NativeAdrenoVm::idle(uint64_t handle) const
{
    if(!attached_)return false;
    VM_LOCKED;
    const Mapping *record=find(handle);
    return record && !record->jobs && completed(record->last_use);
}
IOReturn NativeAdrenoVm::sampleWord(uint64_t iova,uint32_t *cached,uint32_t *dram) const
{
    if(!cached || !dram)return kIOReturnBadArgument;
    *cached=*dram=~0u;
    if(!attached_)return kIOReturnNotReady;
    VM_LOCKED;
    if(poisoned_)return kIOReturnNotReady;
    if(iova&3)return kIOReturnBadArgument;
    for(size_t i=0;i<high_;++i) {
        const Mapping &m=mappings_[i];
        if(!m.memory || iova<m.iova || iova-m.iova>=m.memory->bytes())continue;
        const size_t offset=(size_t)(iova-m.iova);
        if(m.memory->mapping())*cached=static_cast<const volatile uint32_t *>(m.memory->mapping())[offset/4];
        for(size_t j=0;j<m.memory->spanCount();++j) {
            const auto &span=m.memory->spans()[j];
            if(offset<span.offset || offset-span.offset>=span.bytes)continue;
            uint32_t value=0;
            /* DRAM first: the cached reader writes its line back, so read after it DRAM would always agree (int-099). */
            if(native_adreno_dram_read32 && !native_adreno_dram_read32(span.physical+(offset-span.offset),&value))*dram=value;
            /* A write-combined BO's own mapping is non-cacheable: read the cacheable alias instead, so a dirty line is still seen. */
            if(native_adreno_cached_read32 && !native_adreno_cached_read32(span.physical+(offset-span.offset),&value))*cached=value;
            break;
        }
        return kIOReturnSuccess;
    }
    return kIOReturnBadArgument;
}
IOReturn NativeAdrenoVm::reassertWriteCombined(uint32_t *reset) const
{
    if(!reset)return kIOReturnBadArgument;
    if(!attached_)return kIOReturnNotReady;
    VM_LOCKED;
    if(poisoned_)return kIOReturnNotReady;
    for(size_t i=0;i<high_;++i) {
        const Mapping &m=mappings_[i];
        if(!m.memory)continue;
        const IOReturn status=m.memory->reassertWriteCombined(reset);
        if(status)return status;
    }
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::auditRange(uint64_t iova,size_t bytes,uint32_t *lines,uint32_t *dirty,uint32_t *stale) const
{
    if(!lines || !dirty || !stale)return kIOReturnBadArgument;
    if(!attached_)return kIOReturnNotReady;
    VM_LOCKED;
    if(poisoned_)return kIOReturnNotReady;
    if(!native_adreno_line_audit)return kIOReturnUnsupported;
    if(!bytes || iova>UINT64_MAX-bytes)return kIOReturnBadArgument;
    for(size_t i=0;i<high_;++i) {
        const Mapping &m=mappings_[i];
        if(!m.memory || iova<m.iova || iova-m.iova>=m.memory->bytes())continue;
        if(!m.memory->writeCombined())return kIOReturnSuccess;   /* maintained before every submission anyway */
        /* Whole lines; the range is clipped to its BO. */
        size_t at=(size_t)(iova-m.iova)&~(size_t)63;
        const size_t end=(size_t)(iova-m.iova)+bytes>m.memory->bytes()?m.memory->bytes():(size_t)(iova-m.iova)+bytes;
        while(at<end) {
            const NativeAdrenoPhysicalSpan *span=nullptr;
            for(size_t j=0;j<m.memory->spanCount();++j) {
                const auto &s=m.memory->spans()[j];
                if(at>=s.offset && at-s.offset<s.bytes) {span=&s;break;}
            }
            if(!span)return kIOReturnBadArgument;
            const uint64_t physical=span->physical+(at-span->offset);
            size_t chunk=4096-(size_t)(physical&4095);
            if(chunk>span->offset+span->bytes-at)chunk=span->offset+span->bytes-at;
            if(chunk>end-at)chunk=(end-at+63)&~(size_t)63;
            if(native_adreno_line_audit(physical,chunk,dirty,stale))return kIOReturnIOError;
            *lines+=(uint32_t)(chunk/64);at+=chunk;
        }
        return kIOReturnSuccess;
    }
    return kIOReturnBadArgument;
}
static IOReturn peekMapped(const NativeAdrenoMemory *memory,size_t offset,uint32_t *out,size_t count)
{
    for(size_t k=0;k<count && offset+4*k+4<=memory->bytes();++k)out[k]=static_cast<const volatile uint32_t *>(memory->mapping())[offset/4+k];
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::peekWords(uint64_t iova,uint32_t *out,size_t count) const
{
    if(!out || !attached_)return kIOReturnBadArgument;
    VM_LOCKED;
    if(poisoned_)return kIOReturnBadArgument;
    for(size_t k=0;k<count;++k)out[k]=0;
    for(size_t i=0;i<high_;++i) {
        const Mapping &m=mappings_[i];
        if(!m.memory || !m.memory->mapping() || iova<m.iova || iova-m.iova>=m.memory->bytes())continue;
        return peekMapped(m.memory,(size_t)(iova-m.iova),out,count);
    }
    return kIOReturnBadArgument;
}
IOReturn NativeAdrenoVm::peekWordsUncached(uint64_t iova,uint32_t *out,size_t count) const
{
    if(!out || !attached_ || !count)return kIOReturnBadArgument;
    VM_LOCKED;
    if(poisoned_)return kIOReturnBadArgument;
    for(size_t k=0;k<count;++k)out[k]=0;
    for(size_t i=0;i<high_;++i) {
        const Mapping &m=mappings_[i];
        if(!m.memory || !m.memory->mapping() || iova<m.iova || iova-m.iova>=m.memory->bytes())continue;
        const size_t offset=(size_t)(iova-m.iova),first=offset&~(size_t)4095;
        const size_t end=offset+4*count<m.memory->bytes()?offset+4*count:m.memory->bytes();
        const size_t bytes=((end+4095)&~(size_t)4095)-first;
        IOReturn status=NativeAdrenoMaintainOwnedPages(static_cast<unsigned char *>(m.memory->mapping())+first,bytes,NativeAdrenoCleanInvalidate);
        if(status)return status;
        return peekMapped(m.memory,offset,out,count);
    }
    return kIOReturnBadArgument;
}
uint64_t NativeAdrenoVm::handleAt(uint64_t iova) const
{
    if(!attached_ || !mappings_)return 0;
    VM_LOCKED;
    if(!mappings_)return 0;
    /* ranges_ is sorted by address: the range over iova names the mapping (its spans are consecutive). */
    const size_t i=rangeAt(iova);
    if(i>=spanCount() || ranges_[i].iova>iova)return 0;
    for(size_t j=0;j<high_;++j) {
        const Mapping &m=mappings_[j];
        if(m.memory && iova>=m.iova && iova-m.iova<m.memory->bytes())return m.handle;
    }
    return 0;
}
IOReturn NativeAdrenoVm::faultState(uint32_t out[4]) const
{
    if(!out || !attached_)return kIOReturnBadArgument;
    VM_LOCKED;
    if(poisoned_)return kIOReturnBadArgument;
    if(!context_offset_)return kIOReturnNotReady;
    static const uint32_t regs[4]={0x58,0x60,0x64,0x68};
    for(unsigned i=0;i<4;++i)if(io_.read32(io_.cookie,(size_t)context_offset_+regs[i],&out[i]))return kIOReturnIOError;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::holdJob(uint64_t handle)
{
    if(!attached_)return kIOReturnNotReady;
    VM_LOCKED;
    if(poisoned_)return kIOReturnNotReady;
    Mapping *record=find(handle);if(!record)return kIOReturnBadArgument;
    if(record->jobs==UINT32_MAX)return kIOReturnNoResources;
    IOReturn status=record->memory->holdForDevice();if(status)return status;
    ++record->jobs;return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::retireJobAfterCompletion(uint64_t handle)
{
    if(!attached_)return kIOReturnNotReady;
    VM_LOCKED;
    if(poisoned_)return kIOReturnNotReady;
    Mapping *record=find(handle);if(!record || !record->jobs)return kIOReturnBadArgument;
    IOReturn status=record->memory->retireAfterQuiescence();if(status)return status;
    --record->jobs;return kIOReturnSuccess;
}
