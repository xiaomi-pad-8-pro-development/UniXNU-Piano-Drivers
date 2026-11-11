/* SPDX-License-Identifier: MIT */
/* The ABI shell (ARCHITECTURE 2.2, 3.3, 6): the IOKit surface of the running A830. At S9 the orchestrator hands this file the live
 * VM, submit object, ring, clock, timer and DeviceInfo; one IOService (NativeAdrenoGpuPublished, named "NativeAdrenoGpu") is
 * published, and the staged AGXAccelerator (NativeAdrenoAccelerator.cpp) routes its newUserClient here too. Each GPU connection
 * (open type 0x4147) mounts its own NativeAdrenoUserClient core in one of NativeAdrenoMaxClients slots: own handles, own queue
 * range and reserved job slot on the one ring, own window of the one GPU address space (slot 0, the render server's, the large
 * one), own fault and suspend counters. Closing a connection waits for its own jobs only, then unmaps and frees what it created.
 * Locks (M10, ARCHITECTURE 5.4): every connection has its own lock around its core; the submit object's Ring lock and the BO
 * pool's Memory lock (pooled BOs and imports; a BO outside the pool takes none) are the shell's IOLocks; the VM serialises itself;
 * g.lock guards the slot registry and the per-name rows. Fence waits
 * and the submit retry sleep with no lock held (ARCHITECTURE 4.13.3). This is not the Metal or Turnip ABI: it carries
 * NativeAdrenoABI.h unchanged. BO CPU mappings are physical-range views (non-cacheable unless the DeviceInfo is CachedCoherent)
 * matching the GPU's PTEs. The connection class is named AGXDeviceUserClient on purpose: the original sandbox profiles of the
 * processes that use Metal allow IOServiceOpen for that user-client class name, and nothing else in this system can create one
 * (the Apple AGX kext is never loaded without an Apple GPU), so the name only decides whether the Adreno Metal plugin may open
 * its own driver. */
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOUserClient.h>
#include <libkern/c++/OSData.h>
#include <string.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <kern/task.h>
#include <sys/proc.h>   // proc_selfpid for the client's pid
#include "NativeAdrenoUserClient.h"
#include "NativeAdrenoPublish.h"
#include "NativeAdrenoMeter.h"
#include "Adreno830Status.h"
#include "A830Fence.h"
#include "A830PowerState.h"
#include "A830Recovery.h"

extern "C" void proc_selfname(char *, int);
extern "C" int snprintf(char *,size_t,const char *,...) __attribute__((format(printf,3,4)));
/* The plugin's telemetry table (rows of text the Metal bridge keeps), sent as markers: 0xC000|row starts a row, 0x8000|(a<<7)|b appends two characters. */
extern "C" {char native_adreno_metal_rows[48][96];volatile uint32_t native_adreno_metal_rows_seq;}

extern "C" {
/* What the processes that reached this driver did, for the boot diagnostic page. Plain counters and short strings. */
struct NativeAdrenoMetalDiag {
    uint32_t opens, busy, bad, closes, reports, property_reads, foreign_opens, foreign_type, name_reads, calls, last_selector, last_status, ext_entries, ext_selector, ext_status, ext_in, ext_out, tunnel_calls, probes, sel_bits[8], mb_maps, mb_calls, marks_total, marks[16], chk_macf, chk_res, chk_type, type_pass_lo, type_pass_hi, attach_status, inflight, inflight_since_calls, forced_unmaps;
    char opener[3][17];      /* process names of the latest connections, newest first */
    char reader[3][17];      /* process names that read a registry property of the accelerator, newest first */
    char name_reader[17];    /* latest process that read MetalPluginName specifically */
    char starter[3][17];     /* processes that sent plugin marker 1 (the plugin's initWithAcceleratorPort: started), newest first */
    char report[64];         /* the plugin's own result line (SetMetadata text), latest */
    char report2[64];        /* the plugin's Turnip-phase line (text starting "T " or "W "), latest */
    uint32_t grants;         /* opens granted to the render server by this driver in place of the sandbox verdict */
} native_adreno_metal_diag;
/* BO CPU views (clientMemoryForType for a BO token): [0] calls, [1] new descriptors, [2] served from the view cache, [3] refused (not a BO / out of
 * resources), [4] last refusal's IOReturn, [5] threads inside now. int-115 showed a backboardd thread in IOConnectMapMemory64 ~650 times a second. */
volatile uint32_t native_adreno_view_diag[6],native_adreno_view_kept;   /* kept: BoCpuUnmap calls that left the view for another lease */
/* The display hand-off as this driver forwards it: [0] WAIT_SWAP/WAIT_SWAP2 that returned a frame (offers), [1] COMPLETE_SWAP done,
 * [2] calls that failed (not counting a wait that timed out), [3] BAND calls, [4] the last failure's IOReturn. */
volatile uint32_t native_adreno_ngp_stats[5];
/* Fence waits as the callers see them: [0] waits, [1] us waited in total, [2] lock-free sleeps, [3] the longest wait in us,
 * [4] waits that ended as kIOReturnTimeout, [5] waits refused for a lost device. */
volatile uint64_t native_adreno_wait_outer[6];
/* Submissions: [0] retries in the kernel (ring full, caller idle), [1] Busy answers, [2] the sampled submission time (us, every 256th),
 * [3] the longest sampled, [4] samples, [5] refusals for a lost device. */
volatile uint64_t native_adreno_submit_outer[6];
/* Faults the shell delivered: [0] page faults attributed to a connection, [1] page faults no connection owns (kernel range, free or
 * no window), [2] global losses, [3] recoveries begun, [4] recoveries ended well, [5] the last unattributed FAR (low 32 bits),
 * [6] the last attributed slot, [7] the A830Reason of the loss. */
volatile uint32_t native_adreno_fault_stats[8];
}

/* The display hand-off (boot-arg sheng-fb-gpu=1, PD NativeGOPPresenter) reaches the plugin through this already admitted connection:
 * the original sandbox profiles admit this class but not the presenter's own service. Methods 0x7000 + the presenter's NGP_METHOD
 * numbers are forwarded unchanged, without any GPU lock (a wait sleeps); memory type NativeAdrenoScanoutType is the writable scanout.
 * Nothing here touches the GPU. Weak defaults for boards without the presenter (the statically linked kernel refuses undefined weak
 * references, so they are definitions). */
extern "C" __attribute__((weak)) IOReturn native_gop_handoff_call(uint32_t,task_t,const void *,size_t,void *,size_t)
{
    return kIOReturnUnsupported;
}
extern "C" __attribute__((weak)) IOMemoryDescriptor *native_gop_handoff_memory(void)
{
    return nullptr;
}
/* The connection through which task called the hand-off methods closed (the task died or closed it): the presenter treats it as its own
 * user client's stop, abandoning a frame it delivered to that task and dropping its mappings in that task. Without this, a render server
 * that died between WAIT_SWAP and COMPLETE_SWAP left the frame delivered for good: the presenter refused every later offer (the display
 * copied each frame with the CPU) and kept the dead task and its mappings referenced. A weak no-op until the presenter defines it. */
extern "C" __attribute__((weak)) void native_gop_handoff_closed(task_t)
{
}
/* Export (int-147): method NativeAdrenoExportMethod hands one in-order piece of a blob to the board's UFS log, which writes the newest
 * complete blob to its export area. The render server sends its pipeline caches this way. Input: {generation, total, offset, bytes}
 * then the bytes. No GPU state is touched; any connection of the render server may send it. */
extern "C" __attribute__((weak)) int boot_log_export_put(uint32_t,uint32_t,uint32_t,const void *,uint32_t)
{
    return -1;
}
/* DCVS (A830Dcvs.cpp, ARCHITECTURE 4.16): one call per accepted submission, lock-free (the wake boost and the idle detection). A weak
 * empty default keeps the shell linkable without the governor; A830Dcvs.cpp's definition replaces it. */
extern "C" __attribute__((weak)) void adreno830_dcvs_submit(void) {}
enum : uint32_t {NativeAdrenoTunnelMagic=0x4e544e4c /* 'NTNL' */,NativeAdrenoExportMethod=0x7100,NativeAdrenoExportMaxPiece=3968,
                 NativeAdrenoMarkerDoorbell=0x7db0};
extern "C" {volatile uint32_t native_adreno_export_stats[4];}   /* [0] pieces taken, [1] refused, [2] last generation, [3] its total bytes */
/* Hand-off method numbers of NativeGOPWire.h (NGP_METHOD_*), as offsets from NativeAdrenoHandoffBase. */
enum : uint32_t {HandoffWaitSwap=2,HandoffCompleteSwap=3,HandoffBand=4,HandoffWaitSwap2=5};

/* Mailbox: a last-resort command channel for a sandbox that admits no external-method selector at all. It is one shared page (memory
 * type NativeAdrenoMailboxType through IOConnectMapMemory64, which the selector filter does not cover) served by a kernel thread for the
 * render server's connection. The thread sleeps until that connection exists AND has mapped the page (event driven: no resident
 * 1 kHz poll, ARCHITECTURE 5.4), then polls the page every millisecond while it is used and every 16 ms after a second without a
 * request; the open marker 0x4147_7DB0 rings it at once. Layout (little endian dwords): +0 magic, +4 request seq, +8 response seq,
 * +12 method, +16 status, +20 input bytes, +24 output bytes, input at +0x40 (max 0x700), output at +0x800 (max 0x700). */
enum : uint32_t {NativeAdrenoMailboxType=0xC0DE,NativeAdrenoMailboxMagic=0x4d41494c /* 'MAIL' */};

class AGXDeviceUserClient;
#include "NativeGpuTrace.inc"
namespace {
/* A connection's core and its counters, on the heap: when a teardown cannot finish, they outlive the connection object as an orphan
 * that holds its slot until a later attempt releases everything (nothing is forced). */
struct CoreBox {
    NativeAdrenoUserClient core;
    NativeAdrenoStatus status;
    char name[20];   /* the process that opened it (kept for an orphan: the MEM row names who holds the memory) */
    uint32_t quiet_s=0;   /* seconds of native_adreno_client_tick without a call (under g.lock): the slot refusal's table */
};
struct Slot {
    AGXDeviceUserClient *connection;   /* live connection */
    CoreBox *orphan;                   /* a closed connection's core that still holds objects */
};
struct Published {
    bool published=false;
    NativeAdrenoVm *vm=nullptr;
    NativeAdrenoSubmit *submit=nullptr;
    NativeAdrenoRing *ring=nullptr;
    NativeAdrenoRingClock clock{};
    const NativeAdrenoTimer *timer=nullptr;
    const NativeAdrenoAbiDeviceInfo *info=nullptr;
    const NativeAdrenoClientWindow *windows=nullptr;
    IOLock *lock=nullptr;          /* the slot registry, the mailbox owner and the recovery event */
    IOLock *ring_lock=nullptr;     /* NativeAdrenoSubmitLockRing */
    IOLock *memory_lock=nullptr;   /* NativeAdrenoSubmitLockMemory */
    IOService *service=nullptr;
    Slot slots[NativeAdrenoMaxClients]{};
    AGXDeviceUserClient *kernel_client=nullptr;
    CoreBox *kernel_orphan=nullptr;
    uint32_t input_bits=0,output_bits=0;
    uint32_t opened=0,closed=0;
    uint32_t lost=0;          /* the A830Reason of a permanent device loss; 0 while serving */
    uint32_t recovering=0;    /* between native_adreno_recovery_begin and _end */
    NativeAdrenoSlotLog slot_log{};   /* the slot refusal line's rate limit */
    char slot_line[NativeAdrenoSlotLineBytes]{};   /* that line: too long for a stack; under g.lock */
    bool past_4g_noted=false;  /* notePast4G wrote its line */
} g;
IOBufferMemoryDescriptor *gMailbox=nullptr;
volatile uint32_t *gMailboxWords=nullptr;
bool gPollerStarted=false,gMailboxMapped=false;
uint32_t gMailboxDoorbell;   /* event only */
/* The render server's connection serves the mailbox; it is retained while set (under g.lock), so the poller never runs on a freed object. */
AGXDeviceUserClient *gMailboxOwner=nullptr;

uint64_t nowUs()
{
    uint64_t nanoseconds=0;
    absolutetime_to_nanoseconds(mach_absolute_time(),&nanoseconds);
    return nanoseconds/1000;
}
bool lost() {return __atomic_load_n(&g.lost,__ATOMIC_ACQUIRE)!=0;}
bool recovering() {return __atomic_load_n(&g.recovering,__ATOMIC_ACQUIRE)!=0;}
/* IOLockLock for the locks the calls queue on: the call meter sees who waited for which (NativeAdrenoMeter.h). */
void lockMetered(IOLock *lock,unsigned which)
{
    NativeAdrenoMeterLock(which,nowUs,[&] {return IOLockTryLock(lock)!=0;},[&] {IOLockLock(lock);});
}
void lockHook(void *,NativeAdrenoSubmitLock which)
{
    if(which==NativeAdrenoSubmitLockRing)lockMetered(g.ring_lock,Adreno830WaitRing);
    else lockMetered(g.memory_lock,Adreno830WaitMemory);
}
void unlockHook(void *,NativeAdrenoSubmitLock which)
{
    IOLockUnlock(which==NativeAdrenoSubmitLockRing?g.ring_lock:g.memory_lock);
}
void shiftName(char names[3][17]) {
    memcpy(names[2], names[1], 17);
    memcpy(names[1], names[0], 17);
    proc_selfname(names[0], 17);
}
/* The box of slot s (a live connection's or an orphan's), under g.lock. */
CoreBox *slotBox(unsigned s);
}
void nativeAdrenoMailboxPoller(void *,wait_result_t);
static IOReturn runCore(AGXDeviceUserClient *,uint32_t method,const void *input,size_t input_size,void *output,size_t output_size);

/* The two sandbox gates IOServiceOpen applies after newUserClient (XNU IOUserClient.cpp is_io_service_open_extended): the MACF open check on
 * the client object and the sandbox's filter resolver. They are run here for the caller too, only to record which one refuses it. */
extern "C" int mac_iokit_check_open(void *cred,void *client,unsigned type);
extern "C" void *kauth_cred_get(void);
extern const struct io_filter_callbacks *gIOUCFilterCallbacks;

extern "C" {
/* One entry per IOServiceOpen of the accelerator (not markers): who, which type, what the two sandbox gates said. */
struct NativeAdrenoOpenRecord {char name[20]; uint32_t type, macf, res, attach, foreign;};
struct NativeAdrenoOpenRecord native_adreno_open_log[8];
volatile uint32_t native_adreno_open_log_count;
/* One row per process name that opened the plugin type (NativeAdrenoClientRecord, NativeAdrenoPublish.h: the listed clients, the poster
 * extensions among them, the apps, and adrenoprobed): opens granted, refused (sandbox, cap, no free slot, hangs or a lost device),
 * closes, the slot of its live connection (0xff none), the highest GPU address its mappings reached (MiB), its last refusal, and the
 * GPU hangs its jobs caused (NativeAdrenoClientHangLimit). 64 rows since every app may be a client (2026-10-30); once all are used, a
 * new name takes the row of one with no live connection and no hang (NativeAdrenoClientRecordFor), so hangs are still counted. */
enum : unsigned {NativeAdrenoClientRecords=64};
struct NativeAdrenoClientRecord native_adreno_client_log[NativeAdrenoClientRecords];
/* Per row of native_adreno_client_log: the opens counted against sheng-gpu-metal-opens, since the name's last healthy connection (one
 * that lived NativeAdrenoHealthyUs). The cap is there for a crash loop (a plugin that kills its host on every launch); counted over the
 * whole boot it also locked out a healthy process for good once it had restarted often enough (Turnip opens twice per process: twenty
 * render-server or WidgetRenderer launches), and the process then rendered on the CPU. */
volatile uint32_t native_adreno_client_streak[NativeAdrenoClientRecords];
/* The last text line (the plugin's failure reason) each process other than the render server streamed: process name and a 64-byte line,
 * first thirty-two names. */
struct NativeAdrenoClientText {char name[20]; char text[64];};
struct NativeAdrenoClientText native_adreno_client_text[32];
/* Why the latest mount of a connection failed: [0] 1 no free slot, 2 the core's open, 3 the window scope, 4 no memory; [1] its IOReturn;
 * [2] the slot; [3] failures so far. */
volatile uint32_t native_adreno_mount_fail[4];
/* Processes that read MetalPluginName (the Metal discovery), first sixty-four names, with counts. */
struct NativeAdrenoAskerRecord {char name[20]; uint32_t reads;};
struct NativeAdrenoAskerRecord native_adreno_metal_askers[64];
/* GPU hangs attributed at a recovery's start (NativeAdrenoSubmit::halt's oldest range): [0] to a client other than the render server,
 * [1] to the render server or the kernel client, [2] with nothing in flight, [3] the last culprit's slot, [4] recoveries whose cause
 * charges nobody (a GMU, HFI, fenced-write, slumber, wake or self-test fault: native_adreno_recovery_begin(false)). */
volatile uint32_t native_adreno_hang_culprits[5];
/* Slot pressure (M14 P0, the input of the per-process page-table decision): [0] the most GPU connections live at once, [1] opens of a
 * process refused because no slot was free for it (native_adreno_mount_fail why 1). The ramoops meter line shows them. */
volatile uint32_t native_adreno_slot_pressure[2];
/* CB0 page faults by how the jobs in flight compared with the FAR window (NativeAdrenoFaultVerdictFor; the window's connection is the
 * one charged in every case): [0] alike, [1] apart (one other connection alone in flight, or FAR in no window), [2] undecided (nothing
 * or several connections in flight, or no history), [3] found by the poll (no fences at the fault). */
volatile uint32_t native_adreno_fault_attribution[NativeAdrenoFaultVerdicts];
}
/* Every connection slot is a submit range of its own: its queues are [slot * NativeAdrenoAbiMaxQueues, + NativeAdrenoAbiMaxQueues) with
 * one reserved job slot on the ring (ARCHITECTURE 5.4), and recovery reports the ranges with work in flight as a 32-bit mask. */
static_assert(unsigned(NativeAdrenoMaxClients)<=unsigned(NativeAdrenoSubmitRanges) && NativeAdrenoSubmitRanges<=32 &&
              uint32_t(NativeAdrenoSubmitRangeQueues)==uint32_t(NativeAdrenoAbiMaxQueues),"one submit range per connection slot");

class NativeAdrenoGpuPublished : public IOService {
    OSDeclareDefaultStructors(NativeAdrenoGpuPublished)
public:
    IOReturn newUserClient(task_t owningTask,void *securityID,UInt32 type,
        OSDictionary *properties,IOUserClient **handler) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(NativeAdrenoGpuPublished,IOService)

class AGXDeviceUserClient : public IOUserClient {
    OSDeclareDefaultStructors(AGXDeviceUserClient)
    bool live_=false;
    bool handoffOnly_=false;
    bool kernel_=false;
    bool closing_=false;
    CoreBox *box_=nullptr;   /* own core and counters; deleted in free(), so a call that raced the close sees live_ false, never freed memory */
    IOLock *lock_=nullptr;   /* this connection's own lock (M10): the core, its views and live_ */
    unsigned slot_=NativeAdrenoMaxClients;
    char name_[20]{};
    uint64_t opened_us_=0;   /* when mount() succeeded: a connection that lived NativeAdrenoHealthyUs ends its name's open streak */
    task_t handoffTask_=nullptr;   /* referenced: the task that called the hand-off methods through this connection (first call) */
    /* Calls by class since the boot (relaxed, every call), and what native_adreno_client_tick took of them. In the object, not the box:
     * a call that raced the close may still count here, and a box can go away with its orphan. */
    uint32_t classCalls_[Adreno830CallClasses]{},classSeen_[Adreno830CallClasses]{};
    void noteCall(uint32_t method) {__atomic_fetch_add(&classCalls_[NativeAdrenoCallClassOf(method)],1u,__ATOMIC_RELAXED);}
    void noteHandoffTask();
    void handoffClosed();
    /* One user-view descriptor per BO token, reused by every clientMemoryForType call. IOConnectUnmapMemory64 asks clientMemoryForType for
     * the descriptor again and unmaps only a mapping that belongs to THAT descriptor (kIOMapReference); a fresh descriptor per call made the
     * unmap a silent no-op, the user PTEs outlived BoCpuUnmap, and BoFree handed pages with a live user mapping to the VM:
     * pmap_assert_free panicked the freeing core (the "BoFree hang"). The cache holds every BO the client can have. */
    struct View {uint32_t token;IOMemoryDescriptor *descriptor;};
    /* A token is its BO's entry index plus one, so it names its own slot: one view per object the core can hold (its capacity). */
    size_t max_views_=0;
    View *viewSlot(uint32_t token) {return views_ && token && token<=max_views_?&views_[token-1]:nullptr;}
    View *views_=nullptr;   /* on the heap: the client object must stay within the kernel allocator's fixed-size classes */
    void dropViews(uint32_t token);
    static IOReturn action(OSObject *target,void *reference,IOExternalMethodArguments *arguments);
    IOReturn run(uint32_t method,const void *input,size_t input_size,void *output,size_t output_size);
    IOReturn locked(uint32_t method,const void *input,size_t input_size,void *output,size_t output_size);
    IOReturn waitFence(const void *input,size_t input_size,void *output,size_t output_size);
    IOReturn submit(const void *input,size_t input_size,void *output,size_t output_size);
    IOReturn importBo(const void *input,size_t input_size,void *output,size_t output_size);
public:
    void setHandoffOnly() {handoffOnly_=true;}
    IOReturn mount(bool kernel);
    CoreBox *box() const {return box_;}
    /* This second's calls by class into classes, and their sum; the tick's only, under g.lock. */
    uint32_t takeCalls(uint32_t classes[Adreno830CallClasses])
    {
        uint32_t sum=0;
        for(unsigned i=0;i<Adreno830CallClasses;++i) {
            const uint32_t now=__atomic_load_n(&classCalls_[i],__ATOMIC_RELAXED);
            classes[i]=now-classSeen_[i];classSeen_[i]=now;sum+=classes[i];
        }
        return sum;
    }
    friend IOReturn runCore(AGXDeviceUserClient *,uint32_t,const void *,size_t,void *,size_t);
    friend void nativeAdrenoMailboxPoller(void *,wait_result_t);
    IOReturn clientClose() APPLE_KEXT_OVERRIDE;
    void free() APPLE_KEXT_OVERRIDE;
    IOReturn clientMemoryForType(UInt32 type,IOOptionBits *options,IOMemoryDescriptor **memory) APPLE_KEXT_OVERRIDE;
    IOReturn externalMethod(uint32_t selector,IOExternalMethodArguments *arguments,
        IOExternalMethodDispatch *dispatch,OSObject *target,void *reference) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(AGXDeviceUserClient,IOUserClient)

namespace {
CoreBox *slotBox(unsigned s)
{
    if(s>=NativeAdrenoMaxClients)return nullptr;
    if(g.slots[s].connection)return g.slots[s].connection->box();
    return g.slots[s].orphan;
}
bool slotFree(unsigned s) {return !g.slots[s].connection && !g.slots[s].orphan;}
bool anyOrphan()
{
    for(unsigned s=0;s<NativeAdrenoMaxClients;++s)if(g.slots[s].orphan)return true;
    return g.kernel_orphan!=nullptr;
}
/* Retries the teardown of orphaned cores (connections closed while their jobs or objects could not be released); under g.lock. A
 * core whose objects are all released frees its slot. Bounded work: the waits are zero-timeout checks. Run by every mount (not only one
 * that finds no free slot: the render server's own orphan in slot 0 sent its next connection to a 682 MiB window of the old slots 1-3
 * and kept the dead connection's BOs and imports for good), at the end of a recovery, and once a second by native_adreno_client_tick. */
void reapOrphans()
{
    CoreBox **orphans[NativeAdrenoMaxClients+1];
    unsigned count=0;
    for(unsigned s=0;s<NativeAdrenoMaxClients;++s)if(g.slots[s].orphan)orphans[count++]=&g.slots[s].orphan;
    if(g.kernel_orphan)orphans[count++]=&g.kernel_orphan;
    for(unsigned i=0;i<count;++i) {
        CoreBox *box=*orphans[i];
        if(box->core.releaseAll(&g.clock,0)!=kIOReturnSuccess)continue;
        *orphans[i]=nullptr;
        delete box;
        ++g.closed;
    }
}
}

void nativeAdrenoMailboxPoller(void *,wait_result_t)
{
    thread_set_thread_name(current_thread(),"adreno830-mailbox");
    uint32_t last=0;
    uint64_t active_until=0;
    for(;;) {
        IOLockLock(g.lock);
        /* Nothing to serve: sleep until the render server's connection maps the page (or the doorbell rings). */
        while(!gMailboxOwner || !gMailboxMapped || !gMailboxWords)IOLockSleep(g.lock,&gMailboxDoorbell,THREAD_UNINT);
        const uint64_t now=nowUs();
        AbsoluteTime deadline=0;
        clock_interval_to_deadline(now<active_until?1:16,kMillisecondScale,&deadline);
        IOLockSleepDeadline(g.lock,&gMailboxDoorbell,deadline,THREAD_UNINT);
        AGXDeviceUserClient *owner=gMailboxOwner;
        if(owner)owner->retain();
        IOLockUnlock(g.lock);
        if(!owner)continue;
        const uint32_t request=gMailboxWords[1];
        if(request!=last && request!=gMailboxWords[2]) {
            last=request;
            active_until=nowUs()+1000000;
            const uint32_t method=gMailboxWords[3],in_size=gMailboxWords[5],out_size=gMailboxWords[6];
            IOReturn status=kIOReturnBadArgument;
            if(in_size<=0x700 && out_size<=0x700) {
                uint8_t *buffer=static_cast<uint8_t *>(IOMallocData(0xE00));
                if(!buffer)status=kIOReturnNoMemory;
                else {
                    uint8_t *input=buffer,*output=buffer+0x700;
                    memcpy(input,const_cast<uint32_t *>(gMailboxWords)+0x10,in_size);
                    status=runCore(owner,method,input,in_size,output,out_size);
                    if(!status)memcpy(const_cast<uint32_t *>(gMailboxWords)+0x200,output,out_size);
                    IOFreeData(buffer,0xE00);
                }
            }
            native_adreno_metal_diag.mb_calls=native_adreno_metal_diag.mb_calls+1;
            gMailboxWords[4]=uint32_t(status);
            __asm__ volatile("dmb sy" ::: "memory");
            gMailboxWords[2]=request;
            __asm__ volatile("dmb sy" ::: "memory");
        }
        owner->release();
    }
}

/* The client log row of name (NativeAdrenoClientRecordFor: with every row used, the row of a name that holds no slot and never hung the
 * GPU is given to it, and that row's open streak restarts); null only when every row is live or charged with hangs. Under g.lock, which
 * also keeps the slots the liveness check reads. */
static NativeAdrenoClientRecord *clientRecord(const char *name)
{
    auto live=[](const char *holder) {
        for(unsigned s=0;s<NativeAdrenoMaxClients;++s)
            if(const CoreBox *box=slotBox(s))if(!strncmp(box->name,holder,sizeof box->name))return true;
        return false;
    };
    bool fresh=false;
    NativeAdrenoClientRecord *record=NativeAdrenoClientRecordFor(native_adreno_client_log,NativeAdrenoClientRecords,name,live,&fresh);
    if(record && fresh)native_adreno_client_streak[record-native_adreno_client_log]=0;
    return record;
}
/* The slot refusal line (NativeAdrenoSlotLine; adreno830_status_diag_lines, as the per-second line). Under g.lock. */
static void noteSlotRefusal(const char *name,bool large,bool listed,const bool taken[NativeAdrenoMaxClients])
{
    if(!adreno830_status_diag_lines() || !NativeAdrenoSlotLogDue(&g.slot_log,nowUs()))return;
    NativeAdrenoSlotHolder holders[NativeAdrenoMaxClients]={};
    for(unsigned s=0;s<NativeAdrenoMaxClients;++s)
        if(const CoreBox *box=slotBox(s))holders[s]={box->name,box->quiet_s,g.slots[s].orphan!=nullptr};
    NativeAdrenoSlotLine(g.slot_line,name,large?'L':listed?'l':'a',NativeAdrenoClientSmallFree(g.windows,taken),
        NativeAdrenoListedSmallKeep(g.windows),g.slot_log.skipped,holders);
    g.slot_log.skipped=0;
    boot_progress_trace(g.slot_line);
}
/* The boot's first mapping past 4 GiB (NativeAdrenoPast4GLine), on the same condition. Under g.lock, once a boot. */
static void notePast4G(const char *name,unsigned slot,uint64_t iova,uint64_t end)
{
    if(g.past_4g_noted || !NativeAdrenoPast4G(end) || !adreno830_status_diag_lines())return;
    g.past_4g_noted=true;
    NativeAdrenoPast4GLine(g.slot_line,name,slot,iova,end);
    boot_progress_trace(g.slot_line);
}
enum : uint64_t {NativeAdrenoHealthyUs=60000000};   /* a connection this old was no crash loop */
static volatile uint32_t &openStreak(const NativeAdrenoClientRecord *record)
{
    return native_adreno_client_streak[record-native_adreno_client_log];
}
static void noteMountFailure(uint32_t why,IOReturn status,unsigned slot)
{
    native_adreno_mount_fail[0]=why;native_adreno_mount_fail[1]=uint32_t(status);native_adreno_mount_fail[2]=slot;
    native_adreno_mount_fail[3]=native_adreno_mount_fail[3]+1;
}
IOReturn AGXDeviceUserClient::mount(bool kernel)
{
    proc_selfname(name_,sizeof name_);
    CoreBox *box=new CoreBox;
    IOLock *lock=IOLockAlloc();
    if(!box || !lock) {
        if(box)delete box;
        if(lock)IOLockFree(lock);
        noteMountFailure(4,kIOReturnNoMemory,NativeAdrenoMaxClients);
        return kIOReturnNoMemory;
    }
    strlcpy(box->name,name_,sizeof box->name);
    const bool render_server=!kernel && !strncmp(name_,"backboardd",sizeof name_);
    const bool large=!kernel && NativeAdrenoListNamesProcess(NativeAdrenoLargeClientList,name_,sizeof name_);
    const bool listed=!kernel && NativeAdrenoListNamesProcess(NativeAdrenoSandboxGrantList,name_,sizeof name_);
    IOLockLock(g.lock);
    /* An orphan whose jobs completed goes first: its slot (the render server's own slot 0 after a crash) and its memory. */
    if(anyOrphan())reapOrphans();
    /* Slot 0 (the render server's window) is the render server's alone; the large class (NativeAdrenoLargeClientList and a second
     * render-server connection) takes a free large slot, else a small one; a listed process (the grant list: the poster extensions) a
     * free small slot, else a free large one; any other process a small slot while enough are left for the listed ones
     * (NativeAdrenoClientSlotFor). The kernel's own client (a board self-test) has no window and the default queue range. */
    unsigned slot=NativeAdrenoMaxClients;
    bool taken[NativeAdrenoMaxClients]={};
    for(int attempt=0;attempt<2 && slot==NativeAdrenoMaxClients;++attempt) {
        if(kernel)slot=g.kernel_client || g.kernel_orphan?NativeAdrenoMaxClients:0;
        else {
            for(unsigned i=0;i<NativeAdrenoMaxClients;++i)taken[i]=!slotFree(i);
            slot=NativeAdrenoClientSlotFor(render_server,large,listed,g.windows,taken);
        }
        if(slot==NativeAdrenoMaxClients && !attempt)reapOrphans();   /* a slot an orphan held may be free now */
    }
    /* The render server's connections (in whatever slot) and the kernel's own client hold up to 16384 objects; every other connection
     * fewer and it leaves the render server its reserve of the shared VM tables and of the BO pool (NativeAdrenoClientTermsFor). */
    const NativeAdrenoClientTerms terms=NativeAdrenoClientTermsFor(kernel,render_server,large,slot);
    IOReturn status=slot<NativeAdrenoMaxClients?box->core.open(NativeAdrenoAbiVersion,g.vm,g.submit,g.ring,g.input_bits,g.output_bits,
        terms.objects):kIOReturnBusy;
    if(status)noteMountFailure(slot<NativeAdrenoMaxClients?2:1,status,slot);
    if(slot>=NativeAdrenoMaxClients && !kernel) {
        native_adreno_slot_pressure[1]=native_adreno_slot_pressure[1]+1;
        noteSlotRefusal(name_,large,listed,taken);
    }
    if(!status) {
        if(g.timer)box->core.attachTimer(g.timer);
        if(g.info)box->core.attachDeviceInfo(g.info);
        box->core.attachStatus(&box->status);   /* counted per connection from here on (ARCHITECTURE 4.13.4) */
        /* The queue meter's attribution (ARCHITECTURE 4.13.5): the render server's jobs, in whatever slot it got, against everyone's. */
        box->core.setRenderServer(render_server);
        if(terms.reserve)
            (void)box->core.setReserve(NativeAdrenoVmRenderReserveObjects,NativeAdrenoVmRenderReserveSpans,
                NativeAdrenoMemory::poolBytes()/NativeAdrenoPoolRenderShare);
        status=kernel?box->core.setScope(0,0,0):
            box->core.setScope(slot*NativeAdrenoAbiMaxQueues,g.windows[slot].start,g.windows[slot].bytes);
        if(status) {
            noteMountFailure(3,status,slot);
            box->core.releaseAll(&g.clock,0);
        }
    }
    if(!status) {
        max_views_=box->core.capacity();
        box_=box;box=nullptr;lock_=lock;lock=nullptr;slot_=slot;kernel_=kernel;live_=true;opened_us_=nowUs();
        if(kernel)g.kernel_client=this;
        else g.slots[slot].connection=this;
        ++g.opened;
        unsigned live=0;
        for(unsigned s=0;s<NativeAdrenoMaxClients;++s)if(g.slots[s].connection)++live;
        if(live>native_adreno_slot_pressure[0])native_adreno_slot_pressure[0]=live;
        if(NativeAdrenoClientRecord *record=kernel?nullptr:clientRecord(name_))record->slot=slot;
    }
    /* The mailbox page and its server thread belong to the render server's connection in slot 0 only. */
    if(!status && terms.mailbox) {
        if(!gMailbox) {
            gMailbox=IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,kIODirectionInOut,4096,4096);
            if(gMailbox)gMailboxWords=static_cast<volatile uint32_t *>(gMailbox->getBytesNoCopy());
        }
        if(gMailboxWords) {
            memset(const_cast<uint32_t *>(gMailboxWords),0,4096);
            gMailboxWords[0]=NativeAdrenoMailboxMagic;
            gMailboxMapped=false;
            if(!gPollerStarted) {
                thread_t thread=nullptr;
                if(kernel_thread_start(nativeAdrenoMailboxPoller,nullptr,&thread)==KERN_SUCCESS) {thread_deallocate(thread);gPollerStarted=true;}
            }
        }
        if(gMailboxOwner)gMailboxOwner->release();
        retain();gMailboxOwner=this;
    }
    IOLockUnlock(g.lock);
    if(box)delete box;
    if(lock)IOLockFree(lock);
    return status;
}
/* Removes the cached view(s) of token (0 = all) and tears down every user mapping of them, so the BO's pages are never freed while a user
 * PTE still points at them, whether or not the client unmapped. */
void AGXDeviceUserClient::dropViews(uint32_t token)
{
    for(;;) {
        IOMemoryDescriptor *view=nullptr;
        if(lock_)lockMetered(lock_,Adreno830WaitConnection);
        if(token) {View *v=viewSlot(token);if(v && v->descriptor && v->token==token) {view=v->descriptor;v->descriptor=nullptr;v->token=0;}}
        else if(views_)for(size_t i=0;i<max_views_;++i) {View &v=views_[i];if(v.descriptor) {view=v.descriptor;v.descriptor=nullptr;v.token=0;break;}}
        if(lock_)IOLockUnlock(lock_);
        if(!view)return;
        /* Each returned map comes retained (OSPtr is a plain pointer in this kernel) and that is the last reference outside the
         * descriptor's own list: releasing it frees the map, which unmaps the user range. */
        for(;;) {
            IOMemoryMap *map=removeMappingForDescriptor(view);
            if(!map)break;
            map->release();
            native_adreno_metal_diag.forced_unmaps=native_adreno_metal_diag.forced_unmaps+1;
        }
        view->release();
    }
}
IOReturn AGXDeviceUserClient::clientClose()
{
    dropViews(0);
    AGXDeviceUserClient *mailbox=nullptr;
    IOLockLock(g.lock);
    if(gMailboxOwner==this) {mailbox=gMailboxOwner;gMailboxOwner=nullptr;gMailboxMapped=false;}
    IOLockUnlock(g.lock);
    if(lock_) {
        IOLockLock(lock_);
        closing_=true;
        if(live_) {
            const uint64_t peak=box_->core.windowPeak();
            const IOReturn status=box_->core.releaseAll(&g.clock,2000000);
            live_=false;
            IOLockLock(g.lock);
            if(kernel_)g.kernel_client=nullptr;else if(slot_<NativeAdrenoMaxClients)g.slots[slot_].connection=nullptr;
            if(status) {
                /* A teardown that cannot finish (jobs not retired, an object refused) leaves the objects live: the core becomes an orphan
                 * that keeps its slot until a later attempt (a mount that finds no slot, the end of a recovery) releases everything. */
                if(kernel_)g.kernel_orphan=box_;else if(slot_<NativeAdrenoMaxClients)g.slots[slot_].orphan=box_;
                box_=nullptr;
            } else ++g.closed;
            native_adreno_metal_diag.closes=native_adreno_metal_diag.closes+1;
            if(NativeAdrenoClientRecord *record=kernel_?nullptr:clientRecord(name_)) {
                record->closes=record->closes+1;record->slot=0xff;
                if(peak>>20>record->peak_mib)record->peak_mib=uint32_t(peak>>20);
                if(nowUs()-opened_us_>=NativeAdrenoHealthyUs)openStreak(record)=0;
            }
            IOLockUnlock(g.lock);
        }
        IOLockUnlock(lock_);
    }
    /* Sleepers in WaitFence or the submit retry see closing_ and return. */
    if(mailbox)mailbox->release();
    handoffClosed();
    terminate();
    return kIOReturnSuccess;
}
void AGXDeviceUserClient::noteHandoffTask()
{
    if(__atomic_load_n(&handoffTask_,__ATOMIC_ACQUIRE))return;
    task_t task=current_task();
    task_reference(task);
    task_t expected=nullptr;
    if(!__atomic_compare_exchange_n(&handoffTask_,&expected,task,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE))task_deallocate(task);
}
void AGXDeviceUserClient::handoffClosed()
{
    task_t task=__atomic_exchange_n(&handoffTask_,static_cast<task_t>(nullptr),__ATOMIC_ACQ_REL);
    if(!task)return;
    native_gop_handoff_closed(task);
    task_deallocate(task);
}
void AGXDeviceUserClient::free()
{
    handoffClosed();   /* freed without a close (never expected) */
    dropViews(0);
    if(views_) {IODelete(views_,View,max_views_);views_=nullptr;}
    if(box_ && live_ && g.lock) {
        /* Freed without a close (never expected: clientDied closes): the slot must not keep a pointer to this object. */
        IOLockLock(g.lock);
        if(kernel_) {if(g.kernel_client==this) {g.kernel_client=nullptr;g.kernel_orphan=box_;box_=nullptr;}}
        else if(slot_<NativeAdrenoMaxClients && g.slots[slot_].connection==this) {
            g.slots[slot_].connection=nullptr;g.slots[slot_].orphan=box_;box_=nullptr;
        }
        live_=false;
        IOLockUnlock(g.lock);
    }
    if(box_ && !live_) {delete box_;box_=nullptr;}
    if(lock_) {IOLockFree(lock_);lock_=nullptr;}
    IOUserClient::free();
}
IOReturn AGXDeviceUserClient::clientMemoryForType(UInt32 type,IOOptionBits *options,IOMemoryDescriptor **memory)
{
    if(memory)*memory=nullptr;
    if(!memory || !options)return kIOReturnBadArgument;
    IOReturn status=kIOReturnNotFound;
    if(type==NativeAdrenoScanoutType) {
        if(!live_ && !handoffOnly_)return kIOReturnNotReady;
        IOMemoryDescriptor *scanout=native_gop_handoff_memory();
        if(!scanout)return kIOReturnNotReady;
        *memory=scanout;                        /* already retained for the caller */
        *options=kIOMapAnywhere|kIOMapInhibitCache;   /* firmware framebuffer: device memory, never cached by the CPU */
        return kIOReturnSuccess;
    }
    if(type==NativeAdrenoMailboxType) {
        IOLockLock(g.lock);
        if(live_ && gMailbox && gMailboxOwner==this) {
            gMailbox->retain();
            *memory=gMailbox;
            *options=kIOMapAnywhere;   /* ordinary cached memory: both sides are CPUs, so it is coherent */
            native_adreno_metal_diag.mb_maps=native_adreno_metal_diag.mb_maps+1;
            gMailboxMapped=true;
            IOLockWakeup(g.lock,&gMailboxDoorbell,false);
            status=kIOReturnSuccess;
        } else status=kIOReturnNotReady;
        IOLockUnlock(g.lock);
        return status;
    }
    if(!lock_)return kIOReturnNotReady;
    native_adreno_view_diag[0]=native_adreno_view_diag[0]+1;
    /* [5]: threads inside this function right now (a thread stuck here, on a lock or in a mapping, shows as a non-zero count that stays). */
    struct Inside {Inside() {__atomic_fetch_add(&native_adreno_view_diag[5],1,__ATOMIC_RELAXED);} ~Inside() {__atomic_fetch_sub(&native_adreno_view_diag[5],1,__ATOMIC_RELAXED);}} inside;
    /* The GPU maps own BOs non-cacheable; with CachedCoherent (pagani-gpu-coherent=2) they are snooped and the client maps them write-back
     * (the client's cache flags replace these in IOUserClient::mapClientMemory64 either way). */
    const IOOptionBits cache=g.info && (g.info->flags&NativeAdrenoDeviceCachedCoherent)?kIOMapDefaultCache:kIOMapInhibitCache;
    lockMetered(lock_,Adreno830WaitConnection);
    if(!views_ && max_views_)views_=IONewZero(View,max_views_);
    if(!views_) {IOLockUnlock(lock_);return kIOReturnNoMemory;}
    View *v=viewSlot(type);
    if(v && v->descriptor && v->token==type) {
        v->descriptor->retain();*memory=v->descriptor;*options=kIOMapAnywhere|cache;
        native_adreno_view_diag[2]=native_adreno_view_diag[2]+1;
        IOLockUnlock(lock_);
        return kIOReturnSuccess;
    }
    NativeAdrenoMemory *backing=live_?box_->core.memoryForToken(type):nullptr;
    if(backing) {
        /* The BO's pages are mapped non-cacheable by the GPU, so the client needs a view of the same raw physical spans the SMMU maps
         * with the matching cache mode (a mapping of the buffer descriptor with another cache mode is not reliable). A BO's pages can
         * come scattered, one span per page: the descriptor takes them all, as many as the GPU's own table accepts. The descriptor
         * does not retain the pages: the client must unmap before BoCpuUnmap/BoFree (dropViews forces it). */
        enum {MaxSpans=65536};
        const size_t count=backing->spanCount();
        if(!count || count>MaxSpans)status=kIOReturnNoResources;
        else {
            IOAddressRange *ranges=IONewData(IOAddressRange,count);
            if(!ranges)status=kIOReturnNoMemory;
            else {
                for(size_t i=0;i<count;++i)ranges[i]={backing->spans()[i].physical,backing->spans()[i].bytes};
                IOMemoryDescriptor *descriptor=IOMemoryDescriptor::withAddressRanges(ranges,(UInt32)count,
                    static_cast<IOOptionBits>(kIODirectionInOut)|static_cast<IOOptionBits>(kIOMemoryTypePhysical64),nullptr);
                IODeleteData(ranges,IOAddressRange,count);
                if(!descriptor)status=kIOReturnNoMemory;
                else if(!v || v->descriptor) {
                    /* Never hand out a view that cannot be torn down again. */
                    descriptor->release();
                    status=kIOReturnNoResources;
                } else {
                    v->token=type;v->descriptor=descriptor;descriptor->retain();
                    *memory=descriptor;
                    *options=kIOMapAnywhere|cache;
                    native_adreno_view_diag[1]=native_adreno_view_diag[1]+1;
                    status=kIOReturnSuccess;
                }
            }
        }
    }
    IOLockUnlock(lock_);
    if(status){native_adreno_view_diag[3]=native_adreno_view_diag[3]+1;native_adreno_view_diag[4]=(uint32_t)status;}
    return status;
}
/* One core call under this connection's lock. */
IOReturn AGXDeviceUserClient::locked(uint32_t method,const void *input,size_t input_size,void *output,size_t output_size)
{
    if(!lock_)return kIOReturnNotReady;
    lockMetered(lock_,Adreno830WaitConnection);
    const IOReturn status=live_?box_->core.dispatch(method,input,input_size,output,output_size,&g.clock):kIOReturnNotReady;
    IOLockUnlock(lock_);
    return status;
}
/* WaitFence (ARCHITECTURE 4.13.3): one zero-timeout core call under the connection lock validates the fence and reaps it when it already
 * passed; otherwise the thread sleeps in the ring's completion wait (A830Fence::wait through the ring's waiter, or the clock's idle
 * polling without one) holding no lock and doing no MMIO, and reaps with another zero-timeout call when it wakes. Not yet complete,
 * the caller's deadline, and a sleep interrupted by a signal all answer kIOReturnTimeout, which Turnip retries; a lost device answers
 * kIOReturnNotResponding, a closing connection kIOReturnAborted; every other error is the core's. */
IOReturn AGXDeviceUserClient::waitFence(const void *input,size_t input_size,void *output,size_t output_size)
{
    if(!input || input_size!=sizeof(NativeAdrenoAbiWaitFence) || output_size)
        return locked(NativeAdrenoMethodWaitFence,input,input_size,output,output_size);   /* the core refuses the sizes */
    NativeAdrenoAbiWaitFence request{};
    memcpy(&request,input,sizeof(request));
    NativeAdrenoAbiWaitFence poll{request.fence,0};
    const uint64_t began=nowUs();
    native_adreno_wait_outer[0]=native_adreno_wait_outer[0]+1;
    adreno830_status_add64(&adreno830_status.waits,1);
    IOReturn status=locked(NativeAdrenoMethodWaitFence,&poll,sizeof(poll),nullptr,0);
    const uint64_t limit_us=request.timeout_ns?(request.timeout_ns+999)/1000:0;
    while(status==kIOReturnTimeout) {
        if(lost()) {native_adreno_wait_outer[5]=native_adreno_wait_outer[5]+1;status=NativeAdrenoReturnNotResponding;break;}
        if(closing_) {status=NativeAdrenoReturnAborted;break;}
        const uint64_t waited=nowUs()-began;
        if(waited>=limit_us)break;
        const uint64_t left=limit_us-waited;
        uint32_t observed=0;
        native_adreno_wait_outer[2]=native_adreno_wait_outer[2]+1;
        const NativeAdrenoRingFence slept=g.ring->observeFence((uint32_t)request.fence,&g.clock,left>UINT32_MAX?UINT32_MAX:(uint32_t)left,
            &observed);
        status=locked(NativeAdrenoMethodWaitFence,&poll,sizeof(poll),nullptr,0);
        if(status==kIOReturnNotReady && closing_) {status=NativeAdrenoReturnAborted;break;}   /* closed while asleep */
        if(status!=kIOReturnTimeout)break;
        if(slept==NativeAdrenoRingFenceLost) {status=NativeAdrenoReturnNotResponding;break;}
        /* Not yet complete after a sleep that ended: the deadline passed or a signal interrupted it (THREAD_ABORTSAFE): Timeout. */
        if(slept!=NativeAdrenoRingFenceComplete)break;
    }
    const uint64_t waited=nowUs()-began;
    if(NativeAdrenoKoptOn(NativeAdrenoKoptMeter))NativeAdrenoMeterRaise(&native_adreno_meter_fence,waited);
    native_adreno_wait_outer[1]=native_adreno_wait_outer[1]+waited;
    if(waited>native_adreno_wait_outer[3])native_adreno_wait_outer[3]=waited;
    if(status==kIOReturnTimeout) {
        native_adreno_wait_outer[4]=native_adreno_wait_outer[4]+1;
        adreno830_status_add64(&adreno830_status.wait_timeouts,1);
    }
    return status;
}
/* Submit (ARCHITECTURE 5.4): the core's submission under the connection lock; a caller with nothing in flight whose submission finds the
 * ring full (or the ring halted by a recovery) is never told Busy at once: the connection lock is dropped, the thread sleeps on the
 * fence that frees room (in the ring's completion wait), and the submission is made again, for at most 200 ms. */
IOReturn AGXDeviceUserClient::submit(const void *input,size_t input_size,void *output,size_t output_size)
{
    static uint32_t sampled;
    const bool sample=!(__atomic_fetch_add(&sampled,1,__ATOMIC_RELAXED)&255);
    const uint64_t t0=sample?nowUs():0;
    const IOReturn status=NativeAdrenoSubmitWithRetry(
        [&](uint32_t *retry_fence)->IOReturn {
            if(lost())return NativeAdrenoReturnNotResponding;
            if(!lock_)return kIOReturnNotReady;
            /* M13 (ARCHITECTURE 4.17): a submission holds the GPU awake; under P1/P2 acquire() wakes it from slumber, under
             * the default P0 it is two atomics. A transition or a recovery in progress is waited out like a full ring. */
            A830PowerState *power=adreno830_power_state();
            if(power && power->acquire()!=kIOReturnSuccess) {
                if(lost())return NativeAdrenoReturnNotResponding;
                *retry_fence=0;
                native_adreno_submit_outer[0]=native_adreno_submit_outer[0]+1;
                return NativeAdrenoSubmitRetry;
            }
            lockMetered(lock_,Adreno830WaitConnection);
            IOReturn result=live_ && !closing_?box_->core.dispatch(NativeAdrenoMethodSubmit,input,input_size,output,output_size,&g.clock):
                kIOReturnNotReady;
            if(result==NativeAdrenoSubmitRetry)*retry_fence=box_->core.retryFence();
            IOLockUnlock(lock_);
            if(power)power->release();
            if(result==kIOReturnSuccess) {
                /* The fence watchdog learns of the work in flight even when nobody waits on it (ARCHITECTURE 4.13.3). */
                if(A830Fence *fence=A830Fence::instance())fence->submitted(g.ring->lastSubmitted());
            } else if(g.ring->poisoned() && !recovering() && !lost()) {
                /* The fenced WPTR write was dropped 200 times (ARCHITECTURE 4.13.2): the ring stays poisoned until the
                 * recovery restarts the CP (ARCHITECTURE 4.13.4); repeated reports coalesce. */
                adreno830_recovery_report(A830FaultWptrFence,g.ring->wptrStats().failures);
            }
            if(result==NativeAdrenoSubmitRetry)native_adreno_submit_outer[0]=native_adreno_submit_outer[0]+1;
            return result;
        },
        [&](uint32_t fence,uint64_t deadline_us) {
            const uint64_t now=nowUs();
            if(now>=deadline_us || closing_)return;
            uint32_t observed=0;
            /* A halted ring (recovery) gives no fence: a short sleep, then the submission is tried again. */
            if(!fence || recovering() || g.ring->observeFence(fence,&g.clock,(uint32_t)(deadline_us-now),&observed)==NativeAdrenoRingFenceArgument)
                IOSleep(1);
        },
        []{return nowUs();});
    if(status==kIOReturnSuccess) {
        adreno830_status_add64(&adreno830_status.submits,1);
        adreno830_dcvs_submit();
        if(sample) {
            const uint64_t spent=nowUs()-t0;
            native_adreno_submit_outer[2]=spent;native_adreno_submit_outer[4]=native_adreno_submit_outer[4]+1;
            if(spent>native_adreno_submit_outer[3])native_adreno_submit_outer[3]=spent;
        }
    } else if(status==kIOReturnBusy)native_adreno_submit_outer[1]=native_adreno_submit_outer[1]+1;
    else if(status==NativeAdrenoReturnNotResponding)native_adreno_submit_outer[5]=native_adreno_submit_outer[5]+1;
    return status;
}
/* BoImport: the pages belong to the task making this call (an external method runs in the caller's context). A range inside the task's
 * mapping of the hand-off scanout (memory type 0x5343, M9) becomes a BO of the scanout's physical range (ARCHITECTURE 5.3.2). */
IOReturn AGXDeviceUserClient::importBo(const void *input,size_t input_size,void *output,size_t output_size)
{
    const NativeAdrenoCallTimer timer(&g.clock,Adreno830CallImport);
    noteCall(NativeAdrenoMethodBoImport);
    if(!input || input_size!=sizeof(NativeAdrenoAbiBoImport) || !output || output_size!=sizeof(NativeAdrenoAbiBoAllocReply))
        return kIOReturnBadArgument;
    NativeAdrenoAbiBoImport request{};
    memcpy(&request,input,sizeof(request));
    NativeAdrenoAbiBoAllocReply reply{};
    task_t task=current_task();
    NativeAdrenoScanoutMapping scanout{};
    IOMemoryDescriptor *memory=native_gop_handoff_memory();   /* retained for this call; the presenter keeps its own */
    if(memory) {
        /* The task's existing mapping of the scanout descriptor (made through clientMemoryForType), found without creating one. */
        IOMemoryMap *map=memory->createMappingInTask(task,0,kIOMapAnywhere|kIOMapReference,0,0);
        if(map) {
            scanout={task,map->getAddress(),map->getLength(),memory};
            map->release();
        }
    }
    native_adreno_metal_diag.inflight=NativeAdrenoMethodBoImport+1;
    IOReturn status=kIOReturnNotReady;
    if(lock_) {
        lockMetered(lock_,Adreno830WaitConnection);
        if(live_) {
            box_->core.setScanout(scanout.memory?&scanout:nullptr);
            status=box_->core.importBo(&request,task,&reply);
            box_->core.setScanout(nullptr);
        }
        IOLockUnlock(lock_);
    }
    if(memory)memory->release();
    native_adreno_metal_diag.inflight=0;
    native_adreno_metal_diag.calls=native_adreno_metal_diag.calls+1;
    native_adreno_metal_diag.last_selector=NativeAdrenoMethodBoImport;
    native_adreno_metal_diag.last_status=uint32_t(status);
    if(!status)memcpy(output,&reply,sizeof(reply));
    return status;
}
static IOReturn runCore(AGXDeviceUserClient *connection,uint32_t method,const void *input,size_t input_size,void *output,size_t output_size)
{
    /* The call meter: the whole call as its caller sees it, the waits for the connection's lock, a full ring's sleep and the teardown of a
     * freed BO's CPU view included; the waits are counted apart as well (ARCHITECTURE 8.8). */
    const NativeAdrenoCallTimer timer(&g.clock,NativeAdrenoMeterKindOf(method));
    connection->noteCall(method);
    if(method==NativeAdrenoMethodSetMetadata && input && input_size==sizeof(NativeAdrenoAbiMetadata)) {
        const NativeAdrenoAbiMetadata *meta=static_cast<const NativeAdrenoAbiMetadata *>(input);
        /* The raw test's line and the Turnip phase's line ("T ..." result or "W ..." watchdog sample) are kept apart, so neither hides the other. */
        const bool turnip=meta->bytes>=2 && (meta->text[0]=='T' || meta->text[0]=='W') && meta->text[1]==' ';
        char *line=turnip?native_adreno_metal_diag.report2:native_adreno_metal_diag.report;
        uint32_t n=meta->bytes<sizeof(native_adreno_metal_diag.report)-1?meta->bytes:uint32_t(sizeof(native_adreno_metal_diag.report)-1);
        memset(line,0,sizeof(native_adreno_metal_diag.report));
        memcpy(line,meta->text,n);
        native_adreno_metal_diag.reports=native_adreno_metal_diag.reports+1;
    }
    /* A call that is still running when the boot page is photographed shows as inflight = its method number + 1 (0 = none), so a hang in the
     * kernel is attributable to one ABI method. */
    native_adreno_metal_diag.inflight=method+1;
    native_adreno_metal_diag.inflight_since_calls=native_adreno_metal_diag.calls;
    /* The CPU views of the connection's BOs (clientMemoryForType), for the external methods and the mailbox alike (the mailbox used to
     * bypass this, so a BoCpuUnmap through it left the view and its user mapping behind). A view is shared by every CPU mapping of its
     * BO and dropViews removes all of them: only the last lease takes the view down; the earlier ones leave their mapping until it, or
     * the client unmaps it itself (IOConnectUnmapMemory64 works on the cached view). A BO that is freed loses whatever view is left,
     * so no user mapping outlives its pages and a later BO with the same token never gets the old view. */
    uint32_t freed_token=0;
    if(method==NativeAdrenoMethodBoCpuUnmap && input && input_size==sizeof(NativeAdrenoAbiBoCpuUnmap) && connection->lock_) {
        const uint32_t token=static_cast<const NativeAdrenoAbiBoCpuUnmap *>(input)->token;
        lockMetered(connection->lock_,Adreno830WaitConnection);
        const uint32_t leases=connection->live_?connection->box_->core.mappingLeases(token):0;
        IOLockUnlock(connection->lock_);
        if(leases>1)native_adreno_view_kept=native_adreno_view_kept+1;
        else connection->dropViews(token);
    } else if(method==NativeAdrenoMethodBoFree && input && input_size==sizeof(NativeAdrenoAbiBoFree) && connection->lock_) {
        lockMetered(connection->lock_,Adreno830WaitConnection);
        freed_token=connection->live_?connection->box_->core.tokenOf(static_cast<const NativeAdrenoAbiBoFree *>(input)->handle):0;
        IOLockUnlock(connection->lock_);
    }
    IOReturn status;
    if(method==NativeAdrenoMethodWaitFence)status=connection->waitFence(input,input_size,output,output_size);
    else if(method==NativeAdrenoMethodSubmit)status=connection->submit(input,input_size,output,output_size);
    else {
        status=connection->locked(method,input,input_size,output,output_size);
        if(method==NativeAdrenoMethodBoMap && !status && !connection->kernel_ && connection->lock_) {
            lockMetered(connection->lock_,Adreno830WaitConnection);
            const uint64_t peak=connection->live_?connection->box_->core.windowPeak():0;
            IOLockUnlock(connection->lock_);
            IOLockLock(g.lock);
            if(NativeAdrenoClientRecord *record=clientRecord(connection->name_))
                if(peak>>20>record->peak_mib)record->peak_mib=uint32_t(peak>>20);
            notePast4G(connection->name_,connection->slot_,static_cast<const NativeAdrenoAbiBoMap *>(input)->iova,peak);
            IOLockUnlock(g.lock);
        }
        if(freed_token && !status)connection->dropViews(freed_token);
    }
    native_adreno_metal_diag.inflight=0;
    native_adreno_metal_diag.calls=native_adreno_metal_diag.calls+1;
    native_adreno_metal_diag.last_selector=method;
    native_adreno_metal_diag.last_status=uint32_t(status);
    return status;
}
IOReturn AGXDeviceUserClient::run(uint32_t method,const void *input,size_t input_size,void *output,size_t output_size)
{
    if(method==NativeGpuTraceMethod) {
        if(!input || input_size!=sizeof(NativeGpuTraceRecord) || output_size)return kIOReturnBadArgument;
        const auto *record=static_cast<const NativeGpuTraceRecord *>(input);
        if(record->event<30 || record->event>63)return kIOReturnBadArgument;
        native_gpu_trace_record(record->event,proc_selfpid(),record->value,12);
        return kIOReturnSuccess;
    }
    if(method==NativeGpuPixelMethod) {
        if(!input || input_size!=sizeof(NativeGpuPixelPiece) || output_size)return kIOReturnBadArgument;
        return native_gpu_pixel_put(static_cast<const NativeGpuPixelPiece *>(input),proc_selfpid())?kIOReturnNoResources:kIOReturnSuccess;
    }


    if(NativeAdrenoIsHandoffMethod(method)) {
        const uint32_t op=method-NativeAdrenoHandoffBase;
        noteHandoffTask();
        const IOReturn status=native_gop_handoff_call(op,current_task(),input,input_size,output,output_size);
        if(!status && (op==HandoffWaitSwap || op==HandoffWaitSwap2))native_adreno_ngp_stats[0]=native_adreno_ngp_stats[0]+1;
        if(!status && op==HandoffCompleteSwap)native_adreno_ngp_stats[1]=native_adreno_ngp_stats[1]+1;
        if(op==HandoffBand)native_adreno_ngp_stats[3]=native_adreno_ngp_stats[3]+1;
        if(status && status!=kIOReturnTimeout) {native_adreno_ngp_stats[2]=native_adreno_ngp_stats[2]+1;native_adreno_ngp_stats[4]=uint32_t(status);}
        return status;
    }
    if(method==NativeAdrenoExportMethod) {
        struct Piece {uint32_t generation,total,offset,bytes;};
        char caller[20]={};proc_selfname(caller,sizeof caller);
        Piece piece{};
        if(input && input_size>=sizeof(piece))memcpy(&piece,input,sizeof(piece));
        const bool valid=!output_size && input && input_size>=sizeof(piece) && piece.bytes && piece.bytes<=NativeAdrenoExportMaxPiece &&
            input_size==sizeof(piece)+piece.bytes && !strncmp(caller,"backboardd",sizeof caller);
        const int stored=valid?boot_log_export_put(piece.generation,piece.total,piece.offset,
            static_cast<const uint8_t *>(input)+sizeof(piece),piece.bytes):-1;
        if(stored) {native_adreno_export_stats[1]=native_adreno_export_stats[1]+1;return kIOReturnBadArgument;}
        native_adreno_export_stats[0]=native_adreno_export_stats[0]+1;
        native_adreno_export_stats[2]=piece.generation;native_adreno_export_stats[3]=piece.total;
        return kIOReturnSuccess;
    }
    if(method==NativeAdrenoFrontendMethod) {
        if(!adreno830_status_diag_lines())return kIOReturnUnsupported;
        char caller[20]={};proc_selfname(caller,sizeof caller);
        if(!input || input_size!=sizeof(Adreno830StatusFrontend) || output_size || strncmp(caller,"backboardd",sizeof caller))return kIOReturnBadArgument;
        Adreno830StatusFrontend frontend;
        memcpy(&frontend,input,sizeof frontend);
        adreno830_status_frontend(&frontend);
        return kIOReturnSuccess;
    }
    if(handoffOnly_)return kIOReturnUnsupported;   /* a hand-off connection mounts no GPU core */
    if(method==NativeAdrenoMethodBoImport)return importBo(input,input_size,output,output_size);
    return runCore(this,method,input,input_size,output,output_size);
}
IOReturn AGXDeviceUserClient::action(OSObject *target,void *,IOExternalMethodArguments *arguments)
{
    AGXDeviceUserClient *connection=OSDynamicCast(AGXDeviceUserClient,target);
    if(!connection || !arguments)return kIOReturnBadArgument;
    return connection->run(arguments->selector,arguments->structureInput,arguments->structureInputSize,
        arguments->structureOutput,arguments->structureOutputSize);
}
IOReturn AGXDeviceUserClient::externalMethod(uint32_t selector,IOExternalMethodArguments *arguments,
    IOExternalMethodDispatch *,OSObject *target,void *reference)
{
    if(arguments && (selector==NativeGpuTraceMethod || selector==NativeGpuPixelMethod))
        return run(selector,arguments->structureInput,arguments->structureInputSize,
                   arguments->structureOutput,arguments->structureOutputSize);
    /* One entry per ABI method; sizes are the exact wire records. */
    static const IOExternalMethodDispatch table[]={
        {&action,0,0,0,sizeof(NativeAdrenoAbiParams)},
        {&action,0,sizeof(NativeAdrenoAbiBoAlloc),0,sizeof(NativeAdrenoAbiBoAllocReply)},
        {&action,0,sizeof(NativeAdrenoAbiBoMap),0,sizeof(NativeAdrenoAbiBoMapReply)},
        {&action,0,sizeof(NativeAdrenoAbiBoFree),0,0},
        {&action,0,sizeof(NativeAdrenoAbiSubmit),0,sizeof(NativeAdrenoAbiSubmitReply)},
        {&action,0,sizeof(NativeAdrenoAbiWaitFence),0,0},
        {&action,0,0,0,sizeof(NativeAdrenoAbiTimestamp)},
        {&action,0,0,0,sizeof(NativeAdrenoAbiStatus)},
        {&action,0,sizeof(NativeAdrenoAbiMetadata),0,0},
        {&action,0,sizeof(NativeAdrenoAbiMetadata),0,sizeof(NativeAdrenoAbiMetadata)},
        {&action,0,sizeof(NativeAdrenoAbiQueue),0,sizeof(NativeAdrenoAbiQueueReply)},
        {&action,0,sizeof(NativeAdrenoAbiQueue),0,0},
        {&action,0,sizeof(NativeAdrenoAbiBoCpuMap),0,sizeof(NativeAdrenoAbiBoCpuMapReply)},
        {&action,0,sizeof(NativeAdrenoAbiBoCpuUnmap),0,0},
        {&action,0,0,0,sizeof(NativeAdrenoAbiDeviceInfo)},
        {&action,0,sizeof(NativeAdrenoAbiBoImport),0,sizeof(NativeAdrenoAbiBoAllocReply)}};
    native_adreno_metal_diag.ext_entries=native_adreno_metal_diag.ext_entries+1;
    if(selector<256)native_adreno_metal_diag.sel_bits[selector>>5]|=1u<<(selector&31);
    /* Tunnel. The sandbox resolves a filter policy for this class at open time and then admits only the selectors its profile lists,
     * so which selector numbers reach this object is decided outside the driver. A tunnel call carries the real ABI method in its
     * first eight bytes and may use ANY admitted selector: {magic, method, payload...} in, the method's exact output out. Method
     * 0xffffffff is a probe that only proves the selector was admitted. */
    if(arguments && arguments->structureInput && arguments->structureInputSize>=8 &&
        static_cast<const uint32_t *>(arguments->structureInput)[0]==NativeAdrenoTunnelMagic) {
        const uint32_t *header=static_cast<const uint32_t *>(arguments->structureInput);
        if(header[1]==0xffffffffu) {
            native_adreno_metal_diag.probes=native_adreno_metal_diag.probes+1;
            return kIOReturnSuccess;
        }
        native_adreno_metal_diag.tunnel_calls=native_adreno_metal_diag.tunnel_calls+1;
        const IOReturn tunneled=run(header[1],reinterpret_cast<const uint8_t *>(arguments->structureInput)+8,
            arguments->structureInputSize-8,arguments->structureOutput,arguments->structureOutputSize);
        native_adreno_metal_diag.ext_status=uint32_t(tunneled);
        return tunneled;
    }
    native_adreno_metal_diag.ext_selector=selector;
    native_adreno_metal_diag.ext_in=arguments?arguments->structureInputSize:0xffff;
    native_adreno_metal_diag.ext_out=arguments?arguments->structureOutputSize:0xffff;
    if(selector<1 || selector>sizeof(table)/sizeof(table[0])) {
        native_adreno_metal_diag.ext_status=uint32_t(kIOReturnUnsupported);
        return kIOReturnUnsupported;
    }
    const IOReturn result=IOUserClient::externalMethod(selector,arguments,const_cast<IOExternalMethodDispatch *>(&table[selector-1]),
        target?target:this,reference);
    native_adreno_metal_diag.ext_status=uint32_t(result);
    return result;
}
/* Progress markers: open types 0x4147xxxx (xxxx nonzero) are not connections. The plugin sends one before and after each step so that a
 * sandbox that admits neither selectors nor mapping, or a crash, still leaves a trace of how far it got. Always kIOReturnUnsupported. */
static IOReturn noteMarker(uint32_t value)
{
    const uint32_t slot=native_adreno_metal_diag.marks_total&15;
    /* Text stream into the rows table, two characters per marker: 0xC000|row starts a row. Not counted as markers. */
    if(value>=0x8000) {
        static unsigned row=48,length;
        if(value>=0xC000) {
            row=value&0x3f;length=0;
            if(row<48)memset(native_adreno_metal_rows[row],0,sizeof(native_adreno_metal_rows[row]));
            native_adreno_metal_rows_seq=native_adreno_metal_rows_seq+1;
        } else if(row<48) {
            const char pair[2]={char((value>>7)&0x7f),char(value&0x7f)};
            for(unsigned i=0;i<2;++i)if(pair[i] && length<sizeof(native_adreno_metal_rows[row])-1)native_adreno_metal_rows[row][length++]=pair[i];
        }
        return kIOReturnUnsupported;
    }
    if(value==NativeAdrenoMarkerDoorbell) {
        /* The mailbox doorbell: the server thread looks at the page now instead of at its next poll. */
        if(g.lock) {IOLockLock(g.lock);IOLockWakeup(g.lock,&gMailboxDoorbell,false);IOLockUnlock(g.lock);}
        return kIOReturnUnsupported;
    }
    if(value>=0x7000 && value<0x8000) {
        /* One character per marker into report2 (0x7fff starts a line) or report (0x7ffe): needs no connection, so the plugin's Turnip
         * watchdog can report while Turnip itself holds its connection slot. A line from a process other than the render server goes
         * to that process's own row (the render server rewrites report2 every ten seconds). */
        static uint32_t length;
        static char *line=native_adreno_metal_diag.report2;
        static_assert(sizeof(native_adreno_metal_diag.report)==sizeof(native_adreno_metal_diag.report2),"stream lines");
        if(value==0x7fff || value==0x7ffe) {
            line=value==0x7fff?native_adreno_metal_diag.report2:native_adreno_metal_diag.report;
            char who[20]={};
            proc_selfname(who,sizeof who);
            if(value==0x7fff && strncmp(who,"backboardd",sizeof who)) {
                for(auto &entry:native_adreno_client_text) {
                    if(!entry.name[0]) strlcpy(entry.name,who,sizeof entry.name);
                    if(!strncmp(entry.name,who,sizeof entry.name)) {line=entry.text;break;}
                }
            }
            memset(line,0,sizeof(native_adreno_metal_diag.report2));length=0;
        }
        else if(value<0x7100 && length<sizeof(native_adreno_metal_diag.report2)-1) {line[length++]=char(value&0xff);line[length]=0;}
        return kIOReturnUnsupported;
    }
    native_adreno_metal_diag.marks[slot]=value;
    native_adreno_metal_diag.marks_total=native_adreno_metal_diag.marks_total+1;
    if(value==1) {
        memcpy(native_adreno_metal_diag.starter[2],native_adreno_metal_diag.starter[1],17);
        memcpy(native_adreno_metal_diag.starter[1],native_adreno_metal_diag.starter[0],17);
        proc_selfname(native_adreno_metal_diag.starter[0],17);
    }
    return kIOReturnUnsupported;
}
/* A GPU connection while a recovery runs waits for its end (at most 5 s) instead of failing: a render server that restarted after a loss
 * must get its GPU back, not a CPU-only life. Under g.lock. */
static void waitForRecoveryLocked()
{
    AbsoluteTime deadline=0;
    clock_interval_to_deadline(5,kSecondScale,&deadline);
    while(__atomic_load_n(&g.recovering,__ATOMIC_ACQUIRE) && !__atomic_load_n(&g.lost,__ATOMIC_ACQUIRE))
        if(IOLockSleepDeadline(g.lock,&g.recovering,deadline,THREAD_UNINT)==THREAD_TIMED_OUT)break;
}
/* limit > 0 caps the connections ever granted to each process name (sheng-gpu-metal-opens): a plugin that crashes its host process on
 * every launch (the process respawns and loads it again) must not keep a system process in a crash loop. Concurrency is the other bound:
 * NativeAdrenoMaxClients connections at a time (ARCHITECTURE 6.2). */
extern "C" IOReturn native_adreno_new_user_client(IOService *provider,task_t owningTask,void *securityID,uint32_t type,
    OSDictionary *properties,IOUserClient **handler,unsigned limit)
{
    if(handler)*handler=nullptr;
    if(!handler || !g.published)return kIOReturnNotReady;
    if((type>>16)==NativeAdrenoPluginOpenType && (type&0xffff))return noteMarker(type&0xffff);
    shiftName(native_adreno_metal_diag.opener);
    AGXDeviceUserClient *connection=new AGXDeviceUserClient;
    if(!connection)return kIOReturnNoMemory;
    if(!connection->initWithTask(owningTask,securityID,type,properties)) {
        connection->release();
        native_adreno_metal_diag.bad=native_adreno_metal_diag.bad+1;
        return kIOReturnNoResources;
    }
    /* IOService::newUserClient attaches and starts what it creates (a client without a provider chain is also a reason for the sandbox to
     * refuse). Then the two sandbox gates IOServiceOpen would apply afterwards are run here for every opener and logged with the
     * process name and open type, so the boot page shows which processes the sandbox lets reach this class and with which type. */
    unsigned attach_state=0;
    if(provider) {
        const bool attached=connection->attach(provider);
        attach_state=attached?1:2;
        if(attached && !connection->start(provider)) {
            connection->detach(provider);
            attach_state=3;
        }
    }
    native_adreno_metal_diag.attach_status=attach_state;
    /* A refused open must leave nothing attached: the provider's child link kept every refused connection object for the rest of the boot
     * (the IOGPU library's type-1 opens, the per-name cap, no free slot, a lost device), and past 1019 children every IOService::attach
     * to the accelerator stalls 15 s, uninterruptibly, before it fails (XNU IOService::attach, kIOServiceBusyMax). */
    auto discard=[&]() {
        if(attach_state==1)connection->detach(provider);
        connection->release();
    };
    const uint32_t macf=uint32_t(mac_iokit_check_open(kauth_cred_get(),connection,type));
    uint32_t resolver=0xfeedfeed;
    if(gIOUCFilterCallbacks && gIOUCFilterCallbacks->io_filter_resolver) {
        io_filter_policy_t policy=0;
        const IOReturn resolved=gIOUCFilterCallbacks->io_filter_resolver(owningTask,connection,type,&policy);
        resolver=uint32_t(resolved);
        if(resolved==kIOReturnSuccess && policy && gIOUCFilterCallbacks->io_filter_release)gIOUCFilterCallbacks->io_filter_release(policy);
    }
    native_adreno_metal_diag.chk_type=type;
    native_adreno_metal_diag.chk_macf=macf;
    native_adreno_metal_diag.chk_res=resolver;
    {
        NativeAdrenoOpenRecord &record=native_adreno_open_log[native_adreno_open_log_count&7];
        proc_selfname(record.name,sizeof(record.name));
        record.type=type;record.macf=macf;record.res=resolver;record.attach=attach_state;record.foreign=type!=NativeAdrenoPluginOpenType;
        native_adreno_open_log_count=native_adreno_open_log_count+1;
    }
    if(type==NativeAdrenoHandoffOpenType) {
        connection->setHandoffOnly();
        *handler=connection;
        return kIOReturnSuccess;
    }
    /* Only the Adreno plugin's open (type NativeAdrenoPluginOpenType) is served. The original IOGPU library opens the accelerator with type 1
     * (and an API value in the upper bits) for an IOGPUDeviceUserClient, which this driver is not; refuse it cleanly. */
    if(type!=NativeAdrenoPluginOpenType) {
        discard();
        native_adreno_metal_diag.foreign_opens=native_adreno_metal_diag.foreign_opens+1;
        native_adreno_metal_diag.foreign_type=type;
        return kIOReturnUnsupported;
    }
    /* The plugin's open types are granted by this driver: the grant is a property of the client object that the kernel's open path
     * (patches/xnu/native-adreno-sandbox-grant.patch) honours in place of the sandbox verdict. The verdict is still taken above and logged.
     * Which processes run the plugin at all is the plugin's allow-list (NativeAdrenoMetal.clients). */
    const bool kernel=owningTask==kernel_task;
    char opener[20]={};
    proc_selfname(opener,sizeof opener);
    if(!kernel && (macf || resolver!=uint32_t(kIOReturnSuccess))) {
        connection->setProperty("NativeSandboxGrant",kOSBooleanTrue);
        native_adreno_metal_diag.grants=native_adreno_metal_diag.grants+1;
    }
    IOLockLock(g.lock);
    waitForRecoveryLocked();
    /* A global device loss (hang class, no recovery left) refuses a GPU connection (ARCHITECTURE 4.13.4), a page fault never does; a name
     * whose jobs hung the GPU NativeAdrenoClientHangLimit times is refused, never the render server. */
    const bool render_server=!kernel && !strncmp(opener,"backboardd",sizeof opener);
    IOReturn refusal;
    {
        const NativeAdrenoClientRecord *record=kernel?nullptr:clientRecord(opener);
        refusal=NativeAdrenoOpenRefusal(lost(),limit,record?openStreak(record):0,record && !render_server?record->hangs:0);
    }
    IOLockUnlock(g.lock);
    if(!refusal)refusal=connection->mount(kernel);
    /* The row is looked up again under each hold of g.lock: while it was free, another open may have been given this name's row (an
     * idle row without hangs, NativeAdrenoClientRecordFor). */
    if(refusal) {
        discard();
        native_adreno_metal_diag.busy=native_adreno_metal_diag.busy+1;
        IOLockLock(g.lock);
        NativeAdrenoClientRecord *record=kernel?nullptr:clientRecord(opener);
        if(record) {record->refused=record->refused+1;record->last_refusal=uint32_t(refusal);}
        IOLockUnlock(g.lock);
        return refusal;
    }
    IOLockLock(g.lock);
    NativeAdrenoClientRecord *record=kernel?nullptr:clientRecord(opener);
    if(record) {record->opens=record->opens+1;openStreak(record)=openStreak(record)+1;}
    IOLockUnlock(g.lock);
    native_adreno_metal_diag.opens=native_adreno_metal_diag.opens+1;
    *handler=connection;
    return kIOReturnSuccess;
}
extern "C" void native_adreno_note_property_read(const char *key)
{
    if(key && !strcmp(key,"MetalPluginName")) {
        proc_selfname(native_adreno_metal_diag.name_reader,17);
        native_adreno_metal_diag.name_reads=native_adreno_metal_diag.name_reads+1;
        char name[20]={};
        proc_selfname(name,sizeof name);
        /* Unlocked: the discovery runs at process start, a lost count is harmless and a row is claimed by its first byte. */
        for(auto &asker:native_adreno_metal_askers) {
            if(!asker.name[0]) {strlcpy(asker.name,name,sizeof asker.name);asker.reads=1;break;}
            if(!strncmp(asker.name,name,sizeof asker.name)) {asker.reads=asker.reads+1;break;}
        }
    }
    shiftName(native_adreno_metal_diag.reader);
    native_adreno_metal_diag.property_reads=native_adreno_metal_diag.property_reads+1;
}
IOReturn NativeAdrenoGpuPublished::newUserClient(task_t owningTask,void *securityID,UInt32 type,
    OSDictionary *properties,IOUserClient **handler)
{
    return native_adreno_new_user_client(this,owningTask,securityID,type,properties,handler,0);
}

/* ---- Device-level fault plumbing (NativeAdrenoStatus.h) ------------------------------------------------------------------------ */
extern "C" bool native_adreno_note_page_fault(uint64_t far,uint32_t completed,uint32_t issued,bool sampled)
{
    bool attributed=false;
    unsigned slot=NativeAdrenoMaxClients;
    /* The window holding FAR is charged; the jobs in flight when the top half took the fault are only compared with it and counted
     * (M14 P0, ARCHITECTURE 4.12.3). */
    const unsigned at=NativeAdrenoFaultNote(g.published?g.windows:nullptr,g.published?g.submit:nullptr,far,completed,issued,sampled,
        native_adreno_fault_attribution);
    if(at<NativeAdrenoMaxClients) {
        IOLockLock(g.lock);
        if(CoreBox *box=slotBox(at)) {box->status.noteFault();attributed=true;slot=at;}
        IOLockUnlock(g.lock);
    }
    if(attributed) {
        native_adreno_fault_stats[0]=native_adreno_fault_stats[0]+1;native_adreno_fault_stats[6]=slot;
        adreno830_status_add32(&adreno830_status.faults,1);
    } else {
        native_adreno_fault_stats[1]=native_adreno_fault_stats[1]+1;native_adreno_fault_stats[5]=uint32_t(far);
    }
    return attributed;
}
/* Every connection and orphan, under g.lock. */
template<class F> static void forEachBox(F f)
{
    for(unsigned s=0;s<NativeAdrenoMaxClients;++s)if(CoreBox *box=slotBox(s))f(s,box);
    if(g.kernel_client && g.kernel_client->box())f(NativeAdrenoMaxClients,g.kernel_client->box());
    if(g.kernel_orphan)f(NativeAdrenoMaxClients,g.kernel_orphan);
}
extern "C" void native_adreno_device_lost(uint32_t reason)
{
    if(!reason)reason=A830ReasonHang;
    uint32_t expected=0;
    if(!__atomic_compare_exchange_n(&g.lost,&expected,reason,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE))return;   /* already lost */
    native_adreno_fault_stats[2]=native_adreno_fault_stats[2]+1;native_adreno_fault_stats[7]=reason;
    adreno830_status_add32(&adreno830_status.faults,1);
    if(!g.published)return;
    IOLockLock(g.lock);
    forEachBox([](unsigned,CoreBox *box) {box->status.noteFault();});
    /* Opens waiting for a recovery give up now. */
    IOLockWakeup(g.lock,&g.recovering,false);
    IOLockUnlock(g.lock);
}
extern "C" void native_adreno_recovery_begin(bool charge_culprit)
{
    __atomic_store_n(&g.recovering,1u,__ATOMIC_RELEASE);
    native_adreno_fault_stats[3]=native_adreno_fault_stats[3]+1;
    if(!g.published)return;
    /* Halt first, under the ring lock: from here no submission reaches the ring, and the snapshot of the ranges with work in flight is
     * exact. Those connections lose their jobs: faults += 1 (Turnip sees the device lost for that process only), whatever the cause. */
    uint32_t ranges=0,oldest=UINT32_MAX;
    g.submit->halt(&ranges,&oldest);
    IOLockLock(g.lock);
    /* The culprit, only for a cause a client's job can produce (charge_culprit: the hang detector, CP_HW_ERROR, the software watchdog):
     * the range whose job the CP stood in. A client other than the render server is charged one hang against its name
     * (NativeAdrenoClientHangLimit); the render server and the kernel client never are. A GMU, HFI, fenced-write, slumber, wake or
     * self-test fault is the driver's own: whoever's job happened to be in flight is not charged for it. */
    if(!charge_culprit)native_adreno_hang_culprits[4]=native_adreno_hang_culprits[4]+1;
    else if(oldest>=NativeAdrenoMaxClients)native_adreno_hang_culprits[2]=native_adreno_hang_culprits[2]+1;
    else {
        native_adreno_hang_culprits[3]=oldest;
        CoreBox *box=oldest?slotBox(oldest):nullptr;
        if(box && strncmp(box->name,"backboardd",sizeof box->name)) {
            native_adreno_hang_culprits[0]=native_adreno_hang_culprits[0]+1;
            if(NativeAdrenoClientRecord *record=clientRecord(box->name))record->hangs=record->hangs+1;
        } else native_adreno_hang_culprits[1]=native_adreno_hang_culprits[1]+1;
    }
    bool any=false;
    for(unsigned s=0;s<NativeAdrenoMaxClients;++s)
        if(ranges&(1u<<s))if(CoreBox *box=slotBox(s)) {box->status.noteFault();any=true;}
    if((ranges&1u) && !slotBox(0)) {
        /* range 0 without a user connection is the kernel client's */
        if(g.kernel_client && g.kernel_client->box()) {g.kernel_client->box()->status.noteFault();any=true;}
        else if(g.kernel_orphan) {g.kernel_orphan->status.noteFault();any=true;}
    }
    IOLockUnlock(g.lock);
    if(any)adreno830_status_add32(&adreno830_status.faults,1);
}
extern "C" void native_adreno_recovery_end(bool recovered)
{
    if(!g.published) {
        if(!recovered)native_adreno_device_lost(A830ReasonHang);
        __atomic_store_n(&g.recovering,0u,__ATOMIC_RELEASE);
        return;
    }
    if(recovered) {
        /* The GPU was power cycled and the ring restarted: nothing of the outstanding jobs runs any more, so their references go. */
        uint32_t abandoned=0;
        if(g.submit->recover(&abandoned)!=kIOReturnSuccess)recovered=false;
    }
    if(!recovered) {
        native_adreno_device_lost(A830ReasonHang);
        IOLockLock(g.lock);
        __atomic_store_n(&g.recovering,0u,__ATOMIC_RELEASE);
        IOLockWakeup(g.lock,&g.recovering,false);
        IOLockUnlock(g.lock);
        return;
    }
    native_adreno_fault_stats[4]=native_adreno_fault_stats[4]+1;
    IOLockLock(g.lock);
    forEachBox([](unsigned,CoreBox *box) {box->status.noteSuspend();});
    g.submit->resume();
    __atomic_store_n(&g.recovering,0u,__ATOMIC_RELEASE);
    IOLockWakeup(g.lock,&g.recovering,false);
    reapOrphans();
    IOLockUnlock(g.lock);
}
extern "C" void native_adreno_retire_completed(void)
{
    if(g.published && g.submit)(void)g.submit->retireCompleted();
}
/* Once a second (Adreno830Start's memory row on the DCVS thread). First the orphans: a closed connection whose jobs had not completed
 * within its 2 s teardown (or whose objects refused) is released as soon as that is possible, not only when a later open finds every
 * slot taken; only while the GPU is awake (the unmaps touch the SMMU), never during a recovery (its end reaps) or after a loss. Then
 * the figures of the MEM row: the client holding the most GPU memory (own BOs and imports) with its process name, every client's own
 * BOs outside the BO pool, and how many connections and orphans there are. Reads the cores' byte counters without their locks. */
extern "C" unsigned native_adreno_client_tick(Adreno830StatusClients *out)
{
    if(!out || !__atomic_load_n(&g.published,__ATOMIC_ACQUIRE) || !g.lock)return 0;
    *out={};
    IOLockLock(g.lock);
    const bool orphans=anyOrphan();
    IOLockUnlock(g.lock);
    if(orphans && !recovering() && !lost()) {
        A830PowerState *power=adreno830_power_state();
        if(!power || power->accessBegin(false)) {
            IOLockLock(g.lock);
            if(!recovering())reapOrphans();
            IOLockUnlock(g.lock);
            if(power)power->accessEnd(false);
        }
    }
    auto mib=[](uint64_t bytes) {const uint64_t m=(bytes+(1u<<20)-1)>>20;return uint32_t(m<UINT32_MAX?m:UINT32_MAX);};
    uint64_t top=0,top_imported=0,fallback=0;
    IOLockLock(g.lock);
    auto visit=[&](CoreBox *box,bool orphan) {
        if(orphan)++out->orphans;else ++out->clients;
        uint64_t pool=0,other=0,imported=0;
        box->core.usage(&pool,&other,&imported);
        fallback+=other;
        if(pool+other+imported>top) {
            top=pool+other+imported;top_imported=imported;out->top_orphan=orphan?1:0;
            strlcpy(out->top,box->name,sizeof out->top);
        }
    };
    uint32_t classes[Adreno830CallClasses];
    for(unsigned s=0;s<NativeAdrenoMaxClients;++s) {
        if(AGXDeviceUserClient *connection=g.slots[s].connection) {
            CoreBox *box=connection->box();
            if(!box)continue;
            visit(box,false);
            /* This second's busiest connection, and who has been quiet for how long (the slot refusal's table). */
            const uint32_t calls=connection->takeCalls(classes);
            box->quiet_s=calls?0:box->quiet_s+1;
            if(calls>out->second_top_calls) {
                out->second_top_calls=calls;
                memcpy(out->second_classes,classes,sizeof classes);
                strlcpy(out->second_top,box->name,sizeof out->second_top);
            }
        } else if(g.slots[s].orphan) {
            visit(g.slots[s].orphan,true);
            g.slots[s].orphan->quiet_s=g.slots[s].orphan->quiet_s+1;
        }
    }
    if(g.kernel_client && g.kernel_client->box())visit(g.kernel_client->box(),false);
    if(g.kernel_orphan)visit(g.kernel_orphan,true);
    IOLockUnlock(g.lock);
    static uint32_t callsSeen;
    const uint32_t calls=native_adreno_metal_diag.calls;
    out->second_calls=calls-callsSeen;callsSeen=calls;
    out->top_mib=mib(top);out->top_import_mib=mib(top_imported);out->fallback_mib=mib(fallback);
    /* Slot pressure and the fault attribution since the boot, for the ramoops meter line (M14 P0): plain counter reads. */
    out->peak_clients=native_adreno_slot_pressure[0];out->slot_refusals=native_adreno_slot_pressure[1];
    uint32_t overflows=0;
    for(unsigned s=0;s<NativeAdrenoSubmitRanges;++s)overflows+=native_adreno_window_overflow[s][0];
    out->window_overflows=overflows;
    out->reserve_objects=native_adreno_vm_reserve_refused[0];out->reserve_spans=native_adreno_vm_reserve_refused[1];
    out->faults_alike=native_adreno_fault_attribution[NativeAdrenoFaultAlike];
    out->faults_apart=native_adreno_fault_attribution[NativeAdrenoFaultApart];
    out->faults_undecided=native_adreno_fault_attribution[NativeAdrenoFaultUndecided];
    out->faults_polled=native_adreno_fault_attribution[NativeAdrenoFaultPolled];
    /* The calls since the last tick (ARCHITECTURE 8.8): the status block folds them into the meter line's window. Without the meter nothing
     * is taken and the line has no call figures. */
    static uint64_t seen[NativeAdrenoVmKinds][2];
    if(NativeAdrenoKoptOn(NativeAdrenoKoptMeter))NativeAdrenoMeterTake(&out->calls,seen);
    return 1;
}

/* ---- Publication ------------------------------------------------------------------------------------------------------------- */
/* Publishes the service once. Everything passed in must outlive the system (the orchestrator keeps them in statics) and the VM, submit
 * object and ring must already be attached to the live GPU context. */
extern "C" IOReturn native_adreno_publish(NativeAdrenoVm *vm,NativeAdrenoSubmit *submit,NativeAdrenoRing *ring,
    const NativeAdrenoRingClock *clock,const NativeAdrenoTimer *timer,const NativeAdrenoAbiDeviceInfo *info,
    uint32_t input_bits,uint32_t output_bits)
{
    if(g.published || !vm || !submit || !ring || !clock || !clock->now_us || input_bits<31 || input_bits>48 || output_bits<32 ||
        output_bits>48)return kIOReturnBadArgument;
    /* The client windows of ARCHITECTURE 6.2 follow the aperture the DeviceInfo publishes: 16 GiB (va_start + va_size above 4 GiB), 4 GiB
     * (above 2 GiB) or the 2 GiB fallback (pagani-gpu-va=2). The table must fit the input range. */
    const NativeAdrenoClientWindow *windows=NativeAdrenoClientWindowsFor(info?info->va_start+info->va_size:0);
    if(NativeAdrenoClientWindowsEnd(windows)>(UINT64_C(1)<<input_bits))return kIOReturnBadArgument;
    if(!g.lock)g.lock=IOLockAlloc();
    if(!g.ring_lock)g.ring_lock=IOLockAlloc();
    if(!g.memory_lock)g.memory_lock=IOLockAlloc();
    if(!g.lock || !g.ring_lock || !g.memory_lock)return kIOReturnNoMemory;
    const NativeAdrenoSubmitLocks locks={nullptr,lockHook,unlockHook};
    const IOReturn attached=submit->attachLocks(&locks);
    if(attached)return attached;
    /* An unmap waits only for the buffer's own last use (NativeAdrenoVm::noteUse): the VM reads the CP's completed fence. */
    if(ring->memptrs())vm->setFenceSource(ring->memptrs()+1);
    NativeAdrenoGpuPublished *service=new NativeAdrenoGpuPublished;
    if(!service || !service->init() || !service->attach(IOService::getPlatform())) {
        if(service)service->release();
        submit->attachLocks(nullptr);
        return kIOReturnNoMemory;
    }
    g.vm=vm;g.submit=submit;g.ring=ring;g.clock=*clock;g.timer=timer;g.info=info;g.windows=windows;
    g.input_bits=input_bits;g.output_bits=output_bits;g.service=service;
    /* The VM's phase and kind counters count from here on (pagani-gpu-kopt bit 5): the clients' mappings, which is what they are for. */
    native_adreno_vm_now_us=NativeAdrenoKoptOn(NativeAdrenoKoptVmClock)?nowUs:nullptr;
    gpuTraceInit();
    gpuTraceEvent(GpuTraceConfig,0,1,0,0,input_bits,output_bits,0);
    __atomic_store_n(&g.published,true,__ATOMIC_RELEASE);
    service->setName("NativeAdrenoGpu");
    service->setProperty("NativeAdrenoCoherentImports",(unsigned long long)native_adreno_coherent_imports(),32);
    service->setProperty(NativeAdrenoImportCleanKey,1ULL,32);
    service->registerService();
    return kIOReturnSuccess;
}
extern "C" void native_adreno_publish_status(const Adreno830Status *status)
{
    IOService *service=__atomic_load_n(&g.published,__ATOMIC_ACQUIRE)?g.service:nullptr;
    if(!service || !status)return;
    OSData *data=OSData::withBytes(status,sizeof(*status));
    if(!data)return;
    service->setProperty("Adreno830Status",data);
    data->release();
}
/* Copies at most Adreno830StatusRowMax characters of a NUL-terminated or full-length line. */
static void copyRow(char row[Adreno830StatusRowBytes],const char *text,size_t capacity)
{
    memset(row,0,Adreno830StatusRowBytes);
    size_t n=0;
    while(n<capacity && n<Adreno830StatusRowMax && text[n])++n;
    memcpy(row,text,n);
}
extern "C" unsigned native_adreno_report_rows(char rows[3][Adreno830StatusRowBytes])
{
    if(!rows)return 0;
    const NativeAdrenoMetalDiag &diag=native_adreno_metal_diag;
    /* Nothing reported and nobody opened: the diagnostic page stays. */
    if(!diag.reports && !diag.opens && !diag.report[0] && !diag.report2[0])return 0;
    copyRow(rows[0],diag.report[0]?diag.report:"-",sizeof diag.report);
    copyRow(rows[1],diag.report2[0]?diag.report2:"-",sizeof diag.report2);
    char line[Adreno830StatusRowBytes]={};
    snprintf(line,sizeof line,"CLI o%u c%u s%x NGP %u/%u/%u",diag.opens,diag.calls,diag.last_status,native_adreno_ngp_stats[0],
        native_adreno_ngp_stats[1],native_adreno_ngp_stats[2]);
    copyRow(rows[2],line,sizeof line);
    return 3;
}
extern "C" uint32_t native_adreno_coherent_imports(void)
{
    uint32_t level=NativeAdrenoVm::coherence();
    if(level>2)level=2;
    /* Level 2 is a promise about GPU-written imports and own BOs (the plugin skips their invalidates): only with CachedCoherent published. */
    if(level==2 && !(g.info && (g.info->flags&NativeAdrenoDeviceCachedCoherent)))level=1;
    return level;
}
extern "C" uint32_t native_adreno_publish_stats(uint32_t *closed)
{
    if(closed)*closed=g.closed;
    return g.opened;
}
extern "C" IOService *native_adreno_published_service()
{
    return g.service;
}
