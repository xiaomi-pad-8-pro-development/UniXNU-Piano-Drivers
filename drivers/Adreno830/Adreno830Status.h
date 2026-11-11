/* SPDX-License-Identifier: MIT */
#ifndef ADRENO830_STATUS_H
#define ADRENO830_STATUS_H
#include <IOKit/IOTypes.h>
#include <stddef.h>
#include <stdint.h>

/* The A830 status block (ARCHITECTURE 8.2): a fixed-layout global the driver only stores into. Nothing here logs or
 * formats on a hot path: the stages write fields, adreno830_status_publish() lays the overlay rows out when a stage
 * starts, ends, stops or fails (and the DCVS thread once a second), and hands them to the boot overlay (BootProgress)
 * and the registry property only when their text changed. */
enum A830Stage : uint8_t {A830SInputs,A830SPower,A830SSecureCx,A830SSmmu,A830SFirmware,A830SGmu,
                          A830SCp,A830SZap,A830SFence,A830SMetal,A830SDcvs,A830StageCount,A830SRunning=15};
enum A830State : uint8_t {A830Idle,A830Busy,A830Done,A830Failed,A830Stopped,A830Lost,A830Recovering,A830Slumber};
/* Reason codes (ARCHITECTURE 8.3), shown as r<n>. */
enum A830Reason : uint16_t {
    A830ReasonNone,A830ReasonArgument,A830ReasonFirmware,A830ReasonSocInfo,A830ReasonCmdDb,A830ReasonTimeout,
    A830ReasonUnexpectedState,A830ReasonReadback,A830ReasonSmc,A830ReasonRpmh,A830ReasonSmmuRoute,A830ReasonSmmuFault,
    A830ReasonNoMemory,A830ReasonGmuNmi,A830ReasonGmuFault,A830ReasonHfiStatus,A830ReasonF2hError,A830ReasonCpFault,
    A830ReasonHang,A830ReasonIrqWiring,A830ReasonPublish,A830ReasonStopped,
    A830ReasonGmuHostResource,   /* 22: the GMU asked for a host resource (F2H MEM_ALLOC / CNTR_REGISTER), unsupported in RB mode */
    A830ReasonIrqStorm,          /* 23: an interrupt was turned off by the storm guard (run-time event, see irq_off) */
    A830ReasonUnsupported,       /* 24: a boot argument asks for what this build does not have (pagani-gpu-hwsched=1) */
    A830ReasonCount
};
enum : uint32_t {Adreno830StatusMagic=0x41383330 /* 'A830' */,Adreno830StatusVersion=1};
enum : unsigned {Adreno830StatusRows=4,Adreno830StatusRowBytes=68,Adreno830StatusRowMax=66};   /* 1216 px / 18 px = 67 cells */
/* irq_off bits (8.2) and sticky[] indices. */
enum : uint32_t {
    A830IrqOffGpu=1u<<0,A830IrqOffHfi=1u<<1,A830IrqOffGmu=1u<<2,A830IrqOffSmmuCb0=1u<<3,A830IrqOffSmmuCb2=1u<<4,
    A830IrqOffCxHost=1u<<5,A830IrqOffScmWaitq=1u<<6,
};
enum : unsigned {A830StickySmmuGfsr,A830StickyCb0Fsr,A830StickyCb2Fsr,A830StickyGmuAo};
/* qmp (8.2). */
enum : uint8_t {A830QmpNotRun,A830QmpDone,A830QmpTimeout,A830QmpRefused};

/* ---- Diagnostic pages (rows 1-3 with pagani-gpu-status=2): the aux words of each stage (8.4). ----
 * A stage writes its own words; adreno830_status_begin() clears aux when the next stage shows another page. Values
 * that are not known yet print as "-" where a valid bit exists. */
/* S0 page. aux[0]: the valid bits below. aux[1]: ARC level counts, one byte each: gfx, mx, cx, gmxc (0xff = absent in
 * cmd-db). aux[2], aux[3]: BCM SH0 and MC0 as unit | width << 32. aux[4]: ACV vcd | CN0 vcd << 8. */
enum : uint64_t {
    A830S0SocId=1u<<0,A830S0FeatureCode=1u<<1,A830S0SpeedBin=1u<<2,
    A830S0FirmwareChecked=1u<<3,A830S0FirmwareOk=1u<<4,A830S0Arc=1u<<5,A830S0Bcm=1u<<6,
    A830S0SocInfoFallback=1u<<7,   /* the SMEM read failed and the board constants stood in */
};
/* S1/S2 page ("PWR", "SCM", "TCS"):
 *   aux[0] Sm8750GpuPowerTrace step | block << 8 | offset << 32      aux[1] its value | GPU_CC_PLL0 MODE << 32
 *   aux[2] GPU_INIT_REGS x0 (64 bits)                                 aux[3] its x1 (low 32) | SMC instructions << 32
 *   aux[4] SW_FUSE_VALUE | SW_FUSE_FREQ_LIMIT_STATUS << 32
 *   aux[5] the last TCS transaction: A830StatusTcs() packing
 *   valid bits in the top byte of aux[5] say which of the three groups was recorded (A830S1Power, A830S1Scm, A830S1Tcs). */
enum : uint64_t {A830S1Power=1ull<<61,A830S1Scm=1ull<<62,A830S1Tcs=1ull<<63};
static inline uint64_t A830StatusTcs(uint32_t slot,uint32_t status,uint32_t cmd0,uint32_t cmd1,uint32_t cmd2)
{
    return (uint64_t)(slot&0xf)|(uint64_t)(status&0xf)<<4|(uint64_t)(cmd0&0x1ffff)<<8|(uint64_t)(cmd1&0x1ffff)<<25|
           (uint64_t)(cmd2&0x1ffff)<<42;
}
/* S3 page ("SMMU ID", "ACR", "CB0"): aux[0] IDR0 | IDR1 << 32, aux[1] IDR2 | sCR0 << 32, aux[2] sACR | sGFSR << 32,
 * aux[3] S2CR of the GPU slot | (slots gpu | lpac << 8 | gmu << 16) << 32 | the CP_SMMU_APERTURE outcome (A830StatusAperture,
 * bits 56-58) (A830Smmu::fillStatus), aux[4] CB0 SCTLR | ACTLR << 32, aux[5] CB2 SCTLR | ACTLR << 32. The FSRs shown are
 * sticky[A830StickyCb0Fsr/Cb2Fsr], the faults smmu_faults. The ACR row ends with " AP<state>" once fillStatus recorded the
 * aperture (0 not run yet, 1 granted, 2 refused: A830Smmu::apertureState, M14 P0). */
enum : unsigned {A830StatusApertureShift=56};
static inline uint64_t A830StatusAperture(uint32_t state)
{
    return (uint64_t(state<3?state:3)|4u)<<A830StatusApertureShift;   /* bit 58: recorded */
}
/* S4 page ("CXT", "GMU", "SQE"): aux[0] the CX always-on counter written, aux[1] the measured skew in ticks,
 * aux[2] GMU blocks | DDR bytes << 32, aux[3] SQE bytes | AQE bytes << 32, aux[4] TCM bytes. */
/* S5 page ("GMU FW", "HFI", "OOB"): aux[0] core version | HFI version << 32, aux[1] CM3_FW_INIT_RESULT | init ms (16 bits,
 * saturating) << 32 | TCM words read back (16 bits, saturating) << 48, aux[2] HFI status M1 | M2 << 16 | M6 << 32 | M8 << 48,
 * aux[3] M9 | M5 << 16 | OOB ack us << 32, aux[4] GFX_PWR_CLK_STATUS | RPMH_POWER_STATE << 32,
 * aux[5] RSC_STATUS0_DRV0 at the cold boot | GMU2HOST_INTR_INFO << 32. The AO status shown is sticky[A830StickyGmuAo]; GENERAL_11
 * is gmu_general11 and the late HFI drain hfi_late (both global, below). The annibale fields (TCM words, GENERAL_11, RSC status)
 * show only once S0 accepted the annibale profile: the design profile does not read them.
 * HFI statuses are 16 bits; A830HfiNotSent marks a message not sent (or not answered). */
enum : uint16_t {A830HfiNotSent=0xffff};
/* hfi_late: the annibale profile's late HFI drain (A830Hfi::drainLate, field-notes 9-1), counted since the boot: drains |
 * messages found << 8 | late ACKs << 16 | late F2H_MSG_ERR << 24, each byte saturating at 255. Written before the failure of
 * S5 (a failed HFI command drains itself), S6-S8 (the orchestrator drains first) and a hang verdict is published; shown as
 * "L<messages>/<acks>/<errors>" on the S5, S6/S7 and S8 pages once a drain ran. */
static inline uint32_t A830StatusLate(uint32_t drains,uint32_t messages,uint32_t acks,uint32_t errors)
{
    return (drains<255?drains:255)|(messages<255?messages:255)<<8|(acks<255?acks:255)<<16|(errors<255?errors:255)<<24;
}
/* S6/S7 page ("CP", "SQE", "ZAP"): aux[0] CP_RB_RPTR | CP_RB_WPTR << 32, aux[1] RBBM_STATUS | CP_HW_FAULT << 32,
 * aux[2] SQE PC BR | BV << 32, aux[3] CP_PROTECT_STATUS | GMU_RBBM_INT_UNMASKED_STATUS << 32 (A830Cp::dumpStall),
 * aux[4] ZAP x0 of the deciding SMC (64 bits), aux[5] its x1[31:0] | step << 32 | WAITQ sleeps (<= 255) << 40 |
 * SMC instructions (<= 0xffff) << 48 (A830Zap::noteStatus). The ZAP row ends with the late HFI drain (hfi_late) once one ran. */
/* S8 page ("IRQ 332", "COH", "TS"): aux[0] GIC readback | wake us << 32, aux[1] 332 count, aux[2] coherence results
 * (2 bits each: V4, V9a, V9b, V9c from bit 0; A830Coh*), aux[3] CP always-on counter, aux[4] GMU AO counter,
 * aux[5] scratch readback | self-test fence << 32. The COH row ends with the late HFI drain (hfi_late) once one ran. */
enum : uint8_t {A830CohNotRun,A830CohPass,A830CohFail};
/* S9 page: aux[0] native_adreno_publish | publishMetal << 32 (IOReturns), aux[1] publishMetal step | IOGPUFamily load
 * result << 32; generic layout. */
/* S10 and run page ("DCVS", "MEM", "IRQ"): aux[0] DCVS min MHz | max MHz << 16 | first level shown << 32 | level count
 * << 40 | the performance knobs << 48 (A830StatusKnobs: pagani-gpu-maxlevel bits 48-55, pagani-gpu-irq bit 56); aux[1..3]
 * residency per level in permille, 10 bits each, six per word (level i: aux[1 + i / 6] bits (i % 6) * 10; at most 16 levels,
 * aux[3] bits 40-50 hold the design profile's thermal loop, A830StatusThermal below); aux[4] pool
 * MiB | pool MiB in use << 16 | coherence level << 32 | GPU VM objects (15 bits) << 34 | MiB mapped in the GPU VM (15 bits,
 * adreno830_status_vm; the MEM row ends with " vm <MiB>" once it is not 0, then with the GPU clients the DCVS thread stored,
 * Adreno830StatusClients below) << 49; aux[5] the slumber row (seconds | wakes << 24 | last wake
 * ms << 48). Row 3 uses the run-time counters and, once the DCVS thread stored one, the ring's queue period
 * (Adreno830StatusQueue below, kept outside the block). The RUN row (row 0) ends with " m<maxlevel>" and " i<1|0>" after the
 * profile when those knobs are set (i1: 332 live, i0: given but polled, irq_off bit 0), each only when it fits whole, then
 * " <t>C", the hottest GPU sensor in whole degrees (aux[0] bits 57-63, A830StatusTemp; 0 = no valid reading yet). */
enum : unsigned {A830StatusKnobsShift=48,A830StatusTempShift=57};
static inline uint64_t A830StatusKnobs(uint32_t maxlevel,uint32_t irq)
{
    return ((uint64_t)(maxlevel<255?maxlevel:255)|(uint64_t)(irq?1:0)<<8)<<A830StatusKnobsShift;
}
static inline uint64_t A830StatusTemp(int32_t deci)
{
    const int32_t celsius=deci>0?(deci+5)/10:0;
    return (uint64_t)(celsius<127?celsius:127)<<A830StatusTempShift;
}
/* The design profile's thermal loop (A830Dcvs, ARCHITECTURE 4.16) in the run page's aux[3] bits 40-50 (the residency of levels
 * 12-15 uses bits 0-39, so the run page shows at most 16 levels): the target in whole degrees C (bits 40-46, 0 = none: the
 * field is left out), the trend of the filtered reading (bits 47-48, A830StatusTrend*) and A830StatusThermalEmergency (bit 49).
 * The DCVS row shows it after the ceiling: "DCVS <min>-<max> T<target><?|=|+|->[!] t<residency>" ('?' no usable reading, '='
 * steady, '+' rising, '-' falling, '!' the 105 C emergency). */
enum : unsigned {A830StatusThermalShift=40};
enum : uint32_t {A830StatusTrendNone,A830StatusTrendSteady,A830StatusTrendRising,A830StatusTrendFalling};
enum : uint32_t {A830StatusThermalEmergency=1u<<0};
static inline uint64_t A830StatusThermal(int32_t target_deci,uint32_t trend,uint32_t flags)
{
    const int32_t celsius=target_deci>0?(target_deci+5)/10:0;
    return ((uint64_t)(celsius<127?celsius:127)|(uint64_t)(trend&3)<<7|(uint64_t)(flags&1)<<9)<<A830StatusThermalShift;
}
/* The one ring's queue over the last status period (A830Fence's queue meter, ARCHITECTURE 4.13.5): every kernel and framed
 * submission is stamped right before the barrier and WPTR write that hand it to the CP, every completion measured when the
 * fence interrupt's top half (or, while the waits poll, a waiter or the DCVS thread) first sees its seqno in memptrs. The
 * DCVS thread stores one period a second (adreno830_status_queue); it is not part of the 576-byte block, only of rows 2 and 3
 * of the run page: row 3 ends with " | q <avg>/<max>ms d<depth> s<slow>[ sw<contexts>]" (" | q~..." polled, " | q -" idle,
 * "-/-" when nothing completed; sw only when it fits whole), the MEM row (row 2) shows " fw<fw max>ms/<fw over 4 ms>" before its
 * client figures while external work delayed a render-server job (fw max not 0). The job attribution (preemption phase 0,
 * ARCHITECTURE 4.17): a job's service time runs from the later of its stamp and the previous completion to its own; fw is,
 * for a render-server job, the service time of the external jobs still in the ring when it was stamped, without what the
 * oldest of them had run by then. The ramoops meter line (adreno830_status_meter) has every figure. */
struct Adreno830StatusQueue {
    uint32_t submitted;       /* submissions handed to the CP in the period */
    uint32_t completed;       /* completions measured in the period (their stamp was found) */
    uint32_t missed;          /* completions without their stamp (more than A830FenceQueueStamps in flight): in no field below */
    uint32_t avg_us,max_us;   /* stamp to completion seen, over the measured completions */
    uint32_t slow;            /* measured completions that took longer than 16 ms */
    uint32_t depth;           /* the deepest ring a submission saw: its seqno - the completed fence (1: nothing ahead of it) */
    uint32_t polled;          /* 1: completions were seen without the fence interrupt (waits poll): avg/max and rs are upper bounds,
                                 fw and n4 may be low (ARCHITECTURE 4.13.5) */
    uint32_t rs_jobs;         /* render-server (backboardd) completions measured */
    uint32_t rs_avg_us,rs_max_us;      /* their service time */
    uint32_t fw_jobs;         /* of them, those whose external time ahead (fw) is known */
    uint32_t fw_avg_us,fw_max_us;      /* that time */
    uint32_t fw_over4;        /* fw above 4 ms (n4) */
    uint32_t ext_jobs;        /* completions of other processes' jobs measured */
    uint32_t ext_busy_us;     /* their service time in total */
    uint32_t contexts;        /* submissions that entered a new context (a client switch, the first after a reset) */
    uint32_t ext_max_us;      /* the longest service time of those jobs of other processes */
};
/* The calls into the GPU connections since the last tick (the call meter, NativeAdrenoMeter.h, pagani-gpu-kopt bit 4; only the ramoops
 * meter line shows them, ARCHITECTURE 8.8): the longest call of each kind as its caller saw it (every wait inside it included), the
 * longest wait for each lock (0 while every lock was free; the VM's needs pagani-gpu-kopt bit 5 too), the maps of imports and those
 * that took preflight's alias pass with their time, the time of the VM's table transactions (bit 5), the zero fill of pooled BOs, the
 * contiguous attempts of BOs outside the pool and how many BOs went there at all (above 128 KiB they make no attempt: scattered pages). */
enum : unsigned {Adreno830CallAlloc,Adreno830CallMap,Adreno830CallFree,Adreno830CallImport,Adreno830CallSubmit,Adreno830CallKinds};
enum : unsigned {Adreno830WaitConnection,Adreno830WaitRing,Adreno830WaitMemory,Adreno830WaitVm,Adreno830WaitKinds};
/* The classes the per-second line counts a connection's calls by (NativeAdrenoCallClassOf, NativeAdrenoMeter.h): the five kinds above,
 * WaitFence, the CPU maps and unmaps, everything else (timestamps, status, metadata, queues). */
enum : unsigned {Adreno830ClassAlloc,Adreno830ClassMap,Adreno830ClassFree,Adreno830ClassImport,Adreno830ClassSubmit,Adreno830ClassWait,
                 Adreno830ClassCpu,Adreno830ClassOther,Adreno830CallClasses};
struct Adreno830StatusCalls {
    uint32_t max_us[Adreno830CallKinds];
    uint32_t wait_us[Adreno830WaitKinds];
    uint32_t import_maps,shared_maps,shared_us;
    uint32_t map_us,unmap_us;
    uint32_t zero_kib,zero_us;
    uint32_t contig_tries,contig_fails,outside_bos;
    uint32_t fence_us;        /* the longest WaitFence as its caller saw it (the per-second line only) */
};
/* The GPU clients as the publish shell counts them once a second (native_adreno_client_tick, ARCHITECTURE 8.4). Not part of the
 * 576-byte block: laid out at the end of the run page's MEM row as " sc<MiB>" (own BOs of every client outside the BO pool:
 * contiguous or scattered fallbacks, which wire ordinary memory, change page attributes page by page and take one GPU span per
 * page; left out while 0) and " <name>:<MiB>" (the client holding the most GPU memory, own BOs and imports, with its process name
 * cut to the room left; "~" before it: a closed connection whose objects are still held, an orphan). */
struct Adreno830StatusClients {
    char top[20];             /* process name of the client holding the most GPU memory (NUL terminated; "" none) */
    uint32_t top_mib;         /* its own BOs (pool and fallbacks) and imports, MiB rounded up */
    uint32_t top_import_mib;  /* of which imports */
    uint32_t top_orphan;      /* 1: it is an orphan (its connection closed while objects could not be released yet) */
    uint32_t fallback_mib;    /* own BOs of every client and orphan outside the BO pool, MiB rounded up */
    uint32_t clients,orphans; /* live GPU connections, orphaned cores */
    /* Slot pressure and page-fault attribution since the boot (M14 P0; only the ramoops meter line shows them): the most live
     * connections at once, opens refused for want of a free slot, BoMaps refused outside their window, BoMaps refused for the
     * render server's reserve of the shared VM (its mapping records, its table ranges: raising one does nothing for the other), and the
     * CB0 page faults by how the jobs in flight compared with the FAR window,
     * whose connection is the one charged (alike; apart: one other connection alone in flight or FAR in no window; undecided:
     * nothing or several in flight; polled: no fences at the fault). */
    uint32_t peak_clients,slot_refusals,window_overflows,reserve_objects,reserve_spans;
    uint32_t faults_alike,faults_apart,faults_undecided,faults_polled;
    Adreno830StatusCalls calls;   /* since the last tick: adreno830_status_clients folds it into the meter line's window */
    /* Calls into the driver since the last tick (the per-second line, ARCHITECTURE 8.8): all of them, a closed connection's included,
     * and the live connection with the most: its process name, its count and its calls by class. */
    uint32_t second_calls,second_top_calls;
    char second_top[20];
    uint32_t second_classes[Adreno830CallClasses];
};

/* The render server's frontend since the last per-second line (2026-11-01, tunnel method NativeAdrenoFrontendMethod, ARCHITECTURE 8.8; the
 * eight words of AdrenoMetal's FrontendSecond.h): what it did on the CPU that the GPU's figures cannot show. Pipeline states made and their
 * wall time, VkPipeline makings (at draws) and theirs, the surface cleans at commits; of each the sum and the longest single one, microseconds. */
struct Adreno830StatusFrontend {
    uint32_t states,state_us,state_max_us;
    uint32_t makings,making_us,making_max_us;
    uint32_t clean_us,clean_max_us;
};

struct Adreno830Status {
    uint32_t magic;                 /* Adreno830StatusMagic */
    uint16_t version,bytes;         /* Adreno830StatusVersion, sizeof */
    uint8_t stage,state;            /* A830Stage, A830State */
    uint16_t step;                  /* step within the stage (ARCHITECTURE 8.3) */
    uint16_t reason,attempts;       /* A830Reason; GMU cold-boot attempts */
    uint32_t ioreturn;
    uint32_t hfi_late;              /* annibale: the late HFI drain, A830StatusLate() packing (was reserved0) */
    uint64_t reg,value,expect;      /* last physical address (or SMC function, HFI header) touched, value read, value expected */
    uint64_t aux[6];                /* stage specific, see the page layouts */
    uint32_t stage_us[A830StageCount],boot_ms;   /* time per stage, total (after a recovery: of the latest restart) */
    uint32_t seq;                   /* +1 whenever the rows change; the overlay redraws on it */
    uint32_t soc_id,feature_code,speed_bin,smmu_slots;
    uint32_t irq_off;               /* interrupts not wired or turned off by the storm guard: A830IrqOff* */
    uint32_t sticky[4];             /* record only, last non-zero: sGFSR, CB0 FSR, CB2 FSR, GMU AO status */
    uint8_t qmp,coh_limit;          /* QMP: 0 not run, 1 done, 2 timeout, 3 refused; coherence ceiling 0/1/2 */
    uint8_t profile,profile_set;    /* pagani-gpu-profile in effect (A830Profile.h), valid once S0 accepted it (profile_set 1) */
    uint32_t gmu_general11;         /* annibale: GENERAL_11 (0x3D7E72C) the last cold boot read before B6, 0 in RB mode (was
                                       reserved1, the padding before the 64-bit counters) */
    /* run-time counters (relaxed atomic adds from IRQ, submit and DCVS paths) */
    uint64_t submits,retired,irqs,fence_irqs,waits,wait_timeouts,late_wakeups;
    uint32_t faults,smmu_faults,gmu_faults,hangs,recoveries,hfi_errors,wptr_retries,underruns;
    uint32_t mhz,busy_permille,gmu_level,bus_level,votes,vote_failures;
    char rows[Adreno830StatusRows][Adreno830StatusRowBytes];   /* laid out rows: row 0 always, rows 1-3 the diagnostic page */
};
static_assert(offsetof(Adreno830Status,hfi_late)==20 && offsetof(Adreno830Status,reg)==24 &&
              offsetof(Adreno830Status,stage_us)==96 && offsetof(Adreno830Status,irq_off)==164 &&
              offsetof(Adreno830Status,qmp)==184 && offsetof(Adreno830Status,gmu_general11)==188 &&
              offsetof(Adreno830Status,submits)==192 && offsetof(Adreno830Status,rows)==304 &&
              sizeof(Adreno830Status)==576,"Adreno830Status layout is read by the plugin and host tools");

#ifdef __cplusplus
extern "C" {
#endif
extern Adreno830Status adreno830_status;
/* The overlay hooks of PD/drivers/BootProgress; this repository links weak empty defaults. */
void boot_progress_gpu_rows(const char (*rows)[Adreno830StatusRowBytes],unsigned count);
void boot_progress_display_status(const char *detail,uint64_t value);
/* BootProgress's persistent log (RamoopsLog.c, pagani-ramoops=1; PeripheralStatus.h): one line into the Android ramoops
 * region, in DRAM when the call returns; interrupt safe; a no-op while the log is off. The bring-up register trace
 * (A830Trace.h) writes through it only while boot_progress_trace_enabled() says the log runs. Weak defaults here: no line,
 * log off. */
void boot_progress_trace(const char *text);
int boot_progress_trace_enabled(void);
/* Hooks of NativeAdrenoPublish.cpp (weak empty defaults here): the registry property Adreno830Status of the published
 * service, refreshed whenever the rows change (a no-op until S9 published it), and the user-space report page of
 * pagani-gpu-status=3 (the plugin's two latest report rows and the client summary; returns the rows filled, 0..3). */
void native_adreno_publish_status(const Adreno830Status *status);
unsigned native_adreno_report_rows(char rows[3][Adreno830StatusRowBytes]);
/* Once a second from the DCVS thread (the orchestrator's memory row), thread context: the publish shell retries the teardown of
 * orphaned cores while the GPU is awake and fills *clients (Adreno830StatusClients); returns 1, or 0 before S9 published the shell
 * (the weak default here: no client figures). */
unsigned native_adreno_client_tick(Adreno830StatusClients *clients);
#ifdef __cplusplus
}
#endif

/* Overlay mode = pagani-gpu-status: 0 never hand rows over, 1 row 0 only, 2 row 0 plus the diagnostic page, 3 row 0 plus
 * the user-space report page (the diagnostic page while the plugin has reported nothing). */
void adreno830_status_mode(unsigned mode);
unsigned adreno830_status_current_mode(void);
/* pagani-diag (S0, before anything logs): adreno830_status_diag_lines() is what the run-time lines of the test images ask, the per-second
 * line, the slot refusal and the first address past 4 GiB (ARCHITECTURE 8.8, 6.2): the switch, a diagnostic page and BootProgress's log
 * running. Without the switch they stay out of the 32 KiB events ring, whose one-time lines a loop image without the UFS log still needs. */
void adreno830_status_diag(bool on);
bool adreno830_status_current_diag(void);
bool adreno830_status_diag_lines(void);
/* Back to Idle with zeroed fields (kept: magic, version, bytes, the mode, seq); the start of a boot. */
void adreno830_status_reset(void);
void adreno830_status_begin(uint8_t stage);
void adreno830_status_step(uint16_t step);                        /* store only; no publish */
void adreno830_status_end(uint8_t stage,uint32_t elapsed_us);
void adreno830_status_stop(uint8_t stage,uint16_t reason);        /* A830ReasonStopped for pagani-gpu-stop */
void adreno830_status_fail(uint8_t stage,uint16_t step,uint16_t reason,IOReturn ior,uint64_t reg,uint64_t value,uint64_t expect);
/* Run time (thread context only: they publish). Running: S10 done ("GPU RUN ..."). Lost: a global device loss of
 * ARCHITECTURE 4.13.4 ("GPU LOST S<s>.<step> ..."; faults is counted by the caller). Recovering: a recovery re-runs stage
 * `stage`, elapsed_ms since it started ("GPU RECOVER <n> S<s> <ms>ms"; recoveries is counted by the caller). Slumber
 * (P1): "GPU SLP <s>s w<wakes> <ms>ms". */
void adreno830_status_running(void);
void adreno830_status_lost(uint8_t stage,uint16_t step,uint16_t reason,IOReturn ior);
void adreno830_status_recovering(uint8_t stage,uint32_t elapsed_ms);
void adreno830_status_slumber(uint32_t seconds,uint32_t wakes,uint32_t last_wake_ms);
/* Interrupt-safe stores (no publish; the next thread-context publish shows them). */
void adreno830_status_irq_off(uint32_t bits);                     /* A830IrqOff* */
void adreno830_status_sticky(unsigned index,uint32_t value);      /* keeps the last non-zero value */
static inline void adreno830_status_add32(uint32_t *counter,uint32_t n) {__atomic_fetch_add(counter,n,__ATOMIC_RELAXED);}
static inline void adreno830_status_add64(uint64_t *counter,uint64_t n) {__atomic_fetch_add(counter,n,__ATOMIC_RELAXED);}
/* The performance knobs S0 accepted (pagani-gpu-maxlevel, pagani-gpu-irq): kept until the next reset and written into the
 * run page's aux[0] (A830StatusKnobs) by every adreno830_status_dcvs. Store only. */
void adreno830_status_knobs(uint32_t maxlevel,uint32_t irq);
/* The DCVS thread's last valid GPU temperature (deci-degrees C): carried into the run page by the next adreno830_status_dcvs. */
void adreno830_status_temperature(int32_t deci);
/* The design profile's thermal loop (DCVS thread, once a second before the DCVS row): target, trend (A830StatusTrend*) and
 * A830StatusThermal* flags, carried into the run page's aux[3] (A830StatusThermal) by the next adreno830_status_dcvs. Store
 * only; a reset forgets it (the annibale profile never stores one: its DCVS row stays as it was). */
void adreno830_status_thermal(int32_t target_deci,uint32_t trend,uint32_t flags);
/* Bytes mapped in the GPU VM (every live BO: own, pooled, imported, scanout): carried into the MEM row by the next
 * adreno830_status_memory, so a leak of GPU memory shows as a "vm" that only grows. */
void adreno830_status_vm(uint64_t bytes);
/* The ring's queue period (DCVS thread, once a second, A830Fence::takeQueue): kept until the next reset and laid out by every
 * later publish at the end of the run page's row 3. Store only; null forgets it (row 3 goes back to its "wptr" layout). */
void adreno830_status_queue(const Adreno830StatusQueue *queue);
/* The GPU clients (DCVS thread, once a second, native_adreno_client_tick): carried into the MEM row by the next
 * adreno830_status_memory, kept until the next reset; their calls go into the meter line's window here, once. Store only; null
 * forgets them (the MEM row ends at its vm field). */
void adreno830_status_clients(const Adreno830StatusClients *clients);
/* The CP_SMMU_APERTURE outcome (A830Smmu::apertureState: 0 not run, 1 granted, 2 refused), stored by S3 for the meter line's
 * " ap<state>" (M14 P0). Store only; a reset forgets it (" ap-"). */
void adreno830_status_aperture(uint32_t state);
/* The ramoops meter line (M14 P0, ARCHITECTURE 8.8): the DCVS thread calls it once a second after storing the queue period and the
 * client figures. It folds the stored period into a window and, every Adreno830StatusMeterPeriods calls (a minute), while the overlay
 * shows a diagnostic page (pagani-gpu-status 2 or 3; the default 1 logs nothing new) and BootProgress's log runs
 * (boot_progress_trace_enabled), writes one line through boot_progress_trace when its text differs from the last one written:
 * "gpu meter rs <n>[ <avg>/<max>ms] fw <n>[ <avg>/<max>ms] n4 <n> ext <n> <busy>ms ctx <n> <max>/s poll <n>[ conn <live>/<peak> nos <n>
 * ovf <n> vmr <objects>/<spans> pf <alike>/<apart>/<undecided>/<polled>][ ca <alloc>/<map>/<free>/<import>/<submit>][ lk <conn>/<ring>/<memory>/<vm>]
 * [ im <maps>/<alias pass>/<their ms>][ vt <map ms>/<unmap ms>][ zf <MiB>/<ms>][ cs <tries>/<failed>/<BOs outside the pool>] ap<state|->", times in ms with one
 * decimal (averages to the nearest 0.1 ms, maxima up) up to ap's bracket, whole ms rounded up in the call figures (each of those
 * brackets left out while it is all 0, all of them with pagani-gpu-kopt bit 4 off), poll the window's periods whose completions were
 * seen without the fence interrupt (their rs and latencies are upper bounds, fw and n4 may be low: ARCHITECTURE 4.13.5), at most
 * Adreno830StatusMeterBytes - 1 characters (BootProgress continues a line past 200 characters in a '+' record). Nothing is drawn; thread
 * context only. */
enum : unsigned {Adreno830StatusMeterPeriods=60,Adreno830StatusMeterBytes=320};
/* The same call also writes the per-second line (2026-10-31, ARCHITECTURE 8.8), while adreno830_status_diag_lines() (pagani-diag, a
 * diagnostic page, the log running), for a second with a submission or at least Adreno830StatusSecondBurst calls into the driver: what the
 * minute's window cannot tell apart: "gpu sec rs <n>[ <longest>ms] ext <n>[ <longest>ms][ q <longest>ms d<depth>][ fw <longest>ms][ wf <longest>ms] cl <calls>[ ca
 * <alloc>/<map>/<free>/<import>/<submit>][ vt <map ms>/<unmap ms>][ im <import maps>][ pl <states> <ms>/<longest ms>][ vk <makings>
 * <ms>/<longest ms>][ cln <ms>/<longest ms>][ top <name> <calls> <class><n>...]": the service times of the second's longest render-server
 * and external job, its longest stamp to completion and the deepest ring, the longest external time ahead of a render-server job, the
 * longest fence wait, the calls of all connections, the second's longest call of each kind and VM table time, what the render server's
 * frontend did on the CPU since the last line (Adreno830StatusFrontend: pipeline states made, VkPipeline makings, surface cleans, each
 * bracket left out while it is all 0), and the process with the most calls with its classes (al mp fr im su wf cp ot, those not 0). Times
 * in ms with one decimal rounded up, the call and frontend figures in whole ms rounded up; ca, vt, im and wf need pagani-gpu-kopt bit 4,
 * vt bit 5 too. At most Adreno830StatusMeterBytes - 1 characters; a busy second's is about 120, so about 150 bytes a second through the
 * events ring, which the UFS log's rounds (sheng-log=1) keep from being lost: without that log the ring holds about three minutes of them. */
enum : unsigned {Adreno830StatusSecondBurst=1000};
void adreno830_status_meter(void);
/* The render server's report (the tunnel method, thread context): added up until a per-second line takes it; a second without a line
 * passes it on to the next. Sums saturate at 32 bits. Store only. */
void adreno830_status_frontend(const Adreno830StatusFrontend *frontend);
/* The run page inputs (DCVS thread, once a second): frequency, busy, levels and the residency of levels
 * first..first+count-1 in permille (count <= 16), and the memory row. They publish only when the text changed. */
void adreno830_status_dcvs(uint32_t mhz,uint32_t busy_permille,uint32_t gmu_level,uint32_t bus_level,uint32_t min_mhz,
                           uint32_t max_mhz,unsigned first,unsigned count,const uint16_t *residency_permille);
void adreno830_status_memory(uint32_t pool_mib,uint32_t used_mib,uint32_t imports,uint32_t coherent);
/* Pure: lays out the rows a status block shows in this mode and returns how many rows the overlay gets (0, 1 or 4).
 * Every row is NUL terminated and at most Adreno830StatusRowMax characters. Once S0 accepted the profile (profile_set),
 * row 0 ends with " p<profile>" (p1 annibale, p0 design) in every state, as long as the row has room for it: only the
 * longest FAIL and RUN rows (64-66 characters) leave it out. The RUN row then adds the performance knobs of the run page
 * (" m<n>", " i<1|0>"), each when it fits whole. */
unsigned adreno830_status_format(const Adreno830Status *status,char rows[Adreno830StatusRows][Adreno830StatusRowBytes],unsigned mode);
/* The same with the ring's queue period (null: exactly the layout above). Only the run page's rows 2 and 3 change. Row 3: "IRQ
 * <n> wait <n> to <n> late <n> | q <avg>/<max>ms d<depth> s<slow>" (wptr left out; it stays in the block); the four counters
 * take at most 6, 6, 5 and 4 characters (in full while they fit, then in k, M, ... rounded down) and avg <= 999, max <= 9999,
 * depth and slow <= 999, so the worst case is exactly 66 characters. avg is rounded to the nearest ms, max up (a period
 * whose max reads 16ms has no slow completion). " sw<contexts>" (<= 9999) follows when the period entered a new context and the
 * row has room for it whole. Row 2 (MEM): " fw<fw max>ms/<n4>" after " vm" while fw max is not 0 (fw max rounded up, <= 9999;
 * n4 <= 999: at most 13 characters, so the row stays within 66 and the client figures take what is left). */
unsigned adreno830_status_format(const Adreno830Status *status,char rows[Adreno830StatusRows][Adreno830StatusRowBytes],unsigned mode,
                                 const Adreno830StatusQueue *queue);
/* The same with the GPU clients as well (null: the layout above). Only the run page's MEM row changes: after " vm <MiB>" (and the
 * queue period's " fw...") it ends with " sc<MiB>" (own BOs outside the BO pool, while not 0 and while it fits whole) and the largest
 * client " <name>:<MiB>" or " <name>:<own>+<imports>" ("~" before the name for an orphan), the name cut to the room left and the field
 * left out whole below three characters of it, so the row stays within 66 characters with every field whole. */
unsigned adreno830_status_format(const Adreno830Status *status,char rows[Adreno830StatusRows][Adreno830StatusRowBytes],unsigned mode,
                                 const Adreno830StatusQueue *queue,const Adreno830StatusClients *clients);
/* Lays out adreno830_status.rows; when the text changed, seq += 1, (mode != 0) the rows go to boot_progress_gpu_rows
 * and the registry property is refreshed. Thread context only. */
void adreno830_status_publish(void);
/* Short overlay name of a stage (INPUTS, POWER, ...), and the long name used in the boot stage history. */
const char *adreno830_stage_name(uint8_t stage);
const char *adreno830_stage_title(uint8_t stage);
#endif
