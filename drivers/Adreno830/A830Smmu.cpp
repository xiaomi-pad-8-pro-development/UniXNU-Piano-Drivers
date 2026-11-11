/* SPDX-License-Identifier: MIT */
#include "A830Smmu.h"
#include "A830Regs.h"
#include <string.h>
#include <IOKit/IOLib.h>
#include <kern/clock.h>

/* ARCHITECTURE 4.5 (survey, take-over, LPAC, aperture, release), 4.6.4 (TLB maintenance budget), 4.12 (context-fault
 * interrupts, storm guard, polling), 4.17 (restore after a CX collapse). Every register access goes through io_; nothing here
 * gates progress on a register the hypervisor or the firmware controls (sGFSR, sACR, ACTLR, FSR): those are recorded.
 * The annibale profile (A830Profile.h, the header's profile notes) changes the context values, the ACTLR banks, the TLB
 * invalidation, the global writes and the interrupts; everything else is the same in both profiles. */

namespace {
constexpr uint32_t kFault=A830_SMMU_FSR_FAULT_MASK;
constexpr uint32_t kSmrValid=A830_SMMU_SMR_VALID;
constexpr uint64_t kPhysMask=UINT64_C(0x0000fffffffff000);
constexpr unsigned kMaxSmrs=256;
constexpr unsigned kSyncMaxPolls=1u<<20;   /* a clock that never advances must still end the wait */
inline uint32_t lo32(uint64_t v) {return uint32_t(v);}
inline uint32_t hi32(uint64_t v) {return uint32_t(v>>32);}
/* The kit's encoder output uses these TCR fields for the walk of one region: TxSZ [5:0], IRGNx/ORGNx/SHx [13:8]. */
constexpr uint32_t kTcrWalk0=0x3F3Fu;
}

int A830Smmu::read32(uint32_t offset,uint32_t *value) const
{
    *value=0;
    return io_.read32(io_.cookie,offset,value);
}
int A830Smmu::write32(uint32_t offset,uint32_t value) const
{
    ++writes_;
    return io_.write32(io_.cookie,offset,value);
}
int A830Smmu::barrier() const {return io_.barrier(io_.cookie);}

uint16_t A830Smmu::stepBase(unsigned cb,uint16_t sid) const
{
    if(cb==config_.cb_gpu && sid==config_.sid_lpac)return A830SmmuStepLpac;
    if(cb==config_.cb_gpu)return A830SmmuStepGpu;
    if(cb==config_.cb_gmu)return A830SmmuStepGmu;
    return 0x400;
}
IOReturn A830Smmu::fail(uint16_t step,uint16_t reason,uint32_t offset,uint64_t value,uint64_t expect,IOReturn result)
{
    failure_={step,reason,config_.phys+offset,value,expect};
    return result;
}
A830Smmu::Bank *A830Smmu::bank(unsigned cb)
{
    if(!surveyed_ || cb>=MaxBanks || cb>=survey_.banks)return nullptr;
    return &banks_[cb];
}
const A830Smmu::Bank *A830Smmu::bankState(unsigned cb) const {return cb<MaxBanks?&banks_[cb]:nullptr;}
int A830Smmu::streamIndex(const A830SmmuStream &stream) const
{
    for(size_t i=0;i<stream_count_;++i)if(streams_[i].slot==stream.slot)return int(i);
    return -1;
}
void A830Smmu::noteSticky(unsigned cb,uint32_t value)
{
    /* A context FSR keeps only its fault bits (bits [10:9] are the format field, not a fault). */
    if(cb!=0xFFu)value&=kFault;
    if(!value || !config_.status)return;
    const unsigned word=cb==0xFFu?0:cb==config_.cb_gpu?1:cb==config_.cb_gmu?2:4;
    if(word<4)__atomic_store_n(&config_.status->sticky[word],value,__ATOMIC_RELAXED);
}
/* T1SZ/IRGN1/ORGN1/SH1 as the region-0 walk (ARCHITECTURE 4.5.4 step 7: CB0 0xB5A03520 from the kit's 0x80803520), TG1 4K,
 * EPD1 kept: the guard table is only walked once a CP cleared EPD1. */
uint32_t A830Smmu::tcrWithUpper(uint32_t tcr)
{
    return (tcr&~(kTcrWalk0<<16))|((tcr&kTcrWalk0)<<16)|A830_SMMU_TCR_EPD1|A830_SMMU_TCR_TG1_4K;
}

IOReturn A830Smmu::init(const A830SmmuConfig &config)
{
    if(initialized_)return kIOReturnBusy;
    const ArmSmmu4KIo *io=config.io;
    const bool streams=config.sid_gpu!=config.sid_lpac && config.sid_gpu!=config.sid_gmu && config.sid_lpac!=config.sid_gmu &&
        config.sid_gpu<=A830_SMMU_SMR_ID_MASK && config.sid_lpac<=A830_SMMU_SMR_ID_MASK && config.sid_gmu<=A830_SMMU_SMR_ID_MASK &&
        config.slot_gpu!=config.slot_lpac && config.slot_gpu!=config.slot_gmu && config.slot_lpac!=config.slot_gmu &&
        config.slot_gpu<128 && config.slot_lpac<128 && config.slot_gmu<128;
    if(!io || !io->read32 || !io->read64 || !io->write32 || !io->write64 || !io->barrier || io->register_bytes<0x2000 ||
        !config.platform.now_us || !config.platform.delay_us || config.cb_gpu==config.cb_gmu || config.cb_gpu>=MaxBanks ||
        config.cb_gmu>=MaxBanks || !streams || !config.actlr)return kIOReturnBadArgument;
    config_=config;io_=*io;
    /* One profile for the object's life: the restore replays, the VM's TLB I/O and the polling follow what S3 programmed. */
    annibale_=adreno830_annibale()!=0;sgfsr_seen_=0;
    for(unsigned i=0;i<MaxBanks;++i) {
        banks_[i]=Bank{};banks_[i].owner=this;banks_[i].cb=uint8_t(i);banks_[i].intid=i<4?config.irq_cb[i]:0;
    }
    table_io_={this,io_.register_bytes,tableRead32,tableRead64,tableWrite32,tableWrite64,tableBarrier};
    /* annibale: the VM also synchronizes the bank after each map (TLBIASID + TLBSYNC = 1 through table_io_, section 14 row 12). */
    programmer_={this,&table_io_,0,0,0,programmerClaim,programmerProgram,programmerRoute,programmerVerify,programmerFault,
                 programmerDisable,annibale_};
    initialized_=true;
    return kIOReturnSuccess;
}

IOReturn A830Smmu::survey(A830SmmuSurvey *out)
{
    if(!initialized_)return kIOReturnNotReady;
    A830SmmuSurvey s{};
    /* Step 1: the identification registers. */
    if(read32(A830_SMMU_GR0_IDR0,&s.idr0) || read32(A830_SMMU_GR0_IDR1,&s.idr1) || read32(A830_SMMU_GR0_IDR2,&s.idr2))
        return fail(A830SmmuStepIds,A830ReasonUnexpectedState,A830_SMMU_GR0_IDR0,0,0,kIOReturnIOError);
    if(!(s.idr0&A830_SMMU_IDR0_S1TS) || !(s.idr0&A830_SMMU_IDR0_SMS))
        return fail(A830SmmuStepIds,A830ReasonUnexpectedState,A830_SMMU_GR0_IDR0,s.idr0,A830_SMMU_IDR0_S1TS|A830_SMMU_IDR0_SMS,
                    kIOReturnUnsupported);
    if((s.idr1&A830_SMMU_IDR1_PAGESIZE_64K) || A830_SMMU_IDR1_NUMCB(s.idr1)<4)
        return fail(A830SmmuStepIds,A830ReasonUnexpectedState,A830_SMMU_GR0_IDR1,s.idr1,4,kIOReturnUnsupported);
    const uint32_t oas=A830_SMMU_IDR2_OAS(s.idr2);
    if(!(s.idr2&A830_SMMU_IDR2_PTFS_4K) || oas<1 || oas>5)
        return fail(A830SmmuStepIds,A830ReasonUnexpectedState,A830_SMMU_GR0_IDR2,s.idr2,A830_SMMU_IDR2_PTFS_4K|0x10u,
                    kIOReturnUnsupported);
    s.smrs=A830_SMMU_IDR0_NUMSMRG(s.idr0);
    s.banks=A830_SMMU_IDR1_NUMCB(s.idr1);
    s.global_pages=A830_SMMU_GLOBAL_PAGES(s.idr1);
    s.output_bits=A830_SMMU_OAS_BITS(oas);
    if(s.output_bits>A830_SMMU_OUTPUT_BITS_MAX)s.output_bits=A830_SMMU_OUTPUT_BITS_MAX;
    if(!s.smrs || s.banks>s.global_pages || uint64_t(s.global_pages+s.banks)*A830_SMMU_PAGE_BYTES>io_.register_bytes ||
        config_.cb_gpu>=s.banks || config_.cb_gmu>=s.banks)
        return fail(A830SmmuStepIds,A830ReasonUnexpectedState,A830_SMMU_GR0_IDR1,s.idr1,io_.register_bytes,kIOReturnUnsupported);
    /* Step 2: sCR0 is never written and no global reset is made; CLIENTPD would let every stream bypass. */
    if(read32(A830_SMMU_GR0_SCR0,&s.scr0))return fail(A830SmmuStepScr0,A830ReasonUnexpectedState,A830_SMMU_GR0_SCR0,0,0,kIOReturnIOError);
    if(s.scr0&A830_SMMU_SCR0_CLIENTPD)
        return fail(A830SmmuStepScr0,A830ReasonUnexpectedState,A830_SMMU_GR0_SCR0,s.scr0,0,kIOReturnUnsupported);
    if(!(s.scr0&A830_SMMU_SCR0_USFCFG))s.warnings|=A830SmmuWarnUsfcfg;
    /* Steps 3, 4: read only here (releaseCacheLock, clearGlobalFault). */
    if(read32(A830_SMMU_GR0_SACR,&s.sacr) || read32(A830_SMMU_GR0_SGFSR,&s.sgfsr))
        return fail(A830SmmuStepSacr,A830ReasonUnexpectedState,A830_SMMU_GR0_SACR,0,0,kIOReturnIOError);
    if(s.sacr&A830_SMMU_SACR_CACHE_LOCK)s.warnings|=A830SmmuWarnCacheLock;
    if(s.sgfsr)s.warnings|=A830SmmuWarnGlobalFault;
    /* Step 5: no valid SMR may cover one of our SIDs, no valid route may translate to CB0 or CB2 (both are reset next). */
    bool used[kMaxSmrs]={};
    if(s.smrs>A830SmmuMaxSmrs)s.warnings|=A830SmmuWarnSmrs;
    const uint16_t sids[3]={config_.sid_gpu,config_.sid_lpac,config_.sid_gmu};
    for(unsigned n=0;n<s.smrs;++n) {
        uint32_t smr=0,s2cr=0;
        if(read32(A830_SMMU_GR0_SMR(n),&smr) || read32(A830_SMMU_GR0_S2CR(n),&s2cr))
            return fail(A830SmmuStepSlots,A830ReasonUnexpectedState,A830_SMMU_GR0_SMR(n),0,0,kIOReturnIOError);
        if(n<A830SmmuMaxSmrs) {s.smr[n]=smr;s.s2cr[n]=s2cr;}
        if(!(smr&kSmrValid))continue;
        used[n]=true;
        /* piano: the bootloader leaves the GPU SMMU routed for the boot splash (XBL/ABL program the same streams this
         * driver wants), so an existing valid entry for our SIDs is the normal state here, not a conflict to refuse.
         * The driver owns this SMMU from S3 on and resets CB0/CB2 next, so the entry is taken down instead. */
        bool ours=false;
        for(uint16_t sid:sids)
            if(A830_SMMU_SMR_COVERS(smr,sid))ours=true;
        if(A830_SMMU_S2CR_TYPE(s2cr)==A830_SMMU_S2CR_TYPE_TRANS &&
            (A830_SMMU_S2CR_CBNDX(s2cr)==config_.cb_gpu || A830_SMMU_S2CR_CBNDX(s2cr)==config_.cb_gmu))ours=true;
        if(ours) {
            write32(A830_SMMU_GR0_S2CR(n),0);
            write32(A830_SMMU_GR0_SMR(n),0);
            s.warnings|=A830SmmuWarnSlotMoved;
            used[n]=false;
            s.smr[n]=0;s.s2cr[n]=0;
            continue;
        }
    }
    /* The board's preferred slots; a valid one moves to the lowest invalid slot, never 2 (TZ's secure stream). */
    const uint8_t preferred[3]={config_.slot_gpu,config_.slot_lpac,config_.slot_gmu};
    uint8_t chosen[3]={0xFF,0xFF,0xFF};
    auto taken=[&](unsigned slot) {for(uint8_t c:chosen)if(c==slot)return true;return false;};
    /* Every free preferred slot first, so a moved stream never takes another stream's preferred slot. */
    for(unsigned i=0;i<3;++i)
        if(preferred[i]<s.smrs && !used[preferred[i]] && !taken(preferred[i]))chosen[i]=preferred[i];
    for(unsigned i=0;i<3;++i) {
        if(chosen[i]!=0xFF)continue;
        unsigned slot=kMaxSmrs;
        for(unsigned n=0;n<s.smrs && n<128;++n)
            if(!used[n] && n!=A830_SMMU_SLOT_RESERVED && !taken(n)) {slot=n;break;}
        if(slot==kMaxSmrs)
            return fail(A830SmmuStepSlots,A830ReasonSmmuRoute,A830_SMMU_GR0_SMR(preferred[i]<s.smrs?preferred[i]:0),
                        preferred[i]<A830SmmuMaxSmrs?s.smr[preferred[i]]:0,preferred[i],kIOReturnNoResources);
        s.warnings|=A830SmmuWarnSlotMoved;
        chosen[i]=uint8_t(slot);
    }
    s.slot_gpu=chosen[0];s.slot_lpac=chosen[1];s.slot_gmu=chosen[2];
    /* Step 6: CB0..CB3 as found (random at boot on this SoC; nothing is required of them). */
    for(unsigned cb=0;cb<A830SmmuSurveyBanks && cb<s.banks;++cb) {
        const uint32_t off=(s.global_pages+cb)*A830_SMMU_PAGE_BYTES;
        uint32_t ttbr_lo=0,ttbr_hi=0;
        if(read32(off+A830_SMMU_CB_SCTLR,&s.cb_sctlr[cb]) || read32(off+A830_SMMU_CB_ACTLR,&s.cb_actlr[cb]) ||
            read32(off+A830_SMMU_CB_TCR,&s.cb_tcr[cb]) || read32(off+A830_SMMU_CB_TTBR0,&ttbr_lo) ||
            read32(off+A830_SMMU_CB_TTBR0_HI,&ttbr_hi) || read32(off+A830_SMMU_CB_FSR,&s.cb_fsr[cb]))
            return fail(A830SmmuStepBanks,A830ReasonUnexpectedState,off,0,0,kIOReturnIOError);
        s.cb_ttbr0[cb]=uint64_t(ttbr_hi)<<32|ttbr_lo;
        if((cb==config_.cb_gpu || cb==config_.cb_gmu) && (s.cb_sctlr[cb]&A830_SMMU_SCTLR_M))s.warnings|=A830SmmuWarnBankEnabled;
    }
    survey_=s;surveyed_=true;
    programmer_.id0=s.idr0;programmer_.id1=s.idr1;programmer_.id2=s.idr2;
    if(s.sgfsr)noteSticky(0xFFu,s.sgfsr);
    if(out)*out=s;
    return kIOReturnSuccess;
}

IOReturn A830Smmu::releaseCacheLock()
{
    if(!surveyed_)return kIOReturnNotReady;
    uint32_t sacr=0;
    if(read32(A830_SMMU_GR0_SACR,&sacr))return fail(A830SmmuStepSacr,A830ReasonUnexpectedState,A830_SMMU_GR0_SACR,0,0,kIOReturnIOError);
    /* Annibale: sACR is never written (kernel-sequences 14 #9: K90 wrote ACTLR 0x303 with whatever CACHE_LOCK held, and the
     * global space Gunyah emulates is best written as little as possible); the value is only recorded. */
    if(!annibale_ && (sacr&A830_SMMU_SACR_CACHE_LOCK)) {
        /* arm_mmu500_reset: CACHE_LOCK keeps ACTLR from being written. The emulation may drop the write: recorded only. */
        if(write32(A830_SMMU_GR0_SACR,sacr&~A830_SMMU_SACR_CACHE_LOCK) || barrier() || read32(A830_SMMU_GR0_SACR,&sacr))
            return fail(A830SmmuStepSacr,A830ReasonUnexpectedState,A830_SMMU_GR0_SACR,sacr,0,kIOReturnIOError);
    }
    sacr_after_=sacr;
    return kIOReturnSuccess;
}

void A830Smmu::globalCheck()
{
    uint32_t gfsr=0;
    if(read32(A830_SMMU_GR0_SGFSR,&gfsr))return;
    /* Annibale: never written back, so the value stays; only a change is recorded (and counted) as a new global fault. */
    if(annibale_ && __atomic_exchange_n(&sgfsr_seen_,gfsr,__ATOMIC_RELAXED)==gfsr)return;
    if(!gfsr)return;
    uint32_t far_lo=0,far_hi=0;
    (void)read32(A830_SMMU_GR0_SGFAR,&far_lo);(void)read32(A830_SMMU_GR0_SGFAR_HI,&far_hi);
    (void)read32(A830_SMMU_GR0_SGFSYNR0,&sgfsynr_[0]);(void)read32(A830_SMMU_GR0_SGFSYNR1,&sgfsynr_[1]);
    (void)read32(A830_SMMU_GR0_SGFSYNR2,&sgfsynr_[2]);
    sgfar_=uint64_t(far_hi)<<32|far_lo;
    /* Design: written back as read (arm_smmu_device_reset); the emulation may ignore it, the re-read is recorded. */
    if(!annibale_) {(void)write32(A830_SMMU_GR0_SGFSR,gfsr);(void)barrier();}
    (void)read32(A830_SMMU_GR0_SGFSR,&sgfsr_after_);
    __atomic_fetch_add(&global_faults_,1u,__ATOMIC_RELAXED);
    noteSticky(0xFFu,gfsr);
}
IOReturn A830Smmu::clearGlobalFault()
{
    if(!surveyed_)return kIOReturnNotReady;
    uint32_t gfsr=0;
    if(read32(A830_SMMU_GR0_SGFSR,&gfsr))
        return fail(A830SmmuStepGlobalFault,A830ReasonUnexpectedState,A830_SMMU_GR0_SGFSR,0,0,kIOReturnIOError);
    if(!gfsr) {sgfsr_after_=0;return kIOReturnSuccess;}
    globalCheck();
    return kIOReturnSuccess;
}

IOReturn A830Smmu::resetBank(unsigned cb)
{
    Bank *b=bank(cb);
    if(!b)return kIOReturnBadArgument;
    const uint16_t step=uint16_t(stepBase(cb,0xFFFF)+A830SmmuStepReset);
    const uint32_t off=cbOffset(cb);
    /* Not counted as enabled from here: a fault interrupt now only clears FSR. SCTLR = 0 first (random CFIE and M at boot). */
    __atomic_store_n(&b->enabled,false,__ATOMIC_RELEASE);
    uint32_t sctlr=0,fsr=0;
    if(write32(off+A830_SMMU_CB_SCTLR,0) || barrier() || read32(off+A830_SMMU_CB_SCTLR,&sctlr))
        return fail(step,A830ReasonUnexpectedState,off+A830_SMMU_CB_SCTLR,0,0,kIOReturnIOError);
    if(sctlr)return fail(step,A830ReasonReadback,off+A830_SMMU_CB_SCTLR,sctlr,0,kIOReturnIOError);
    if(write32(off+A830_SMMU_CB_FSR,kFault) || barrier() || read32(off+A830_SMMU_CB_FSR,&fsr))
        return fail(step,A830ReasonUnexpectedState,off+A830_SMMU_CB_FSR,0,kFault,kIOReturnIOError);
    b->fsr_boot=fsr;
    if(fsr&kFault)noteSticky(cb,fsr);
    b->reset=true;b->no_progress=0;
    return kIOReturnSuccess;
}

unsigned A830Smmu::attachIrqs()
{
    unsigned wired=0;
    const uint8_t banks[2]={config_.cb_gpu,config_.cb_gmu};
    const uint32_t bits[2]={A830SmmuIrqOffGpu,A830SmmuIrqOffGmu};
    for(unsigned i=0;i<2;++i) {
        Bank &b=banks_[banks[i]];
        auto off=[&] {
            b.irq_off=true;
            if(config_.status)__atomic_fetch_or(&config_.status->irq_off,bits[i],__ATOMIC_RELAXED);
        };
        if(b.irq_attached) {wired+=!b.irq_off;continue;}   /* no detach exists: one attach per boot */
        /* Annibale: no SMMU interrupt in the first round (kernel-sequences 14 #13; SCTLR has no CFIE): polled, as unwired. */
        if(annibale_ || !b.reset || !b.intid || !config_.platform.spi_attach) {off();continue;}
        uint32_t readback=0;uint64_t router=0;
        const IOReturn status=config_.platform.spi_attach(b.intid,false,contextIrq,&b,&readback,&router);
        b.irq_readback=readback;
        if(status!=kIOReturnSuccess) {off();continue;}
        b.irq_attached=true;
        /* A level, enabled Group 1 SPI at priority 0xA0; anything else is polled as if it were not wired. */
        if((readback&0xFF07u)!=0xA003u) {off();continue;}
        b.irq_off=false;
        if(config_.status)__atomic_fetch_and(&config_.status->irq_off,~bits[i],__ATOMIC_RELAXED);
        ++wired;
    }
    return wired;
}

IOReturn A830Smmu::claimStream(const A830SmmuStream &stream)
{
    if(!surveyed_)return kIOReturnNotReady;
    if(stream.slot>=survey_.smrs || stream.slot>=128 || stream.sid>A830_SMMU_SMR_ID_MASK || stream.cb>=survey_.banks ||
        stream.cb>=MaxBanks)return kIOReturnBadArgument;
    for(size_t i=0;i<stream_count_;++i)
        if((streams_[i].slot==stream.slot)!=(streams_[i].sid==stream.sid) ||
            (streams_[i].slot==stream.slot && streams_[i].cb!=stream.cb))return kIOReturnBusy;
    int index=streamIndex(stream);
    if(index<0) {
        if(stream_count_>=MaxStreams)return kIOReturnNoResources;
        index=int(stream_count_++);streams_[index]=stream;
    }
    claimed_[index]=false;routed_[index]=false;
    const uint16_t base=stepBase(stream.cb,stream.sid);
    uint32_t value=0;
    /* Step 1: FAULT with SHCFG = NSH (never SHCFG 0 or a translate to CB1: the combination aborted on this SoC). */
    if(write32(A830_SMMU_GR0_S2CR(stream.slot),A830_SMMU_S2CR_FAULT_NSH) || barrier() ||
        read32(A830_SMMU_GR0_S2CR(stream.slot),&value))
        return fail(uint16_t(base+A830SmmuStepClaimS2cr),A830ReasonUnexpectedState,A830_SMMU_GR0_S2CR(stream.slot),0,
                    A830_SMMU_S2CR_FAULT_NSH,kIOReturnIOError);
    if((value&A830_SMMU_S2CR_TYPE_MASK)!=(A830_SMMU_S2CR_FAULT_NSH&A830_SMMU_S2CR_TYPE_MASK))
        return fail(uint16_t(base+A830SmmuStepClaimS2cr),A830ReasonReadback,A830_SMMU_GR0_S2CR(stream.slot),value,
                    A830_SMMU_S2CR_FAULT_NSH,kIOReturnIOError);
    /* Step 2: the exact SID, mask 0. */
    const uint32_t smr=A830_SMMU_SMR(stream.sid,0);
    if(write32(A830_SMMU_GR0_SMR(stream.slot),smr) || barrier() || read32(A830_SMMU_GR0_SMR(stream.slot),&value))
        return fail(uint16_t(base+A830SmmuStepClaimSmr),A830ReasonUnexpectedState,A830_SMMU_GR0_SMR(stream.slot),0,smr,
                    kIOReturnIOError);
    if(value!=smr)
        return fail(uint16_t(base+A830SmmuStepClaimSmr),A830ReasonReadback,A830_SMMU_GR0_SMR(stream.slot),value,smr,
                    kIOReturnIOError);
    claimed_[index]=true;
    return kIOReturnSuccess;
}

IOReturn A830Smmu::tlbSyncAt(uint32_t off)
{
    /* TLBSYNC (any value: ~0 as Linux, 1 as K90 in the annibale profile), then TLBSTATUS.SACTIVE polled: ten spins, then 1, 2,
     * 4 ... 1024 us apart, 10 ms in all (ARCHITECTURE 4.6.4). */
    if(write32(off+A830_SMMU_CB_TLBSYNC,annibale_?A830SmmuAnnibaleTlbSync:A830_SMMU_TLBI_ALL_VALUE) || barrier())
        return kIOReturnIOError;
    const uint64_t start=config_.platform.now_us(config_.platform.cookie);
    for(unsigned polls=0;;++polls) {
        uint32_t status=0;
        if(read32(off+A830_SMMU_CB_TLBSTATUS,&status))return kIOReturnIOError;
        if(!(status&A830_SMMU_TLBSTATUS_SACTIVE)) {
            const uint64_t spent=config_.platform.now_us(config_.platform.cookie)-start;
            if(spent>sync_slowest_us_)sync_slowest_us_=uint32_t(spent>UINT32_MAX?UINT32_MAX:spent);
            return barrier()?kIOReturnIOError:kIOReturnSuccess;
        }
        if(config_.platform.now_us(config_.platform.cookie)-start>=SyncBudgetUs || polls>=kSyncMaxPolls) {
            __atomic_fetch_add(&sync_timeouts_,1u,__ATOMIC_RELAXED);
            return kIOReturnTimeout;
        }
        if(polls>=SyncSpin) {
            const unsigned shift=polls-SyncSpin<10?polls-SyncSpin:10;
            config_.platform.delay_us(config_.platform.cookie,(1u<<shift)<SyncBackoffMaxUs?(1u<<shift):SyncBackoffMaxUs);
        }
    }
}
int A830Smmu::invalidateBank(unsigned cb,uint32_t off,uint32_t *reg)
{
    /* Design: TLBIALL. Annibale: TLBIASID of the bank's ASID, the only invalidation K90 used (every leaf has nG, so the ASID
     * covers the whole space; kernel-sequences 7.5). Both are 32-bit stores. */
    *reg=off+(annibale_?A830_SMMU_CB_TLBIASID:A830_SMMU_CB_TLBIALL);
    return write32(*reg,annibale_?bankAsid(cb):A830_SMMU_TLBI_ALL_VALUE) || barrier();
}

IOReturn A830Smmu::programContext(unsigned cb,const ArmSmmu4KContextImage &image,uint32_t actlr,uint64_t ttbr1,
    A830SmmuReadback *readback)
{
    Bank *b=bank(cb);
    /* ttbr1: 0, or the context's ASID with a 4 KiB-aligned guard table. */
    /* piano: the ACTLR the board asks for is not what this hypervisor hands back (Gunyah owns the secure CX policy),
     * so a zero readback is this board's normal state rather than a refusal. The context is programmed with what the
     * SMMU actually reports. */
    /* The guard table only exists for the design TCR; the annibale profile programs T0SZ 25 with TTBR1 = 0 (kernel
     * sequences 7.4, section 14 #11), so its ASID comparison must not run there: on this board the caller still passes
     * the encoded image's TTBR1 and the comparison refused the GPU context at step 7 with kIOReturnBadArgument. */
    if(!b || (!annibale_ && ((ttbr1&~kPhysMask&UINT64_C(0x0000ffffffffffff)) ||
                             ((ttbr1&kPhysMask) && ttbr1>>48!=image.ttbr0>>48))))
        return kIOReturnBadArgument;
    if(readback)*readback={};
    const uint16_t base=stepBase(cb,0xFFFF);
    const uint32_t off=cbOffset(cb);
    auto step=[&](uint16_t n) {return uint16_t(base+n);};
    ArmSmmu4KContextImage want=image;
    want.ttbr1=ttbr1;
    if(annibale_) {
        /* kernel-sequences 7.4 for both banks. TCR T0SZ 25 walks the kit's tables unchanged: they start at level 1 from the
         * same root page, whose 512 slots the kit writes (only 0..3 used), so the IOVAs between 4 GiB and 512 GiB stay
         * invalid. TCR has no TTBR1 walk fields (EPD1 alone closes the upper region), so TTBR1 is 0 and the design's guard
         * table is not used (kernel-sequences 14 #11: the guard only goes with the design TCR). Our PTEs use MAIR indexes 0
         * and 1 only, the same bytes in both MAIR0 values. No CFIE: faults are polled. */
        want.sctlr=A830SmmuAnnibaleSctlr;want.tcr=A830SmmuAnnibaleTcr;want.ttbr1=0;
        want.mair0=A830SmmuAnnibaleMair0;want.mair1=0;
    } else if(ttbr1&kPhysMask)want.tcr=tcrWithUpper(image.tcr);
    want.reserved_zero=0;
    /* Annibale: ACTLR on the GPU bank only (K90's direct-ring CB0 0x303); the GMU bank's is left as found. */
    const bool actlr_write=!annibale_ || cb==config_.cb_gpu;
    b->image=want;b->actlr=actlr;b->actlr_written=actlr_write;b->programmed=false;
    __atomic_store_n(&b->enabled,false,__ATOMIC_RELEASE);
    A830SmmuReadback got{};
    uint32_t value=0,hi=0;
    auto io=[&](uint16_t n,uint32_t reg) {return fail(step(n),A830ReasonUnexpectedState,reg,0,0,kIOReturnIOError);};
    auto check=[&](uint16_t n,uint32_t reg,uint32_t read,uint32_t expect) {
        return read==expect?kIOReturnSuccess:fail(step(n),A830ReasonReadback,reg,read,expect,kIOReturnIOError);
    };
    IOReturn status;
    /* Step 3: SCTLR = 0 (the bank may be enabled with random values; it must read 0 before anything else is written). */
    if(write32(off+A830_SMMU_CB_SCTLR,0) || barrier() || read32(off+A830_SMMU_CB_SCTLR,&value))
        return io(A830SmmuStepSctlrOff,off+A830_SMMU_CB_SCTLR);
    if((status=check(A830SmmuStepSctlrOff,off+A830_SMMU_CB_SCTLR,value,0)))return status;
    /* Step 4: ACTLR (0x303 by default; pagani-gpu-actlr), recorded only; a bank the profile leaves alone is read only. */
    if((actlr_write && (write32(off+A830_SMMU_CB_ACTLR,actlr) || barrier())) || read32(off+A830_SMMU_CB_ACTLR,&b->actlr_read))
        return io(A830SmmuStepActlr,off+A830_SMMU_CB_ACTLR);
    /* Step 5: VA64, then stage 1 translate with stage 2 bypass. */
    if(write32(A830_SMMU_GR1_CBA2R(cb),want.cba2r) || write32(A830_SMMU_GR1_CBAR(cb),want.cbar) || barrier() ||
        read32(A830_SMMU_GR1_CBA2R(cb),&got.cba2r) || read32(A830_SMMU_GR1_CBAR(cb),&got.cbar))
        return io(A830SmmuStepCbar,A830_SMMU_GR1_CBAR(cb));
    if((status=check(A830SmmuStepCbar,A830_SMMU_GR1_CBA2R(cb),got.cba2r,want.cba2r)) ||
        (status=check(A830SmmuStepCbar,A830_SMMU_GR1_CBAR(cb),got.cbar,want.cbar)))return status;
    /* Step 6: TCR2; TBI0/TBI1 may read 1 (read-only 1 on this SMMU). */
    if(write32(off+A830_SMMU_CB_TCR2,want.tcr2) || barrier() || read32(off+A830_SMMU_CB_TCR2,&got.tcr2))
        return io(A830SmmuStepTcr2,off+A830_SMMU_CB_TCR2);
    if((got.tcr2&~(A830_SMMU_TCR2_TBI0|A830_SMMU_TCR2_TBI1))!=want.tcr2)
        return fail(step(A830SmmuStepTcr2),A830ReasonReadback,off+A830_SMMU_CB_TCR2,got.tcr2,want.tcr2,kIOReturnIOError);
    /* Step 7: TCR before any TTBR. */
    if(write32(off+A830_SMMU_CB_TCR,want.tcr) || barrier() || read32(off+A830_SMMU_CB_TCR,&got.tcr))
        return io(A830SmmuStepTcr,off+A830_SMMU_CB_TCR);
    if((status=check(A830SmmuStepTcr,off+A830_SMMU_CB_TCR,got.tcr,want.tcr)))return status;
    /* Steps 8, 9: TTBR0 and TTBR1 as two 32-bit halves each (SCTLR.M is 0). */
    if(write32(off+A830_SMMU_CB_TTBR0,lo32(want.ttbr0)) || write32(off+A830_SMMU_CB_TTBR0_HI,hi32(want.ttbr0)) || barrier() ||
        read32(off+A830_SMMU_CB_TTBR0,&value) || read32(off+A830_SMMU_CB_TTBR0_HI,&hi))
        return io(A830SmmuStepTtbr0,off+A830_SMMU_CB_TTBR0);
    got.ttbr0=uint64_t(hi)<<32|value;
    if(got.ttbr0!=want.ttbr0)
        return fail(step(A830SmmuStepTtbr0),A830ReasonReadback,off+A830_SMMU_CB_TTBR0,got.ttbr0,want.ttbr0,kIOReturnIOError);
    if(write32(off+A830_SMMU_CB_TTBR1,lo32(want.ttbr1)) || write32(off+A830_SMMU_CB_TTBR1_HI,hi32(want.ttbr1)) || barrier() ||
        read32(off+A830_SMMU_CB_TTBR1,&value) || read32(off+A830_SMMU_CB_TTBR1_HI,&hi))
        return io(A830SmmuStepTtbr1,off+A830_SMMU_CB_TTBR1);
    got.ttbr1=uint64_t(hi)<<32|value;
    if(got.ttbr1!=want.ttbr1)
        return fail(step(A830SmmuStepTtbr1),A830ReasonReadback,off+A830_SMMU_CB_TTBR1,got.ttbr1,want.ttbr1,kIOReturnIOError);
    /* Step 10: MAIR0 0xFF44 (index 0 NC, index 1 WBRWA; annibale 0xF404FF44), MAIR1 0. */
    if(write32(off+A830_SMMU_CB_MAIR0,want.mair0) || write32(off+A830_SMMU_CB_MAIR1,want.mair1) || barrier() ||
        read32(off+A830_SMMU_CB_MAIR0,&got.mair0) || read32(off+A830_SMMU_CB_MAIR1,&got.mair1))
        return io(A830SmmuStepMair,off+A830_SMMU_CB_MAIR0);
    if((status=check(A830SmmuStepMair,off+A830_SMMU_CB_MAIR0,got.mair0,want.mair0)) ||
        (status=check(A830SmmuStepMair,off+A830_SMMU_CB_MAIR1,got.mair1,want.mair1)))return status;
    /* Step 11: FSR written with ones to clear. */
    if(write32(off+A830_SMMU_CB_FSR,kFault) || barrier())return io(A830SmmuStepFsr,off+A830_SMMU_CB_FSR);
    /* Step 12: TLBIALL (annibale: TLBIASID of this context's ASID) and TLBSYNC (10 ms). */
    uint32_t tlbi=0;
    if(invalidateBank(cb,off,&tlbi))return io(A830SmmuStepTlb,tlbi);
    if((status=tlbSyncAt(off))!=kIOReturnSuccess)
        return fail(step(A830SmmuStepTlb),status==kIOReturnTimeout?A830ReasonTimeout:A830ReasonUnexpectedState,
                    off+A830_SMMU_CB_TLBSTATUS,A830_SMMU_TLBSTATUS_SACTIVE,0,status);
    /* Step 13: enable (0x1167; annibale 0x27); from the write on, contextIrq counts this bank's faults. ASIDPNE may read 0. */
    if(write32(off+A830_SMMU_CB_SCTLR,want.sctlr))return io(A830SmmuStepEnable,off+A830_SMMU_CB_SCTLR);
    __atomic_store_n(&b->enabled,true,__ATOMIC_RELEASE);
    b->no_progress=0;
    if(barrier() || read32(off+A830_SMMU_CB_SCTLR,&got.sctlr))return io(A830SmmuStepEnable,off+A830_SMMU_CB_SCTLR);
    if((got.sctlr|A830_SMMU_SCTLR_ASIDPNE)!=(want.sctlr|A830_SMMU_SCTLR_ASIDPNE) ||
        (got.sctlr&A830_SMMU_SCTLR_ASIDPNE&~want.sctlr)) {
        (void)disableBank(cb);
        return fail(step(A830SmmuStepEnable),A830ReasonReadback,off+A830_SMMU_CB_SCTLR,got.sctlr,want.sctlr,kIOReturnIOError);
    }
    /* Step 14: everything again with the tolerance rules. */
    ArmSmmu4KContextImage seen{};
    uint32_t lo_bits=0,hi_bits=0;
    if(read32(A830_SMMU_GR1_CBAR(cb),&seen.cbar) || read32(A830_SMMU_GR1_CBA2R(cb),&seen.cba2r) ||
        read32(off+A830_SMMU_CB_TCR2,&seen.tcr2) || read32(off+A830_SMMU_CB_TCR,&seen.tcr) ||
        read32(off+A830_SMMU_CB_TTBR0,&lo_bits) || read32(off+A830_SMMU_CB_TTBR0_HI,&hi_bits))
        return io(A830SmmuStepReadback,off);
    seen.ttbr0=uint64_t(hi_bits)<<32|lo_bits;
    if(read32(off+A830_SMMU_CB_TTBR1,&lo_bits) || read32(off+A830_SMMU_CB_TTBR1_HI,&hi_bits) ||
        read32(off+A830_SMMU_CB_MAIR0,&seen.mair0) || read32(off+A830_SMMU_CB_MAIR1,&seen.mair1) ||
        read32(off+A830_SMMU_CB_SCTLR,&seen.sctlr) || read32(off+A830_SMMU_CB_FSR,&got.fsr))
        return io(A830SmmuStepReadback,off);
    seen.ttbr1=uint64_t(hi_bits)<<32|lo_bits;
    const uint32_t mismatch=(seen.cbar!=want.cbar)|(seen.cba2r!=want.cba2r)<<1|(seen.tcr!=want.tcr)<<2|
        ((seen.tcr2&~(A830_SMMU_TCR2_TBI0|A830_SMMU_TCR2_TBI1))!=want.tcr2)<<3|(seen.mair0!=want.mair0)<<4|
        (seen.mair1!=want.mair1)<<5|(seen.ttbr0!=want.ttbr0)<<6|(seen.ttbr1!=want.ttbr1)<<7|
        ((seen.sctlr|A830_SMMU_SCTLR_ASIDPNE)!=(want.sctlr|A830_SMMU_SCTLR_ASIDPNE))<<8;
    got.sctlr=seen.sctlr;got.actlr=b->actlr_read;got.mismatch=mismatch;
    if(readback) {
        *readback=got;
        readback->cbar=seen.cbar;readback->cba2r=seen.cba2r;readback->tcr2=seen.tcr2;readback->tcr=seen.tcr;
        readback->ttbr0=seen.ttbr0;readback->ttbr1=seen.ttbr1;readback->mair0=seen.mair0;readback->mair1=seen.mair1;
    }
    if(got.fsr&kFault)noteSticky(cb,got.fsr);
    if(mismatch) {
        (void)disableBank(cb);
        return fail(step(A830SmmuStepReadback),A830ReasonReadback,off,mismatch,0,kIOReturnIOError);
    }
    b->programmed=true;
    return kIOReturnSuccess;
}

IOReturn A830Smmu::routeStream(const A830SmmuStream &stream)
{
    const int index=streamIndex(stream);
    Bank *b=bank(stream.cb);
    if(index<0 || !b || streams_[index].sid!=stream.sid || streams_[index].cb!=stream.cb)return kIOReturnBadArgument;
    if(!claimed_[index] || !b->programmed || !__atomic_load_n(&b->enabled,__ATOMIC_ACQUIRE))return kIOReturnNotReady;
    const uint16_t base=stepBase(stream.cb,stream.sid);
    const uint32_t want=A830_SMMU_S2CR_TRANS_NSH(stream.cb);
    uint32_t value=0;
    /* Step 15: translate to the bank, SHCFG = NSH; type, bank, SHCFG and PRIVCFG must read back. */
    if(write32(A830_SMMU_GR0_S2CR(stream.slot),want) || barrier() || read32(A830_SMMU_GR0_S2CR(stream.slot),&value))
        return fail(uint16_t(base+A830SmmuStepRoute),A830ReasonUnexpectedState,A830_SMMU_GR0_S2CR(stream.slot),0,want,
                    kIOReturnIOError);
    if((value&A830_SMMU_S2CR_CHECK_MASK)!=want)
        return fail(uint16_t(base+A830SmmuStepRoute),A830ReasonReadback,A830_SMMU_GR0_S2CR(stream.slot),value,want,
                    kIOReturnIOError);
    routed_[index]=true;
    /* Step 16: the bank's FSR, record only (the CM3 is in reset, GX is off: normally 0). */
    const uint32_t off=cbOffset(stream.cb);
    uint32_t fsr=0;
    if(!read32(off+A830_SMMU_CB_FSR,&fsr) && (fsr&kFault)) {
        (void)write32(off+A830_SMMU_CB_FSR,fsr);(void)barrier();
        noteSticky(stream.cb,fsr);
    }
    return kIOReturnSuccess;
}

IOReturn A830Smmu::attachLpac(uint8_t slot,uint16_t sid,unsigned cb)
{
    if(cb>=MaxBanks)return kIOReturnBadArgument;
    const A830SmmuStream stream={slot,uint8_t(cb),sid};
    Bank *b=bank(cb);
    if(!b || !b->programmed || !__atomic_load_n(&b->enabled,__ATOMIC_ACQUIRE))return kIOReturnNotReady;
    IOReturn status=claimStream(stream);
    if(status==kIOReturnSuccess)status=routeStream(stream);
    return status;
}

IOReturn A830Smmu::lpacRouted(uint32_t *readback)
{
    if(readback)*readback=0;
    const A830SmmuStream lpac=lpacStream();
    const int index=streamIndex(lpac);
    if(!surveyed_ || index<0 || !routed_[index])
        return fail(uint16_t(A830SmmuStepLpac+A830SmmuStepRoute),A830ReasonSmmuRoute,A830_SMMU_GR0_S2CR(lpac.slot),0,
                    A830_SMMU_S2CR_TRANS_NSH(lpac.cb),kIOReturnNotReady);
    uint32_t value=0;
    if(read32(A830_SMMU_GR0_S2CR(lpac.slot),&value))
        return fail(uint16_t(A830SmmuStepLpac+A830SmmuStepRoute),A830ReasonUnexpectedState,A830_SMMU_GR0_S2CR(lpac.slot),0,0,
                    kIOReturnIOError);
    if(readback)*readback=value;
    if((value&A830_SMMU_S2CR_CHECK_MASK)!=A830_SMMU_S2CR_TRANS_NSH(lpac.cb))
        return fail(uint16_t(A830SmmuStepLpac+A830SmmuStepRoute),A830ReasonSmmuRoute,A830_SMMU_GR0_S2CR(lpac.slot),value,
                    A830_SMMU_S2CR_TRANS_NSH(lpac.cb),kIOReturnIOError);
    return kIOReturnSuccess;
}

IOReturn A830Smmu::grantAperture(unsigned cb)
{
    if(!surveyed_ || cb>=survey_.banks)return kIOReturnBadArgument;
    if(!config_.platform.aperture) {aperture_state_=0;return kIOReturnSuccess;}
    int64_t x0=0;
    const int status=config_.platform.aperture(config_.platform.cookie,cb,&x0);
    aperture_x0_=x0;
    /* Only CP_SMMU_TABLE_UPDATE (per-process tables, M14) needs it: the result is recorded, never a failure. */
    aperture_state_=!status && !x0?1:2;
    return kIOReturnSuccess;
}

IOReturn A830Smmu::verifyRoutes(unsigned cb)
{
    const Bank *b=bank(cb);
    if(!b)return kIOReturnBadArgument;
    const uint16_t step=uint16_t(stepBase(cb,0xFFFF)+A830SmmuStepVerify);
    if(!b->programmed || !__atomic_load_n(&b->enabled,__ATOMIC_ACQUIRE))
        return fail(step,A830ReasonUnexpectedState,cbOffset(cb),0,0,kIOReturnNotReady);
    unsigned routes=0;
    for(size_t i=0;i<stream_count_;++i) {
        const A830SmmuStream &s=streams_[i];
        if(s.cb!=cb || !routed_[i])continue;
        uint32_t smr=0,s2cr=0;
        if(read32(A830_SMMU_GR0_SMR(s.slot),&smr) || read32(A830_SMMU_GR0_S2CR(s.slot),&s2cr))
            return fail(step,A830ReasonUnexpectedState,A830_SMMU_GR0_SMR(s.slot),0,0,kIOReturnIOError);
        if(smr!=A830_SMMU_SMR(s.sid,0))
            return fail(step,A830ReasonSmmuRoute,A830_SMMU_GR0_SMR(s.slot),smr,A830_SMMU_SMR(s.sid,0),kIOReturnIOError);
        if((s2cr&A830_SMMU_S2CR_CHECK_MASK)!=A830_SMMU_S2CR_TRANS_NSH(cb))
            return fail(step,A830ReasonSmmuRoute,A830_SMMU_GR0_S2CR(s.slot),s2cr,A830_SMMU_S2CR_TRANS_NSH(cb),kIOReturnIOError);
        ++routes;
    }
    if(!routes)return fail(step,A830ReasonSmmuRoute,cbOffset(cb),0,1,kIOReturnNotReady);
    const uint32_t off=cbOffset(cb);
    uint32_t sctlr=0,tcr=0,lo=0,hi=0;
    if(read32(off+A830_SMMU_CB_SCTLR,&sctlr) || read32(off+A830_SMMU_CB_TCR,&tcr) || read32(off+A830_SMMU_CB_TTBR0,&lo) ||
        read32(off+A830_SMMU_CB_TTBR0_HI,&hi))return fail(step,A830ReasonUnexpectedState,off,0,0,kIOReturnIOError);
    if(!(sctlr&A830_SMMU_SCTLR_M))return fail(step,A830ReasonReadback,off+A830_SMMU_CB_SCTLR,sctlr,b->image.sctlr,kIOReturnIOError);
    const uint64_t ttbr0=uint64_t(hi)<<32|lo;
    if(ttbr0!=b->image.ttbr0)return fail(step,A830ReasonReadback,off+A830_SMMU_CB_TTBR0,ttbr0,b->image.ttbr0,kIOReturnIOError);
    /* The CP may clear EPD1 once it ran (seen on this SoC); nothing else of TCR may change. */
    if(tcr!=b->image.tcr && tcr!=(b->image.tcr&~A830_SMMU_TCR_EPD1))
        return fail(step,A830ReasonReadback,off+A830_SMMU_CB_TCR,tcr,b->image.tcr,kIOReturnIOError);
    return kIOReturnSuccess;
}

IOReturn A830Smmu::faultStream(const A830SmmuStream &stream)
{
    const int index=streamIndex(stream);
    if(!surveyed_ || index<0)return kIOReturnBadArgument;
    uint32_t value=0;
    const uint16_t step=uint16_t(stepBase(stream.cb,stream.sid)+A830SmmuStepRelease);
    /* The SMR stays valid: the SID keeps matching this slot and faults instead of reaching a bank. */
    if(write32(A830_SMMU_GR0_S2CR(stream.slot),A830_SMMU_S2CR_FAULT_NSH) || barrier() ||
        read32(A830_SMMU_GR0_S2CR(stream.slot),&value))
        return fail(step,A830ReasonUnexpectedState,A830_SMMU_GR0_S2CR(stream.slot),0,A830_SMMU_S2CR_FAULT_NSH,kIOReturnIOError);
    routed_[index]=false;
    if((value&A830_SMMU_S2CR_TYPE_MASK)!=(A830_SMMU_S2CR_FAULT_NSH&A830_SMMU_S2CR_TYPE_MASK))
        return fail(step,A830ReasonReadback,A830_SMMU_GR0_S2CR(stream.slot),value,A830_SMMU_S2CR_FAULT_NSH,kIOReturnIOError);
    return kIOReturnSuccess;
}

IOReturn A830Smmu::stopBank(unsigned cb)
{
    Bank *b=bank(cb);
    if(!b)return kIOReturnBadArgument;
    const uint16_t step=uint16_t(stepBase(cb,0xFFFF)+A830SmmuStepRelease);
    const uint32_t off=cbOffset(cb);
    __atomic_store_n(&b->enabled,false,__ATOMIC_RELEASE);
    uint32_t sctlr=0;
    if(write32(off+A830_SMMU_CB_SCTLR,0) || barrier() || read32(off+A830_SMMU_CB_SCTLR,&sctlr))
        return fail(step,A830ReasonUnexpectedState,off+A830_SMMU_CB_SCTLR,0,0,kIOReturnIOError);
    if(sctlr)return fail(step,A830ReasonReadback,off+A830_SMMU_CB_SCTLR,sctlr,0,kIOReturnIOError);
    return kIOReturnSuccess;
}
IOReturn A830Smmu::disableBank(unsigned cb)
{
    IOReturn status=stopBank(cb);
    if(status!=kIOReturnSuccess)return status;
    const uint16_t step=uint16_t(stepBase(cb,0xFFFF)+A830SmmuStepRelease);
    const uint32_t off=cbOffset(cb);
    uint32_t tlbi=0;
    if(invalidateBank(cb,off,&tlbi))return fail(step,A830ReasonUnexpectedState,tlbi,0,0,kIOReturnIOError);
    status=tlbSyncAt(off);
    if(status!=kIOReturnSuccess)
        return fail(step,status==kIOReturnTimeout?A830ReasonTimeout:A830ReasonUnexpectedState,off+A830_SMMU_CB_TLBSTATUS,
                    A830_SMMU_TLBSTATUS_SACTIVE,0,status);
    return kIOReturnSuccess;
}

IOReturn A830Smmu::release(const A830SmmuStream &stream)
{
    IOReturn status=faultStream(stream);
    if(status==kIOReturnSuccess)status=disableBank(stream.cb);
    return status;
}

IOReturn A830Smmu::releaseAll()
{
    if(!surveyed_)return kIOReturnSuccess;
    IOReturn first=kIOReturnSuccess;
    for(size_t i=0;i<stream_count_;++i) {
        const IOReturn status=faultStream(streams_[i]);
        if(status!=kIOReturnSuccess && first==kIOReturnSuccess)first=status;
    }
    /* The first two steps of 4.5.7 only: the CP may still run, so no TLB maintenance is relied on and no table is freed. */
    for(unsigned cb=0;cb<MaxBanks && cb<survey_.banks;++cb) {
        if(!banks_[cb].programmed && !__atomic_load_n(&banks_[cb].enabled,__ATOMIC_ACQUIRE))continue;
        const IOReturn status=stopBank(cb);
        if(status!=kIOReturnSuccess && first==kIOReturnSuccess)first=status;
    }
    return first;
}

bool A830Smmu::pushFault(const A830SmmuFault &fault)
{
    for(;;) {
        uint32_t head=__atomic_load_n(&fault_head_,__ATOMIC_RELAXED);
        const uint32_t tail=__atomic_load_n(&fault_tail_,__ATOMIC_ACQUIRE);
        if(head-tail>=A830SmmuFaultRing) {__atomic_fetch_add(&fault_lost_,1u,__ATOMIC_RELAXED);return false;}
        if(__atomic_compare_exchange_n(&fault_head_,&head,head+1,false,__ATOMIC_ACQ_REL,__ATOMIC_RELAXED)) {
            faults_[head%A830SmmuFaultRing]=fault;
            __atomic_store_n(&fault_seq_[head%A830SmmuFaultRing],head+1,__ATOMIC_RELEASE);
            return true;
        }
    }
}
void A830Smmu::recordFault(Bank &b,uint32_t fsr,bool polled,bool sampled,uint32_t completed,uint32_t issued)
{
    const uint32_t off=cbOffset(b.cb);
    A830SmmuFault fault{};
    uint32_t far_lo=0,far_hi=0,frsynra=0;
    (void)read32(off+A830_SMMU_CB_FAR,&far_lo);(void)read32(off+A830_SMMU_CB_FAR_HI,&far_hi);
    (void)read32(off+A830_SMMU_CB_FSYNR0,&fault.fsynr0);(void)read32(off+A830_SMMU_CB_FSYNR1,&fault.fsynr1);
    (void)read32(A830_SMMU_GR1_CBFRSYNRA(b.cb),&frsynra);
    fault.far=uint64_t(far_hi)<<32|far_lo;fault.fsr=fsr;fault.sid=uint16_t(frsynra);fault.cb=b.cb;fault.polled=polled;
    fault.sampled=sampled;fault.completed=completed;fault.issued=issued;
    (void)pushFault(fault);
    /* Written back to clear; a stalled transaction (stall mode only) is terminated. */
    (void)write32(off+A830_SMMU_CB_FSR,fsr);
    if(fsr&A830_SMMU_FSR_SS)(void)write32(off+A830_SMMU_CB_RESUME,A830_SMMU_RESUME_TERMINATE);
    (void)barrier();
    __atomic_fetch_add(&b.faults,uint64_t(1),__ATOMIC_RELAXED);
    if(config_.status)__atomic_fetch_add(&config_.status->smmu_faults,1u,__ATOMIC_RELAXED);
    noteSticky(b.cb,fsr);
}
void A830Smmu::stormGuard(Bank &b,bool progress)
{
    if(progress) {b.no_progress=0;return;}
    if(++b.no_progress<StormLimit || b.irq_off)return;
    /* 64 handlings in a row without progress: mask at the device (SCTLR.CFIE), then at the GIC; the bank is polled from now. */
    const uint32_t off=cbOffset(b.cb);
    uint32_t sctlr=0;
    if(!read32(off+A830_SMMU_CB_SCTLR,&sctlr))(void)write32(off+A830_SMMU_CB_SCTLR,sctlr&~A830_SMMU_SCTLR_CFIE);
    (void)barrier();
    if(config_.platform.spi_disable && b.intid)(void)config_.platform.spi_disable(b.intid);
    b.irq_off=true;
    __atomic_fetch_add(&storms_,1u,__ATOMIC_RELAXED);
    if(config_.status)
        __atomic_fetch_or(&config_.status->irq_off,b.cb==config_.cb_gpu?A830SmmuIrqOffGpu:A830SmmuIrqOffGmu,__ATOMIC_RELAXED);
}
void A830Smmu::contextIrq(void *context)
{
    Bank *b=static_cast<Bank *>(context);
    if(!b || !b->owner)return;
    A830Smmu *s=b->owner;
    /* The jobs in flight right now, before any register read (M14 P0): the faulting one is among them unless it completed in the
     * few microseconds since the fault. */
    uint32_t completed=0,issued=0;
    const bool sampled=s->config_.platform.fences!=nullptr;
    if(sampled)s->config_.platform.fences(s->config_.platform.cookie,&completed,&issued);
    const uint32_t off=s->cbOffset(b->cb);
    __atomic_fetch_add(&b->irqs,uint64_t(1),__ATOMIC_RELAXED);
    uint32_t fsr=0;
    if(s->read32(off+A830_SMMU_CB_FSR,&fsr) || !(fsr&kFault)) {s->stormGuard(*b,false);return;}
    if(!__atomic_load_n(&b->enabled,__ATOMIC_ACQUIRE)) {
        /* A value from before this driver enabled the bank: cleared, neither recorded nor counted. */
        (void)s->write32(off+A830_SMMU_CB_FSR,fsr);(void)s->barrier();
    } else {
        s->recordFault(*b,fsr,false,sampled,completed,issued);
        if(s->config_.platform.fault_notify)s->config_.platform.fault_notify(s->config_.platform.cookie);
    }
    uint32_t again=0;
    const bool progress=!s->read32(off+A830_SMMU_CB_FSR,&again) && (!(again&kFault) || (again&kFault)!=(fsr&kFault));
    s->stormGuard(*b,progress);
}
void A830Smmu::globalIrq(void *context)
{
    A830Smmu *s=static_cast<A830Smmu *>(context);
    if(s && s->surveyed_)s->globalCheck();
}
unsigned A830Smmu::drainFaults(void (*sink)(void *cookie,const A830SmmuFault &fault),void *cookie)
{
    unsigned drained=0;
    for(;;) {
        const uint32_t tail=__atomic_load_n(&fault_tail_,__ATOMIC_RELAXED);
        if(tail==__atomic_load_n(&fault_head_,__ATOMIC_ACQUIRE))break;
        if(__atomic_load_n(&fault_seq_[tail%A830SmmuFaultRing],__ATOMIC_ACQUIRE)!=tail+1)break;   /* still being written */
        const A830SmmuFault fault=faults_[tail%A830SmmuFaultRing];
        __atomic_store_n(&fault_tail_,tail+1,__ATOMIC_RELEASE);
        ++drained;
        if(sink)sink(cookie,fault);
    }
    return drained;
}
void A830Smmu::poll()
{
    if(!surveyed_)return;
    const uint8_t banks[2]={config_.cb_gpu,config_.cb_gmu};
    for(uint8_t cb:banks) {
        Bank &b=banks_[cb];
        if(!b.reset || (b.irq_attached && !b.irq_off))continue;
        const uint32_t off=cbOffset(cb);
        uint32_t fsr=0;
        if(read32(off+A830_SMMU_CB_FSR,&fsr) || !(fsr&kFault))continue;
        if(__atomic_load_n(&b.enabled,__ATOMIC_ACQUIRE)) {
            recordFault(b,fsr,true);
            if(config_.platform.fault_notify)config_.platform.fault_notify(config_.platform.cookie);
        } else {(void)write32(off+A830_SMMU_CB_FSR,fsr);(void)barrier();}
    }
    globalCheck();
}
void A830Smmu::recordSticky()
{
    if(!surveyed_)return;
    uint32_t gfsr=0;
    if(!read32(A830_SMMU_GR0_SGFSR,&gfsr) && gfsr)noteSticky(0xFFu,gfsr);
    const uint8_t banks[2]={config_.cb_gpu,config_.cb_gmu};
    for(uint8_t cb:banks) {
        const uint32_t off=cbOffset(cb);
        uint32_t fsr=0;
        if(read32(off+A830_SMMU_CB_FSR,&fsr) || !(fsr&kFault))continue;
        (void)write32(off+A830_SMMU_CB_FSR,fsr);(void)barrier();
        noteSticky(cb,fsr);
    }
}
void A830Smmu::snapshot(uint32_t fsr[2],uint64_t far[2])
{
    const uint8_t banks[2]={config_.cb_gpu,config_.cb_gmu};
    for(unsigned i=0;i<2;++i) {
        fsr[i]=0;far[i]=0;
        if(!surveyed_)continue;
        const uint32_t off=cbOffset(banks[i]);
        uint32_t lo=0,hi=0;
        (void)read32(off+A830_SMMU_CB_FSR,&fsr[i]);
        (void)read32(off+A830_SMMU_CB_FAR,&lo);(void)read32(off+A830_SMMU_CB_FAR_HI,&hi);
        far[i]=uint64_t(hi)<<32|lo;
    }
}
unsigned A830Smmu::restoreIrqs()
{
    unsigned restored=0;
    const uint8_t banks[2]={config_.cb_gpu,config_.cb_gmu};
    const uint32_t bits[2]={A830SmmuIrqOffGpu,A830SmmuIrqOffGmu};
    for(unsigned i=0;i<2;++i) {
        Bank &b=banks_[banks[i]];
        if(!b.irq_attached || !b.irq_off || (b.irq_readback&0xFF07u)!=0xA003u)continue;
        const uint32_t off=cbOffset(b.cb);
        uint32_t sctlr=0;
        if(__atomic_load_n(&b.enabled,__ATOMIC_ACQUIRE) && !read32(off+A830_SMMU_CB_SCTLR,&sctlr))
            (void)write32(off+A830_SMMU_CB_SCTLR,sctlr|A830_SMMU_SCTLR_CFIE);
        (void)barrier();
        if(config_.platform.spi_enable && config_.platform.spi_enable(b.intid)!=kIOReturnSuccess)continue;
        b.no_progress=0;b.irq_off=false;
        if(config_.status)__atomic_fetch_and(&config_.status->irq_off,~bits[i],__ATOMIC_RELAXED);
        ++restored;
    }
    return restored;
}

bool A830Smmu::routesIntact()
{
    if(!surveyed_)return false;
    for(size_t i=0;i<stream_count_;++i) {
        if(!routed_[i])continue;
        const A830SmmuStream &s=streams_[i];
        uint32_t smr=0,s2cr=0;
        if(read32(A830_SMMU_GR0_SMR(s.slot),&smr) || read32(A830_SMMU_GR0_S2CR(s.slot),&s2cr) || smr!=A830_SMMU_SMR(s.sid,0) ||
            (s2cr&A830_SMMU_S2CR_CHECK_MASK)!=A830_SMMU_S2CR_TRANS_NSH(s.cb))return false;
    }
    for(unsigned cb=0;cb<MaxBanks && cb<survey_.banks;++cb) {
        const Bank &b=banks_[cb];
        if(!b.programmed || !__atomic_load_n(&b.enabled,__ATOMIC_ACQUIRE))continue;
        const uint32_t off=cbOffset(cb);
        uint32_t sctlr=0,lo=0,hi=0;
        if(read32(off+A830_SMMU_CB_SCTLR,&sctlr) || read32(off+A830_SMMU_CB_TTBR0,&lo) || read32(off+A830_SMMU_CB_TTBR0_HI,&hi) ||
            !(sctlr&A830_SMMU_SCTLR_M) || (uint64_t(hi)<<32|lo)!=b.image.ttbr0)return false;
    }
    return true;
}
IOReturn A830Smmu::save()
{
    if(!surveyed_)return kIOReturnNotReady;
    recordSticky();
    /* What restore() will replay must be what the hardware holds now. */
    for(unsigned cb=0;cb<MaxBanks && cb<survey_.banks;++cb) {
        if(!banks_[cb].programmed)continue;
        const IOReturn status=verifyRoutes(cb);
        if(status!=kIOReturnSuccess)return status;
    }
    return kIOReturnSuccess;
}
IOReturn A830Smmu::restore()
{
    if(!surveyed_)return kIOReturnNotReady;
    bool routed[MaxStreams];
    for(size_t i=0;i<MaxStreams;++i)routed[i]=routed_[i];
    const A830SmmuStream lpac=lpacStream();
    /* The order of S3: the GMU bank, then the GPU bank, the LPAC route, the aperture. */
    const uint8_t banks[2]={config_.cb_gmu,config_.cb_gpu};
    for(uint8_t cb:banks) {
        Bank &b=banks_[cb];
        if(!b.programmed)continue;
        const ArmSmmu4KContextImage image=b.image;
        IOReturn status=resetBank(cb);
        for(size_t i=0;status==kIOReturnSuccess && i<stream_count_;++i)
            if(streams_[i].cb==cb && !(streams_[i].slot==lpac.slot && streams_[i].sid==lpac.sid && cb==lpac.cb))
                status=claimStream(streams_[i]);
        A830SmmuReadback readback;
        if(status==kIOReturnSuccess)status=programContext(cb,image,b.actlr,image.ttbr1,&readback);
        for(size_t i=0;status==kIOReturnSuccess && i<stream_count_;++i)
            if(streams_[i].cb==cb && routed[i] && !(streams_[i].slot==lpac.slot && streams_[i].sid==lpac.sid && cb==lpac.cb))
                status=routeStream(streams_[i]);
        if(status==kIOReturnSuccess && cb==config_.cb_gpu) {
            const int index=streamIndex(lpac);
            if(index>=0 && routed[index])status=attachLpac(lpac.slot,lpac.sid,lpac.cb);
            if(status==kIOReturnSuccess && aperture_state_)status=grantAperture(cb);
        }
        if(status!=kIOReturnSuccess) {
            failure_.step=uint16_t(failure_.step|A830SmmuStepRestore);
            return status;
        }
    }
    return kIOReturnSuccess;
}

void A830Smmu::fillStatus(Adreno830Status *status) const
{
    if(!status)return;
    const A830SmmuSurvey &s=survey_;
    const Bank &gpu=banks_[config_.cb_gpu],&gmu=banks_[config_.cb_gmu];
    uint32_t s2cr=0;
    if(surveyed_)(void)read32(A830_SMMU_GR0_S2CR(s.slot_gpu),&s2cr);
    uint32_t sctlr[2]={0,0};
    if(surveyed_) {
        (void)read32(cbOffset(config_.cb_gpu)+A830_SMMU_CB_SCTLR,&sctlr[0]);
        (void)read32(cbOffset(config_.cb_gmu)+A830_SMMU_CB_SCTLR,&sctlr[1]);
    }
    const uint32_t slots=s.slot_gpu|uint32_t(s.slot_lpac)<<8|uint32_t(s.slot_gmu)<<16;
    status->aux[0]=s.idr0|uint64_t(s.idr1)<<32;
    status->aux[1]=s.idr2|uint64_t(s.scr0)<<32;
    status->aux[2]=s.sacr|uint64_t(s.sgfsr)<<32;
    status->aux[3]=s2cr|uint64_t(slots)<<32|A830StatusAperture(aperture_state_);
    status->aux[4]=sctlr[0]|uint64_t(gpu.actlr_read)<<32;
    status->aux[5]=sctlr[1]|uint64_t(gmu.actlr_read)<<32;
    status->smmu_slots=slots;
}

const NativeAdrenoSmmuProgrammer *A830Smmu::programmer()
{
    return initialized_ && surveyed_?&programmer_:nullptr;
}

/* The VM's TLB I/O (ARCHITECTURE 4.6.4): the kit issues TLBIVAL with write64 (one 64-bit store here as on io_), TLBIASID and
 * TLBSYNC with write32, and polls TLBSTATUS with read32 up to its poll budget. A TLBSYNC write starts the clock; a busy status
 * after ten spins waits 1, 2, 4 ... 1024 us before returning, and a sync still busy after 10 ms reads as an I/O error, so the
 * kit fails the transaction and the VM keeps every page (no retry).
 * Annibale: the kit's invalidations (TLBIVA*, TLBIASID, TLBIALL) of a bank are dropped and each TLBSYNC it writes goes out as
 * TLBIASID of the bank's own ASID followed by TLBSYNC = 1 (kernel-sequences 7.5, 14 #12). The kit always writes its
 * invalidations after the leaves are published and completes them with a TLBSYNC, so the one TLBIASID written right before
 * that TLBSYNC covers every dropped one (all leaves are nG: the ASID is the whole space); a TLBSYNC without an invalidation
 * only costs an extra one. */
int A830Smmu::tableBank(size_t at) const
{
    if(!surveyed_)return -1;
    const size_t base=cbOffset(0);
    if(at<base || (at-base)/A830_SMMU_PAGE_BYTES>=MaxBanks)return -1;
    return int((at-base)/A830_SMMU_PAGE_BYTES);
}
int A830Smmu::tableRead32(void *cookie,size_t at,uint32_t *value)
{
    A830Smmu *s=static_cast<A830Smmu *>(cookie);
    const int failed=s->io_.read32(s->io_.cookie,at,value);
    const uint32_t base=s->cbOffset(0);
    if(failed || !s->surveyed_ || at<base || (at&0xFFF)!=A830_SMMU_CB_TLBSTATUS || !(*value&A830_SMMU_TLBSTATUS_SACTIVE))
        return failed;
    const size_t bank=(at-base)/A830_SMMU_PAGE_BYTES;
    if(bank>=MaxBanks)return failed;
    const uint64_t spent=s->config_.platform.now_us(s->config_.platform.cookie)-s->sync_started_us_[bank];
    if(spent>=SyncBudgetUs) {__atomic_fetch_add(&s->sync_timeouts_,1u,__ATOMIC_RELAXED);return 1;}
    const uint32_t polls=++s->sync_polls_[bank];
    if(polls>SyncSpin) {
        const unsigned shift=polls-SyncSpin-1<10?polls-SyncSpin-1:10;
        s->config_.platform.delay_us(s->config_.platform.cookie,(1u<<shift)<SyncBackoffMaxUs?(1u<<shift):SyncBackoffMaxUs);
    }
    return 0;
}
int A830Smmu::tableRead64(void *cookie,size_t at,uint64_t *value)
{
    A830Smmu *s=static_cast<A830Smmu *>(cookie);
    return s->io_.read64(s->io_.cookie,at,value);
}
int A830Smmu::tableWrite32(void *cookie,size_t at,uint32_t value)
{
    A830Smmu *s=static_cast<A830Smmu *>(cookie);
    const int bank=s->tableBank(at);
    const uint32_t reg=uint32_t(at&0xFFF);
    if(bank>=0 && s->annibale_) {
        if(reg==A830_SMMU_CB_TLBIASID || reg==A830_SMMU_CB_TLBIALL)return 0;   /* the TLBIASID before the TLBSYNC covers it */
        if(reg==A830_SMMU_CB_TLBSYNC) {
            ++s->writes_;
            const int failed=s->io_.write32(s->io_.cookie,at-reg+A830_SMMU_CB_TLBIASID,s->bankAsid(unsigned(bank)));
            if(failed)return failed;
            value=A830SmmuAnnibaleTlbSync;
        }
    }
    if(bank>=0 && reg==A830_SMMU_CB_TLBSYNC) {
        s->sync_started_us_[bank]=s->config_.platform.now_us(s->config_.platform.cookie);
        s->sync_polls_[bank]=0;
    }
    ++s->writes_;
    return s->io_.write32(s->io_.cookie,at,value);
}
int A830Smmu::tableWrite64(void *cookie,size_t at,uint64_t value)
{
    A830Smmu *s=static_cast<A830Smmu *>(cookie);
    const uint32_t reg=uint32_t(at&0xFFF);
    if(s->annibale_ && s->tableBank(at)>=0 && (reg==A830_SMMU_CB_TLBIVA || reg==A830_SMMU_CB_TLBIVAA ||
        reg==A830_SMMU_CB_TLBIVAL || reg==A830_SMMU_CB_TLBIVAAL))return 0;   /* the TLBIASID before the TLBSYNC covers it */
    ++s->writes_;
    return s->io_.write64(s->io_.cookie,at,value);
}
int A830Smmu::tableBarrier(void *cookie)
{
    A830Smmu *s=static_cast<A830Smmu *>(cookie);
    return s->io_.barrier(s->io_.cookie);
}

IOReturn A830Smmu::programmerClaim(void *cookie,unsigned bank,const ArmSmmu4KStream *stream)
{
    A830Smmu *s=static_cast<A830Smmu *>(cookie);
    if(!stream || bank>=MaxBanks || stream->slot>=128)return kIOReturnBadArgument;
    return s->claimStream({uint8_t(stream->slot),uint8_t(bank),stream->sid});
}
IOReturn A830Smmu::programmerProgram(void *cookie,unsigned bank,const ArmSmmu4KContextImage *image,uint64_t ttbr1,
    NativeAdrenoSmmuProgramReport *report)
{
    A830Smmu *s=static_cast<A830Smmu *>(cookie);
    if(report)*report={};
    if(!image || bank>=MaxBanks)return kIOReturnBadArgument;
    const uint32_t writes=s->writes_;
    A830SmmuReadback readback{};
    const IOReturn status=s->programContext(bank,*image,s->config_.actlr,ttbr1,&readback);
    if(report) {
        const Bank &b=s->banks_[bank];
        report->want=b.image;
        report->got={readback.cbar,readback.cba2r,readback.tcr,readback.tcr2,readback.mair0,readback.mair1,readback.sctlr,0,
                     readback.ttbr0,readback.ttbr1};
        /* actlr 0: not written (the annibale profile's GMU bank); actlr_read is what the bank holds either way. */
        report->actlr=b.actlr_written?b.actlr:0;report->actlr_read=b.actlr_read;report->fsr=readback.fsr;
        report->mismatch=readback.mismatch;
        report->step=status==kIOReturnSuccess?uint32_t(s->stepBase(bank,0xFFFF)+A830SmmuStepReadback):s->failure_.step;
        report->writes=s->writes_-writes;
        report->context_offset=s->cbOffset(bank);
    }
    return status;
}
IOReturn A830Smmu::programmerRoute(void *cookie,unsigned bank,const ArmSmmu4KStream *stream)
{
    A830Smmu *s=static_cast<A830Smmu *>(cookie);
    if(!stream || bank>=MaxBanks || stream->slot>=128)return kIOReturnBadArgument;
    return s->routeStream({uint8_t(stream->slot),uint8_t(bank),stream->sid});
}
IOReturn A830Smmu::programmerVerify(void *cookie,unsigned bank,const ArmSmmu4KStream *streams,size_t count)
{
    A830Smmu *s=static_cast<A830Smmu *>(cookie);
    if(!streams || !count || bank>=MaxBanks)return kIOReturnBadArgument;
    for(size_t i=0;i<count;++i) {
        const int index=s->streamIndex({uint8_t(streams[i].slot),uint8_t(bank),streams[i].sid});
        if(index<0 || s->streams_[index].sid!=streams[i].sid || s->streams_[index].cb!=bank || !s->routed_[index])
            return s->fail(uint16_t(s->stepBase(bank,streams[i].sid)+A830SmmuStepVerify),A830ReasonSmmuRoute,
                           A830_SMMU_GR0_S2CR(streams[i].slot<128?streams[i].slot:0),0,A830_SMMU_S2CR_TRANS_NSH(bank),
                           kIOReturnNotReady);
    }
    return s->verifyRoutes(bank);
}
IOReturn A830Smmu::programmerFault(void *cookie,unsigned bank,const ArmSmmu4KStream *streams,size_t count)
{
    A830Smmu *s=static_cast<A830Smmu *>(cookie);
    if(!streams || bank>=MaxBanks)return kIOReturnBadArgument;
    IOReturn first=kIOReturnSuccess;
    for(size_t i=0;i<count;++i) {
        const IOReturn status=s->faultStream({uint8_t(streams[i].slot),uint8_t(bank),streams[i].sid});
        if(status!=kIOReturnSuccess && first==kIOReturnSuccess)first=status;
    }
    return first;
}
IOReturn A830Smmu::programmerDisable(void *cookie,unsigned bank)
{
    return static_cast<A830Smmu *>(cookie)->disableBank(bank);
}

#if defined(__aarch64__)
namespace {
bool mmioFits(const A830SmmuMmio *m,uint32_t at,uint32_t bytes)
{
    return m && m->base && !(at&(bytes-1)) && at<=m->bytes && bytes<=m->bytes-at;
}
}
/* Out of line, "r" constraints only: every value travels in a general register (the emulation refuses a WZR/XZR source, Linux
 * QCOM_DUMMY_VAL), and a 64-bit register is written with one 64-bit store (ARCHITECTURE 4.5.2). */
__attribute__((noinline)) static int a830SmmuRead32(void *cookie,uint32_t off,uint32_t *value)
{
    const A830SmmuMmio *m=static_cast<const A830SmmuMmio *>(cookie);
    if(!value || !mmioFits(m,off,4))return 1;
    uint32_t read;
    __asm__ volatile("ldr %w0, [%1]" : "=r"(read) : "r"(m->base+off) : "memory");
    *value=read;
    return 0;
}
__attribute__((noinline)) static int a830SmmuRead64(void *cookie,uint32_t off,uint64_t *value)
{
    const A830SmmuMmio *m=static_cast<const A830SmmuMmio *>(cookie);
    if(!value || !mmioFits(m,off,8))return 1;
    uint64_t read;
    __asm__ volatile("ldr %x0, [%1]" : "=r"(read) : "r"(m->base+off) : "memory");
    *value=read;
    return 0;
}
__attribute__((noinline)) static int a830SmmuWrite32(void *cookie,uint32_t off,uint32_t v)
{
    const A830SmmuMmio *m=static_cast<const A830SmmuMmio *>(cookie);
    if(!mmioFits(m,off,4))return 1;
    __asm__ volatile("str %w0, [%1]" : : "r"(v),"r"(m->base+off) : "memory");
    return 0;
}
__attribute__((noinline)) static int a830SmmuWrite64(void *cookie,uint32_t off,uint64_t v)
{
    const A830SmmuMmio *m=static_cast<const A830SmmuMmio *>(cookie);
    if(!mmioFits(m,off,8))return 1;
    __asm__ volatile("str %x0, [%1]" : : "r"(v),"r"(m->base+off) : "memory");
    return 0;
}
static int mmioRead32(void *cookie,size_t at,uint32_t *value) {return at>UINT32_MAX?1:a830SmmuRead32(cookie,uint32_t(at),value);}
static int mmioRead64(void *cookie,size_t at,uint64_t *value) {return at>UINT32_MAX?1:a830SmmuRead64(cookie,uint32_t(at),value);}
static int mmioWrite32(void *cookie,size_t at,uint32_t value) {return at>UINT32_MAX?1:a830SmmuWrite32(cookie,uint32_t(at),value);}
static int mmioWrite64(void *cookie,size_t at,uint64_t value) {return at>UINT32_MAX?1:a830SmmuWrite64(cookie,uint32_t(at),value);}
static int mmioBarrier(void *) {__asm__ volatile("dsb sy" ::: "memory");return 0;}
IOReturn A830SmmuBindMmio(A830SmmuMmio *mmio,ArmSmmu4KIo *io)
{
    if(!io)return kIOReturnBadArgument;
    *io={};
    if(!mmio || !mmio->base || (uintptr_t(mmio->base)&4095) || mmio->bytes<0x2000 || mmio->bytes>UINT32_MAX)
        return kIOReturnBadArgument;
    *io={mmio,mmio->bytes,mmioRead32,mmioRead64,mmioWrite32,mmioWrite64,mmioBarrier};
    return kIOReturnSuccess;
}
#else
IOReturn A830SmmuBindMmio(A830SmmuMmio *,ArmSmmu4KIo *io)
{
    if(io)*io={};
    return kIOReturnUnsupported;
}
#endif

#ifndef ADRENO830_HOST_TEST
namespace {
struct KernelPlatform {Sm8750Scm *scm;void (*notify)(void *);void *cookie;} kernelPlatform;
uint64_t kernelNowUs(void *)
{
    uint64_t nanoseconds=0;
    absolutetime_to_nanoseconds(mach_absolute_time(),&nanoseconds);
    return nanoseconds/1000;
}
void kernelDelayUs(void *,uint32_t microseconds) {IODelay(microseconds);}
void kernelNotify(void *cookie)
{
    const KernelPlatform *k=static_cast<const KernelPlatform *>(cookie);
    if(k->notify)k->notify(k->cookie);
}
int kernelAperture(void *cookie,uint32_t context_bank,int64_t *x0)
{
    const KernelPlatform *k=static_cast<const KernelPlatform *>(cookie);
    if(!k->scm)return -1;
    Sm8750SmcResult result{};
    const Sm8750ScmStatus status=Sm8750ScmSetGpuSmmuAperture(k->scm,context_bank,&result);
    if(x0)*x0=result.x0;
    return status==Sm8750ScmOk?0:1;
}
}
IOReturn A830SmmuKernelPlatform(A830SmmuPlatform *platform,Sm8750Scm *scm,void (*notify)(void *),void *notify_cookie)
{
    if(!platform)return kIOReturnBadArgument;
    kernelPlatform={scm,notify,notify_cookie};
    *platform={&kernelPlatform,kernelNowUs,kernelDelayUs,nullptr,nullptr,nullptr,kernelNotify,scm?kernelAperture:nullptr,nullptr};
    return kIOReturnSuccess;
}
#endif
