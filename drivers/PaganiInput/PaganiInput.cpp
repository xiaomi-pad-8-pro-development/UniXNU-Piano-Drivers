// SPDX-License-Identifier: BSD-2-Clause-Patent
#include "OriginalHidDeviceABI.h"
#include "Mapping.h"
#include "Buttons.h"
#include <PaganiPeripherals.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <libkern/c++/OSString.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include <SmmuTakeover.h>
#include <Rpmh.h>
extern "C" {
#include "S3910.h"
#include <GeniSpi.h>
#include "Nvt/NvtProtocol.h"
#include "Nvt/NvtTouchCore.h"
#ifdef PAGANI_TOUCH_NVT
#include "Nvt/NvtFirmware_boe.inc"
#include "Nvt/NvtFirmware_csot.inc"
#endif
void boot_progress_stage(const char *);
}

#include "Reports.h"
#include "HidEvents.h"
#include "Drv2Lock.h"
#include "InputScript.h"
#include "../BootProgress/PeripheralStatus.h"

// Touch frames parsed from the controller (presses, moves and releases). The A830 GPU driver's DCVS raises its floor
// while this count changes (Adreno830 ARCHITECTURE 4.16 touch boost; slumber pre-wake, 4.17). Weak: the GPU driver
// carries the same zero definition, so either links without the other and the kernel link keeps one word.
extern "C" {
__attribute__((weak)) volatile uint32_t pagani_input_touch_events;
// Key reports accepted by the HID parser (a press, a release or a ring/silent change). PaganiArmCpu's governor reads it while the
// display is off (pagani-screenoff bit 0): a key there lifts the screen-off CPU levels. Weak like the touch counter.
__attribute__((weak)) volatile uint32_t pagani_input_key_events;
// The display's power as PaganiPanel publishes it from the display's power callback (bit 0 off, the count of changes above
// it); the input script presses Power only while it reads off. Weak zero like the counters: without PaganiPanel it reads on.
__attribute__((weak)) volatile uint32_t pagani_screen_state;
// The Power key is down, by its debounced level. A long press ends in the PMIC's reset, which no code sees coming, so the UFS log (BootProgress
// UfsLog.h) stays off the wire for as long. Weak zero like the counters: UfsLog.c links without this driver and reads it released.
__attribute__((weak)) volatile uint32_t pagani_input_power_held;
}

static void dmaCache(void *bytes,size_t size,unsigned operation) {
    uint64_t ctr;__asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
    uintptr_t line=4ULL<<((ctr>>16)&15),end=(uintptr_t)bytes+size;
    for (uintptr_t at=(uintptr_t)bytes&~(line-1);at<end;at+=line) {
        if (operation==0) __asm__ volatile("dc cvac, %0" :: "r"(at) : "memory");
        else if (operation==1) __asm__ volatile("dc civac, %0" :: "r"(at) : "memory");
        else __asm__ volatile("dc ivac, %0" :: "r"(at) : "memory");
    }
    __asm__ volatile("dsb sy" ::: "memory");
}
static void dmaClean(void *,void *p,size_t n) {dmaCache(p,n,0);}
static void dmaCleanInval(void *,void *p,size_t n) {dmaCache(p,n,1);}
static void dmaInval(void *,void *p,size_t n) {dmaCache(p,n,2);}

class PaganiInput : public IOHIDDevice {
    OSDeclareDefaultStructors(PaganiInput)
    IOWorkLoop *loop=nullptr;
    IOTimerEventSource *timer=nullptr;
    IOService *touchEvents=nullptr;
    bool touch=false,running=false,timerAdded=false;
    PeripheralWindow core,cnfg,obs,gcc,se,wrapper,smmu,rsc,cmddb,pins[6],plusKey;
    IOBufferMemoryDescriptor *dmaMemory=nullptr;
    bool dmaExposed=false,firstReadCaptured=false;
    geni_spi spi={};
    s3910 controller={};
    s3910_frame previous={};
    pagani_keys keys=pagani_keys_initial();
    int pon=-1,gpio=-1;
    uint8_t lastButtons=0;
    bool plusConfigured=false;
    uint32_t plusSaved=0;
    uint32_t savedPins[6]={};
    uint32_t savedReset=0;
    unsigned pinsConfigured=0;
    const char *phase="allocate";
    uint32_t sample=0;
    uint32_t touchBinding=0,touchUsage=0xffffffff,lastInputXY=0;
    uint64_t reports=0;
    int touchError=0;
    char touchPhase[24]={};
    uint32_t screenWidth=0,screenHeight=0;
    pagani_script script={};
#ifdef PAGANI_TOUCH_NVT
    // NT36532 THP (Xiaomi piano): the protocol of Nvt/ over the same SE DMA transport. Buffers: Nvt/NvtProtocol.h sizes.
    nvt_bus nvtBus={};
    nvt_chip nvt={};
    uint8_t *nvtScratch=nullptr,*nvtRx=nullptr;
    nvt_fw *nvtFw=nullptr;
    nvt_fw_info nvtInfo={};
    nvt_touch_core *nvtCore=nullptr;
    uint8_t *nvtFrame=nullptr;
    int16_t *nvtMatrix=nullptr;
    unsigned nvtCounter=~0u,nvtFrames=0,nvtTypes=0;
    // One THP scan: the 32-byte header window first, the full frame only when its counter moved. The piano firmware marks touch
    // scans type 3 (UEFI TouchProbe); the tracker takes them as its data frames (type 4). Contacts go out in the TouchComm frame
    // (0..32767 per axis), so the HID path stays the one the S3910 boards use.
    // After a firmware download the controller is still starting: a read before it raises the frame-ready line once can
    // make it reset. Until the line has changed level once, only that line (a TLMM read, not the bus) is sampled.
    int nvtAttnSeen=-1;
    int pollNvt(s3910_frame &frame) {
        if (nvtAttnSeen<1) {
            int level=(int)(PeripheralWindow::read(&pins[5],4)&1);
            if (nvtAttnSeen<0) {nvtAttnSeen=level?2:3;return 0;}   // 2/3: first level high/low
            if (level==(nvtAttnSeen==2)) return 0;
            nvtAttnSeen=1;
        }
        uint8_t hdr[NVT_THP_PEEK_LEN];
        int rc=nvt_peek_frame_header(&nvt,hdr);if (rc) return rc;
        if (!nvt_peek_valid(hdr)) return 0;
        unsigned counter=nvt_peek_counter(hdr);
        if (counter==nvtCounter) return 0;
        rc=nvt_read_frame(&nvt,nvtFrame);if (rc) return rc;
        if (!nvt_frame_header_valid(nvtFrame)) return 0;
        nvtCounter=nvt_frame_counter(nvtFrame);++nvtFrames;
        unsigned type=nvt_frame_type(nvtFrame);
        if (type<32) nvtTypes|=1u<<type;
        if (type==3) type=4;
        nvt_frame_matrix(nvtFrame,nvtMatrix);
        nvt_contact contacts[S3910_CONTACTS];
        int n=nvt_touch_process(nvtCore,nvtMatrix,type,contacts,S3910_CONTACTS);
        if (n<0) return 0;
        // No finger now and none in the last report: nothing to tell iPadOS (an empty report per scan kept the screen waking).
        bool was=false;for (auto &c:previous.contact) was|=c.active!=0;
        if (!n && !was) return 0;
        frame={};
        for (int i=0;i<n;++i) {
            unsigned slot=(unsigned)contacts[i].slot%S3910_CONTACTS;
            frame.contact[slot].x=(uint16_t)((uint32_t)contacts[i].x*32767u/NVT_LONG_AXIS_MAX);
            // The sensor's short axis runs bottom to top on the piano (slide test 2026-11-12), the screen's top to bottom.
            frame.contact[slot].y=(uint16_t)(32767u-(uint32_t)contacts[i].y*32767u/NVT_SHORT_AXIS_MAX);
            frame.contact[slot].active=1;
        }
        return 1;
    }
    PeripheralWindow lcdId;
    static constexpr size_t kNvtScratch=NVT_TRANSFER_LEN+8,kNvtRx=NVT_READ_MAX+2;
    static int nvtXfer(void *ctx,const uint8_t *tx,uint8_t *rx,size_t len) {
        return static_cast<PaganiInput *>(ctx)->transferTouch(tx,rx,len);
    }
    static void nvtDelay(void *,unsigned ms) {IOSleep(ms);}
#endif
    void observe(const char *stage,int result=0) {
#ifdef PAGANI_TOUCH_NVT
        // Bring-up: every touch step on the boot screen as it starts, so a bus hang leaves its step as the last line.
        if (touch && stage!=phase && !__atomic_load_n(&running,__ATOMIC_RELAXED)) {
            char line[48];snprintf(line,sizeof line,"touch: %s",stage);boot_progress_stage(line);
            // And to the kernel log, with time for the USB console to send it before the step runs.
            IOLog("%s (sample %x)\n",line,sample);
        }
#endif
        phase=stage;
        if (touch && !result && !drawDue()) return;
        boot_progress_peripheral(touch?PAGANI_TOUCH:PAGANI_KEYS,phase,result,sample,reports);
    }
    // While touch runs, the status rows redraw at most five times a second: a redraw per scan (120 Hz) took the screen to 16 fps.
    uint64_t lastDrawMs=0,lastXYLogMs=0;
    bool drawDue() {
        if (!__atomic_load_n(&running,__ATOMIC_RELAXED)) return true;
        uint64_t ms=nowMs();if (ms-lastDrawMs<200) return false;
        lastDrawMs=ms;return true;
    }
    void observeTouch(int result) {
        snprintf(touchPhase,sizeof touchPhase,"T%u %02x %s",controller.version,controller.command,controller.stage);
        sample=controller.detail;observe(touchPhase,result);
    }
    int transferTouch(const uint8_t *tx,uint8_t *rx,size_t size) {
        spi.snap_valid=0;
#ifdef PAGANI_TOUCH_NVT
        bool capture=false;   // no RESET pin on this board: pins[4] is never mapped
#else
        bool capture=rx && !firstReadCaptured;
#endif
        uint32_t pin=capture?(PeripheralWindow::read(&pins[5],4)&1):0;
        if (spi.use_dma) dmaExposed=true;
        int rc=geni_spi_transfer(&spi,tx,rx,size,0);
        // Preserve the driver's pre-cancel snapshot; live registers now describe recovery.
        if (rc && spi.snap_valid) boot_progress_touch_spi(spi.info.fw_revision,spi.info.if_disable,
            spi.snap_status,spi.snap_irq,spi.snap_txfs,spi.snap_rxfs);
        if (rc && spi.snap_valid && spi.use_dma) boot_progress_touch_dma(
            spi.snap_dma_tx,spi.snap_dma_rx,spi.snap_dma_txin,spi.snap_dma_rxin);
        if (!rc && capture) {
            uint32_t head=0;
            for (size_t i=0;i<size && i<4;++i) head|=(uint32_t)rx[i]<<(8*i);
            pin|=(PeripheralWindow::read(&pins[5],4)&1)<<1;
            pin|=(PeripheralWindow::read(&pins[4],4)&3)<<2;
            boot_progress_touch_read((uint32_t)size,head,pin,(uint32_t)spi.use_dma,
                spi.se.rd(spi.se.ctx,SE_GENI_STATUS),spi.last_tx_in,spi.last_rx_in,
                spi.se.rd(spi.se.ctx,SM8750_SPI_DELAY_COUNTERS));
            firstReadCaptured=true;
        }
        return rc;
    }
    static int readTouch(void *ctx,uint8_t *data,size_t size) {
        uint8_t dummy[S3910_PAYLOAD_MAX+7];
        if (!data || !size) return -22;
        if (size>sizeof dummy) return -90;
        // Synaptics reads clock 0xff on MOSI, as in the stock HBP SPI transport.
        memset(dummy,0xff,size);
        return static_cast<PaganiInput *>(ctx)->transferTouch(dummy,data,size);
    }
    static int writeTouch(void *ctx,const uint8_t *data,size_t size) {
        return static_cast<PaganiInput *>(ctx)->transferTouch(data,nullptr,size);
    }
    static void sleepTouch(void *,unsigned ms) {IOSleep(ms);}
    int setupDma() {
#ifdef PAGANI_TOUCH_NVT
        constexpr size_t bytes=65536;   // one NT36532 firmware block (NVT_TRANSFER_LEN + 8) per direction
        static_assert(NVT_TRANSFER_LEN+8<=bytes && NVT_THP_FRAME_LEN+2<=bytes,"NT36532 transfer fits DMA buffer");
#else
        constexpr size_t bytes=4096;
#endif
        static_assert(S3910_PAYLOAD_MAX+7<=bytes,"touch transfer fits DMA page");
        observe("DMA allocate");
        dmaMemory=IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,
            static_cast<IOOptionBits>(kIODirectionInOut)|static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous)|
            static_cast<IOOptionBits>(kIOMemoryMapperNone),2*bytes,bytes);
        if (!dmaMemory) return -12;
        IOByteCount contiguous=0;
        uint64_t physical=dmaMemory->getPhysicalSegment(0,&contiguous,kIOMemoryMapperNone);
        auto virt=static_cast<uint8_t *>(dmaMemory->getBytesNoCopy());
        if (!virt || !physical || (physical&(bytes-1)) || contiguous<2*bytes) return -14;
        memset(virt,0,2*bytes);dmaCache(virt,2*bytes,1);
        observe("SMMU map");
        if (!smmu.map(PAGANI_TOUCH_SMMU,PAGANI_TOUCH_SMMU_SIZE)) return -12;
        auto read=[this](unsigned at) {return PeripheralWindow::read(&smmu,at);};
        auto write=[this](unsigned at,uint32_t value) {PeripheralWindow::write(&smmu,at,value);};
        // Refuse an existing translated/shared QUP stream; do not alter firmware bank 0.
        auto plan=planUfsSmmu(PAGANI_TOUCH_SMMU_SID,physical,2*bytes,read,PAGANI_TOUCH_SMMU_SIZE);
        sample=(plan.slot<<16)|plan.bank;observe(plan.error?"SMMU conflict":"SMMU configure");
        const char *error=configureUfsSmmu(plan,PAGANI_TOUCH_SMMU_SID,read,write);
        if (error) {
            IOLog("Pagani touch SMMU: %s slot=%u smr=%x s2cr=%x\n",error,plan.slot,plan.smr,plan.s2cr);
            boot_progress_touch_smmu(plan.slot,plan.bank,plan.smr,plan.s2cr);return -16;
        }
        observe("SMMU sync");unsigned polls=0;
        if (!syncUfsSmmu(read,write,[&] {IODelay(10);return ++polls>=10000;},sample)) return -110;
        spi.use_dma=1;
        spi.dma={nullptr,dmaClean,dmaCleanInval,dmaInval,virt,virt+bytes,physical,physical+bytes,bytes};
        return 0;
    }
    int setupButtons() {
        observe("SPMI map");
        if (!core.map(PAGANI_SPMI_CORE,0x3000) || !cnfg.map(PAGANI_SPMI_CNFG,0x4000) || !obs.map(PAGANI_SPMI_OBS,0x80000)) return -12;
        auto c=core.io(),f=cnfg.io();
        observe("SPMI channels");
        pon=spmi_find(&c,&f,PAGANI_PON_PPID,PAGANI_SPMI_EE);
        gpio=spmi_find(&c,&f,PAGANI_GPIO_PPID,PAGANI_SPMI_EE);
        sample=((uint32_t)pon<<16)|((uint32_t)gpio&0xffff);
        if (pon<0 || gpio<0) return -19;
        auto o=obs.io();uint8_t subtype=0,type=0,mode=0,enable=0;
        observe("PON subtype");
        int rc=spmi_read8(&o,PAGANI_SPMI_EE,pon,5,&subtype);sample=subtype;
        if (rc || subtype!=9) return rc?rc:-19;
        observe("GPIO type");rc=spmi_read8(&o,PAGANI_SPMI_EE,gpio,4,&type);sample=type;
        if (rc || type!=0x10) return rc?rc:-19;
        observe("GPIO subtype");rc=spmi_read8(&o,PAGANI_SPMI_EE,gpio,5,&subtype);sample=subtype;
        if (rc || subtype<0x10 || subtype>0x13) return rc?rc:-19;
        observe("GPIO mode");rc=spmi_read8(&o,PAGANI_SPMI_EE,gpio,0x40,&mode);sample=mode;
        if (rc || (mode&3)) return rc?rc:-19;
        observe("GPIO enable");rc=spmi_read8(&o,PAGANI_SPMI_EE,gpio,0x46,&enable);sample=enable;
        if (rc || !(enable&0x80)) return rc?rc:-19;
        setupPlusKey();
        return 0;
    }
    // Optional Plus key: Android's input configuration of TLMM GPIO 97, checked by readback; the original CFG returns at
    // cleanup. Without it the ring/silent switch stays at ring and the other keys still start.
    void setupPlusKey() {
        observe("Plus key");
#ifndef PAGANI_PLUS_KEY_GPIO
        return;   // the board has no Plus key
#else
        if (!plusKey.map(PAGANI_TLMM_GPIO(PAGANI_PLUS_KEY_GPIO),0x1000)) return;
        plusSaved=PeripheralWindow::read(&plusKey,0);
        uint32_t config=pagani_plus_key_config(plusSaved);
        PeripheralWindow::write(&plusKey,0,config);plusConfigured=true;
        sample=PeripheralWindow::read(&plusKey,0);
        if ((sample&0x3ff)!=(config&0x3ff)) releasePlusKey();
#endif
    }
    void releasePlusKey() {
        if (plusConfigured) PeripheralWindow::write(&plusKey,0,plusSaved);
        plusConfigured=false;plusKey.release();
    }
    // One poll: PMIC levels over SPMI and the Plus key from TLMM. A changed report goes out at once and a failed send is
    // retried. The elements start at 0 (silent), so the first poll reports ring before backboardd copies the switch level.
    void pollButtons(uint64_t ms) {
        uint8_t p=0,g=1;auto io=obs.io();
        int rc=spmi_read8(&io,PAGANI_SPMI_EE,pon,0x10,&p);
        if (!rc) rc=spmi_read8(&io,PAGANI_SPMI_EE,gpio,0x10,&g);
        uint32_t level=plusConfigured?PeripheralWindow::read(&plusKey,4)&1:1;
        uint8_t valid=(uint8_t)((rc?0:PAGANI_KEYS_PMIC)|(plusConfigured?PAGANI_KEY_PLUS:0));
        uint8_t report=pagani_keys_step(&keys,valid,pagani_buttons(p,g,level),ms);
        // What the PMIC sees, so before the script's keys, and only what a read confirmed: a failed read lets the key go in the report, not here.
        if (keys.filter.known&PAGANI_KEY_POWER) __atomic_store_n(&pagani_input_power_held,(report&PAGANI_KEY_POWER)?1u:0u,__ATOMIC_RELAXED);
        report=pagani_script_keys(report,ms);   // InputScript.h: unchanged unless an input script is armed
        sample=pagani_keys_sample(p,g,level,keys.ringer.ring,plusConfigured);
        if (report!=lastButtons) {
            uint8_t bytes[]={2,report};
            if (send(bytes,sizeof bytes)) {lastButtons=report;__atomic_fetch_add(&pagani_input_key_events,1u,__ATOMIC_RELAXED);}
        } else observe("PMIC sample",rc);
    }
    int setupTouchPower() {
        observe("power map");
        if (!rsc.map(PAGANI_TOUCH_RSC,PAGANI_TOUCH_RSC_SIZE) ||
            !cmddb.map(PAGANI_CMD_DB,PAGANI_CMD_DB_SIZE)) return -12;
        auto db=cmddb.io(),power=rsc.io();uint32_t addresses[2]={};
        observe("power VDD lookup");
        int rc=rpmh_vrm_address(&db,PAGANI_CMD_DB_SIZE,PAGANI_TOUCH_VDD,&addresses[0]);if (rc) return rc;
        observe("power VCC lookup");
        rc=rpmh_vrm_address(&db,PAGANI_CMD_DB_SIZE,PAGANI_TOUCH_VCC,&addresses[1]);if (rc) return rc;
        observe("power resource");
        if (addresses[0]!=PAGANI_TOUCH_VDD_VRM || addresses[1]!=PAGANI_TOUCH_VCC_VRM) return -71;
        const unsigned millivolts[]={PAGANI_TOUCH_VDD_MV,PAGANI_TOUCH_VCC_MV};
        // DRV-2 is shared with the A830 GPU votes: both rail requests go out under the DRV-2 lock.
        observe("power lock");
        uint32_t waits=0;rc=pagani_drv2_acquire(PaganiTouchDrv2Owner,IODelay,&waits);sample=waits;
        if (rc) return rc;
        for (unsigned i=0;i<2;++i) {
            sample=addresses[i];observe(i?"power VCC":"power VDD");
            rpmh_result result={};rc=sm8750_rpmh_vrm_enable(&power,addresses[i],millivolts[i],&result);
            if (rc) {
                pagani_drv2_release();
                boot_progress_touch_power(addresses[i],result.hardware,result.tcs,result.status,
                    result.command[0],result.command[1],result.command[2]);
                return rc;
            }
        }
        pagani_drv2_release();
        IOSleep(PAGANI_TOUCH_POWER_MS);return 0;
    }
#ifdef PAGANI_TOUCH_NVT
    // QUP1 SE2 has no DFS clock source on SM8750 (gcc_qupv3_wrap1_s2_clk_src is a fixed divider of the QSPI reference):
    // vote the wrapper and SE branches, wait for them, and keep the source the boot firmware left running.
    int nvtClock() {
        auto g=gcc.io();
        sm8750_gcc_vote_report report={};
        int rc=sm8750_gcc_vote_set(&g,PAGANI_TOUCH_GCC_VOTE,SM8750_GCC_OWNER_TOUCH,&report);sample=report.after;
        if (rc) return rc;
        const uint32_t branches[]=PAGANI_TOUCH_GCC_BRANCHES;
        for (uint32_t branch:branches) {
            unsigned n=0;
            for (;n<200;++n) {
                uint32_t v=peripheral_read(&g,branch);
                if ((v&2) || !(v&0x80000000u)) break;
                IODelay(1);
            }
            if (n==200) {sample=branch;return -110;}
        }
        return 0;
    }
    int setupTouch() {
        observe("GCC map");
        if (!gcc.map(PAGANI_GCC_BASE,0x60000)) return -12;
        observe("SPI clock");int rc=nvtClock();if (rc) return rc;
        observe("SPI map");
        if (!se.map(PAGANI_TOUCH_SE,0x4000) || !wrapper.map(PAGANI_TOUCH_WRAPPER,0x1000)) return -12;
        spi.se={&se,PeripheralWindow::read,PeripheralWindow::write,PeripheralWindow::delay};
        spi.wrapper={&wrapper,PeripheralWindow::read,PeripheralWindow::write,PeripheralWindow::delay};
        observe("GENI firmware");rc=geni_spi_read_info(&spi);if (rc) return rc;
        sample=spi.info.geni_status;
        if (spi.info.protocol!=GENI_PROTO_SPI) return GENI_ERR_PROTO;
        if (spi.info.geni_status&(GENI_STATUS_M_ACTIVE|(1u<<12))) return -16;
        spi.poll_only=1;spi.poll_limit=200000;
        if (spi.info.fifo_disabled) {rc=setupDma();if (rc) return rc;spi.poll_only=2;}
        observe("SPI init");rc=geni_spi_init(&spi);if (rc) return rc;
        observe("SPI mode");rc=geni_spi_setup(&spi,0,0);if (rc) return rc;
        // The divider the UEFI touch driver runs this engine with (pianoPkg TouchProbe: 4 and 1 lose the controller).
        observe("SPI rate");
        rc=geni_spi_set_clock(&spi,PAGANI_TOUCH_HZ,PAGANI_TOUCH_HZ*PAGANI_TOUCH_CLK_DIV*spi.oversampling,0);if (rc) return rc;
        const unsigned gpios[]=PAGANI_TOUCH_PINS;
        const unsigned drives[]=PAGANI_TOUCH_DRIVES;
        for (unsigned i=0;i<4;++i) {
            sample=gpios[i];observe("touch GPIO");
            if (!pins[i].map(PAGANI_TLMM_GPIO(gpios[i]),0x1000)) return -12;
            savedPins[i]=PeripheralWindow::read(&pins[i],0);
            PeripheralWindow::write(&pins[i],0,(savedPins[i]&~0x3ffu)|(drives[i]<<6)|(PAGANI_TOUCH_SPI_FUNC<<2));
            ++pinsConfigured;
        }
        // ATTN (frame ready) is only read; its Android pinctrl state is gpio input, which is the reset state.
        if (!pins[5].map(PAGANI_TLMM_GPIO(PAGANI_TOUCH_ATTN),0x1000)) return -12;
        nvtScratch=static_cast<uint8_t *>(IOMalloc(kNvtScratch));nvtRx=static_cast<uint8_t *>(IOMalloc(kNvtRx));
        if (!nvtScratch || !nvtRx) return -12;
        nvtBus={this,nvtXfer,nvtDelay};
        nvt.bus=&nvtBus;nvt.scratch=nvtScratch;nvt.rxbuf=nvtRx;
        observe("NVT identify");rc=nvt_identify(&nvt);
        sample=(uint32_t)nvt.id[2]|(uint32_t)nvt.id[3]<<8|(uint32_t)nvt.id[4]<<16|(uint32_t)nvt.id[5]<<24;
        IOLog("Pagani touch NT36532 id %02x %02x %02x %02x %02x %02x cascade %d rc %d\n",nvt.id[0],nvt.id[1],nvt.id[2],
            nvt.id[3],nvt.id[4],nvt.id[5],nvt.cascade,rc);
        if (rc) return rc;
        // Panel vendor from the LCD id pin, as the Android driver reads it: 0 CSOT, 1 BOE. Each panel has its own firmware.
        observe("NVT panel");
        if (!lcdId.map(PAGANI_TLMM_GPIO(PAGANI_TOUCH_LCD_ID),0x1000)) return -12;
        const bool boe=(PeripheralWindow::read(&lcdId,4)&1)!=0;
        lcdId.release();
        nvtFw=static_cast<nvt_fw *>(IOMalloc(sizeof(nvt_fw)));if (!nvtFw) return -12;
        rc=boe?nvt_fw_parse(nvtFw,nvt_fw_boe,nvt_fw_boe_len):nvt_fw_parse(nvtFw,nvt_fw_csot,nvt_fw_csot_len);
        IOLog("Pagani touch firmware %s: %u partitions rc %d\n",boe?"BOE":"CSOT",nvtFw->count,rc);
        if (rc) return rc;
        observe("NVT download");rc=nvt_download_firmware(&nvt,nvtFw);
        IOLog("Pagani touch download rc %d attempt %u stage %u stage-rc %d\n",rc,nvt.dl_attempt,nvt.dl_stage,nvt.dl_rc);
        sample=(nvt.dl_attempt<<8)|nvt.dl_stage;
        if (rc) return rc;
        observe("NVT start");rc=nvt_start_scanning(&nvt,&nvtInfo);
        IOLog("Pagani touch fw ver %u type %u grid %ux%u max %ux%u rc %d\n",nvtInfo.fw_ver,nvtInfo.fw_type,nvtInfo.x_num,
            nvtInfo.y_num,nvtInfo.abs_x_max,nvtInfo.abs_y_max,rc);
        sample=nvtInfo.fw_ver;
        // The piano THP firmware does not take the host commands (it never clears them; the UEFI driver sends none), so a timeout
        // there is not fatal: frames run without them.
        if (rc==NVT_ERR_BUS) return rc;
        nvtCore=static_cast<nvt_touch_core *>(IOMalloc(sizeof(nvt_touch_core)));
        nvtFrame=static_cast<uint8_t *>(IOMalloc(NVT_THP_FRAME_LEN));
        nvtMatrix=static_cast<int16_t *>(IOMalloc(NVT_NODES*sizeof(int16_t)));
        if (!nvtCore || !nvtFrame || !nvtMatrix) return -12;
        nvt_touch_reset(nvtCore);
        controller.ready=1;phase="NVT frames";return 0;
    }
#else
    int setupTouch() {
        observe("GCC map");
        if (!gcc.map(PAGANI_GCC_BASE,0x60000)) return -12;
        auto g=gcc.io();unsigned hz=0,select=0;
        observe("SPI clock");
        int rc=sm8750_touch_clock(&g,&hz,&select);if (rc) return rc;
        sample=hz;observe("SPI map");
        if (!se.map(PAGANI_TOUCH_SE,0x4000) || !wrapper.map(PAGANI_TOUCH_WRAPPER,0x1000)) return -12;
        spi.se={&se,PeripheralWindow::read,PeripheralWindow::write,PeripheralWindow::delay};
        spi.wrapper={&wrapper,PeripheralWindow::read,PeripheralWindow::write,PeripheralWindow::delay};
        // shortcut: retain boot firmware SPI microcode until cold firmware loading is implemented.
        observe("GENI firmware");rc=geni_spi_read_info(&spi);if (rc) return rc;
        sample=spi.info.geni_status;
        if (spi.info.protocol!=GENI_PROTO_SPI) return GENI_ERR_PROTO;
        if (spi.info.geni_status&(GENI_STATUS_M_ACTIVE|(1u<<12))) return -16;
        spi.poll_only=1;spi.poll_limit=20000;
        if (spi.info.fifo_disabled) {
            rc=setupDma();if (rc) return rc;
            // ArmCpu masks unused SPIs at distributor init; retain SE status generation for polling.
            spi.poll_only=2;
        }
        observe("SPI init");rc=geni_spi_init(&spi);if (rc) return rc;
        observe("SPI mode");rc=geni_spi_setup(&spi,0,0);if (rc) return rc;
        observe("SPI rate");rc=geni_spi_set_clock(&spi,PAGANI_TOUCH_HZ,hz,select);if (rc) return rc;
        auto serial=se.io();observe("SPI timing");
        rc=sm8750_spi_timing(&serial,PAGANI_TOUCH_CS_DELAY,PAGANI_TOUCH_WORD_DELAY,&sample);if (rc) return rc;
        rc=setupTouchPower();if (rc) return rc;
        const unsigned gpios[]=PAGANI_TOUCH_PINS;
        const unsigned drives[]=PAGANI_TOUCH_DRIVES;
        for (unsigned i=0;i<6;++i) {
            sample=gpios[i];observe("touch GPIO");
            if (!pins[i].map(PAGANI_TLMM_GPIO(gpios[i]),0x1000)) return -12;
            savedPins[i]=PeripheralWindow::read(&pins[i],0);
            uint32_t cfg=(savedPins[i]&~0x3ffu)|(drives[i]<<6)|(i<4?4:0);
            if (i==4) {
                savedReset=PeripheralWindow::read(&pins[i],4)&2;
                PeripheralWindow::write(&pins[i],4,2); // Set RESET high before enabling output.
                cfg|=1u<<9;
            }
            PeripheralWindow::write(&pins[i],0,cfg);++pinsConfigured;
        }
        PeripheralWindow::write(&pins[4],4,0);IOSleep(PAGANI_TOUCH_RESET_MS);
        PeripheralWindow::write(&pins[4],4,2);IOSleep(PAGANI_TOUCH_STARTUP_MS);
        controller.bus={this,readTouch,writeTouch,sleepTouch};
        observe("TouchComm start");rc=s3910_start(&controller);
        observeTouch(rc);
        return rc;
    }
#endif
    IOReturn bindTouchEvents() {
        if (touchEvents) return touchEvents->isInactive()?kIOReturnNotReady:kIOReturnSuccess;
        IOService *driver=pagani_hid::findDriver(this,4);
        if (!driver) return kIOReturnNotReady;
        touchBinding=1;
        touchUsage=(pagani_hid::number(driver,"PrimaryUsagePage")<<16)|pagani_hid::number(driver,"PrimaryUsage");
        if (*reinterpret_cast<uintptr_t **>(driver)==pagani_original_hid_driver_vtable) touchBinding|=2;
        if (touchUsage==0xd0004) touchBinding|=4;
        auto digitizer=OSDynamicCast(OSDictionary,driver->getProperty("Digitizer"));
        if (driver->getProperty("Built-In")==kOSBooleanTrue && driver->getProperty("DisplayIntegrated")==kOSBooleanTrue &&
            digitizer && digitizer->getObject("collectionDispatch")==kOSBooleanTrue) touchBinding|=8;
        if (touchBinding!=15 || !pagani_hid::installDispatch(driver)) {driver->release();return kIOReturnUnsupported;}
        touchBinding|=16;touchEvents=driver;return kIOReturnSuccess;
    }
    void showTouchEvents() {
        if (!drawDue()) return;
        boot_progress_touch_hid(touchBinding,touchUsage,__atomic_load_n(&pagani_hid::events,__ATOMIC_RELAXED),
            __atomic_load_n(&pagani_hid::digitizers,__ATOMIC_RELAXED),__atomic_load_n(&pagani_hid::edges,__ATOMIC_RELAXED),
            lastInputXY,__atomic_load_n(&pagani_hid::lastXY,__ATOMIC_RELAXED),__atomic_load_n(&pagani_hid::lastMask,__ATOMIC_RELAXED));
    }
    bool send(const void *bytes,size_t size) {
        if (touch) {
            IOReturn rc=bindTouchEvents();
            if (rc) {observe(rc==kIOReturnNotReady?"HID wait":"HID bind",rc);return false;}
        }
        auto buffer=IOBufferMemoryDescriptor::withBytes(bytes,size,kIODirectionNone);
        IOReturn rc=buffer?handleReport(buffer,0,0):kIOReturnNoMemory;
        if (!rc) ++reports;
        observe("HID report",rc);
        OSSafeReleaseNULL(buffer);return rc==kIOReturnSuccess;
    }
    void releaseContacts() {
        bool active=false;for (auto &c:previous.contact) active|=c.active!=0;
        if (!active) return;
        s3910_frame empty={};uint8_t report[62];s3910_hid_report(&empty,&previous,report);
        if (send(report,sizeof report)) previous=empty;
    }
    // Input script (InputScript.h, pagani-input-script): the touch device runs the timeline from this timer, the key device
    // presses the script's keys from its poll (pagani_script_keys). Unarmed, nothing below runs.
    static uint64_t nowMs() {uint64_t ns=0;absolutetime_to_nanoseconds(mach_absolute_time(),&ns);return ns/1000000;}
    // The synthetic finger takes the real contacts' report builder and send, and counts as a parsed touch frame.
    static int scriptTouch(void *ctx,int down,uint16_t x,uint16_t y) {
        auto self=static_cast<PaganiInput *>(ctx);const s3910_frame frame=pagani_script_contact(down,x,y);
        uint8_t report[62];s3910_hid_report(&frame,&self->previous,report);
        if (!self->send(report,sizeof report)) return 0;
        self->previous=frame;__atomic_fetch_add(&pagani_input_touch_events,1u,__ATOMIC_RELAXED);return 1;
    }
    static void scriptKeys(void *,uint8_t bits,uint64_t until) {pagani_script_hold(bits,until);}
    static void scriptLog(void *,const char *line) {boot_progress_trace(line);}
    pagani_script_io scriptIo() {pagani_script_io io={this,scriptTouch,scriptKeys,scriptLog};return io;}
    void scriptEnd() {if (!script.armed) __atomic_store_n(&pagani_script_link.on,0u,__ATOMIC_RELEASE);}
    // Touch device start: pagani-input-script=0 or absent leaves everything as it was (no line, no state).
    void scriptArm() {
        uint32_t mask=0,at=PAGANI_SCRIPT_AT_DEFAULT;
        if (!PE_parse_boot_argn("pagani-input-script",&mask,sizeof mask) || !mask) return;
        PE_parse_boot_argn("pagani-input-script-at",&at,sizeof at);
        // pagani-input-script-at counts from here (the touch driver's start, a few seconds into the boot log): mach absolute time is
        // the raw counter on this board (cpu_base_timebase 0), which runs from the SoC reset, about a minute before XNU starts, so
        // an absolute deadline started the 2026-10-31 00:30 run's script some 59 s early, out of the log's surviving window.
        at+=(uint32_t)(nowMs()/1000);
        char line[PAGANI_SCRIPT_LINE];
        if (pagani_script_arm(&script,mask,at,screenWidth,screenHeight,line,sizeof line))
            __atomic_store_n(&pagani_script_link.on,1u,__ATOMIC_RELEASE);
        boot_progress_trace(line);
    }
    // A real contact report (pagani_script_real_frame): returns whether to deliver it.
    bool scriptFrame() {
        auto io=scriptIo();
        const int deliver=pagani_script_real_frame(&script,s3910_objects(controller.payload,controller.length),nowMs(),&io);
        scriptEnd();return deliver!=0;
    }
    void scriptTick() {
        const uint64_t ms=nowMs();
        if (!pagani_script_due(&script,ms)) return;
        pagani_script_input in={};
        in.ms=ms;
        in.display_off=(__atomic_load_n(&pagani_screen_state,__ATOMIC_ACQUIRE)&1u)!=0;
        in.keys_alive=(uint8_t)pagani_script_keys_alive(ms);
        in.controller_ready=controller.ready;
        in.hid_bound=script.running || bindTouchEvents()==kIOReturnSuccess;
        in.real_keys=__atomic_load_n(&pagani_script_link.real_keys,__ATOMIC_RELAXED);
        in.binding=touchBinding;
        in.real_touch=(uint8_t)pagani_script_real_contacts(&script,&previous);
        auto io=scriptIo();pagani_script_tick(&script,&in,&io);scriptEnd();
    }
    static void tick(OSObject *owner,IOTimerEventSource *) {
        auto self=static_cast<PaganiInput *>(owner);
        if (!__atomic_load_n(&self->running,__ATOMIC_ACQUIRE)) return;
        if (self->touch) {
#ifdef PAGANI_TOUCH_NVT
            if (self->controller.ready) {
                s3910_frame frame={};int rc=self->pollNvt(frame);
                self->sample=(self->nvtFrames&0xffff)|(self->nvtTypes<<16);
#else
            if (self->controller.ready) self->sample=PeripheralWindow::read(&self->pins[5],4)&1;
            if (self->controller.ready && !self->sample) {
                s3910_frame frame={};int rc=s3910_poll(&self->controller,&frame);
#endif
                if (rc==1) {
                    __atomic_fetch_add(&pagani_input_touch_events,1u,__ATOMIC_RELAXED);
                    self->sample=0;for (unsigned i=0;i<S3910_CONTACTS;++i) if (frame.contact[i].active) self->sample|=1u<<i;
                    for (auto &c:frame.contact) if (c.active) {self->lastInputXY=(uint32_t)c.x<<16|c.y;break;}
#ifdef PAGANI_TOUCH_NVT
                    // Bring-up: where the first finger is, twice a second, to check the axes against the screen.
                    if (self->sample && nowMs()-self->lastXYLogMs>=500) {
                        self->lastXYLogMs=nowMs();
                        IOLog("touch xy %u %u fingers %x reports %u\n",self->lastInputXY>>16,self->lastInputXY&0xffff,self->sample,(unsigned)self->reports);
                    }
#endif
                    if (!self->script.armed || self->scriptFrame()) {
                        uint8_t report[62];s3910_hid_report(&frame,&self->previous,report);
                        if (self->send(report,sizeof report)) self->previous=frame;
                    }
                    self->showTouchEvents();
                } else if (rc<0) {
                    self->controller.ready=0;self->touchError=rc;self->setProperty("InputError",(uint32_t)rc,32);
                } else {
#ifdef PAGANI_TOUCH_NVT
                    self->observe("NVT idle");
#else
                    self->sample=self->controller.code;self->observe("TouchComm idle");
#endif
                }
            }
            else if (self->controller.ready && !self->reports) self->observe("ATTN wait");
            if (!self->controller.ready) {self->releaseContacts();self->observeTouch(self->touchError);}
            if (self->script.armed) self->scriptTick();
        } else {
            uint64_t ns=0;absolutetime_to_nanoseconds(mach_absolute_time(),&ns);
            self->pollButtons(ns/1000000);
        }
        // shortcut: poll the ATTN line and PMIC levels until GIC detach/lifetime support is available.
        self->timer->setTimeoutMS(self->touch?8:PAGANI_KEYS_POLL_MS);
    }
    void cleanup() {
        __atomic_store_n(&running,false,__ATOMIC_RELEASE);
        if (timer) {timer->cancelTimeout();if (timerAdded) loop->removeEventSource(timer);}
        timerAdded=false;OSSafeReleaseNULL(timer);OSSafeReleaseNULL(loop);
        OSSafeReleaseNULL(touchEvents);
        for (unsigned i=0;i<pinsConfigured;++i) {
            if (i==4) PeripheralWindow::write(&pins[i],4,savedReset);
            PeripheralWindow::write(&pins[i],0,savedPins[i]);
        }
        pinsConfigured=0;for (auto &pin:pins) pin.release();
#ifdef PAGANI_TOUCH_NVT
        if (nvtScratch) IOFree(nvtScratch,kNvtScratch);
        if (nvtRx) IOFree(nvtRx,kNvtRx);
        if (nvtFw) IOFree(nvtFw,sizeof(nvt_fw));
        if (nvtCore) IOFree(nvtCore,sizeof(nvt_touch_core));
        if (nvtFrame) IOFree(nvtFrame,NVT_THP_FRAME_LEN);
        if (nvtMatrix) IOFree(nvtMatrix,NVT_NODES*sizeof(int16_t));
        nvtFw=nullptr;nvtCore=nullptr;nvtFrame=nullptr;nvtMatrix=nullptr;
        nvtScratch=nvtRx=nullptr;
#endif
        releasePlusKey();
        if (se.base && spi.use_dma) PeripheralWindow::write(&se,SE_IRQ_EN,0);
        // shortcut: retain the exposed 8 KiB until reboot; DMA reset completion is not yet qualified on this board.
        if (dmaExposed) dmaMemory=nullptr;
        OSSafeReleaseNULL(dmaMemory);
        core.release();cnfg.release();obs.release();se.release();wrapper.release();gcc.release();smmu.release();
        // Leave shared rail votes enabled; cleanup must not cut power to another consumer.
        rsc.release();cmddb.release();
    }
public:
    void selectTouch() {touch=true;}
    IOWorkLoop *getWorkLoop() const override {return loop;}
    OSString *newTransportString() const override {return OSString::withCString(touch?"SPI":"SPMI");}
    OSString *newProductString() const override {return OSString::withCString(touch?"Pagani S3910":"Pagani buttons");}
    IOReturn newReportDescriptor(IOMemoryDescriptor **out) const override {
        if (!out) return kIOReturnBadArgument;
        *out=IOBufferMemoryDescriptor::withBytes(touch?touchDescriptor:buttonsDescriptor,
            touch?sizeof touchDescriptor:sizeof buttonsDescriptor,kIODirectionNone);
        return *out?kIOReturnSuccess:kIOReturnNoMemory;
    }
    bool start(IOService *provider) override {
        observe("work loop");
        loop=IOWorkLoop::workLoop();if (!loop) {observe(phase,kIOReturnNoMemory);return false;}
        setProperty("HIDDefaultBehavior",true);setProperty("Built-In",true);
        if (touch) {
            setProperty("DisplayIntegrated",true);setProperty("DigitizerCollectionDispatch",true);
            PE_Video video={};
            if (PE_current_console(&video) || video.v_rotate || video.v_width<=2*PAGANI_TOUCH_EDGE_PIXELS ||
                video.v_height<=2*PAGANI_TOUCH_EDGE_PIXELS || !pagani_hid::installPersonality()) {
                observe("HID identity",kIOReturnUnsupported);cleanup();return false;
            }
            pagani_hid::edgeX=PAGANI_TOUCH_EDGE_PIXELS*65536u/video.v_width;
            pagani_hid::edgeY=PAGANI_TOUCH_EDGE_PIXELS*65536u/video.v_height;
            screenWidth=(uint32_t)video.v_width;screenHeight=(uint32_t)video.v_height;
        }
        int rc=touch?setupTouch():setupButtons();
        if (rc) {observe(phase,rc);IOLog("Pagani %s %s failed: %d\n",touch?"touch":"buttons",phase,rc);cleanup();return false;}
        observe("HID start");
        if (!IOHIDDevice::start(provider)) {observe(phase,kIOReturnError);cleanup();return false;}
        timer=IOTimerEventSource::timerEventSource(this,tick);
        timerAdded=timer && loop->addEventSource(timer)==kIOReturnSuccess;
        if (!timerAdded) {observe("timer",kIOReturnNoResources);IOHIDDevice::stop(provider);cleanup();return false;}
        observe("ready");
        if (touch) scriptArm();
        __atomic_store_n(&running,true,__ATOMIC_RELEASE);timer->setTimeoutMS(1);return true;
    }
    void stop(IOService *provider) override {
        __atomic_store_n(&running,false,__ATOMIC_RELEASE);
        if (timer) {timer->cancelTimeout();if (timerAdded) {loop->removeEventSource(timer);timerAdded=false;}}
        if (script.armed) {auto io=scriptIo();pagani_script_abort(&script,"driver stopped",&io);scriptEnd();}
        // Release the keys but keep the ring/silent level: a switch does not move when its driver stops.
        if (touch) releaseContacts();else {uint8_t report[]={2,(uint8_t)(keys.ringer.ring?PAGANI_REPORT_RING:0)};send(report,2);}
        IOHIDDevice::stop(provider);cleanup();
    }
    void free() override {cleanup();IOHIDDevice::free();}
};
OSDefineMetaClassAndStructors(PaganiInput,IOHIDDevice)
extern "C" IOService *pagani_input_allocate(bool touch) {
    auto device=new PaganiInput;if (device && touch) device->selectTouch();return device;
}
