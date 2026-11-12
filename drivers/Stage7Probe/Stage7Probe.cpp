// SPDX-License-Identifier: BSD-2-Clause-Patent
// Adapted from UniXNU-sheng's LegacyDisplayProbe and KeyStoreProbe for SM8750.
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOUserClient.h>
#include <libkern/c++/OSData.h>
#include <libkern/c++/OSSymbol.h>
#include <libkern/c++/OSKext.h>
#include <pexpert/pexpert.h>
#include <mach/kmod.h>
#include <kern/clock.h>
#include <kern/thread_call.h>
#include "../BootProgress/PeripheralStatus.h"
extern "C" kmod_info_t g_kernel_kmod_info;
extern "C" kern_return_t OSKextLoadKextWithIdentifier(const char *);
extern "C" void native_legacy_display_static_init(void) __asm__("__GLOBAL__sub_I_NativeLegacyDisplay.cpp");
// GPU scanout hand-off (sheng-fb-gpu=1, ARCHITECTURE 5.3.2, M9): the presenter the display offers its frames to (NativeGOPPresenter.cpp,
// a late object in the same archive).
extern "C" void native_gop_presenter_static_init(void) __asm__("__GLOBAL__sub_I_NativeGOPPresenter.cpp");
extern "C" IOService *native_gop_presenter_create(void);
extern "C" void native_gop_presenter_set_source(IOService *presenter,IOService *nub);
extern "C" bool native_gop_gpu_offer(IOService *,void *display,IOMemoryDescriptor *,uint64_t stride,uint32_t width,uint32_t height);
extern "C" bool native_gop_gpu_offer_frame(IOService *,void *display,IOMemoryDescriptor *,uint64_t stride,uint32_t width,uint32_t height,
                                           OSObject *surface,uint64_t targetOffset);
// The display processor probe and underrun monitor (NativeMobileDisplay DpuProbe.cpp; pagani-dpu-probe, pagani-te, pagani-fb-flip).
extern "C" void native_dpu_probe_start(void);
extern "C" void native_keyevents_static_init(void) __asm__("__GLOBAL__sub_I_NativeKeyStoreEvents.cpp");
extern "C" IOService *native_legacy_display_allocate(void),*native_keyevents_allocate(void);
extern "C" void pagani_input_static_init(void) __asm__("__GLOBAL__sub_I_PaganiInput.cpp");
extern "C" void pagani_panel_static_init(void) __asm__("__GLOBAL__sub_I_PaganiPanel.cpp");
extern "C" IOService *pagani_input_allocate(bool),*pagani_panel_allocate(void);
// The A830 GPU start (UniXNU-Pagani_SM8750 package/PaganiGpuBoard.cpp, appended to the Adreno830 archive) replaces this
// default. The kernel is linked statically, where an undefined weak reference is an error, so the default is a weak
// definition; a kernel without the GPU driver keeps it and pagani-gpu=1 does nothing.
extern "C" __attribute__((weak)) void pagani_gpu_start(void) {}
// The ADSP bring-up (UniXNU-Pagani-Drivers drivers/PaganiAdsp, appended to the Adreno830 archive) replaces this default; it only
// starts its own kernel thread (pagani-adsp=1) and returns at once.
extern "C" __attribute__((weak)) void pagani_adsp_start(void) {}
// The battery power source (drivers/PaganiBattery, appended to the Adreno830 archive) replaces this default the same way. It reads
// its own boot argument (pagani-battery=1; pagani-adsp=1 does not imply it) and returns at once without it, so it is called unconditionally.
extern "C" __attribute__((weak)) void pagani_battery_start(void) {}
// The audio service (drivers/PaganiAudio, component PaganiAudio) replaces this default the same way. It reads its own boot argument
// pagani-audio and returns at once without it, so it is called unconditionally.
extern "C" __attribute__((weak)) void pagani_audio_start(void) {}
// The vibrator (drivers/PaganiHaptics, appended to the Adreno830 archive) replaces this default the same way. It reads its own boot
// argument pagani-haptics (absent: off; 2: dry run) and returns at once; its own kernel thread does the rest.
extern "C" __attribute__((weak)) void pagani_haptics_start(void) {}
// The speakers (drivers/PaganiSpeaker, appended to the Adreno830 archive) replace this default the same way. It reads its own boot
// argument pagani-speaker (absent: off, nothing allocated, mapped or started) and returns at once; its own kernel thread does the rest.
extern "C" __attribute__((weak)) void pagani_speaker_start(void) {}
// The Wi-Fi (drivers/PaganiWifi, appended to the Adreno830 archive) replaces this default the same way. It reads its own boot
// argument pagani-wifi (absent: off) and returns at once; its own kernel thread does the rest.
extern "C" __attribute__((weak)) void pagani_wifi_start(void) {}
// The Bluetooth (drivers/PaganiBt, appended to the Adreno830 archive) replaces this default the same way. It reads its own boot argument
// pagani-bt (absent: off) and returns at once; its own kernel thread does the rest and ends.
extern "C" __attribute__((weak)) void pagani_bt_start(void) {}
// The camera (drivers/PaganiCamera, appended to the Adreno830 archive) replaces this default the same way. It reads its own boot argument
// pagani-cam=N (absent: off) and returns at once; its own kernel thread goes to level N and ends. With pagani-torch-ios=1 it also publishes
// the plug-in's log service; nothing else.
extern "C" __attribute__((weak)) void pagani_camera_start(void) {}
// The sensors (drivers/PaganiSensors, appended to the Adreno830 archive) replace this default the same way. It reads its own boot argument
// pagani-sensors (absent or 0: off, nothing registered) and returns at once; it opens PaganiAdsp IPC router ports, so it comes after
// pagani_adsp_start, and its own kernel thread does the rest.
extern "C" int piano_usb_bringup(void);   // defined in PianoUsb.cpp, included at the end of this file
extern "C" __attribute__((weak)) void pagani_sensors_start(void) {}
// The torch (drivers/PaganiTorch, appended to the Adreno830 archive) replaces this default the same way. It reads its own boot argument
// pagani-torch (absent: off; 2: dry run) and returns at once; its own kernel thread does the rest.
extern "C" __attribute__((weak)) void pagani_torch_start(void) {}
static IOService *pendingDisplay,*gpuPresenter;
static bool displayInitialized,ephemeralState,frameStatus;
static uint64_t frames,clients,aksCalls,aksOpens;
extern "C" {
// Scanout copy statistics in microseconds: last, largest, count, sum (BootProgress shows them on the platform row).
volatile uint64_t native_legacy_copy_stats[4];
// GPU hand-off: frames the display took back after an offer (refused offers are not counted) and the last status.
volatile uint64_t native_legacy_gpu_fallbacks;
volatile uint32_t native_legacy_gpu_last_status;
}

static void progress(const char *detail,uint64_t value) {boot_progress_display_status(detail,value);}
static bool enabled(const char *name) {
    unsigned value=0;return PE_parse_boot_argn(name,&value,sizeof value) && value==1;
}
static bool registerClass(void (*initializer)(void)) {
    void *handle=OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if(!handle)return false;
    initializer();return OSMetaClass::postModLoad(handle)==kIOReturnSuccess;
}
static bool startPeripheral(IOService *service,const char *label,unsigned device) {
    IOService *provider=IOService::getResourceService();
    bool attached=service && provider && service->init() && service->attach(provider);
    if (!attached) boot_progress_peripheral(device,"service attach",kIOReturnNoResources,0,0);
    bool started=attached && service->start(provider);
    progress(label,started);
    if (!started) {if (attached) service->detach(provider);OSSafeReleaseNULL(service);}
    // A started peripheral retains its service lifetime until reboot, like the display provider.
    return started;
}
// What became of the panel's service, for the log: a service reports by itself (PaganiPanel.cpp), but one that never came up has nothing
// that would, and the events ring has lost the boot's records long before anyone reads it. So the line comes back on a timer, the first
// time when the boot's records are out, as long as there is no service. No effect without the ramoops log.
enum { PANEL_ABSENT_FIRST_S=45, PANEL_ABSENT_S=600 };
static char panelAbsent[96];
static thread_call_t panelAbsentCall;
static void panelAbsentRepeat(unsigned seconds) {
    uint64_t deadline=0;clock_interval_to_deadline(seconds,NSEC_PER_SEC,&deadline);
    thread_call_enter_delayed(panelAbsentCall,deadline);
}
static void panelAbsentReport(thread_call_param_t,thread_call_param_t) {
    boot_progress_trace(panelAbsent);
    panelAbsentRepeat(PANEL_ABSENT_S);
}
static thread_call_t touchLaterCall;
static void touchLater(thread_call_param_t,thread_call_param_t) {
    startPeripheral(pagani_input_allocate(true),"touch started (delayed)",PAGANI_TOUCH);
}
static void startPeripherals(bool hidReady) {
    boot_progress_peripheral_enable(enabled("pagani-peripheral-status"));
    boot_progress_platform_enable(enabled("pagani-platform-status"));
    frameStatus=enabled("pagani-platform-status") || enabled("pagani-peripheral-status");
    for (unsigned i=0;i<PAGANI_SPI;++i) boot_progress_peripheral(i,"disabled",0,0,0);
    if (enabled("pagani-touch") || enabled("pagani-buttons")) {
        bool ready=hidReady && registerClass(pagani_input_static_init);
        const char *stage=hidReady?"HID class":"HID resource";
        if (enabled("pagani-buttons")) {
            boot_progress_peripheral(PAGANI_KEYS,stage,ready?0:kIOReturnNotReady,0,0);
            if (ready) startPeripheral(pagani_input_allocate(false),"physical buttons started",PAGANI_KEYS);
        }
        if (enabled("pagani-touch")) {
            boot_progress_peripheral(PAGANI_TOUCH,stage,ready?0:kIOReturnNotReady,0,0);
            // pagani-touch-delay=<s>: start touch that long after the display instead of here, for bring-up on a board whose
            // touch setup can still hang the bus (the boot and a USB console are up by then).
            unsigned delay=0;PE_parse_boot_argn("pagani-touch-delay",&delay,sizeof delay);
            if (ready && delay && (touchLaterCall=thread_call_allocate(touchLater,nullptr))) {
                uint64_t deadline=0;clock_interval_to_deadline(delay,NSEC_PER_SEC,&deadline);
                thread_call_enter_delayed(touchLaterCall,deadline);
            } else if (ready) startPeripheral(pagani_input_allocate(true),"S3910 touch started",PAGANI_TOUCH);
        }
    }
    // pagani-panel is a bit mask since the brightness work: bit 0 runs the panel service, the others are PaganiPanel's own (PanelBrightness.h).
    // Without bit 0 nothing here runs: no AppleARMPlatform, no class, no backlight for the system to find.
    unsigned panel=0;PE_parse_boot_argn("pagani-panel",&panel,sizeof panel);
    if (panel&1) {
        kern_return_t loaded=OSKextLoadKextWithIdentifier("com.apple.driver.AppleARMPlatform");
        progress("AppleARMPlatform load",(uint32_t)loaded);
        bool ready=!loaded && registerClass(pagani_panel_static_init);
        boot_progress_peripheral(PAGANI_PANEL,loaded?"platform load":"panel class",loaded?loaded:(ready?0:kIOReturnNotReady),0,0);
        bool started=ready && startPeripheral(pagani_panel_allocate(),"DSI panel control started",PAGANI_PANEL);
        snprintf(panelAbsent,sizeof panelAbsent,"panel: stage7 AppleARMPlatform load %x, class %s, service %s",(uint32_t)loaded,
                 ready?"registered":loaded?"not tried":"failed",started?"started":ready?"failed":"not tried");
        boot_progress_trace(panelAbsent);
        if (!started && boot_progress_trace_enabled() && (panelAbsentCall=thread_call_allocate(panelAbsentReport,nullptr))) panelAbsentRepeat(PANEL_ABSENT_FIRST_S);
    }
    // The USB gadget (piano-usb=1) is started by NativeUfs: the same windows are known readable
    // there, while by this point in the boot the core can already be gated.
}

// The GPU hand-off presenter (sheng-fb-gpu=1): attached to the resource service, not under the display subsystem's node (the original
// display code enumerates that node's children: sheng int-075/077, SpringBoard found no display and crashed with the presenter there,
// registered or not); its framebuffer memory comes from the display's nub. The Adreno driver reaches it through native_gop_handoff_*.
static void startPresenter(IOService *nub) {
    if(!enabled("sheng-fb-gpu") || gpuPresenter)return;
    if(!registerClass(native_gop_presenter_static_init)){progress("GPU hand-off class failed",0);return;}
    IOService *presenter=native_gop_presenter_create();
    IOService *home=IOService::getResourceService();
    bool attached=false;
    if(presenter && home)native_gop_presenter_set_source(presenter,nub);
    if(presenter && home && presenter->init() && (attached=presenter->attach(home)) && presenter->start(home))gpuPresenter=presenter;
    else {if(attached)presenter->detach(home);OSSafeReleaseNULL(presenter);}
    progress("GPU hand-off presenter",gpuPresenter!=nullptr);
}
static bool console(PE_Video &video) {
    return !PE_current_console(&video) && video.v_baseAddr && video.v_width && video.v_height &&
        video.v_width<=16384 && video.v_height<=16384 && video.v_depth==32 && !video.v_rotate &&
        video.v_rowBytes>=video.v_width*4 && video.v_rowBytes<=UINT64_MAX/video.v_height;
}
extern "C" {
__attribute__((noreturn)) void native_legacy_backend_guard(unsigned slot,void *object,void *caller) {
    panic("SM8750 unsupported display slot %u object %p caller %p",slot,object,caller);
}
bool native_legacy_gop_mapping_observed(IOService *,IOMemoryMap *mapping,uint32_t width,uint32_t height) {
    progress("GOP mapping ready",((uint64_t)width<<32)|height);
    return mapping && mapping->getVirtualAddress();
}
void native_legacy_registration_observed(IOService *) {progress("display registered",1);}
void native_legacy_swap_stage(unsigned) {}
volatile uint64_t native_gop_vsync[3];
void native_legacy_vsync_observed(unsigned event,unsigned hz) {
    if(event==4 || event==5)__atomic_store_n(&native_gop_vsync[0],hz,__ATOMIC_RELAXED);
    if(event==1)__atomic_fetch_add(&native_gop_vsync[1],1,__ATOMIC_RELAXED);
    if(event==3)__atomic_fetch_add(&native_gop_vsync[2],1,__ATOMIC_RELAXED);
    if(event==3)boot_progress_display_event(PAGANI_DISPLAY_VSYNC_LATE);
    else if(event==4)progress("software display refresh Hz",hz);
    else if(event==6)progress("display TE locked Hz",hz);
    else if(event==8)progress("display TE lost",hz);
    else if(event==9)progress("panel switched to Hz",hz);
}
void native_legacy_client_enter(task_t,uint32_t type) {progress("display client enter",type);}
void native_legacy_client_observed(IOReturn status,IOUserClient *) {progress("display client",status?(uint32_t)status:++clients);}
void native_legacy_pm_observed(unsigned stage,IOReturn status,unsigned long) {if(status)progress("display PM error",((uint64_t)stage<<32)|(uint32_t)status);}
void native_legacy_frame_observed(unsigned,uint64_t,uint64_t,uint64_t,uint64_t,uint64_t) {}
void native_legacy_frame_colors_observed(const uint32_t *,const uint64_t *,uint64_t) {}
void native_legacy_swap_validation_observed(bool accepted,unsigned reason,const void *,const void *) {if(!accepted)progress("display swap rejected",reason);}
void native_legacy_surface_rejected(uint64_t,uint64_t,uint32_t format,uint32_t,uint64_t,uint64_t,uint64_t) {progress("surface format rejected",format);}
bool native_legacy_prepare_scanout(void) {return boot_progress_display_handoff()!=0;}
void native_legacy_transfer_observed(IOReturn status,const void *,uint64_t) {
    if(status) {
        // A failed first copy must not permanently hide the diagnostic console.
        if(!frames)boot_progress_display_failed();
        progress("display copy failed",(uint32_t)status);
    } else {
        // The frame count goes to BootProgress every frame (one store; the panic screen prints it). The per-frame status line is diagnostics:
        // only the first frame and the opted-in overlays pay for it.
        boot_progress_display_frames(++frames);
        if(frames==1 || frameStatus) {progress("system frame copied",frames);boot_progress_peripheral_frame();}
    }
}
void native_legacy_copy_timing_observed(uint64_t ticks,uint64_t) {
    uint64_t hz=0;__asm__ volatile("mrs %0, cntfrq_el0":"=r"(hz));
    const uint64_t us=hz?ticks*1000000/hz:0;
    native_legacy_copy_stats[0]=us;
    if(us>native_legacy_copy_stats[1])native_legacy_copy_stats[1]=us;
    native_legacy_copy_stats[2]=native_legacy_copy_stats[2]+1;
    native_legacy_copy_stats[3]=native_legacy_copy_stats[3]+us;
}
void native_legacy_retirement_observed(IOService *) {}
// Every frame that reached the panel (NativeLegacyDisplay frameDone and the blank fill), with its path (PAGANI_FRAME_CPU / _GPU) and mach
// time: BootProgress's frame-rate ring behind the FPS row and the ramoops F/J records (pagani-fps). No lock: two atomic adds and a store.
void native_legacy_frame_presented(unsigned path,uint64_t when) {boot_progress_frame(path,when);}
// The same with where the frame's time went (arrival and the gap since the previous one, offer, taken, copied; PeripheralStatus.h
// pagani_frame_timing): its breakdown goes into the ring beside it, for the SLOW row (text row 8) and the J records. No lock.
void native_legacy_frame_timed(unsigned path,uint64_t when,const pagani_frame_timing *timing) {boot_progress_frame_timed(path,when,timing);}
// The display path's stage (PAGANI_STAGE_*: what it waits for now, since `when`) and its events per second (PAGANI_DISPLAY_*: refused
// offers, failed GPU copies, failures after early retirement, empty waits of the GPU client). One atomic store or add, any thread.
void native_legacy_frame_stage(unsigned stage,uint64_t when) {boot_progress_frame_stage(stage,when);}
void native_legacy_display_event(unsigned event) {boot_progress_display_event(event);}
bool native_legacy_gpu_offer(void *display,IOMemoryDescriptor *memory,uint64_t stride,uint32_t width,uint32_t height) {
    return gpuPresenter && native_gop_gpu_offer(gpuPresenter,display,memory,stride,width,height);
}
bool native_legacy_gpu_offer_frame(void *display,IOMemoryDescriptor *memory,uint64_t stride,uint32_t width,uint32_t height,OSObject *surface,
                                   uint64_t targetOffset) {
    return gpuPresenter && native_gop_gpu_offer_frame(gpuPresenter,display,memory,stride,width,height,surface,targetOffset);
}
void native_legacy_gpu_fallback_observed(int status) {
    native_legacy_gpu_last_status=(uint32_t)status;native_legacy_gpu_fallbacks=native_legacy_gpu_fallbacks+1;
    if(native_legacy_gpu_fallbacks==1)progress("GPU frame taken back",(uint32_t)status);
}
void native_keyevents_open_refused(void) {progress("key state open refused",0);}
bool native_aks_state_enabled(void) {return ephemeralState;}
uint32_t native_aks_lock_state(void) {return 0;}
void native_aks_call(uint32_t selector,int32_t result) {++aksCalls;if(result)progress("key operation unavailable",selector);}
void native_aks_opened(void) {++aksOpens;}
}
static void startKeyState(void) {
    if(!enabled("offline-key-events") || !enabled("sheng-aks"))return;
    IORegistryEntry *chosen=IORegistryEntry::fromPath("/chosen",gIODTPlane);
    OSData *data=chosen?OSDynamicCast(OSData,chosen->getProperty("ephemeral-storage")):nullptr;
    uint32_t ephemeral=0;
    if(data && data->getLength()==sizeof ephemeral)memcpy(&ephemeral,data->getBytesNoCopy(),sizeof ephemeral);
    OSSafeReleaseNULL(chosen);
    if(ephemeral!=1)return;
    const OSSymbol *name=OSSymbol::withCString("AppleKeyStore");
    bool absent=name && !OSMetaClass::getMetaClassWithName(name);
    OSSafeReleaseNULL(name);
    IOService *provider=IOService::getResourceService();
    if(!absent || !provider || !registerClass(native_keyevents_static_init))return;
    IOService *service=native_keyevents_allocate();
    bool attached=service && service->init() && service->attach(provider);
    // Publish state queries only after the volatile-storage guard passed.
    ephemeralState=true;
    if(attached && service->start(provider)){progress("volatile key state registered",1);return;}
    ephemeralState=false;
    if(attached)service->detach(provider);
    OSSafeReleaseNULL(service);
    progress("volatile key state failed",0);
}
extern "C" void native_legacy_display_probe(void) {
    if(displayInitialized)return;
    displayInitialized=true;
    progress("display class registration enter",0);
    if(!registerClass(native_legacy_display_static_init)){progress("display class failed",0);return;}
    PE_Video video={};
    if(!console(video)){progress("GOP console unavailable",0);return;}
    progress("display allocation enter",0);
    IOService *display=native_legacy_display_allocate();
    progress("display init enter",display!=nullptr);
    if(display && display->init())pendingDisplay=display;
    else OSSafeReleaseNULL(display);
    progress("display class initialized",pendingDisplay!=nullptr);
}
extern "C" void native_legacy_display_late_probe(void) {
    progress("key state start enter",0);
    startKeyState();
    // Load original event dispatch before attaching physical HID devices.
    progress("IOHIDFamily load enter",0);
    progress("IOHIDFamily load",(uint32_t)OSKextLoadKextWithIdentifier("com.apple.iokit.IOHIDFamily"));
    progress("IOHIDEventDriver load enter",0);
    progress("IOHIDEventDriver matching",(uint32_t)OSKext::loadKextWithIdentifier(
        "com.apple.iokit.IOHIDEventDriver",true,false,kOSKextExcludeNone,kOSKextExcludeNone,nullptr));
    OSDictionary *hidMatching=IOService::serviceMatching("IOHIDResource");
    IOService *hidResource=hidMatching?IOService::waitForMatchingService(hidMatching,5000000000ULL):nullptr;
    progress("IOHIDResource ready",hidResource!=nullptr);
    bool hidReady=hidResource!=nullptr;
    OSSafeReleaseNULL(hidResource);OSSafeReleaseNULL(hidMatching);
    IOService *display=pendingDisplay;
    pendingDisplay=nullptr;
    if(!display){progress("no initialized display",0);return;}
    progress("IOSurface load enter",0);
    kern_return_t loaded=OSKextLoadKextWithIdentifier("com.apple.iokit.IOSurface");
    OSDictionary *matching=IOService::nameMatching("IOCoreSurfaceRoot");
    IOService *surface=matching?IOService::waitForMatchingService(matching,5000000000ULL):nullptr;
    bool surfaceReady=!loaded && surface;
    OSSafeReleaseNULL(surface);OSSafeReleaseNULL(matching);
    progress("IOSurface ready",surfaceReady);
    PE_Video video={};
    if(!surfaceReady || !console(video)){display->release();return;}
    IOPlatformDevice *nub=new IOPlatformDevice;
    IOService *parent=IOService::getResourceService();
    IORegistryEntry *dtParent=IORegistryEntry::fromPath("/arm-io",gIODTPlane);
    IODeviceMemory *memory=IODeviceMemory::withRange(video.v_baseAddr,video.v_rowBytes*video.v_height);
    OSArray *ranges=OSArray::withCapacity(1);
    const char type[]="display-subsystem";
    OSData *typeData=OSData::withBytes(type,sizeof type);
    bool nubAttached=false,nubStarted=false,dtAttached=false,attached=false,started=false;
    if(nub && parent && dtParent && memory && ranges && typeData && nub->init() && ranges->setObject(memory)) {
        nub->setDeviceMemory(ranges);nub->setName("NativeGOPFramebuffer");
        nubAttached=nub->attach(parent);
        nubStarted=nubAttached && nub->start(parent);
        if(nubStarted && nub->setProperty("device_type",typeData)) {
            nub->setName("gop-display",gIODTPlane);
            dtAttached=nub->attachToParent(dtParent,gIODTPlane);
            SInt32 score=0;
            progress("GOP display probe enter",dtAttached);
            attached=dtAttached && display->probe(nub,&score)==display && display->attach(nub);
            progress("GOP display start enter",attached);
            started=attached && display->start(nub);
        }
    }
    progress("GOP display started",started);
    if (started) {
        startPeripherals(hidReady);
        startPresenter(nub);
        native_dpu_probe_start();
    }
    // A830 bring-up, synchronous: IOSurface and the display are up and the touch DRV-2 votes are done, so the GPU's own
    // RPMh votes do not race them; backboardd finds the Metal service once launchd runs (Adreno830 ARCHITECTURE 3.1).
    if (enabled("pagani-gpu")) pagani_gpu_start();
    // The audio DSP after the GPU (their QMP, SCM and DRV-2 uses then never overlap at boot); asynchronous, never blocks the display.
    if (enabled("pagani-adsp")) pagani_adsp_start();
    // The battery after the GPU and ADSP starts, and after the touch setup's unlocked update of the GCC vote register its I2C clocks
    // share; its own kernel thread does the rest (gauge bus, battery manager channel), never blocking the display.
    pagani_battery_start();
    // The audio service: shared rings and a kernel thread only (no hardware with its silent backend); the plug-in finds it once
    // launchd starts audiomxd.
    pagani_audio_start();
    // The speakers after the audio service, which they serve through the adsp backend; asynchronous, never blocks the display.
    pagani_speaker_start();
    // The vibrator after the audio service (its API is for the audio side) and the battery (its low-battery rule reads the battery's
    // status); asynchronous, never blocks the display.
    pagani_haptics_start();
    // The Wi-Fi after the ADSP, battery and haptics starts (its RPMh, QMP and GCC vote uses then follow theirs at boot); asynchronous,
    // never blocks the display.
    pagani_wifi_start();
    // The Bluetooth after the Wi-Fi: both vote the same rails on DRV-2, the same values, never a disable; asynchronous, never blocks the display.
    pagani_bt_start();
    // The camera after the Bluetooth: its rails are its own, its ARC votes never touch mx; asynchronous, never blocks the display.
    pagani_camera_start();
    // The sensors last: the iOS sensor services (registered before launchd, MobileGestalt asks early) and, with pagani-adsp=1, the
    // sensor hub's client on PaganiAdsp's IPC router (its registry exists once pagani_adsp_start ran); asynchronous, never blocks.
    pagani_sensors_start();
    // The torch: no dependency on the others; asynchronous, never blocks the display.
    pagani_torch_start();
    OSSafeReleaseNULL(ranges);OSSafeReleaseNULL(memory);OSSafeReleaseNULL(typeData);
    if(!started) {
        if(attached)display->detach(nub);
        if(dtAttached)nub->detachFromParent(dtParent,gIODTPlane);
        if(nubStarted)nub->stop(parent);
        if(nubAttached)nub->detach(parent);
        OSSafeReleaseNULL(nub);OSSafeReleaseNULL(display);
    }
    // On success the service and its real framebuffer provider live until reboot.
    OSSafeReleaseNULL(dtParent);
}

// The USB gadget lives in its own file but is compiled here: the arm driver recipe compiles exactly
// one source file per component, so a second .cpp in the directory is never built.
#include "PianoUsb.cpp"
