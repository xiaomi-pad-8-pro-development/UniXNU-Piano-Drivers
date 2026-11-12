// SPDX-License-Identifier: BSD-2-Clause-Patent
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include "PeripheralStatus.h"
#include "RamoopsLog.h"
#include "GunyahWdog.h"
#include "UfsLog.h"
#include "FrameRate.h"
#include "CrashTrace.h"
#if !defined(BOOT_PROGRESS_TEST) || defined(BOOT_PROGRESS_STALL_TEST)
#include "StallProbe.h"
extern int PE_parse_boot_argn(const char *arg_string, void *arg_ptr, int max_arg);
#endif
#ifdef BOOT_PROGRESS_TEST
// The host test (Test/console.c) supplies the frame-rate clock and the boot-args.
static uint64_t test_clock;
static int test_boot_arg(const char *name, uint32_t *value);
#endif
#ifndef BOOT_PROGRESS_TEST
#include <pexpert/pexpert.h>
#include <pexpert/arm64/boot.h>
#include <machine/machine_routines.h>
#include <kern/clock.h>
#include <kern/thread_call.h>
#include <kern/boot_progress.h>
#include <kern/early_boot_progress.h>
#include <arm64/proc_reg.h>
_Static_assert(ARM_PGBYTES == 4096 && (T0SZ_BOOT == 25 || T0SZ_BOOT == 26) && CACHE_ATTRINDX_WRITECOMB == 1,
               "Early framebuffer map requires the native 4 KiB bootstrap ABI");
#endif
#include <DiagFont.h>
#ifndef BOOT_PROGRESS_TEST
#include <BootLogo.h>
#endif

_Static_assert(offsetof(boot_args, Video) == 0x28, "DarwinBoot video ABI");
_Static_assert(offsetof(boot_args, physBase) == 0x10, "DarwinBoot memory ABI");

extern volatile uint32_t armdt_platform_configured, armcpu_registered, armcpu_gic_ready;
extern volatile uint64_t armcpu_timer_interrupts;
extern volatile uint32_t armcpu_gic_boot[8];
extern void early_boot_progress_vectors_done(void);
extern volatile int native_owner_cache_stage __attribute__((weak));
extern volatile uint64_t native_ufs_block_reads __attribute__((weak));
extern volatile uint64_t native_ufs_block_failures __attribute__((weak));
// Scanout copy statistics from Stage7Probe (native_legacy_copy_timing_observed), in microseconds: last, largest, count, sum.
extern volatile uint64_t native_legacy_copy_stats[4] __attribute__((weak));
extern unsigned vm_page_free_count;
extern uint32_t memorystatus_available_pages;

// These values are also read by the stackless fatal exception entry.
uint64_t early_boot_args_pa, early_boot_phys_base, early_boot_virt_base;
uint64_t early_boot_stage;
static const uint64_t font[] = GLYPH_FONT;
static uintptr_t framebuffer, framebuffer_pa;
// The write-combined alias made once the kernel VM is up (early_boot_progress_vm_ready). XNU's console maps the same framebuffer
// Device-nGnRnE (video_console.c initialize_screen, VM_WIMG_IO) and rebinds us to that mapping; we keep drawing through this alias
// when the new mapping is the same physical framebuffer, so the console and the display path use one memory type (F1-C7, MA-1).
static uintptr_t framebuffer_wc;
static uint64_t framebuffer_wc_bytes;
static unsigned width, height, stride, column, row = 6;
static uint64_t poll_count;
static thread_call_t poll_call;
static int display_owned, logo_visible;
static int busy;
static unsigned console_lock, console_smp;
static uint64_t panic_cpu;
enum { SCALE = 3, CELL = 6 * SCALE, LINE = 14 * SCALE, LOG_TOP = 6 };
_Static_assert(LINE == PAGANI_STATUS_LINE, "PeripheralStatus.h states the text row height");
enum { HISTORY = 8, STAGE_BYTES = 80 };
static char stages[HISTORY][STAGE_BYTES], root_stage[STAGE_BYTES], root_detail[192];
static char display_stage[STAGE_BYTES], exit_stage[STAGE_BYTES];
static uint64_t display_value;
static unsigned stage_count, panic_state, panic_limit, panic_clipped;
// pagani-overlay=0: no diagnostic text on the screen (the status band, the GPU, FPS, CPU and peripheral rows, the boot text); the
// ramoops log keeps all of it, the boot logo and a panic screen still show. Default 1. The text covered the top-left corner of the
// iOS UI (back buttons, the status bar) on the phone layout (the user, 2026-10-30).
static unsigned overlay_hidden;
static int overlay_off(void) { return __atomic_load_n(&overlay_hidden, __ATOMIC_RELAXED) && !panic_state; }
static uint64_t root_sequence, root_value, root_tick, fault_esr, fault_pc, fault_far, fault_psr;
static int fault_valid;
char boot_progress_panic_text[2048];
extern int (vsnprintf)(char *, size_t, const char *, va_list);
extern int (snprintf)(char *, size_t, const char *, ...);
static char storage_status[96];
static char timer_status[96];
static char display_status[96];
static char peripheral_status[PAGANI_PERIPHERALS][80];
static int peripheral_overlay, platform_overlay;
// Status rows: the GPU's (boot_progress_gpu_rows, text rows 10-13) and the display probe's (boot_progress_dpu_rows, rows 14-18), stored
// under the console lock and drawn by poll() only when they changed. Rows between the two sets inside the band stay black. status_shown
// keeps what each row 10-19 shows (status_owned: the row belongs to the band), so only changed rows are painted. The FPS row (text row 9,
// pagani-fps=1) and the SLOW row above it (text row 8) are kept the same way by fps_status/fps_shown/fps_owned and
// slow_status/slow_shown/slow_owned further down. The CPU row (boot_progress_cpu_row, PaganiArmCpu pagani-cpufreq) is kept the same way in
// cpu_status (cpu_row: one is stored) on row 14, or on row 19 below the display probe rows while those are shown; status_shown covers 10-19.
enum { STATUS_TOP = PAGANI_GPU_TOP_ROW, STATUS_ROWS = PAGANI_CPU_ROW_LOW + 1 - PAGANI_GPU_TOP_ROW };
_Static_assert(PAGANI_GPU_TOP_ROW + PAGANI_GPU_ROWS <= PAGANI_DPU_TOP_ROW, "GPU and display probe rows do not overlap");
_Static_assert(PAGANI_CPU_ROW == PAGANI_DPU_TOP_ROW && PAGANI_CPU_ROW_LOW == PAGANI_DPU_TOP_ROW + PAGANI_DPU_ROWS && STATUS_ROWS <= 32 &&
               PAGANI_CPU_ROW_WIDTH < PAGANI_GPU_ROW_BYTES, "the CPU row takes row 14, or the row below the display probe rows");
static char gpu_status[PAGANI_GPU_ROWS][PAGANI_GPU_ROW_BYTES];
static char dpu_status[PAGANI_DPU_ROWS][PAGANI_GPU_ROW_BYTES];
static char cpu_status[PAGANI_GPU_ROW_BYTES];
static char status_shown[STATUS_ROWS][PAGANI_GPU_ROW_BYTES];
static unsigned gpu_count, dpu_count, cpu_row, status_owned;
static uint32_t gpu_seq, gpu_drawn_seq, dpu_seq, dpu_drawn_seq, cpu_seq, cpu_drawn_seq;
static int status_dirty;
// Frames the display path presented (boot_progress_display_frames), printed by the panic screen.
static uint64_t display_frames;
// The frame rate (FrameRate.h, pagani-fps): every frame that reached the panel (boot_progress_frame, no lock) and what the 1 Hz observer
// makes of it: the FPS row on text row 9 (fps_row; part of the band while shown, drawn by status_draw like the other rows, only when its
// text changed) and the ramoops F line and J (jitter) records. fps_mode: 0 off, 1 the row and the log, 2 the log only. fps_next, the
// scan and the jitter limit belong to poll() alone; fps_status/fps_shown/fps_owned are under the console lock like the other rows.
// Where the time went (boot_progress_frame_timed, _stage, _display_event): the SLOW row on text row 8 right above it, kept the same way
// (slow_status/slow_shown/slow_owned, slow_next from poll()), from the breakdowns in the ring, the display path's stage word
// (frame_stage_now: what it waits for and since when) and the display events counted per second (display_events against display_seen,
// the counts of the observer's previous pass).
_Static_assert(PAGANI_FRAME_CPU == FRAME_CPU && PAGANI_FRAME_GPU == FRAME_GPU, "frame paths");
_Static_assert(PAGANI_FPS_ROW + 1 == PAGANI_GPU_TOP_ROW && 4 + PAGANI_PERIPHERALS <= PAGANI_FPS_ROW,
               "the FPS row sits right above the GPU rows, below the peripheral overlay (rows 4-8)");
_Static_assert(PAGANI_SLOW_ROW + 1 == PAGANI_FPS_ROW && LOG_TOP < PAGANI_SLOW_ROW && PAGANI_STAGES <= 8,
               "the SLOW row sits right above the FPS row, below the log's first row; stages fit the stage word");
// The SLOW row is at most SLOW_WIDTH characters on the screen (its " done" part only while it fits); the log's copy always has it.
enum { SLOW_WIDTH = 65, SLOW_LOG_WIDTH = 96, FPS_EXTRA_BYTES = 128, SLOW_STUCK_MS = FRAME_JITTER_MS };
static struct frame_ring presented;
static unsigned fps_mode, fps_row, fps_owned, slow_owned;
static uint32_t fps_seq, fps_drawn_seq;
static char fps_status[PAGANI_GPU_ROW_BYTES], fps_shown[PAGANI_GPU_ROW_BYTES];
static char fps_next[PAGANI_GPU_ROW_BYTES], fps_logged[PAGANI_GPU_ROW_BYTES];
static char slow_status[PAGANI_GPU_ROW_BYTES], slow_shown[PAGANI_GPU_ROW_BYTES], slow_next[PAGANI_GPU_ROW_BYTES];
static char fps_extra[FPS_EXTRA_BYTES], fps_extra_logged[FPS_EXTRA_BYTES];
static uint64_t fps_scanned;
static struct frame_scan fps_scan;
static struct frame_limit jitter_limit;
static uint64_t frame_stage_now;
static uint64_t display_events[PAGANI_DISPLAY_EVENTS], display_seen[PAGANI_DISPLAY_EVENTS];
// The stall probe (StallProbe.h, pagani-diag), at the end of this file: the frame path pokes the sampler awake, the observer's pass writes
// the second's records, a panic stops it first thing. Nothing of it exists in Test/console.c; Test/stall.c tests the header, Test/stall-glue.c the glue.
#if !defined(BOOT_PROGRESS_TEST) || defined(BOOT_PROGRESS_STALL_TEST)
static void stall_frame(void);
static void stall_halt(void);
static const char *stall_fps_extra(const char *extra);
static void stall_second(uint32_t frames);
static int stall_touched(void);
#else
static void stall_frame(void) {}
static void stall_halt(void) {}
static const char *stall_fps_extra(const char *extra) { return extra; }
static void stall_second(uint32_t frames) { (void)frames; }
static int stall_touched(void) { return 0; }
#endif
extern unsigned processor_count, processor_avail_count;
extern unsigned long gPhysSize;
extern volatile uint32_t armcpu_core_timer[8], armcpu_core_sgi[8];

static void barrier(void)
{
#ifndef BOOT_PROGRESS_TEST
    __asm__ volatile("dsb sy" ::: "memory");
#endif
}

static uint64_t irq_disable(void)
{
    uint64_t flags = 0;
#ifndef BOOT_PROGRESS_TEST
    __asm__ volatile("mrs %0, daif\n\tmsr daifset, #15" : "=r"(flags) :: "memory");
#endif
    return flags;
}

static void irq_enable(uint64_t flags)
{
#ifndef BOOT_PROGRESS_TEST
    __asm__ volatile("msr daif, %0" :: "r"(flags) : "memory");
#else
    (void)flags;
#endif
}

static uint64_t console_cpu(void)
{
    uint64_t cpu = 0;
#ifndef BOOT_PROGRESS_TEST
    __asm__ volatile("mrs %0, mpidr_el1" : "=r"(cpu));
#else
    cpu = test_cpu;
#endif
    return (cpu & 0xff00ffffffULL) + 1;
}

static int panic_other_cpu(void)
{
    uint64_t owner = __atomic_load_n(&panic_cpu, __ATOMIC_ACQUIRE);
    return owner && owner != console_cpu();
}

// Skip contended diagnostics; a stopped CPU must never block the panic screen.
static uint64_t irq_save(void)
{
    uint64_t flags = irq_disable();
    if (__atomic_load_n(&panic_cpu, __ATOMIC_ACQUIRE) ||
        (console_smp && __atomic_exchange_n(&console_lock, 1, __ATOMIC_ACQUIRE))) {
        irq_enable(flags);
        return UINT64_MAX;
    }
    return flags;
}

static void irq_restore(uint64_t flags)
{
    if (console_smp) __atomic_store_n(&console_lock, 0, __ATOMIC_RELEASE);
    irq_enable(flags);
}

static uint64_t frequency(void)
{
    uint64_t value = 19200000;
#ifndef BOOT_PROGRESS_TEST
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(value));
#endif
    return value;
}

static uint64_t counter(void)
{
    uint64_t value = 0;
#ifndef BOOT_PROGRESS_TEST
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(value));
#endif
    return value;
}

// The frame-rate clock: mach time, the clock the display path stamps its frames with (it counts at cntfrq_el0, frequency()).
static uint64_t frame_clock(void)
{
#ifndef BOOT_PROGRESS_TEST
    return mach_absolute_time();
#else
    return test_clock;
#endif
}

static int boot_arg(const char *name, uint32_t *value)
{
#ifndef BOOT_PROGRESS_TEST
    return PE_parse_boot_argn(name, value, sizeof(*value)) ? 1 : 0;
#else
    return test_boot_arg(name, value);
#endif
}

static void copy_line(char *to, size_t size, const char *from)
{
    size_t i = 0;
    if (from) for (; i + 1 < size && from[i]; i++)
        to[i] = from[i] >= 32 && from[i] <= 126 ? from[i] : ' ';
    to[i] = 0;
}

static void record_stage(const char *s)
{
    copy_line(stages[stage_count % HISTORY], STAGE_BYTES, s);
    stage_count++;
}

static int geometry(uintptr_t base, uint64_t w, uint64_t h, uint64_t pitch, uint64_t depth)
{
    return base != 0 && (base & 3) == 0 && (depth & 255) == 32 &&
        w >= CELL && w <= 8192 && h >= (LOG_TOP + 1) * LINE && h <= 8192 &&
        pitch >= w * 4 && pitch <= 32768 && (pitch & 3) == 0 &&
        pitch * h <= 64 * 1024 * 1024 && base <= UINTPTR_MAX - pitch * h;
}

// The text row of the CPU row: 14, or right below the display probe rows while they are shown.
static unsigned cpu_row_at(unsigned dpu)
{
    return dpu ? PAGANI_CPU_ROW_LOW : PAGANI_CPU_ROW;
}

// Text rows [top, bottom) of the status band: the SLOW row 8 and the FPS row 9, GPU rows from 10, display probe rows from 14, the CPU row
// (14, or 19 with the display probe rows), and the rows between them.
static int status_band(unsigned *top, unsigned *bottom)
{
    // pagani-overlay=0: no band at all, so the display path copies whole frames (the presenter skips the band it reports).
    if (__atomic_load_n(&overlay_hidden, __ATOMIC_RELAXED)) return 0;
    unsigned gpu = __atomic_load_n(&gpu_count, __ATOMIC_ACQUIRE), dpu = __atomic_load_n(&dpu_count, __ATOMIC_ACQUIRE);
    unsigned fps = __atomic_load_n(&fps_row, __ATOMIC_ACQUIRE), cpu = __atomic_load_n(&cpu_row, __ATOMIC_ACQUIRE);
    if (!gpu && !dpu && !fps && !cpu) return 0;
    *top = fps ? PAGANI_SLOW_ROW : gpu ? PAGANI_GPU_TOP_ROW : PAGANI_DPU_TOP_ROW;
    *bottom = cpu ? cpu_row_at(dpu) + 1 : dpu ? PAGANI_DPU_TOP_ROW + dpu : gpu ? PAGANI_GPU_TOP_ROW + gpu : PAGANI_FPS_ROW + 1;
    return 1;
}

static void clear_line(unsigned y)
{
    if (panic_other_cpu() || overlay_off()) return;
    if ((display_owned || (logo_visible && y >= LOG_TOP * LINE)) && !panic_state) return;
    if (!framebuffer || y + LINE > height) return;
    for (unsigned dy = 0; dy < LINE; dy++) {
        volatile uint32_t *pixels = (volatile uint32_t *)framebuffer + (y + dy) * stride;
        for (unsigned x = 0; x < width; x++) pixels[x] = 0xff000000;
    }
}

static void render_boot_logo(void)
{
    int left = ((int)width - BOOT_LOGO_WIDTH) / 2 + BOOT_LOGO_X;
    int top = ((int)height - BOOT_LOGO_HEIGHT) / 2 + BOOT_LOGO_Y;
    if (!framebuffer || left < 0 || top < 0 || left + BOOT_LOGO_WIDTH > (int)width ||
        top + BOOT_LOGO_HEIGHT > (int)height) return;
    for (unsigned y = 0; y < height; y++) {
        volatile uint32_t *line = (volatile uint32_t *)framebuffer + y * stride;
        for (unsigned x = 0; x < width; x++) line[x] = 0xff000000;
    }
    for (unsigned y = 0; y < BOOT_LOGO_HEIGHT; y++) {
        volatile uint32_t *line = (volatile uint32_t *)framebuffer + (top + y) * stride + left;
        for (unsigned x = 0; x < BOOT_LOGO_WIDTH; x++) {
            uint32_t level = boot_logo[y * BOOT_LOGO_WIDTH + x];
            line[x] = 0xff000000 | level * 0x010101;
        }
    }
    logo_visible = 1;
}

static void glyph_pixels(unsigned char c, unsigned x, unsigned y, uint32_t color)
{
    if (panic_other_cpu() || overlay_off()) return;
    if (!framebuffer || x + CELL > width || y + LINE > height) return;
    if (c < 32 || c > 126) c = '?';
    uint64_t bits = font[c - 32];
    for (unsigned gy = 0; gy < 12; gy++) {
        unsigned scan = (unsigned)((gy < 6 ? bits >> 32 : bits & 0xffffffff) >> ((gy % 6) * 5));
        for (unsigned gx = 0; gx < 6; gx++) {
            uint32_t pixel = gx < 5 && ((scan >> gx) & 1) ? color : 0xff000000;
            for (unsigned dy = 0; dy < SCALE; dy++) {
                volatile uint32_t *line = (volatile uint32_t *)framebuffer + (y + gy * SCALE + dy) * stride + x + gx * SCALE;
                for (unsigned dx = 0; dx < SCALE; dx++) line[dx] = pixel;
            }
        }
    }
}

static void glyph(unsigned char c, unsigned x, unsigned y, uint32_t color)
{
    if ((display_owned || (logo_visible && y >= LOG_TOP * LINE)) && !panic_state) return;
    glyph_pixels(c, x, y, color);
}

static void putc_raw(char c)
{
    if (panic_other_cpu()) return;
    if (panic_state) ramoops_log_panic_putc(c);   // the persistent log keeps what the screen clips
    if (!framebuffer) return;
    if (panic_state && row >= panic_limit) { panic_clipped = 1; return; }
    if (c == '\t') c = ' ';
    if (c == '\r') { column = 0; return; }
    if (c == '\n' || (column + 1) * CELL > width) {
        column = 0;
        row++;
        if (panic_state && row >= panic_limit) { panic_clipped = 1; return; }
        if ((row + 1) * LINE > height) row = LOG_TOP;
        // The log skips the status band instead of writing over it.
        unsigned top, bottom;
        if (!panic_state && status_band(&top, &bottom) && row >= top && row < bottom) {
            row = bottom;
            if ((row + 1) * LINE > height) row = LOG_TOP;
        }
        clear_line(row * LINE);
        if (c == '\n') return;
    }
    glyph((unsigned char)c, column++ * CELL, row * LINE, 0xffffffff);
}

static void text_raw(const char *s)
{
    while (*s) putc_raw(*s++);
}

static void line_raw(const char *s)
{
    while (*s && column < width / CELL) putc_raw(*s++);
}

static void hex_raw(uint64_t n)
{
    for (int shift = 60; shift >= 0; shift -= 4) {
        unsigned digit = (unsigned)(n >> shift) & 15;
        putc_raw((char)(digit < 10 ? '0' + digit : 'a' + digit - 10));
    }
}

static void value_raw(const char *name, uint64_t n)
{
    text_raw(name); text_raw(" 0x"); hex_raw(n); putc_raw('\n');
}

void boot_progress_bind(uintptr_t base, unsigned w, unsigned h, unsigned pitch, unsigned depth)
{
    if (panic_state) return;
    if (!geometry(base, w, h, pitch, depth)) return;
    // The console's own mapping of our framebuffer: keep the write-combined alias (same physical base, fits in it).
    if (framebuffer_wc && base != framebuffer_wc && (uint64_t)pitch * h <= framebuffer_wc_bytes &&
        (uint64_t)ml_vtophys((vm_offset_t)base) == (uint64_t)framebuffer_pa)
        base = framebuffer_wc;
    framebuffer = base; width = w; height = h; stride = pitch / 4;
    if ((row + 1) * LINE > height) row = LOG_TOP;
    if ((column + 1) * CELL > width) column = 0;
}

// Stop diagnostic drawing before the first system frame.
int boot_progress_display_handoff(void)
{
    uint64_t flags;
    // A skipped handoff strands the pending swap; wait for the bounded writer.
    while ((flags = irq_save()) == UINT64_MAX) {
        if (__atomic_load_n(&panic_cpu, __ATOMIC_ACQUIRE)) return 0;
#ifndef BOOT_PROGRESS_TEST
        __asm__ volatile("yield" ::: "memory");
#endif
    }
    int ready = framebuffer && !panic_state && !busy;
    if (ready) { display_owned = 1; barrier(); }
    irq_restore(flags);
    return ready;
}

void boot_progress_display_failed(void)
{
    uint64_t flags = irq_save();
    if (flags == UINT64_MAX) return;
    if (!panic_state) display_owned = 0;
    irq_restore(flags);
}

int boot_progress_active(void) { return framebuffer != 0; }

void boot_progress_putc(char c)
{
    ramoops_log_putc(c);
    uint64_t flags = irq_save();
    if (flags == UINT64_MAX) return;
    if (!busy && !panic_state) { busy = 1; putc_raw(c); barrier(); busy = 0; }
    irq_restore(flags);
}

void boot_progress_stage(const char *s)
{
    ramoops_log_line(RAMOOPS_KIND_STAGE, s);

    uint64_t flags = irq_save();
    if (flags == UINT64_MAX) return;
    if (!busy && !panic_state) {
        busy = 1;
        record_stage(s);
        if (logo_visible) {
            unsigned saved_row = row, saved_column = column;
            row = 0; column = 0; clear_line(0); line_raw(s);
            row = saved_row; column = saved_column;
        }
        if (column) putc_raw('\n');
        text_raw("[XNU] "); text_raw(s); putc_raw('\n'); barrier();
        busy = 0;
    }
    irq_restore(flags);
}

void boot_progress_root(const char *stage, const char *detail, uint64_t value)
{
    ramoops_log_root(stage, detail, value);
    uint64_t flags = irq_save();
    if (flags == UINT64_MAX) return;
    if (!panic_state && !busy) {
        busy = 1;
        copy_line(root_stage, sizeof(root_stage), stage);
        copy_line(root_detail, sizeof(root_detail), detail);
        root_value = value; root_tick = counter(); root_sequence++;
        record_stage(stage);
        char *entry = stages[(stage_count - 1) % HISTORY];
        unsigned n = 0;
        while (entry[n]) n++;
        if (root_detail[0] && n + 3 < STAGE_BYTES) {
            entry[n++] = ':'; entry[n++] = ' ';
            copy_line(entry + n, STAGE_BYTES - n, root_detail);
        }
        if (column) putc_raw('\n');
        text_raw("[ROOT] "); text_raw(root_stage); text_raw(" value=0x"); hex_raw(value); putc_raw('\n');
        if (root_detail[0]) { text_raw(root_detail); putc_raw('\n'); }
        unsigned saved_row = row, saved_column = column;
        row = 3; column = 0; clear_line(3 * LINE);
        text_raw("ROOT ");
        for (unsigned i = 0; root_stage[i] && column < width / CELL; i++) putc_raw(root_stage[i]);
        row = saved_row; column = saved_column;
        barrier(); busy = 0;
    }
    irq_restore(flags);
}

void boot_progress_display_status(const char *detail, uint64_t value)
{
    uint64_t flags = irq_save();
    if (flags == UINT64_MAX) return;
    if (!panic_state && !busy) {
        copy_line(display_stage, sizeof(display_stage), detail);
        display_value = value;
    }
    irq_restore(flags);
    boot_progress_root("stage7", detail, value);
}

void boot_progress_peripheral(unsigned device, const char *stage, int32_t result, uint32_t value, uint64_t reports)
{
    if (device >= PAGANI_SPI) return;
    static const char *names[] = {"KEYS", "TOUCH", "PANEL"};
    uint64_t flags = irq_save();
    if (flags == UINT64_MAX) return;
    if (!panic_state) snprintf(peripheral_status[device], sizeof(peripheral_status[device]),
        "%s %.18s r=%x v=%x n=%llu", names[device], stage ? stage : "?",
        (unsigned)result, value, (unsigned long long)reports);
    irq_restore(flags);
}

void boot_progress_touch_spi(uint32_t firmware, uint32_t disabled, uint32_t state, uint32_t irq, uint32_t tx, uint32_t rx)
{
    uint64_t flags = irq_save();
    if (flags == UINT64_MAX) return;
    if (!panic_state) {
        snprintf(peripheral_status[PAGANI_SPI], sizeof(peripheral_status[PAGANI_SPI]),
            "SPI fw=%x if=%x st=%x irq=%x", firmware, disabled, state, irq);
        snprintf(peripheral_status[PAGANI_SPI_FIFO], sizeof(peripheral_status[PAGANI_SPI_FIFO]),
            "SPI FIFO tx=%x rx=%x", tx, rx);
    }
    irq_restore(flags);
}

void boot_progress_touch_dma(uint32_t tx, uint32_t rx, uint32_t txin, uint32_t rxin)
{
    uint64_t flags = irq_save();
    if (flags == UINT64_MAX) return;
    if (!panic_state) snprintf(peripheral_status[PAGANI_SPI_FIFO], sizeof(peripheral_status[PAGANI_SPI_FIFO]),
        "SPI DMA tx=%x rx=%x ti=%x ri=%x", tx, rx, txin, rxin);
    irq_restore(flags);
}

void boot_progress_touch_read(uint32_t length, uint32_t head, uint32_t pins, uint32_t dma,
                              uint32_t state, uint32_t txin, uint32_t rxin, uint32_t delay)
{
    uint64_t flags = irq_save();
    if (flags == UINT64_MAX) return;
    if (!panic_state) {
        snprintf(peripheral_status[PAGANI_SPI], sizeof(peripheral_status[PAGANI_SPI]),
            "SPI RX n=%x h=%08x a=%x/%x rst=%x", length, head, pins&1, (pins>>1)&1, (pins>>2)&3);
        snprintf(peripheral_status[PAGANI_SPI_FIFO], sizeof(peripheral_status[PAGANI_SPI_FIFO]),
            "SPI dma=%x ti=%x ri=%x st=%x dly=%x", dma, txin, rxin, state, delay);
    }
    irq_restore(flags);
}

void boot_progress_touch_power(uint32_t address, uint32_t hardware, uint32_t tcs, uint32_t state,
                               uint32_t cmd0, uint32_t cmd1, uint32_t cmd2)
{
    uint64_t flags = irq_save();
    if (flags == UINT64_MAX) return;
    if (!panic_state) {
        snprintf(peripheral_status[PAGANI_SPI], sizeof(peripheral_status[PAGANI_SPI]),
            "PWR a=%x hw=%x t=%x", address, hardware, tcs);
        snprintf(peripheral_status[PAGANI_SPI_FIFO], sizeof(peripheral_status[PAGANI_SPI_FIFO]),
            "PWR st=%x c=%x/%x/%x", state, cmd0, cmd1, cmd2);
    }
    irq_restore(flags);
}

void boot_progress_touch_hid(uint32_t binding, uint32_t usage, uint32_t events, uint32_t digitizers,
                             uint32_t edges, uint32_t input, uint64_t output, uint32_t mask)
{
    uint64_t flags = irq_save();
    if (flags == UINT64_MAX) return;
    if (!panic_state) {
        snprintf(peripheral_status[PAGANI_SPI], sizeof(peripheral_status[PAGANI_SPI]),
            "HID b=%x use=%x/%x ev=%u/%u e=%u", binding, usage>>16, usage&0xffff, events, digitizers, edges);
        snprintf(peripheral_status[PAGANI_SPI_FIFO], sizeof(peripheral_status[PAGANI_SPI_FIFO]),
            "XY in=%x/%x out=%x/%x mask=%x", input>>16, input&0xffff, (uint32_t)(output>>32), (uint32_t)output, mask);
    }
    irq_restore(flags);
}

void boot_progress_touch_smmu(uint32_t slot, uint32_t bank, uint32_t smr, uint32_t s2cr)
{
    uint64_t flags = irq_save();
    if (flags == UINT64_MAX) return;
    if (!panic_state) {
        snprintf(peripheral_status[PAGANI_SPI], sizeof(peripheral_status[PAGANI_SPI]),
            "SMMU slot=%x bank=%x", slot, bank);
        snprintf(peripheral_status[PAGANI_SPI_FIFO], sizeof(peripheral_status[PAGANI_SPI_FIFO]),
            "SMMU smr=%x s2cr=%x", smr, s2cr);
    }
    irq_restore(flags);
}

void boot_progress_peripheral_enable(int enabled)
{
    __atomic_store_n(&peripheral_overlay, enabled != 0, __ATOMIC_RELEASE);
}

void boot_progress_platform_enable(int enabled)
{
    __atomic_store_n(&platform_overlay, enabled != 0, __ATOMIC_RELEASE);
}

// The first platform row: CPU registration and per-core timer/IPI activity, then C<last>/<mean>/<largest>us F<copies>, the CPU scanout
// copy time per copied frame from Stage7Probe (native_legacy_copy_timing_observed) when it is linked in.
static void platform_line(char *line, size_t size)
{
    unsigned timer = 0, ipi = 0;
    for (unsigned cpu = 0; cpu < 8; cpu++) {
        if (__atomic_load_n(&armcpu_core_timer[cpu], __ATOMIC_RELAXED)) timer |= 1U << cpu;
        if (__atomic_load_n(&armcpu_core_sgi[cpu], __ATOMIC_RELAXED)) ipi |= 1U << cpu;
    }
    if (native_legacy_copy_stats) {
        uint64_t copies = native_legacy_copy_stats[2];
        snprintf(line, size, "CPU reg=%u online=%u T=%02x I=%02x C%llu/%llu/%lluus F%llu",
                 processor_count, processor_avail_count, timer, ipi,
                 (unsigned long long)native_legacy_copy_stats[0],
                 (unsigned long long)(copies ? native_legacy_copy_stats[3] / copies : 0),
                 (unsigned long long)native_legacy_copy_stats[1], (unsigned long long)copies);
    } else snprintf(line, size, "CPU reg=%u online=%u T=%02x I=%02x", processor_count, processor_avail_count, timer, ipi);
}

// Only the opted-in status rows may repaint after console handoff.
void boot_progress_peripheral_frame(void)
{
    int platform = __atomic_load_n(&platform_overlay, __ATOMIC_ACQUIRE);
    if (!platform && !__atomic_load_n(&peripheral_overlay, __ATOMIC_ACQUIRE)) return;
    uint64_t flags = irq_save();
    if (flags == UINT64_MAX) return;
    if (!display_owned || panic_state || busy || !framebuffer) { irq_restore(flags); return; }
    char line[80];
    // Rows 4-8; the SLOW row (pagani-fps=1) keeps row 8, so the fifth peripheral row is not shown then.
    unsigned rows = platform ? 2U : PAGANI_PERIPHERALS;
    if (__atomic_load_n(&fps_row, __ATOMIC_ACQUIRE) && 4 + rows > PAGANI_SLOW_ROW) rows = PAGANI_SLOW_ROW - 4;
    for (unsigned i = 0; i < rows; i++) {
        if (platform && i == 0) {
            platform_line(line, sizeof(line));
        } else if (platform) {
            snprintf(line, sizeof(line), "RAM MiB=%lu free=%u avail=%u", gPhysSize >> 20,
                     vm_page_free_count >> 8, memorystatus_available_pages >> 8);
        } else copy_line(line, sizeof(line), peripheral_status[i]);
        if (!line[0]) continue;
        unsigned at = 0;
        for (unsigned x = 0; x + CELL <= width; x += CELL) {
            if (panic_state) { irq_restore(flags); return; }
            unsigned char c = line[at];
            if (c) at++;
            glyph_pixels(c ? c : ' ', x, (i + 4) * LINE, 0xffffffff);
        }
    }
    barrier(); irq_restore(flags);
}

// One status text row painted directly like the per-frame overlay (neither the display hand-off nor the boot logo hides it): black,
// then the text. Only called by poll() with the console held.
static void status_paint(unsigned text_row, const char *text)
{
    if (overlay_off()) return;
    unsigned top = text_row * LINE;
    if (top + LINE > height) return;
    for (unsigned y = top; y < top + LINE; y++) {
        volatile uint32_t *pixels = (volatile uint32_t *)framebuffer + y * stride;
        for (unsigned x = 0; x < width; x++) pixels[x] = 0xff000000;
    }
    for (unsigned i = 0; text && text[i] && (i + 1) * CELL <= width; i++)
        glyph_pixels((unsigned char)text[i], i * CELL, top, 0xffffffff);
}

static int status_same(const char *a, const char *b)
{
    unsigned i = 0;
    for (; a[i] && a[i] == b[i]; i++) {}
    return a[i] == b[i];
}

// Text rows 8-19 as the SLOW, FPS, GPU, display probe and CPU rows want them: rows of the band get their text (blank between the sets),
// rows that left the band are cleared once and handed back to the presentation path.
static void status_draw(void)
{
    if (panic_state || panic_other_cpu() || !framebuffer) return;
    uint32_t gseq = __atomic_load_n(&gpu_seq, __ATOMIC_ACQUIRE), dseq = __atomic_load_n(&dpu_seq, __ATOMIC_ACQUIRE);
    uint32_t cseq = __atomic_load_n(&cpu_seq, __ATOMIC_ACQUIRE);
    unsigned fps = __atomic_load_n(&fps_row, __ATOMIC_ACQUIRE);
    if (gseq == gpu_drawn_seq && dseq == dpu_drawn_seq && cseq == cpu_drawn_seq && fps_seq == fps_drawn_seq && !status_dirty &&
        fps_owned == fps && slow_owned == fps) return;
    unsigned top = 0, bottom = 0;
    int band = status_band(&top, &bottom);
    if (fps) {
        if (!slow_owned || status_dirty || !status_same(slow_shown, slow_status)) {
            status_paint(PAGANI_SLOW_ROW, slow_status);
            copy_line(slow_shown, sizeof(slow_shown), slow_status);
            slow_owned = 1;
        }
        if (!fps_owned || status_dirty || !status_same(fps_shown, fps_status)) {
            status_paint(PAGANI_FPS_ROW, fps_status);
            copy_line(fps_shown, sizeof(fps_shown), fps_status);
            fps_owned = 1;
        }
    } else {
        if (slow_owned) {
            status_paint(PAGANI_SLOW_ROW, "");
            slow_owned = 0;
            slow_shown[0] = 0;
        }
        if (fps_owned) {
            status_paint(PAGANI_FPS_ROW, "");
            fps_owned = 0;
            fps_shown[0] = 0;
        }
    }
    for (unsigned r = 0; r < STATUS_ROWS; r++) {
        unsigned text_row = STATUS_TOP + r;
        const char *text = 0;
        if (text_row >= PAGANI_GPU_TOP_ROW && text_row < PAGANI_GPU_TOP_ROW + gpu_count)
            text = gpu_status[text_row - PAGANI_GPU_TOP_ROW];
        else if (text_row >= PAGANI_DPU_TOP_ROW && text_row < PAGANI_DPU_TOP_ROW + dpu_count)
            text = dpu_status[text_row - PAGANI_DPU_TOP_ROW];
        else if (cpu_row && text_row == cpu_row_at(dpu_count))
            text = cpu_status;
        else if (band && text_row >= top && text_row < bottom)
            text = "";
        unsigned bit = 1U << r;
        if (!text) {
            if (status_owned & bit) { status_paint(text_row, ""); status_owned &= ~bit; status_shown[r][0] = 0; }
            continue;
        }
        if ((status_owned & bit) && !status_dirty && status_same(status_shown[r], text)) continue;
        status_paint(text_row, text);
        copy_line(status_shown[r], PAGANI_GPU_ROW_BYTES, text);
        status_owned |= bit;
    }
    gpu_drawn_seq = gseq; dpu_drawn_seq = dseq; cpu_drawn_seq = cseq; fps_drawn_seq = fps_seq; status_dirty = 0;
}

// Stores rows for status_draw. A skipped update is never drawn (the callers publish only on change), so this waits for the bounded
// console writer instead of dropping it.
static void status_store(char (*to)[PAGANI_GPU_ROW_BYTES], unsigned capacity, unsigned *count, uint32_t *seq,
                         const char (*rows)[PAGANI_GPU_ROW_BYTES], unsigned n)
{
    if (!rows) n = 0;
    if (n > capacity) n = capacity;
    uint64_t flags;
    while ((flags = irq_save()) == UINT64_MAX) {
        if (__atomic_load_n(&panic_cpu, __ATOMIC_ACQUIRE)) return;
#ifndef BOOT_PROGRESS_TEST
        __asm__ volatile("yield" ::: "memory");
#endif
    }
    if (!panic_state && !busy) {
        for (unsigned r = 0; r < capacity; r++) copy_line(to[r], PAGANI_GPU_ROW_BYTES, r < n ? rows[r] : "");
        __atomic_store_n(count, n, __ATOMIC_RELEASE);
        __atomic_store_n(seq, *seq + 1, __ATOMIC_RELEASE);
    }
    irq_restore(flags);
}

void boot_progress_gpu_rows(const char (*rows)[PAGANI_GPU_ROW_BYTES], unsigned count)
{
    ramoops_log_rows(RAMOOPS_KIND_GPU, rows, count);
    status_store(gpu_status, PAGANI_GPU_ROWS, &gpu_count, &gpu_seq, rows, count);
}

void boot_progress_dpu_rows(const char (*rows)[PAGANI_GPU_ROW_BYTES], unsigned count)
{
    ramoops_log_rows(RAMOOPS_KIND_DPU, rows, count);
    status_store(dpu_status, PAGANI_DPU_ROWS, &dpu_count, &dpu_seq, rows, count);
}

// The CPU row (PaganiArmCpu), cut to PAGANI_CPU_ROW_WIDTH characters; a new sequence only when the text changed. Like status_store it waits
// for the bounded console writer (a skipped update would never be drawn). Not logged: PaganiArmCpu traces its own events.
void boot_progress_cpu_row(const char *text)
{
    char line[PAGANI_GPU_ROW_BYTES];
    copy_line(line, PAGANI_CPU_ROW_WIDTH + 1, text);
    const unsigned shown = line[0] != 0;
    uint64_t flags;
    while ((flags = irq_save()) == UINT64_MAX) {
        if (__atomic_load_n(&panic_cpu, __ATOMIC_ACQUIRE)) return;
#ifndef BOOT_PROGRESS_TEST
        __asm__ volatile("yield" ::: "memory");
#endif
    }
    if (!panic_state && !busy && (shown != cpu_row || !status_same(cpu_status, line))) {
        copy_line(cpu_status, sizeof(cpu_status), line);
        __atomic_store_n(&cpu_row, shown, __ATOMIC_RELEASE);
        __atomic_store_n(&cpu_seq, cpu_seq + 1, __ATOMIC_RELEASE);
    }
    irq_restore(flags);
}

int boot_progress_gpu_band(unsigned *y0, unsigned *y1)
{
    unsigned top, bottom;
    if (!status_band(&top, &bottom) || !framebuffer || bottom * LINE > height) return 0;
    if (y0) *y0 = top * LINE;
    if (y1) *y1 = bottom * LINE;
    return 1;
}

void boot_progress_display_frames(uint64_t frames)
{
    __atomic_store_n(&display_frames, frames, __ATOMIC_RELAXED);
}

void boot_progress_frame(unsigned path, uint64_t when)
{
    frame_ring_record(&presented, path, when ? when : frame_clock());
    stall_frame();
}

// The frame with its stages: the breakdown (microseconds) goes into the ring beside its slot. Any thread, no lock.
void boot_progress_frame_timed(unsigned path, uint64_t when, const struct pagani_frame_timing *timing)
{
    const uint64_t stamp = when ? when : frame_clock();
    if (!timing) {
        frame_ring_record(&presented, path, stamp);
        stall_frame();
        return;
    }
    struct frame_breakdown breakdown;
    frame_breakdown_of(timing, stamp, frequency(), &breakdown);
    frame_ring_record_breakdown(&presented, path, stamp, &breakdown);
    stall_frame();
}

void boot_progress_frame_stage(unsigned stage, uint64_t when)
{
    if (stage >= PAGANI_STAGES) return;
    __atomic_store_n(&frame_stage_now, frame_stage_word(stage, when ? when : frame_clock()), __ATOMIC_RELAXED);
}

void boot_progress_display_event(unsigned event)
{
    if (event < PAGANI_DISPLAY_EVENTS) __atomic_fetch_add(&display_events[event], 1, __ATOMIC_RELAXED);
}

// pagani-fps: 0 off, 1 the FPS row and the log, 2 the log only (other values: 1). Absent: 1 with the GPU status rows (pagani-gpu=1 and
// pagani-gpu-status not 0, the GPU driver's default being 1) or with pagani-platform-status=1, else 0. Thread context, before poll().
static void fps_configure(void)
{
    uint32_t mode = 0;
    if (!boot_arg("pagani-fps", &mode)) {
        uint32_t gpu = 0, status = 1, platform = 0;
        (void)boot_arg("pagani-gpu", &gpu);
        (void)boot_arg("pagani-gpu-status", &status);
        (void)boot_arg("pagani-platform-status", &platform);
        mode = (gpu == 1 && status != 0) || platform == 1;
    }
    if (mode > 2) mode = 1;
    fps_scanned = __atomic_load_n(&presented.count, __ATOMIC_ACQUIRE);
    for (unsigned i = 0; i < PAGANI_DISPLAY_EVENTS; i++) display_seen[i] = __atomic_load_n(&display_events[i], __ATOMIC_RELAXED);
    __atomic_store_n(&fps_mode, mode, __ATOMIC_RELEASE);
    __atomic_store_n(&fps_row, mode == 1, __ATOMIC_RELEASE);
    if (mode == 1) boot_progress_stage("fps: rows 8-9 and ramoops F/J records (pagani-fps=1)");
    else if (mode == 2) boot_progress_stage("fps: ramoops F/J records only (pagani-fps=2)");
}

// This pass's display events: the counts since the previous pass (at most 999 shown anyway).
static uint32_t display_delta(unsigned event)
{
    const uint64_t value = __atomic_load_n(&display_events[event], __ATOMIC_RELAXED), delta = value - display_seen[event];
    display_seen[event] = value;
    return delta > 100000 ? 100000 : (uint32_t)delta;
}

// The 1 Hz observer's frame-rate pass, outside the console lock: the ring scan, the next FPS and SLOW row texts (fps_next, slow_next;
// poll() stores them for status_draw), then the log: the F line while frames come (or when it changed) with the SLOW row while it
// matters, and the jitters oldest first as the limit lets them, each with its breakdown.
static void fps_observe(void)
{
    if (!__atomic_load_n(&fps_mode, __ATOMIC_ACQUIRE)) return;
    const uint64_t hz = frequency();
    const uint64_t count = __atomic_load_n(&presented.count, __ATOMIC_ACQUIRE);
    const uint64_t now = frame_clock();
    frame_ring_scan(&presented, count, now, hz, fps_scanned, &fps_scan);
    fps_scanned = fps_scan.next;
    frame_row(fps_next, sizeof(fps_next), &fps_scan);
    const uint32_t refused = display_delta(PAGANI_DISPLAY_REFUSED), failed = display_delta(PAGANI_DISPLAY_FAILED);
    const uint32_t early = display_delta(PAGANI_DISPLAY_EARLY_FAILED), timeouts = display_delta(PAGANI_DISPLAY_TIMEOUT);
    struct frame_slow slow;
    slow.have = fps_scan.intervals != 0;
    slow.interval_us = fps_scan.max_us;
    slow.timed = slow.have && frame_ring_breakdown(&presented, fps_scan.max_n, &slow.b);
    slow.fallbacks = refused + failed;
    slow.timeouts = timeouts;
    const uint64_t stage = __atomic_load_n(&frame_stage_now, __ATOMIC_RELAXED);
    const uint64_t since = frame_stage_stamp(stage);
    slow.stage = frame_stage_of(stage);
    slow.stage_us = hz && now > since ? (now - since) / hz * 1000000 + (now - since) % hz * 1000000 / hz : 0;
    frame_slow_row(slow_next, sizeof(slow_next), SLOW_WIDTH, &slow);
    if (!boot_progress_trace_enabled()) return;
    // The F line carries the SLOW row while it says something: a slow interval, a frame stuck in one stage, a fallback.
    const int stuck = !slow.have && slow.stage != PAGANI_STAGE_NONE && slow.stage != PAGANI_STAGE_IDLE &&
                      slow.stage_us > (uint64_t)SLOW_STUCK_MS * 1000;
    fps_extra[0] = 0;
    if ((slow.have && slow.interval_us > (uint64_t)FRAME_JITTER_MS * 1000) || stuck || refused || failed || early) {
        char text[SLOW_LOG_WIDTH + 1];
        struct frame_text t;
        t.at = fps_extra;
        t.used = 0;
        t.room = sizeof(fps_extra);
        frame_slow_row(text, sizeof(text), SLOW_LOG_WIDTH, &slow);
        frame_puts(&t, " | ");
        frame_puts(&t, text);
        if (refused || failed || early) {
            frame_puts(&t, " | ref ");
            frame_putu(&t, frame_count_shown(refused));
            frame_puts(&t, " fail ");
            frame_putu(&t, frame_count_shown(failed));
            frame_puts(&t, " early ");
            frame_putu(&t, frame_count_shown(early));
        }
    }
    const int touched = stall_touched();
    if (fps_scan.fps || fps_scan.jitters || fps_scan.lost || touched || !status_same(fps_logged, fps_next) ||
        !status_same(fps_extra_logged, fps_extra)) {
        ramoops_log_fps(fps_next, fps_scan.jitters, fps_scan.lost, stall_fps_extra(fps_extra));
        copy_line(fps_logged, sizeof(fps_logged), fps_next);
        copy_line(fps_extra_logged, sizeof(fps_extra_logged), fps_extra);
    }
    stall_second(fps_scan.fps);
    jitter_limit.skipped += fps_scan.jitters - fps_scan.kept;     // older than the kept ones
    for (unsigned i = fps_scan.kept; i-- > 0;) {
        const struct frame_jitter *j = &fps_scan.events[i];
        if (!frame_limit_take(&jitter_limit, now, hz)) continue;
        struct frame_breakdown breakdown;
        char detail[128];
        const int timed = frame_ring_breakdown(&presented, j->n, &breakdown);
        frame_jitter_detail(detail, sizeof(detail), timed ? &breakdown : 0, slow.fallbacks, slow.timeouts);
        ramoops_log_jitter(j->interval_us, j->path, hz ? j->age * 1000000 / hz : 0, jitter_limit.skipped, detail);
        jitter_limit.skipped = 0;
    }
}

// Parks the CPU until reset. The loop waits on a flag nothing sets instead of being declared noreturn: the host test returns from here
// (BOOT_PROGRESS_TEST) and the callers keep their return statements for it.
static volatile int panic_released;
static void panic_halt(void)
{
    barrier();
#ifndef BOOT_PROGRESS_TEST
    while (!panic_released) __asm__ volatile("wfe");
#endif
}

static void panic_open(const char *title)
{
    panic_state = 1; busy = 1; panic_clipped = 0;
    panic_limit = height / LINE - 2;
    for (unsigned y = 0; y + LINE <= height; y += LINE) clear_line(y);
    for (unsigned y = 0; y < 12; y++) {
        volatile uint32_t *pixels = (volatile uint32_t *)framebuffer + y * stride;
        for (unsigned x = 0; x < width; x++)
            pixels[x] = x < width / 3 ? 0xffff4040 : x < 2 * width / 3 ? 0xff40ff40 : 0xff4080ff;
    }
    row = 1; column = 0; text_raw(title); putc_raw('\n');
}

static void panic_footer(const char *status)
{
    unsigned clipped = panic_clipped;
    panic_limit = height / LINE;
    row = panic_limit - 2; column = 0; clear_line(row * LINE); clear_line((row + 1) * LINE);
    text_raw(clipped ? "SCREEN TEXT CLIPPED\n" : "\n");
    text_raw(status); barrier();
}

void boot_progress_panic(const char *format, va_list *args, unsigned long caller,
                         unsigned reason, uint64_t options)
{
    stall_halt();
    // No screen: the persistent log still records the panic and resets when configured; otherwise XNU's own path goes on.
    if (!framebuffer) { gunyah_wdog_panic(); ramoops_log_panic_headless(format, args, caller, reason, options); return; }
    uint64_t flags = irq_disable();
    uint64_t owner = 0, cpu = console_cpu();
    if (!__atomic_compare_exchange_n(&panic_cpu, &owner, cpu, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE) && owner != cpu) {
        panic_halt(); return;
    }
    if (panic_state) {
        panic_footer("NESTED PANIC: first message preserved"); gunyah_wdog_panic(); ramoops_log_panic_reset();
        panic_halt(); return;
    }
    gunyah_wdog_panic();   // pagani-wdog: no more pets; disabled when RamoopsLog resets below (after a UFS log flush), else the bite resets
    uint64_t sp = 0, fp = 0, mpidr = 0, sctlr = 0, tcr = 0, ttbr1 = 0;
#ifndef BOOT_PROGRESS_TEST
    __asm__ volatile("mov %0, sp" : "=r"(sp));
    __asm__ volatile("mov %0, x29" : "=r"(fp));
    __asm__ volatile("mrs %0, mpidr_el1" : "=r"(mpidr));
    __asm__ volatile("mrs %0, sctlr_el1" : "=r"(sctlr));
    __asm__ volatile("mrs %0, tcr_el1" : "=r"(tcr));
    __asm__ volatile("mrs %0, ttbr1_el1" : "=r"(ttbr1));
#endif
    panic_open("OnePlus 13T / XNU PANIC");
    text_raw("panic entry reached; values below are hexadecimal\n");
    value_raw("caller", caller); value_raw("MPIDR", mpidr);
    text_raw("last stage: "); text_raw(stage_count ? stages[(stage_count - 1) % HISTORY] : "none"); putc_raw('\n');
    text_raw("root: "); text_raw(root_sequence ? root_stage : "not entered"); putc_raw('\n');
    if (root_sequence) {
        value_raw("root seq", root_sequence); value_raw("root value", root_value);
        uint64_t hz = frequency();
        value_raw("root stage age (seconds)", hz ? (counter() - root_tick) / hz : 0);
    }
    value_raw("display frames", __atomic_load_n(&display_frames, __ATOMIC_RELAXED));
    unsigned message_row = row;
    text_raw("MESSAGE (formatting...)\n"); barrier();
    if (format && args) {
        va_list copy;
        va_copy(copy, *args);
        int n = vsnprintf(boot_progress_panic_text, sizeof(boot_progress_panic_text), format, copy);
        va_end(copy);
        if (n < 0) copy_line(boot_progress_panic_text, sizeof(boot_progress_panic_text), "<panic formatting failed>");
        if (n >= (int)sizeof(boot_progress_panic_text)) panic_clipped = 1;
    } else {
        copy_line(boot_progress_panic_text, sizeof(boot_progress_panic_text), format ? format : "<no panic format>");
    }
    for (unsigned r = message_row; r < row && r < panic_limit; r++) clear_line(r * LINE);
    row = message_row; column = 0; text_raw("MESSAGE\n");
    text_raw(boot_progress_panic_text); putc_raw('\n');
    text_raw("PANIC HANDLER STATE\n");
    text_raw("SP "); hex_raw(sp); text_raw(" FP "); hex_raw(fp); putc_raw('\n');
    text_raw("DAIF "); hex_raw(flags); text_raw(" reason "); hex_raw(reason); putc_raw('\n');
    value_raw("options", options); value_raw("SCTLR_EL1", sctlr);
    value_raw("TCR_EL1", tcr); value_raw("TTBR1_EL1", ttbr1);
    if (fault_valid) {
        text_raw("RECORDED KERNEL EXCEPTION\n");
        value_raw("ESR_EL1", fault_esr); value_raw("ELR_EL1", fault_pc);
        value_raw("FAR_EL1", fault_far); value_raw("SPSR_EL1", fault_psr);
    } else text_raw("No fatal exception recorded before this panic\n");
    text_raw("poll "); hex_raw(poll_count); text_raw(" timer "); hex_raw(armcpu_timer_interrupts); putc_raw('\n');
    text_raw("RECENT STAGES (oldest first)\n");
    unsigned count = stage_count < HISTORY ? stage_count : HISTORY;
    for (unsigned i = stage_count - count; i < stage_count; i++) {
        text_raw(stages[i % HISTORY]); putc_raw('\n');
    }
    const char *footer = ramoops_log_panic_footer();
    if (!footer) footer = gunyah_wdog_panic_footer();   // no software reset: the watchdog bite resets
    panic_footer(footer ? footer : "HALTED: no software reboot");
    ramoops_log_panic_reset();   // pagani-panic-reset / pagani-reset-after: wait, then the warm reset
    panic_halt();
}

// No kernel services, stack protector, SIMD, or relocated pointers on this path.
void early_boot_progress_init(boot_args *args)
{
    ramoops_log_suspend();
    console_smp = 0; console_lock = 0; panic_cpu = 0;
    early_boot_args_pa = (uintptr_t)args;
    early_boot_phys_base = args->physBase;
    early_boot_virt_base = args->virtBase;
    early_boot_stage = 1;
    display_owned = 0; logo_visible = 0; panic_state = 0; stage_count = 0; root_sequence = 0; fault_valid = 0;
    poll_count = 0; display_value = 0;
    display_stage[0] = 0; exit_stage[0] = 0;
    peripheral_overlay = 0; platform_overlay = 0;
    for (unsigned i = 0; i < PAGANI_PERIPHERALS; i++) peripheral_status[i][0] = 0;
    for (unsigned i = 0; i < PAGANI_GPU_ROWS; i++) gpu_status[i][0] = 0;
    for (unsigned i = 0; i < PAGANI_DPU_ROWS; i++) dpu_status[i][0] = 0;
    for (unsigned i = 0; i < STATUS_ROWS; i++) status_shown[i][0] = 0;
    gpu_count = 0; dpu_count = 0; status_owned = 0; gpu_seq = 0; gpu_drawn_seq = 0; dpu_seq = 0; dpu_drawn_seq = 0;
    cpu_status[0] = 0; cpu_row = 0; cpu_seq = 0; cpu_drawn_seq = 0;
    status_dirty = 0; display_frames = 0;
    // The frame-rate state; the ring's slots stay (a slot passes only for the frame number that wrote it).
    fps_mode = 0; fps_row = 0; fps_owned = 0; fps_seq = 0; fps_drawn_seq = 0; fps_scanned = 0;
    fps_status[0] = 0; fps_shown[0] = 0; fps_next[0] = 0; fps_logged[0] = 0;
    slow_owned = 0; slow_status[0] = 0; slow_shown[0] = 0; slow_next[0] = 0; fps_extra[0] = 0; fps_extra_logged[0] = 0;
    frame_stage_now = 0;
    for (unsigned i = 0; i < PAGANI_DISPLAY_EVENTS; i++) { display_events[i] = 0; display_seen[i] = 0; }
    presented.count = 0; presented.paths[0] = 0; presented.paths[1] = 0;
    jitter_limit.credit = 0; jitter_limit.stamp = 0; jitter_limit.skipped = 0; jitter_limit.primed = 0;
    framebuffer_wc = 0; framebuffer_wc_bytes = 0;
    boot_progress_panic_text[0] = 0;
    framebuffer = 0; column = 0; row = LOG_TOP; busy = 0;
    framebuffer_pa = 0;
    if (!geometry(args->Video.v_baseAddr, args->Video.v_width, args->Video.v_height,
                  args->Video.v_rowBytes, args->Video.v_depth)) return;
    boot_progress_bind(args->Video.v_baseAddr, (unsigned)args->Video.v_width, (unsigned)args->Video.v_height,
                       (unsigned)args->Video.v_rowBytes, (unsigned)args->Video.v_depth);
    framebuffer_pa = framebuffer;
    for (unsigned y = 0; y + LINE <= height; y += LINE) clear_line(y);
    unsigned saved = row;
    row = 0; text_raw("OnePlus 13T / source XNU"); row = saved; column = 0;
    boot_progress_stage("S1: XNU entry, MMU off");
    value_raw("args PA", (uintptr_t)args);
    value_raw("RAM PA", args->physBase);
    value_raw("RAM VA", args->virtBase);
    value_raw("RAM bytes", args->memSize);
    value_raw("CNTFRQ Hz", frequency());
    render_boot_logo();
    barrier();
}

// Called with MMU off. Extend the existing TTBR0 identity map only over GOP.
int early_boot_progress_map(uint64_t *l1, uintptr_t free_page, uintptr_t end)
{
    const uint64_t block = 1ULL << 21, table_mask = 0x0000fffffffff000ULL;
    if (!framebuffer) return 1;
    uint64_t first = framebuffer_pa & ~(block - 1);
    uint64_t limit = (framebuffer_pa + (uint64_t)stride * 4 * height + block - 1) & ~(block - 1);
    // The kernel selects a full or half L1 table; never alias addresses beyond it.
    if (limit > (1ULL << (64 - T0SZ_BOOT)) || (free_page & 4095) || (end & 4095)) return 0;
    for (uint64_t pa = first; pa < limit; pa += block) {
        uint64_t *slot = &l1[(pa >> 30) & 511];
        if (*slot == 0) {
            if (free_page > end || end - free_page < 4096) return 0;
            volatile uint64_t *page = (volatile uint64_t *)free_page;
            for (unsigned i = 0; i < 512; i++) page[i] = 0;
            *slot = free_page | 3;
            free_page += 4096;
        }
        if ((*slot & 3) != 3) return 0;
        uint64_t *l2 = (uint64_t *)(uintptr_t)(*slot & table_mask);
        // Attr1 = normal non-cacheable in XNU's MAIR, outer-shareable, AF, EL1 RW, XN.
        uint64_t mapping = pa | (1ULL << 54) | (1ULL << 53) | 0x605;
        if (l2[(pa >> 21) & 511] != 0 && l2[(pa >> 21) & 511] != mapping) return 0;
        l2[(pa >> 21) & 511] = mapping;
    }
    barrier();
    early_boot_stage = 2;
    boot_progress_stage("S2: bootstrap maps ready");
    return 1;
}

void early_boot_progress_mmu_on(void) { early_boot_stage = 3; boot_progress_stage("S3: MMU enabled"); }
void early_boot_progress_arm_init(void) { early_boot_stage = 4; boot_progress_stage("S4: arm_init entered"); }
void early_boot_progress_suspend(void) { early_boot_stage = 0x50; barrier(); framebuffer = 0; }

void early_boot_progress_vm_ready(void)
{
    uint64_t bytes = (uint64_t)stride * 4 * height;
    uintptr_t mapped = framebuffer_pa ? ml_io_map_wcomb(framebuffer_pa, bytes) : 0;
    if (mapped) { framebuffer = mapped; framebuffer_wc = mapped; framebuffer_wc_bytes = bytes; }
    early_boot_progress_vectors_done();
    console_smp = 1;
    early_boot_stage = 6;
    // The persistent log (pagani-ramoops) starts here; it begins with the stages that ran before it could be mapped.
    if (ramoops_log_vm_ready()) {
        unsigned count = stage_count < HISTORY ? stage_count : HISTORY;
        for (unsigned i = stage_count - count; i < stage_count; i++) ramoops_log_line(RAMOOPS_KIND_STAGE, stages[i % HISTORY]);
    }
    boot_progress_stage("S6: kernel VM and exception vectors ready");
    const char *ramoops = ramoops_log_status();
    if (ramoops) boot_progress_stage(ramoops);
}

void boot_progress_init(void)
{
    early_boot_stage = 5;
    boot_progress_stage("S5: PE_init_platform ready");
    value_raw("kernel timebase Hz", gPEClockFrequencyInfo.timebase_frequency_hz);
    barrier();
}

// The assembly fatal path re-enters with physical addresses and MMU/cache off.
void early_boot_progress_fatal(boot_args *args, uint64_t esr, uint64_t elr, uint64_t far, uint64_t spsr, uint64_t stage)
{
    early_boot_progress_init(args);
    if (framebuffer) panic_open("OnePlus 13T / EARLY EXCEPTION");
    value_raw("last checkpoint", stage);
    value_raw("ESR_EL1", esr); value_raw("ELR_EL1", elr);
    value_raw("FAR_EL1", far); value_raw("SPSR_EL1", spsr);
    if (framebuffer) panic_footer("HALTED: no software reboot");
    panic_halt();
}

void boot_progress_kernel_exception(uint64_t esr, uint64_t elr, uint64_t far)
{
    if (panic_other_cpu()) { panic_halt(); return; }
    uint64_t flags = panic_state ? irq_disable() : irq_save();
    if (flags == UINT64_MAX) return;
    if (panic_state) {
        panic_limit = height / LINE;
        row = panic_limit - 2; column = 0; clear_line(row * LINE); clear_line((row + 1) * LINE);
        text_raw("NESTED EXCEPTION ESR "); hex_raw(esr); putc_raw('\n');
        text_raw("ELR "); hex_raw(elr); text_raw(" FAR "); hex_raw(far);
        gunyah_wdog_panic(); ramoops_log_panic_reset(); panic_halt(); return;
    }
    ramoops_log_exception(esr, elr, far);
    fault_valid = 1; fault_esr = esr; fault_pc = elr; fault_far = far;
#ifndef BOOT_PROGRESS_TEST
    __asm__ volatile("mrs %0, spsr_el1" : "=r"(fault_psr));
#endif
    record_stage("kernel exception");
    text_raw("[XNU] kernel exception\n");
    value_raw("ESR_EL1", esr); value_raw("ELR_EL1", elr); value_raw("FAR_EL1", far);
    barrier(); irq_restore(flags);
}

static void poll(void *unused0, void *unused1)
{
    (void)unused0; (void)unused1;
    if (panic_state) return;
    fps_observe();
    uint64_t flags = irq_save();
    if (flags == UINT64_MAX) goto schedule;
    if (framebuffer && !busy) {
        busy = 1;
        ++poll_count;
        if (logo_visible && !display_owned && poll_count >= 30) {
            logo_visible = 0;
            for (unsigned y = 0; y + LINE <= height; y += LINE) clear_line(y);
            row = LOG_TOP; column = 0;
            text_raw("No system frame after 30s; boot continues\n");
            unsigned count = stage_count < HISTORY ? stage_count : HISTORY;
            for (unsigned i = stage_count - count; i < stage_count; i++) {
                text_raw(stages[i % HISTORY]); putc_raw('\n');
            }
            text_raw(root_stage); putc_raw('\n');
            text_raw(root_detail); putc_raw('\n'); value_raw("root value", root_value);
            status_dirty = 1;   // the page cleared the status rows as well
        }
        unsigned saved_row = row, saved_column = column;
        row = 1; column = 0; clear_line(LINE);
        snprintf(timer_status, sizeof(timer_status), "poll %016llx timer %016llx",
                 (unsigned long long)poll_count, (unsigned long long)armcpu_timer_interrupts);
        line_raw(timer_status);
        row = 2; column = 0; clear_line(2 * LINE);
        if (&native_owner_cache_stage && native_owner_cache_stage &&
            &native_ufs_block_reads && &native_ufs_block_failures) {
            snprintf(storage_status, sizeof(storage_status), "free=%u avail=%u UFS=%llu err=%llu",
                     vm_page_free_count, memorystatus_available_pages,
                     (unsigned long long)native_ufs_block_reads, (unsigned long long)native_ufs_block_failures);
            line_raw(storage_status);
        } else {
            snprintf(storage_status, sizeof(storage_status), "DT/CPU/GIC %d/%d/%d PPI-enable %016llx",
                     !!armdt_platform_configured, !!armcpu_registered, !!armcpu_gic_ready,
                     (unsigned long long)armcpu_gic_boot[1]);
            line_raw(storage_status);
        }
        row = 4; column = 0; clear_line(4 * LINE);
        snprintf(display_status, sizeof(display_status), "DISPLAY %.48s %llx", display_stage,
                 (unsigned long long)display_value);
        line_raw(display_status);
        row = 5; column = 0; clear_line(5 * LINE); line_raw(exit_stage);
        if (__atomic_load_n(&fps_row, __ATOMIC_ACQUIRE)) {
            // Rows 8 and 9 share the sequence: either text changed, status_draw paints that row only.
            if (!status_same(fps_status, fps_next)) {
                copy_line(fps_status, sizeof(fps_status), fps_next);
                fps_seq++;
            }
            if (!status_same(slow_status, slow_next)) {
                copy_line(slow_status, sizeof(slow_status), slow_next);
                fps_seq++;
            }
        }
        status_draw();
        row = saved_row; column = saved_column; barrier(); busy = 0;
    }
    irq_restore(flags);
    boot_progress_peripheral_frame();
schedule:;
    gunyah_wdog_tick();   // pagani-wdog: the observer pets too (thread context: stops with the scheduler)
    ramoops_log_tick(poll_count, armcpu_timer_interrupts, vm_page_free_count, memorystatus_available_pages);
    uint64_t deadline;
    clock_interval_to_deadline(1, NSEC_PER_SEC, &deadline);
    thread_call_enter_delayed(poll_call, deadline);
}

// The reset paths of the test loop: RamoopsLog (PSCI facts, the pagani-reset-after timer), then pagani-wdog, the
// hypervisor watchdog that resets a kernel which stopped scheduling (GunyahWdog.h).
static void start_reset_paths(void)
{
    ramoops_log_start();
    const char *seq = ramoops_reset_seq_status();   // pagani-reset-seq 0x100: the probe's answers, also on the screen
    if (seq) boot_progress_stage(seq);
    gunyah_wdog_start();
    const char *wdog = gunyah_wdog_status();
    if (wdog) boot_progress_stage(wdog);
    ufs_log_start();                        // sheng-log=1: the log's writer thread (UfsLog.h); nothing without it
}

#if !defined(BOOT_PROGRESS_TEST) || defined(BOOT_PROGRESS_STALL_TEST)
static void stall_configure(void);
#else
static void stall_configure(void) {}
#endif

void boot_progress_start(void)
{
    uint32_t overlay = 1;
    if (boot_arg("pagani-overlay", &overlay) && !overlay) {
        boot_progress_stage("overlay: diagnostic text hidden (pagani-overlay=0); the ramoops log keeps it");
        __atomic_store_n(&overlay_hidden, 1, __ATOMIC_RELAXED);
    }
    fps_configure();
    stall_configure();
    poll_call = thread_call_allocate_with_options(poll, NULL, THREAD_CALL_PRIORITY_KERNEL, THREAD_CALL_OPTIONS_ONCE);
    if (!poll_call) { boot_progress_stage("timer observer allocation failed"); start_reset_paths(); return; }
    boot_progress_stage("timer observer scheduled; counters are hexadecimal");
    thread_call_enter(poll_call);
    start_reset_paths();
}

// Process observations retain identity so repeated execs and restart loops differ.
void boot_progress_irq(uint64_t pc, uint64_t psr, uint64_t sp) { (void)pc; (void)psr; (void)sp; }
void boot_progress_poll(void) {}
void boot_progress_user_breakpoint(void *state, unsigned esr) { (void)state; (void)esr; }
void boot_progress_user_abort(void *state, unsigned esr, uint64_t far) { (void)state; (void)esr; (void)far; }
void boot_progress_critical_exit(int pid, char *s, unsigned long n) { (void)pid; (void)s; (void)n; }
// pagani-exec-log=1: every exec step and exit to the log and status line. Off by default: a line per process start costs the
// console and the screen a redraw each, and a normal boot runs hundreds of them. Failed exits are always kept.
static int exec_log(void)
{
    static int value = -1;
    if (value < 0) {
        uint32_t arg = 0;
        value = PE_parse_boot_argn("pagani-exec-log", &arg, sizeof(arg)) && arg;
    }
    return value;
}
void boot_progress_process_stage(int pid, const char *name, const char *stage)
{
    if (!exec_log()) return;
    char line[STAGE_BYTES];
    snprintf(line, sizeof(line), "pid=%d %.20s %.36s", pid, name ? name : "?", stage ? stage : "?");
    // The on-medium snapshots read the kernel message buffer; a process record kept only in the RAM
    // ring is lost at power-off. Printed here, at the entry point the console never re-enters.
    printf("bp: %s\n", line);
    boot_progress_stage(line);
}
void boot_progress_exit(int pid, const char *name, int status, unsigned ns, uint64_t code)
{
    char line[STAGE_BYTES];
    snprintf(line, sizeof(line), "exit pid=%d %.16s status=%x ns=%u code=%llx", pid,
             name ? name : "?", (unsigned)status, ns, (unsigned long long)code);
    uint64_t flags = irq_save();
    if (flags == UINT64_MAX) return;
    if (!panic_state && !busy) copy_line(exit_stage, sizeof(exit_stage), line);
    irq_restore(flags);
    if (!exec_log() && !status) return;
    printf("bp: %s\n", line);
    boot_progress_stage(line);
}

// Fatal signals (CrashTrace.h): the device repository's patches/xnu/crash-trace.patch calls this from postsig_locked, through
// osfmk/kern/bsd_kern.c, on the thread taking the signal with interrupts enabled and no lock held, before the S exit record.
// The X records go to the events ring. pagani-crash=0 turns it off, =2 writes every crash in full (no rate limit).
static struct crash_table crash_table;
static uint32_t crash_lock_word, crash_mode = UINT32_MAX;

static uint64_t crash_lock(void)
{
    uint64_t flags = irq_disable();
    for (uint32_t spins = 0; __atomic_exchange_n(&crash_lock_word, 1, __ATOMIC_ACQUIRE); spins++) {
        if (spins >= (1u << 20) || __atomic_load_n(&panic_cpu, __ATOMIC_ACQUIRE)) {
            irq_enable(flags);
            return UINT64_MAX;
        }
#ifndef BOOT_PROGRESS_TEST
        __asm__ volatile("yield" ::: "memory");
#endif
    }
    return flags;
}

static void crash_unlock(uint64_t flags)
{
    __atomic_store_n(&crash_lock_word, 0, __ATOMIC_RELEASE);
    irq_enable(flags);
}

static void crash_emit(void *context, const char *line)
{
    (void)context;
    ramoops_log_line(RAMOOPS_KIND_CRASH, line);
}

void boot_progress_crash(const struct boot_progress_crash *crash)
{
    static const struct crash_sink sink = {crash_emit, crash_lock, crash_unlock, NULL};
    uint32_t mode = __atomic_load_n(&crash_mode, __ATOMIC_RELAXED);
    if (mode == UINT32_MAX) {
        mode = CRASH_MODE_ON;
        (void)boot_arg("pagani-crash", &mode);
        __atomic_store_n(&crash_mode, mode, __ATOMIC_RELAXED);
    }
    crash_report(&crash_table, crash, mode, &sink);
}

#if !defined(BOOT_PROGRESS_TEST) || defined(BOOT_PROGRESS_STALL_TEST)
// The stall probe's glue (StallProbe.h says what runs where and why that is safe). XNU's side is phone-stall-probe.patch: weak here, a kernel
// without it reports that once and goes on without the probe. Only the sampler's thread call touches stall_watch, stall_gap, stall_limit and
// the targets; the observer reads their counters; the frame path and the hooks touch stall_state and the words noted below.
#pragma weak pagani_stall_on
#pragma weak boot_progress_stall_abi
#pragma weak boot_progress_stall_thread_read
#pragma weak boot_progress_stall_regs_here
#pragma weak boot_progress_stall_xcall
#pragma weak boot_progress_stall_task_find
#pragma weak boot_progress_stall_task_release
#pragma weak boot_progress_stall_foreground
#pragma weak boot_progress_stall_thread_find
#pragma weak boot_progress_stall_thread_release
#pragma weak boot_progress_stall_thread_times
#pragma weak boot_progress_stall_map
#pragma weak boot_progress_stall_map_release
#pragma weak boot_progress_stall_map_read
#pragma weak boot_progress_stall_image
#pragma weak boot_progress_stall_cpu_read
#pragma weak boot_progress_stall_vm_read
#pragma weak boot_progress_stall_mem_read
extern uint32_t pagani_stall_on;
uint32_t boot_progress_stall_abi(void);
int boot_progress_stall_thread_read(void *thread, struct boot_progress_stall_thread *out);
int boot_progress_stall_regs_here(void *thread, struct boot_progress_stall_regs *out);
int boot_progress_stall_xcall(unsigned cpu, void (*function)(void *), void *parameter);
void *boot_progress_stall_task_find(const char *name, int32_t pid, struct boot_progress_stall_task *out);
void boot_progress_stall_task_release(void *task);
int32_t boot_progress_stall_foreground(const char *not_a, const char *not_b, uint32_t *count);
void *boot_progress_stall_thread_find(void *task, const char *name, uint64_t tid);
void boot_progress_stall_thread_release(void *thread);
unsigned boot_progress_stall_thread_times(void *task, uint64_t *tids, uint64_t *times, unsigned max);
void *boot_progress_stall_map(void *task);
void boot_progress_stall_map_release(void *map);
int boot_progress_stall_map_read(void *map, uint64_t address, void *buffer, uint32_t bytes);
int boot_progress_stall_image(void *task, uint64_t address, struct boot_progress_crash_image *image);
void boot_progress_stall_cpu_read(unsigned cpu, struct boot_progress_stall_cpu *out);
void boot_progress_stall_vm_read(struct boot_progress_stall_vm *out);
void boot_progress_stall_mem_read(struct boot_progress_stall_mem *out);
// Strong here, weak no-ops in XNU.
void boot_progress_stall_run(const struct boot_progress_stall_run *run);
void boot_progress_stall_idle(unsigned cpu);
void boot_progress_stall_ipi_send(unsigned cpu);
void boot_progress_stall_ipi_recv(unsigned cpu);
void boot_progress_stall_kbt(int32_t pid, unsigned group, int end);
// The inputs of the triggers and of the M records. Weak zero like everywhere else: a kernel without that driver reads nothing new.
extern uint32_t adreno830_cpu_hint(void) __attribute__((weak));
extern volatile uint64_t native_ufs_read_ticks __attribute__((weak)), native_ufs_verify_ticks __attribute__((weak));
extern volatile uint64_t native_ram_block_resident_bytes __attribute__((weak));
extern volatile uint32_t native_adreno_pool_stats[8] __attribute__((weak));     // [3] MiB in use, [4] peak MiB
__attribute__((weak)) volatile uint32_t pagani_input_touch_events;

// One thread the samples look at, by reference.
struct stall_target {
    void *task, *thread;
    uint64_t looked;                        // when it was last looked up
    struct boot_progress_stall_task info;
};

// The file mapping of one address, between the sampler and the low-priority thread call.
struct stall_image_work {
    void *task;
    uint64_t address;
    int32_t pid;
    uint32_t state, found;
    struct boot_progress_crash_image image;
};
enum { STALL_IMAGE_FREE, STALL_IMAGE_ASKED, STALL_IMAGE_DONE };

// A sample's records: the head, two of CPUs, a th and a bt of each thread.
enum { STALL_THREADS = 3, STALL_SAMPLE_BASE_LINES = 3, STALL_THREAD_LINES = 2 };
#define STALL_SAMPLE_LINES(threads) (STALL_SAMPLE_BASE_LINES + STALL_THREAD_LINES * (threads))

// The threads of a sample.
struct stall_pick {
    struct stall_target *target;
    const char *tag;
};

static struct stall_state stall;
static uint32_t stall_mode, stall_armed, stall_dead;
static thread_call_t stall_call, stall_image_call;
static uint32_t stall_ticking;
// Sampler alone.
static struct stall_watch stall_watching;
static struct stall_gap stall_gaps;
static struct stall_limit stall_limits, stall_kbt_limits, stall_tail_limits;
static struct stall_busy stall_kbd_busy;
static struct stall_known stall_known_maps;
static struct stall_target stall_rs, stall_sb, stall_fg, stall_app, stall_kbd;
static struct stall_slot stall_slots[STALL_CPUS];
static struct stall_cpu_view stall_cpus[STALL_CPUS];
static struct crash_trace stall_walked;
static struct boot_progress_crash stall_input;
static struct boot_progress_stall_thread stall_read;
static struct boot_progress_stall_task stall_image_task;
static char stall_line[STALL_LINE + 1];
static uint32_t stall_samples, stall_watch_stalls, stall_gpu_seen;
static uint64_t stall_gpu_at, stall_last_tick;
static struct stall_image_work stall_image;
// The sampler writes these, the observer takes them.
static uint64_t stall_tick_max, stall_touch_at;
static uint32_t stall_free_min = UINT32_MAX, stall_images;
static uint32_t stall_touch_seen;
// Taken under the lock below by any thread, and by the sampler.
static struct stall_kbt stall_open;
static uint32_t stall_open_lock;
// The observer alone.
static struct stall_seen stall_seen_totals;
static struct stall_second stall_this_second;
static uint64_t stall_seen_hungry[STALL_CPUS], stall_seen_stuck[STALL_CPUS];
static uint64_t stall_seen_reads, stall_seen_read_ticks, stall_seen_verify_ticks;
static struct boot_progress_stall_vm stall_seen_vm;
static uint32_t stall_seen_timer[STALL_CPUS], stall_seen_touches;
static uint64_t stall_m_until, stall_mem_next;
static uint32_t stall_touches;              // of the pass under way: stall_touched sets it, stall_fps_extra and stall_second read it
static char stall_extra[FPS_EXTRA_BYTES + 48];
static char stall_second_line[STALL_LINE + 1];

static int stall_halted(void)
{
    if (__atomic_load_n(&stall_dead, __ATOMIC_ACQUIRE)) return 1;
    if (!__atomic_load_n(&panic_cpu, __ATOMIC_ACQUIRE) && !panic_state && !ramoops_log_resetting()) return 0;
    __atomic_store_n(&stall_dead, 1, __ATOMIC_RELEASE);
    return 1;
}

static void stall_halt(void)
{
    __atomic_store_n(&stall_dead, 1, __ATOMIC_RELEASE);
    if (&pagani_stall_on) __atomic_store_n(&pagani_stall_on, 0, __ATOMIC_RELEASE);
}

// The hooks: on, they are all this probe adds to those paths (StallProbe.h).
void boot_progress_stall_run(const struct boot_progress_stall_run *run) { stall_note_run(&stall, run); }
void boot_progress_stall_idle(unsigned cpu) { stall_note_idle(&stall, cpu); }
void boot_progress_stall_ipi_send(unsigned cpu) { stall_note_ipi_send(&stall, cpu, frame_clock()); }
void boot_progress_stall_ipi_recv(unsigned cpu) { stall_note_ipi_recv(&stall, cpu, frame_clock()); }

static void stall_arm(void)
{
    uint32_t idle = 0;
    if (__atomic_load_n(&stall_dead, __ATOMIC_ACQUIRE)) return;
    if (__atomic_compare_exchange_n(&stall_armed, &idle, 1, 0, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED)) thread_call_enter(stall_call);
}

static void stall_frame(void)
{
    if (__atomic_load_n(&stall_mode, __ATOMIC_RELAXED) && !__atomic_load_n(&stall_armed, __ATOMIC_RELAXED)) stall_arm();
}

// The table of open kbt pairs: a lock word like crash_lock's, held for a few loads and stores. UINT64_MAX: not taken.
static uint64_t stall_open_take(void)
{
    uint64_t flags = irq_disable();
    for (uint32_t spins = 0; __atomic_exchange_n(&stall_open_lock, 1, __ATOMIC_ACQUIRE); spins++) {
        if (spins >= (1u << 16) || __atomic_load_n(&panic_cpu, __ATOMIC_ACQUIRE)) {
            irq_enable(flags);
            return UINT64_MAX;
        }
        __asm__ volatile("yield" ::: "memory");
    }
    return flags;
}

static void stall_open_give(uint64_t flags)
{
    __atomic_store_n(&stall_open_lock, 0, __ATOMIC_RELEASE);
    irq_enable(flags);
}

void boot_progress_stall_kbt(int32_t pid, unsigned group, int end)
{
    if (!__atomic_load_n(&stall_mode, __ATOMIC_RELAXED)) return;
    const uint64_t flags = stall_open_take();
    if (flags == UINT64_MAX) return;
    stall_kbt_note(&stall_open, pid, group, end, frame_clock(), frequency());
    stall_open_give(flags);
    if (!end) stall_frame();
}

static void stall_emit(const char *line)
{
    ramoops_log_line(RAMOOPS_KIND_STALL, line);
}

// The cross-call's handler, on the CPU of the thread with interrupts masked. An answer that comes late finds the slot as the sampler left it.
static void stall_xcall(void *parameter)
{
    struct stall_slot *slot = parameter;
    stall_slot_answer(slot, (uint32_t)boot_progress_stall_regs_here(stall_slot_wanted(slot), &slot->regs));
}

// Asks `cpu` for `thread`'s interrupted state. "ok" and *regs, "mv" (it is not on that CPU any more), "to" (no answer in STALL_XCALL_US, or
// an earlier question still unanswered), "no" (the call did not go out).
static const char *stall_ask(unsigned cpu, void *thread, const struct boot_progress_stall_regs **regs, uint64_t hz)
{
    if (cpu >= STALL_CPUS) return "no";
    struct stall_slot *slot = &stall_slots[cpu];
    if (stall_halted()) return "no";
    if (!stall_slot_ask(slot, thread)) return "to";
    if (boot_progress_stall_xcall(cpu, stall_xcall, slot)) {
        stall_slot_cancel(slot);
        return "no";
    }
    const uint64_t began = frame_clock(), patience = hz / 1000000 * STALL_XCALL_US;
    while (!stall_slot_answered(slot)) {
        if (frame_clock() - began >= patience) return "to";
        __asm__ volatile("yield" ::: "memory");
    }
    if (!stall_slot_found(slot)) return "mv";
    *regs = &slot->regs;
    return "ok";
}

static void stall_target_drop(struct stall_target *t)
{
    if (t->thread) boot_progress_stall_thread_release(t->thread);
    if (t->task) boot_progress_stall_task_release(t->task);
    t->thread = t->task = NULL;
}

// A task by name or pid, at most once in STALL_LOOKUP_MS when it failed; its map record the first time. 1 when t->task holds it.
static int stall_target_task(struct stall_target *t, const char *name, int32_t pid, struct stall_limit *limit, unsigned level, uint64_t now, uint64_t hz)
{
    if (t->task) return 1;
    if (t->looked && now - t->looked < hz / 1000 * STALL_LOOKUP_MS) return 0;
    t->looked = now;
    t->task = boot_progress_stall_task_find(name, pid, &t->info);
    if (!t->task) return 0;
    if (stall_limit_take(limit, now, hz, 1, level)) {
        stall_line_map(stall_line, &t->info);
        stall_emit(stall_line);
    }
    return 1;
}

// One thread of a sample: its state, the state of its CPU when it runs, its frame chain; a th and a bt record. Reads under references only.
static void stall_sample_thread(struct stall_target *t, const char *tag, uint32_t id, uint64_t hz)
{
    if (!boot_progress_stall_thread_read(t->thread, &stall_read) || !stall_read.active) {
        stall_target_drop(t);
        struct crash_text x;
        stall_head(&x, stall_line, id, "no ");
        crash_put_str(&x, tag);
        stall_emit(stall_line);
        return;
    }
    const struct boot_progress_stall_regs *regs = &stall_read.user;
    const char *asked = NULL;
    if (stall_read.cpu >= 0) {
        asked = stall_ask((unsigned)stall_read.cpu, t->thread, &regs, hz);
        // It moved: its PCB may be good by now, or it is on another CPU; one more look.
        if (asked[0] == 'm' && boot_progress_stall_thread_read(t->thread, &stall_read) && stall_read.cpu >= 0) {
            regs = &stall_read.user;
            asked = stall_ask((unsigned)stall_read.cpu, t->thread, &regs, hz);
        }
    }
    const uint64_t now = frame_clock();
    struct stall_thread_view view;
    view.id = id;
    view.tag = tag;
    view.pid = t->info.pid;
    view.name = t->info.name;
    view.t = &stall_read;
    view.regs = regs;
    view.now = now;
    view.hz = hz;
    view.since = stall_read.cpu >= 0 && stall_read.cpu < STALL_CPUS ? stall_cpus[stall_read.cpu].on.since : 0;
    view.xcall = asked;
    stall_line_thread(stall_line, &view);
    stall_emit(stall_line);
    void *map = boot_progress_stall_map(t->task);
    struct boot_progress_crash *c = &stall_input;
    c->pc = regs->pc;
    c->lr = regs->lr;
    c->fp = map && regs->valid ? regs->fp : 0;
    c->user_limit = t->info.user_limit;
    c->cache_start = t->info.cache_start;
    c->cache_end = t->info.cache_end;
    c->cache_slide = t->info.cache_slide;
    c->main_header = t->info.main_header;
    c->context = map;
    c->read = boot_progress_stall_map_read;
    c->image = NULL;
    stall_trace_take(c, &stall_walked);
    if (map) boot_progress_stall_map_release(map);
    stall_line_bt(stall_line, id, tag, t->info.pid, regs, &stall_walked);
    stall_emit(stall_line);
    // A frame outside the shared cache: its mapping is looked up at a low priority, once (Q img).
    if (__atomic_load_n(&stall_image.state, __ATOMIC_ACQUIRE) == STALL_IMAGE_FREE) {
        const uint64_t address = stall_trace_unknown(&stall_walked, &stall_known_maps, t->info.pid);
        // Not into t->info: a list in use writes its pid, and that pid looked up next would be another process.
        void *task = address ? boot_progress_stall_task_find(NULL, t->info.pid, &stall_image_task) : NULL;
        if (task) {
            stall_image.task = task;
            stall_image.address = address;
            stall_image.pid = t->info.pid;
            __atomic_store_n(&stall_image.state, STALL_IMAGE_ASKED, __ATOMIC_RELEASE);
            thread_call_enter(stall_image_call);
        }
    }
}

static void stall_image_lookup(void *unused0, void *unused1)
{
    (void)unused0; (void)unused1;
    if (__atomic_load_n(&stall_image.state, __ATOMIC_ACQUIRE) != STALL_IMAGE_ASKED || stall_halted()) return;
    stall_image.found = (uint32_t)boot_progress_stall_image(stall_image.task, stall_image.address, &stall_image.image);
    __atomic_store_n(&stall_image.state, STALL_IMAGE_DONE, __ATOMIC_RELEASE);
}

// What the lookup found goes to the log and the table; the page of an address without a file mapping counts as known too.
static void stall_image_finish(uint64_t now, uint64_t hz)
{
    if (__atomic_load_n(&stall_image.state, __ATOMIC_ACQUIRE) != STALL_IMAGE_DONE) return;
    boot_progress_stall_task_release(stall_image.task);
    if (stall_image.found == 1) {
        stall_known_add(&stall_known_maps, stall_image.pid, stall_image.image.start, stall_image.image.end);
        if (stall_limit_take(&stall_limits, now, hz, 1, STALL_LEVEL_HIGH) || stall_limit_take(&stall_kbt_limits, now, hz, 1, STALL_LEVEL_KBT)) {
            stall_line_img(stall_line, stall_image.pid, &stall_image.image);
            stall_emit(stall_line);
            __atomic_store_n(&stall_images, stall_images + 1, __ATOMIC_RELAXED);
        }
    } else {
        stall_known_add(&stall_known_maps, stall_image.pid, stall_image.address & ~0xfffULL, (stall_image.address & ~0xfffULL) + 0x1000);
    }
    __atomic_store_n(&stall_image.state, STALL_IMAGE_FREE, __ATOMIC_RELEASE);
}

// The head of a sample and the CPUs, as close to the trigger as the sampler gets, then the threads.
static void stall_sample(struct stall_sample_head *head, struct stall_limit *limit, unsigned level, const struct stall_pick *picks, unsigned threads,
                         uint64_t now, uint64_t hz)
{
    if (!stall_limit_take(limit, now, hz, STALL_SAMPLE_LINES(threads), level)) return;
    head->id = ++stall_samples;
    const uint64_t touch = __atomic_load_n(&stall_touch_at, __ATOMIC_RELAXED);
    head->touch_ms = touch ? stall_ms(now > touch ? now - touch : 0, hz) : UINT64_MAX;      // the observer may have stamped it after `now`
    head->gpu_ms = stall_gpu_at ? stall_ms(now - stall_gpu_at, hz) : UINT64_MAX;
    stall_line_head(stall_line, head);
    stall_emit(stall_line);
    for (unsigned c = 0; c < STALL_CPUS; c++) stall_cpus[c].ok = stall_oncore_read(&stall.cpu[c], &stall_cpus[c].on);
    for (unsigned c = 0; c < STALL_CPUS; c += 4) {
        stall_line_cpus(stall_line, head->id, c, 4, stall_cpus, now, hz);
        stall_emit(stall_line);
    }
    for (unsigned i = 0; i < threads; i++) {
        if (stall_halted()) return;
        if (picks[i].target->thread) {
            stall_sample_thread(picks[i].target, picks[i].tag, head->id, hz);
        } else {
            struct crash_text x;
            stall_head(&x, stall_line, head->id, "no ");
            crash_put_str(&x, picks[i].tag);
            stall_emit(stall_line);
        }
    }
}

// A task whose thread did not turn up for STALL_LOOKUP_MS goes back to the lookup by name: when that process exited, this task has no thread left to
// find and stall_sample_thread, the one that lets go of a dead target, never gets a thread to look at.
static void stall_target_retry(struct stall_target *t, uint64_t now, uint64_t hz)
{
    if (t->task && !t->thread && now - t->looked >= hz / 1000 * STALL_LOOKUP_MS) stall_target_drop(t);
}

// backboardd's render server and SpringBoard's main thread, by reference; a lost one is looked up again.
static void stall_look_system(uint64_t now, uint64_t hz)
{
    if (stall_target_task(&stall_rs, "backboardd", 0, &stall_limits, STALL_LEVEL_HIGH, now, hz) && !stall_rs.thread) {
        stall_rs.thread = boot_progress_stall_thread_find(stall_rs.task, "render-server", 0);   // com.apple.coreanimation.render-server
        if (!stall_rs.thread) stall_rs.thread = boot_progress_stall_thread_find(stall_rs.task, "coreanimation", 0);
    }
    if (stall_target_task(&stall_sb, "SpringBoard", 0, &stall_limits, STALL_LEVEL_HIGH, now, hz) && !stall_sb.thread)
        stall_sb.thread = boot_progress_stall_thread_find(stall_sb.task, NULL, 0);
    stall_target_retry(&stall_rs, now, hz);
    stall_target_retry(&stall_sb, now, hz);
}

// The App on screen, asked for each gap sample: it changes with every switch, and a role is one word of a task (StallProbe.h). *focal for the head;
// 1 when stall_fg holds the main thread of the App. Nobody in the role is the home screen: nothing is held for it. A busy list leaves the target alone
// for the next sample, and this one goes without.
static int stall_look_foreground(int32_t *focal, uint64_t now, uint64_t hz)
{
    uint32_t count = 0;
    const int32_t pid = boot_progress_stall_foreground("backboardd", "SpringBoard", &count);
    *focal = pid < 0 ? -1 : (int32_t)count;
    if (pid < 0) return 0;
    if (stall_fg.task && stall_fg.info.pid != pid) {
        stall_target_drop(&stall_fg);
        stall_fg.looked = 0;
    }
    if (!pid) return 0;
    if (stall_target_task(&stall_fg, NULL, pid, &stall_limits, STALL_LEVEL_HIGH, now, hz) && !stall_fg.thread)
        stall_fg.thread = boot_progress_stall_thread_find(stall_fg.task, NULL, 0);
    stall_target_retry(&stall_fg, now, hz);
    return stall_fg.task != NULL;
}

// The App of an open kbt pair, its main thread, and kbd's thread with the most CPU time since the last look.
static void stall_look_keyboard(int32_t pid, uint64_t now, uint64_t hz)
{
    static uint64_t tids[STALL_BUSY_THREADS], times[STALL_BUSY_THREADS];
    if (stall_app.task && stall_app.info.pid != pid) {
        stall_target_drop(&stall_app);
        stall_app.looked = 0;
    }
    if (stall_target_task(&stall_app, NULL, pid, &stall_kbt_limits, STALL_LEVEL_KBT, now, hz) && !stall_app.thread)
        stall_app.thread = boot_progress_stall_thread_find(stall_app.task, NULL, 0);
    if (!stall_target_task(&stall_kbd, "kbd", 0, &stall_kbt_limits, STALL_LEVEL_KBT, now, hz)) return;
    const unsigned n = boot_progress_stall_thread_times(stall_kbd.task, tids, times, STALL_BUSY_THREADS);
    if (n > STALL_BUSY_THREADS) return;         // its lock was busy: the last look, and the base of the deltas, stand
    const uint64_t tid = stall_busiest(&stall_kbd_busy, tids, times, n);
    if (stall_kbd.thread) boot_progress_stall_thread_release(stall_kbd.thread);
    stall_kbd.thread = tid ? boot_progress_stall_thread_find(stall_kbd.task, NULL, tid) : NULL;
    if (!n) stall_target_drop(&stall_kbd);      // no thread left: kbd exited, the next sample looks for the new one
}

// One pass over the CPUs: 0 or more hungry or with an IPI stuck; the first of each kind in *hungry, *stuck.
static void stall_watch_pass(uint64_t now, uint64_t hz, int *hungry, int *stuck)
{
    *hungry = *stuck = -1;
    for (unsigned c = 0; c < STALL_CPUS; c++) {
        boot_progress_stall_cpu_read(c, &stall_cpus[c].sched);
        if (stall_watch_cpu(&stall_watching, c, &stall_cpus[c].sched) && *hungry < 0) *hungry = (int)c;
        if (stall_watch_ipi(&stall_watching, c, STALL_LOAD(&stall.cpu[c].pending), now, hz) && *stuck < 0) *stuck = (int)c;
    }
}

static uint32_t stall_frames_before(uint64_t count, uint64_t newest, uint64_t hz)
{
    uint32_t n = 0;
    for (unsigned back = 2; back <= STALL_CADENCE_FRAMES + 1 && count >= back; back++) {
        uint64_t stamp = 0;
        unsigned path = 0;
        if (!frame_ring_get(&presented, count - back, &stamp, &path) || newest < stamp || stall_ms(newest - stamp, hz) > STALL_ACTIVE_MS) break;
        n++;
    }
    return n;
}

static const struct stall_pick stall_picks_gap[STALL_THREADS] = {{&stall_rs, "rs"}, {&stall_sb, "sb"}, {&stall_fg, "fg"}};
static const struct stall_pick stall_picks_kbt[2] = {{&stall_app, "app"}, {&stall_kbd, "kbd"}};

static void stall_tick(void *unused0, void *unused1)
{
    (void)unused0; (void)unused1;
    if (stall_halted() || __atomic_exchange_n(&stall_ticking, 1, __ATOMIC_ACQUIRE)) return;
    const uint64_t hz = frequency(), now = frame_clock();
    if (stall_last_tick && now - stall_last_tick > __atomic_load_n(&stall_tick_max, __ATOMIC_RELAXED))
        __atomic_store_n(&stall_tick_max, now - stall_last_tick, __ATOMIC_RELAXED);
    stall_last_tick = now;
    const uint32_t touches = __atomic_load_n(&pagani_input_touch_events, __ATOMIC_RELAXED);
    if (touches != stall_touch_seen) {
        stall_touch_seen = touches;
        __atomic_store_n(&stall_touch_at, now, __ATOMIC_RELAXED);
    }
    const uint32_t gpu = adreno830_cpu_hint ? adreno830_cpu_hint() : 0;
    if (gpu != stall_gpu_seen) {
        stall_gpu_seen = gpu;
        stall_gpu_at = now;
    }
    if (vm_page_free_count < __atomic_load_n(&stall_free_min, __ATOMIC_RELAXED)) __atomic_store_n(&stall_free_min, vm_page_free_count, __ATOMIC_RELAXED);
    stall_image_finish(now, hz);
    int hungry, stuck;
    stall_watch_pass(now, hz, &hungry, &stuck);
    const uint64_t count = __atomic_load_n(&presented.count, __ATOMIC_ACQUIRE);
    uint64_t newest = stall_gaps.newest;
    unsigned path = 0;
    if (count) (void)frame_ring_get(&presented, count - 1, &newest, &path);
    struct stall_gap_in in;
    in.now = now;
    in.hz = hz;
    in.frames = count;
    in.newest = newest;
    in.frames_before = count ? stall_frames_before(count, newest, hz) : 0;
    in.gpu = gpu;
    in.gpu_at = stall_gpu_at;
    in.touch_at = __atomic_load_n(&stall_touch_at, __ATOMIC_RELAXED);
    in.kbt_open = 0;
    struct stall_sample_head head;
    head.stall = head.sample = head.gap_ms = head.open = head.sign = 0;
    head.focal = 0;
    head.age_ms = head.since_ms = 0;
    head.frames_before = in.frames_before;
    head.pid = 0;
    head.name = NULL;
    head.why = NULL;
    head.cpu = -1;
    struct stall_kbt_due due;
    uint64_t flags = stall_open_take();
    int keyed = 0, open = 1;
    if (flags != UINT64_MAX) {
        keyed = stall_kbt_take(&stall_open, now, hz, &due);
        open = stall_kbt_any(&stall_open);
        stall_open_give(flags);
        in.kbt_open = (uint32_t)open;
    }
    if (keyed) {                                // the one sample of this tick; the others are still due at the next
        stall_look_keyboard(due.pid, now, hz);
        if (!stall_app.task && stall_app.looked == now && !stall_app.info.pid) {   // the list was read, the process is not on it
            flags = stall_open_take();
            if (flags != UINT64_MAX) {
                stall_kbt_forget(&stall_open, due.pid);
                stall_open_give(flags);
            }
        }
        head.why = stall_kbt_names[due.group < STALL_KBT_GROUPS ? due.group : 0];
        head.stall = due.id;
        head.sample = due.sample;
        head.age_ms = due.age_ms;
        head.pid = due.pid;
        head.name = stall_app.task ? stall_app.info.name : "?";
        head.open = due.open;
        stall_sample(&head, &stall_kbt_limits, STALL_LEVEL_KBT, stall_picks_kbt, 2, now, hz);
    } else if (stuck >= 0 || hungry >= 0) {
        stall_look_system(now, hz);
        head.why = stuck >= 0 ? "ipi" : "idleq";
        head.stall = ++stall_watch_stalls;
        head.sample = 1;
        head.cpu = stuck >= 0 ? stuck : hungry;
        stall_sample(&head, &stall_limits, STALL_LEVEL_HIGH, stall_picks_gap, 2, now, hz);
    } else {
        const uint32_t ms = stall_gap_step(&stall_gaps, &in);
        if (ms) {
            stall_look_system(now, hz);
            const int app = stall_look_foreground(&head.focal, now, hz);
            head.gap_ms = ms;
            head.stall = stall_gaps.stalls;
            head.sample = stall_gaps.samples;
            head.age_ms = stall_ms(now - stall_gap_from(&stall_gaps), hz);
            head.since_ms = stall_ms(now - stall_gaps.newest, hz);
            head.sign = stall_gaps.sign;
            const unsigned level = stall_gap_level(&stall_gaps, ms);
            stall_sample(&head, level == STALL_LEVEL_TAIL ? &stall_tail_limits : &stall_limits, level, stall_picks_gap, app ? 3 : 2, now, hz);
        }
    }
    __atomic_store_n(&stall_ticking, 0, __ATOMIC_RELEASE);
    if (stall_halted()) return;
    const uint64_t window = hz / 1000 * STALL_ARMED_MS, touched = in.touch_at;
    if (open || stall_gap_waiting(&stall_gaps) || (newest && now - newest < window) || (touched && now - touched < window)) {
        uint64_t deadline;
        clock_interval_to_deadline(STALL_TICK_MS, 1000000, &deadline);
        thread_call_enter_delayed(stall_call, deadline);
        return;
    }
    // Asleep until a frame, a touch or a kbt pair: stall_frame and the observer arm it again. stall_last_tick starts over.
    stall_last_tick = 0;
    __atomic_store_n(&stall_armed, 0, __ATOMIC_RELEASE);
    if (__atomic_load_n(&presented.count, __ATOMIC_ACQUIRE) != count) stall_arm();
}

static int stall_touched(void)
{
    if (!__atomic_load_n(&stall_mode, __ATOMIC_RELAXED)) return 0;
    const uint32_t value = __atomic_load_n(&pagani_input_touch_events, __ATOMIC_RELAXED);
    stall_touches = value - stall_seen_touches;
    stall_seen_touches = value;
    if (stall_touches) {
        // The sampler was asleep: this second is as close as the time of that touch gets.
        if (!__atomic_load_n(&stall_armed, __ATOMIC_RELAXED)) __atomic_store_n(&stall_touch_at, frame_clock(), __ATOMIC_RELAXED);
        stall_arm();
    }
    return stall_touches != 0;
}

static const char *stall_fps_extra(const char *extra)
{
    if (!__atomic_load_n(&stall_mode, __ATOMIC_RELAXED)) return extra;
    const uint32_t late = display_delta(PAGANI_DISPLAY_VSYNC_LATE);
    const uint64_t touch = __atomic_load_n(&stall_touch_at, __ATOMIC_RELAXED), now = frame_clock();
    unsigned used = 0;
    for (; extra && extra[used] && used + 1 < sizeof(stall_extra); used++) stall_extra[used] = extra[used];
    stall_extra[used] = 0;
    stall_fps_tail(stall_extra + used, (unsigned)sizeof(stall_extra) - used, stall_touches, touch ? stall_ms(now - touch, frequency()) : UINT64_MAX, late);
    return stall_extra;
}

static void stall_meter(const char *line)
{
    ramoops_log_line(RAMOOPS_KIND_METER, line);
}

// The observer's pass: every total moves on each second; the records until the first frame, then while the sampler is awake and STALL_M_TAIL_S past
// the last frame or touch.
static void stall_second(uint32_t frames)
{
    if (!__atomic_load_n(&stall_mode, __ATOMIC_RELAXED)) return;
    const uint64_t hz = frequency();
    static struct stall_ipi_line ipi;
    static struct stall_io_line io;
    static struct stall_mem_line mem;
    static struct boot_progress_stall_vm vm;
    static uint32_t timers[STALL_CPUS];
    stall_second_take(&stall, &stall_seen_totals, &stall_this_second);
    for (unsigned c = 0; c < STALL_CPUS; c++) {
        const uint32_t timer = __atomic_load_n(&armcpu_core_timer[c], __ATOMIC_RELAXED);
        timers[c] = timer - stall_seen_timer[c];
        stall_seen_timer[c] = timer;
        ipi.hungry[c] = stall_delta32(STALL_LOAD(&stall_watching.hungry[c]), &stall_seen_hungry[c]);
        ipi.stuck[c] = stall_delta32(STALL_LOAD(&stall_watching.stuck[c]), &stall_seen_stuck[c]);
    }
    ipi.tick_ticks = __atomic_exchange_n(&stall_tick_max, 0, __ATOMIC_RELAXED);
    ipi.images = __atomic_load_n(&stall_images, __ATOMIC_RELAXED);
    boot_progress_stall_vm_read(&vm);
    const uint64_t reads = &native_ufs_block_reads ? native_ufs_block_reads : 0;
    const uint64_t read_ticks = &native_ufs_read_ticks ? native_ufs_read_ticks : 0;
    const uint64_t verify_ticks = &native_ufs_verify_ticks ? native_ufs_verify_ticks : 0;
    io.reads = reads - stall_seen_reads;
    io.read_ticks = read_ticks - stall_seen_read_ticks;
    io.verify_ticks = verify_ticks - stall_seen_verify_ticks;
    stall_seen_reads = reads;
    stall_seen_read_ticks = read_ticks;
    stall_seen_verify_ticks = verify_ticks;
    io.faults = vm.faults - stall_seen_vm.faults;
    io.pageins = vm.pageins - stall_seen_vm.pageins;
    io.compressions = vm.compressions - stall_seen_vm.compressions;
    io.decompressions = vm.decompressions - stall_seen_vm.decompressions;
    io.page_waits = vm.page_waits - stall_seen_vm.page_waits;
    for (unsigned t = 0; t < 3; t++) {
        io.throttled[t] = vm.throttled[t + 1] - stall_seen_vm.throttled[t + 1];
        stall_seen_vm.throttled[t + 1] = vm.throttled[t + 1];
    }
    stall_seen_vm.faults = vm.faults;
    stall_seen_vm.pageins = vm.pageins;
    stall_seen_vm.compressions = vm.compressions;
    stall_seen_vm.decompressions = vm.decompressions;
    stall_seen_vm.page_waits = vm.page_waits;
    const uint32_t lowest = __atomic_exchange_n(&stall_free_min, UINT32_MAX, __ATOMIC_RELAXED);
    io.free_min = lowest < vm_page_free_count ? lowest : vm_page_free_count;
    io.available = memorystatus_available_pages;
    io.used = STALL_LOAD(&stall_limits.used);
    io.over_rate = STALL_LOAD(&stall_limits.over_rate);
    io.over_budget = STALL_LOAD(&stall_limits.over_budget);
    io.kbt_used = STALL_LOAD(&stall_kbt_limits.used);
    io.kbt_over_rate = STALL_LOAD(&stall_kbt_limits.over_rate);
    io.kbt_over_budget = STALL_LOAD(&stall_kbt_limits.over_budget);
    io.tail_used = STALL_LOAD(&stall_tail_limits.used);
    io.tail_over_rate = STALL_LOAD(&stall_tail_limits.over_rate);
    io.tail_over_budget = STALL_LOAD(&stall_tail_limits.over_budget);
    // A second with nothing to show may be the middle of a hitch, and its totals are already taken: there is no later.
    const uint64_t now = frame_clock();
    if (frames || stall_touches) stall_m_until = now + hz * STALL_M_TAIL_S;
    // Before the first frame nothing keeps the sampler awake and nothing marks the seconds that matter: the boot's long waits are all of them.
    if (__atomic_load_n(&presented.count, __ATOMIC_ACQUIRE) && now >= stall_m_until && !__atomic_load_n(&stall_armed, __ATOMIC_RELAXED)) return;
    stall_line_lat(stall_second_line, &stall_this_second, hz);
    stall_meter(stall_second_line);
    stall_line_cpu(stall_second_line, &stall_this_second, timers);
    stall_meter(stall_second_line);
    stall_line_ipi(stall_second_line, &stall_this_second, &ipi, hz);
    stall_meter(stall_second_line);
    stall_line_io(stall_second_line, &io, hz);
    stall_meter(stall_second_line);
    if (!stall_mem_due(now, hz, &stall_mem_next)) return;
    boot_progress_stall_mem_read(&mem.mem);
    mem.ram_bytes = &native_ram_block_resident_bytes ? native_ram_block_resident_bytes : 0;
    mem.pool_mib = native_adreno_pool_stats ? native_adreno_pool_stats[3] : 0;
    mem.pool_peak_mib = native_adreno_pool_stats ? native_adreno_pool_stats[4] : 0;
    stall_line_mem(stall_second_line, &mem);
    stall_meter(stall_second_line);
}

// Thread context, before the observer's first pass. Everything stays off without pagani-diag.
static void stall_configure(void)
{
    uint32_t diag = 0;
    if (!boot_arg("pagani-diag", &diag) || !diag) return;
    if (!__atomic_load_n(&fps_mode, __ATOMIC_ACQUIRE) || !boot_progress_trace_enabled()) {
        boot_progress_stage("stall probe: off, it needs pagani-fps and the ramoops log");
        return;
    }
    if (!boot_progress_stall_abi || boot_progress_stall_abi() != BOOT_PROGRESS_STALL_ABI) {
        boot_progress_stage("stall probe: off, this kernel has no matching phone-stall-probe.patch");
        return;
    }
    stall_state_init(&stall, frequency());
    stall_call = thread_call_allocate_with_options(stall_tick, NULL, THREAD_CALL_PRIORITY_HIGH, THREAD_CALL_OPTIONS_ONCE);
    stall_image_call = thread_call_allocate_with_options(stall_image_lookup, NULL, THREAD_CALL_PRIORITY_LOW, THREAD_CALL_OPTIONS_ONCE);
    if (!stall_call || !stall_image_call) {
        boot_progress_stage("stall probe: off, no thread call");
        return;
    }
    stall_seen_touches = stall_touch_seen = __atomic_load_n(&pagani_input_touch_events, __ATOMIC_RELAXED);
    __atomic_store_n(&stall_mode, 1, __ATOMIC_RELEASE);
    __atomic_store_n(&pagani_stall_on, 1, __ATOMIC_RELEASE);
    boot_progress_stage("stall probe: ramoops M/Q records (pagani-diag)");
}
#endif
