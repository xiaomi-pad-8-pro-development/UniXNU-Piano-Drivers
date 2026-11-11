// SPDX-License-Identifier: MIT
// Copyright (c) 2026, UniXNU contributors.
// The log on UFS (UfsLog.h). Ported from the writer of UniXNU-sheng, package/BootLog.c (MIT; UniXNU-UniXNU-sheng commit 6b11bb5): the window
// layout, the thread's start, the error and GPT stops, the two export halves and the panic flush are its; the slots here hold ramoops
// snapshots instead of composed text.
#include <stdint.h>
#include <stddef.h>
#include "UfsLog.h"
#include "RamoopsLog.h"
#include "../NativeUfs/NativeUfsLog.h"

_Static_assert(sizeof(struct ufs_log_header) == 128, "read-ufslog.py reads this layout");
_Static_assert(offsetof(struct ufs_log_header, crc) == sizeof(struct ufs_log_header) - 4, "the CRC comes last");

// The kernel message buffer (bsd/sys/msgbuf.h); only these leading fields are read.
struct ufs_log_msgbuf {
    int msg_magic, msg_size, msg_bufx, msg_bufr;
    char *msg_bufc;
};
extern struct ufs_log_msgbuf *msgbufp;

// The NativeUfs symbols stay weak, like native_ufs_block_reads in BootProgress.c: a kernel without NativeUfs just has no log.
int native_ufs_log_reset(void);
#pragma weak native_ufs_log_reset
#pragma weak native_ufs_log_open
#pragma weak native_ufs_log_write
#pragma weak native_ufs_export_blocks
#pragma weak native_ufs_export_write
#pragma weak native_ufs_log_sync
#pragma weak native_ufs_log_close
#pragma weak native_ufs_store_gpt_check
#pragma weak native_ufs_log_panic
#pragma weak native_ufs_log_panic_write
#pragma weak native_ufs_log_panic_sync

#ifdef UFS_LOG_TEST
// Test/ufslog.c supplies the platform: boot-args, the counter, memory, the export lock, the watchdog's answers and the thread (which it
// drives itself through ufs_log_writer_open and ufs_log_writer_tick). NativeUfs is a model of it there.
int ufs_log_platform_arg(const char *name, uint32_t *value);
uint64_t ufs_log_platform_counter(void);
uint64_t ufs_log_platform_frequency(void);
void *ufs_log_platform_alloc(uint32_t bytes);
void *ufs_log_platform_lock_alloc(void);
void ufs_log_platform_lock(void *lock);
void ufs_log_platform_unlock(void *lock);
int ufs_log_platform_start(void);
int ufs_log_platform_wdog_armed(void);
void ufs_log_platform_stop_pets(void);
void ufs_log_platform_pause(void);
int ufs_log_platform_power_held(void);
#else
#include "GunyahWdog.h"
#include <pexpert/pexpert.h>
#include <kern/clock.h>
// The scheduler headers drag in task, IPC and kalloc internals whose inline code warns under this file's -Weverything recipe (GunyahWdog.c).
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-warning-option"
#pragma clang diagnostic ignored "-Wcast-function-type-strict"
#pragma clang diagnostic ignored "-Wallocator-wrappers"
#pragma clang diagnostic ignored "-Wenum-enum-conversion"
#include <kern/locks.h>
#include <kern/sched.h>
#include <kern/thread.h>
#pragma clang diagnostic pop
// IOKit/IOLib.h and IOLocks.h do not build in this file's osfmk recipe; these are their exported functions.
extern void *IOMallocAligned_external(vm_size_t size, vm_size_t alignment);
extern lck_mtx_t *IOLockAlloc(void);
extern void IOSleep(unsigned milliseconds);

static int ufs_log_platform_arg(const char *name, uint32_t *value)
{
    return PE_parse_boot_argn(name, value, (int)sizeof(*value)) ? 1 : 0;
}

static uint64_t ufs_log_platform_counter(void)
{
    uint64_t value;
    __asm__ volatile("isb\n\tmrs %0, cntvct_el0" : "=r"(value) :: "memory");
    return value;
}

static uint64_t ufs_log_platform_frequency(void)
{
    uint64_t value;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(value));
    return value;
}

static void *ufs_log_platform_alloc(uint32_t bytes) { return IOMallocAligned_external(bytes, UFS_LOG_BLOCK); }
static void *ufs_log_platform_lock_alloc(void) { return IOLockAlloc(); }
static void ufs_log_platform_lock(void *lock) { lck_mtx_lock(lock); }
static void ufs_log_platform_unlock(void *lock) { lck_mtx_unlock(lock); }
static int ufs_log_platform_start(void);
static int ufs_log_platform_wdog_armed(void) { return gunyah_wdog_armed(); }
static void ufs_log_platform_stop_pets(void) { gunyah_wdog_stop_pets(); }
static void ufs_log_platform_pause(void) { __asm__ volatile("yield"); }
// PaganiInput's debounced Power key. Weak zero like its other words: a kernel without that driver reads it released.
extern volatile uint32_t pagani_input_power_held, pagani_panel_probe_quiet;
__attribute__((weak)) volatile uint32_t pagani_input_power_held;
__attribute__((weak)) volatile uint32_t pagani_panel_probe_quiet;
static int ufs_log_platform_power_held(void)
{
    return __atomic_load_n(&pagani_input_power_held, __ATOMIC_RELAXED) != 0 || __atomic_load_n(&pagani_panel_probe_quiet, __ATOMIC_ACQUIRE) != 0;
}
#endif

enum {
    MSG_BYTES = UFS_LOG_MSG_BLOCKS * UFS_LOG_BLOCK,
    // The writer's memory, allocated once the window is open: two snapshots (the one going out and what the slot holds), the message
    // buffer, a header block, and the panic path's own snapshot and header block, which it must not share with a writer it interrupted.
    BUFFER_BYTES = 3 * UFS_LOG_SLOT_BYTES + MSG_BYTES + 2 * UFS_LOG_BLOCK,
};

struct writer {
    uint64_t first, blocks, start, hz, next_round;
    uint32_t slot_count, period_ms, advance, burst, panic_on;
    int gpt_noted;
    uint8_t *fresh, *shown, *msg, *block, *panic_slot, *panic_block;
    uint32_t current;                       // the logical slot of the last snapshot
    int have_slot, shown_valid;             // a slot holds a snapshot; `shown` is what the current slot holds
    uint64_t base_events, base_console;     // the rings' totals where the current slot has to reach back to
    uint64_t last_events, last_console;     // the rings' totals of the last snapshot
    uint32_t last_sequence;
    int msg_position;                       // msg_bufx at the last message buffer write, -1: none yet
    uint32_t msg_bytes;
    uint32_t errors, write_errors, rounds;
    uint32_t max_round_us, stat_round_us;   // a round, first command to last flush: ever, and in the rounds since the last statistics record
    uint32_t max_command_us, stat_command_us;   // one call to NativeUfs, its wait for the transport included
    uint32_t export_seen;                   // the export generation the medium's header names
    int held;                               // the Power key was down at the last pass: its round has gone
    uint32_t presses;                       // how many times it went down
};
static struct writer w;
static struct ufs_log_header headers[2];
// The newest header on the medium, never the one a round is filling. The panic path reads it from any context.
static struct ufs_log_header *volatile live;
static int writer_stopped;                  // set under export_lock, so no export is on its way to NativeUfs when the window closes
// A resetter came through ufs_log_quiesce: nothing starts any more, also when it did not get the transport and the reset goes on regardless.
static int reset_coming;
// The way out (UfsLog.h): one CPU at a time, and for good once it has the transport.
enum { LAST_IDLE, LAST_BUSY, LAST_DONE };
static int last_state;

// The export: boot_log_export_put takes the blob's pieces in order and writes them through a staging buffer straight into the half of
// the export region that does not hold the last complete blob; once the last piece is written that blob is the complete one, and the
// writer's next round flushes it and names it. A blob starts only once the one before it is named: then the half it goes to is not the
// one the medium's header names, so a reset never leaves that header pointing at a blob half overwritten. The lock serialises pieces and
// guards the counters.
static void *export_lock;
static uint8_t *export_stage;
static uint32_t export_region;              // blocks of the export region, 0 = none
static uint32_t export_total, export_filled, export_generation, export_target, export_ready, export_ready_bytes, export_ready_first;
static uint32_t export_named;               // the generation the header on the medium names

static const uint32_t ufs_crc_nibble[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
};

static uint32_t ufs_crc(const uint8_t *bytes, uint32_t n)
{
    uint32_t crc = 0xffffffffu;
    for (uint32_t i = 0; i < n; i++) {
        crc ^= bytes[i];
        crc = (crc >> 4) ^ ufs_crc_nibble[crc & 15];
        crc = (crc >> 4) ^ ufs_crc_nibble[crc & 15];
    }
    return ~crc;
}

// This file builds -ffreestanding and without runtime helpers (build-boot-progress.py); memcpy's cousins here are plain loops.
static void ufs_copy(void *to, const void *from, uint32_t n)
{
    uint8_t *t = to;
    const uint8_t *f = from;
    for (uint32_t i = 0; i < n; i++) t[i] = f[i];
}

static void ufs_zero(void *to, uint32_t n)
{
    uint8_t *t = to;
    for (uint32_t i = 0; i < n; i++) t[i] = 0;
}

static int ufs_same_magic(const void *a, const char *b)
{
    const uint8_t *bytes = a;
    for (uint32_t i = 0; i < 8; i++) if (bytes[i] != (uint8_t)b[i]) return 0;
    return 1;
}

static int ufs_same_text(const char *a, const char *b)
{
    while (*a && *a == *b) { a++; b++; }
    return *a == *b;
}

// Whole blocks, 4096-byte aligned.
static int ufs_same_block(const uint8_t *a, const uint8_t *b)
{
    const uint32_t *x = (const uint32_t *)(uintptr_t)a, *y = (const uint32_t *)(uintptr_t)b;
    for (uint32_t i = 0; i < UFS_LOG_BLOCK / 4; i++) if (x[i] != y[i]) return 0;
    return 1;
}

struct ufs_line {
    char text[200];
    uint32_t used;
};

static void ufs_put_str(struct ufs_line *l, const char *s)
{
    while (s && *s && l->used + 1 < sizeof(l->text)) l->text[l->used++] = *s++;
    l->text[l->used] = 0;
}

static void ufs_put_dec(struct ufs_line *l, uint64_t value)
{
    char digits[20];
    uint32_t n = 0;
    do { digits[n++] = (char)('0' + value % 10); value /= 10; } while (value);
    while (n && l->used + 1 < sizeof(l->text)) l->text[l->used++] = digits[--n];
    l->text[l->used] = 0;
}

static uint32_t elapsed_us(uint64_t since)
{
    const uint64_t ticks = ufs_log_platform_counter() - since, us = w.hz ? ticks * 1000000 / w.hz : 0;
    return us > UINT32_MAX ? UINT32_MAX : (uint32_t)us;
}

// The longest of `took` so far, and of the rounds since the last statistics record.
static void note_longest(uint32_t took, uint32_t *ever, uint32_t *lately)
{
    if (took > *ever) *ever = took;
    if (took > *lately) *lately = took;
}

static int timed_write(uint64_t rel, uint32_t blocks, const void *src)
{
    const uint64_t began = ufs_log_platform_counter();
    const int ok = native_ufs_log_write(rel, blocks, src);
    note_longest(elapsed_us(began), &w.max_command_us, &w.stat_command_us);
    return ok;
}

static int timed_sync(void)
{
    const uint64_t began = ufs_log_platform_counter();
    const int ok = native_ufs_log_sync();
    note_longest(elapsed_us(began), &w.max_command_us, &w.stat_command_us);
    return ok;
}

static uint32_t slot_position(uint32_t logical)
{
    return logical < UFS_LOG_PINNED ? logical : UFS_LOG_PINNED + (logical - UFS_LOG_PINNED) % (w.slot_count - UFS_LOG_PINNED);
}

// The message buffer, oldest byte first, NUL-padded to its blocks; returns the bytes copied. Of a buffer larger than the blocks, the newest.
static uint32_t copy_msgbuf(uint8_t *out)
{
    ufs_zero(out, MSG_BYTES);
    const struct ufs_log_msgbuf *m = msgbufp;
    if (!m || !m->msg_bufc || m->msg_size <= 0) return 0;
    const uint32_t size = (uint32_t)m->msg_size;
    uint32_t x = (uint32_t)m->msg_bufx, n = 0;
    if (x >= size) x = 0;
    for (uint32_t i = size > MSG_BYTES ? size - MSG_BYTES : 0; i < size; i++) {
        const uint32_t at = x + i;
        const char c = m->msg_bufc[at < size ? at : at - size];
        if (c) out[n++] = (uint8_t)c;
    }
    return n;
}

// A header block: the rest of the block zero, the CRC over the fields.
static void header_block(uint8_t *block, struct ufs_log_header *h)
{
    h->crc = ufs_crc((const uint8_t *)h, (uint32_t)offsetof(struct ufs_log_header, crc));
    ufs_zero(block, UFS_LOG_BLOCK);
    ufs_copy(block, h, sizeof(*h));
}

// A permanent stop: the window closes, so nothing (the export, a bug) can write after it, and the panic path stays away. The medium keeps
// the last header that went through. State 3 and 4 are for the log, the only place they can go.
static void writer_stop(uint32_t state, const char *why, uint64_t value)
{
    struct ufs_line l;
    l.used = 0;
    ufs_put_str(&l, "ufslog stopped for good (state ");
    ufs_put_dec(&l, state);
    ufs_put_str(&l, "): ");
    ufs_put_str(&l, why);
    ufs_put_str(&l, " ");
    ufs_put_dec(&l, value);
    ramoops_log_line(RAMOOPS_KIND_RESET, l.text);
    if (export_lock) ufs_log_platform_lock(export_lock);
    __atomic_store_n(&writer_stopped, 1, __ATOMIC_RELEASE);
    if (export_lock) ufs_log_platform_unlock(export_lock);
    native_ufs_log_close();
}

// Returns 1 when the window is open and the first header is on the medium, 0 to try again, -1 when it was refused for good.
int ufs_log_writer_open(void);
int ufs_log_writer_open(void)
{
    uint64_t first = 0, blocks = 0;
    const char *why = "";
    struct ufs_line l;
    if (!native_ufs_log_open(&first, &blocks, &why)) {
        if (ufs_same_text(why, "no root layout")) return 0;
        /* Bring-up: the storage geometry appears only once the root windows are published, so the
         * first attempt can be refused for another reason. Retry for up to two minutes before
         * stopping the writer for good. */
        {
            static unsigned refusal_retries;
            if (++refusal_retries < 120) {
                native_ufs_log_reset();
                return 0;
            }
        }
        l.used = 0;
        ufs_put_str(&l, "ufslog refused: ");
        ufs_put_str(&l, why);
        ramoops_log_line(RAMOOPS_KIND_RESET, l.text);
        return -1;
    }
    const uint32_t count = blocks > UFS_LOG_SLOT_FIRST ? (uint32_t)((blocks - UFS_LOG_SLOT_FIRST) / UFS_LOG_SLOT_BLOCKS) : 0;
    uint8_t *buffer = count > UFS_LOG_PINNED ? ufs_log_platform_alloc(BUFFER_BYTES) : NULL;
    void *lock = buffer ? ufs_log_platform_lock_alloc() : NULL;
    if (!lock) {
        writer_stop(UFS_LOG_STATE_ERRORS, "no memory or too small a window: slots", count);
        return -1;
    }
    w.first = first;
    w.blocks = blocks;
    w.slot_count = count;
    w.fresh = buffer;
    w.shown = buffer + UFS_LOG_SLOT_BYTES;
    w.panic_slot = buffer + 2 * UFS_LOG_SLOT_BYTES;
    w.msg = buffer + 3 * UFS_LOG_SLOT_BYTES;
    w.block = w.msg + MSG_BYTES;
    w.panic_block = w.block + UFS_LOG_BLOCK;
    w.msg_position = -1;
    w.hz = ufs_log_platform_frequency();
    if (!w.hz) w.hz = 19200000;
    w.start = ufs_log_platform_counter();
    w.next_round = w.start;
    uint32_t value = 0;
    w.period_ms = ufs_log_platform_arg("pagani-ufslog-ms", &value) && value ? value : UFS_LOG_PERIOD_MS;
    if (w.period_ms < UFS_LOG_PERIOD_MIN_MS) w.period_ms = UFS_LOG_PERIOD_MIN_MS;
    // The last snapshot runs with interrupts masked, on a bus that may be why the phone is resetting: it stays off by default when a CPU
    // stuck in it would leave nothing to reset the phone.
    value = ufs_log_platform_wdog_armed() || ramoops_log_reset_in_ms();
    (void)ufs_log_platform_arg("pagani-ufslog-panic", &value);
    w.panic_on = value != 0;
    struct ramoops_snapshot s;
    const unsigned copy = ramoops_log_snapshot(NULL, 0, &s);
    const uint32_t smallest = s.events_bytes < s.console_bytes ? s.events_bytes : s.console_bytes;
    w.advance = copy && smallest - smallest / 8 < UFS_LOG_ADVANCE ? smallest - smallest / 8 : UFS_LOG_ADVANCE;
    w.burst = w.advance / 2;
    export_region = (uint32_t)native_ufs_export_blocks();
    __atomic_store_n(&export_lock, lock, __ATOMIC_RELEASE);     // the export's way in: it finds all of the above
    const uint32_t gpt = native_ufs_store_gpt_check();
    if (gpt) {
        writer_stop(UFS_LOG_STATE_GPT, "the GPT differs from the root start: check", gpt);
        return -1;
    }
    struct ufs_log_header *h = &headers[0];
    ufs_zero(h, sizeof(*h));
    ufs_copy(h->magic, UFS_LOG_MAGIC, sizeof(h->magic));
    h->version = UFS_LOG_VERSION;
    h->header_bytes = sizeof(*h);
    h->boot_id = w.start;
    h->first_lba = first;
    h->ramoops_run = s.run;
    h->window_blocks = (uint32_t)blocks;
    h->msg_first = UFS_LOG_MSG_FIRST;
    h->msg_blocks = UFS_LOG_MSG_BLOCKS;
    h->slot_first = UFS_LOG_SLOT_FIRST;
    h->slot_blocks = UFS_LOG_SLOT_BLOCKS;
    h->slot_count = count;
    h->slot_pinned = UFS_LOG_PINNED;
    h->state = UFS_LOG_STATE_WRITING;
    h->export_blocks = h->export_below = export_region;
    h->period_ms = w.period_ms;
    // A new run replaces the old header first, so a run that dies before its first round does not pass for the one before it.
    header_block(w.block, h);
    if (!native_ufs_log_write(0, 1, w.block) || !native_ufs_log_sync()) {
        h->write_errors = ++w.write_errors;
        w.errors = 1;
    }
    __atomic_store_n(&live, h, __ATOMIC_RELEASE);
    l.used = 0;
    ufs_put_str(&l, "ufslog open at LBA ");
    ufs_put_dec(&l, first);
    ufs_put_str(&l, ": ");
    ufs_put_dec(&l, count);
    ufs_put_str(&l, " slots, export ");
    ufs_put_dec(&l, export_region);
    ufs_put_str(&l, " blocks, every ");
    ufs_put_dec(&l, w.period_ms);
    ufs_put_str(&l, w.panic_on ? " ms" : " ms, no last snapshot");
    ramoops_log_line(RAMOOPS_KIND_RESET, l.text);
    return 1;
}

// The snapshot into a slot: whole when `shown` is NULL, else the blocks that differ from it. A run of changed blocks is one command.
static int write_slot(uint32_t position, const uint8_t *fresh, const uint8_t *shown)
{
    const uint64_t base = (uint64_t)UFS_LOG_SLOT_FIRST + (uint64_t)position * UFS_LOG_SLOT_BLOCKS;
    for (uint32_t b = 0; b < UFS_LOG_SLOT_BLOCKS;) {
        if (shown && ufs_same_block(fresh + b * UFS_LOG_BLOCK, shown + b * UFS_LOG_BLOCK)) {
            b++;
            continue;
        }
        uint32_t n = 1;
        while (b + n < UFS_LOG_SLOT_BLOCKS &&
               !(shown && ufs_same_block(fresh + (b + n) * UFS_LOG_BLOCK, shown + (b + n) * UFS_LOG_BLOCK))) n++;
        if (!timed_write(base + b, n, fresh + b * UFS_LOG_BLOCK)) return 0;
        b += n;
    }
    return 1;
}

// The statistics record, every UFS_LOG_CHECK_ROUNDS rounds: the writer's cost to the root volume's reads, which wait for the one transport.
static void writer_stats(const struct ufs_log_header *h)
{
    struct ufs_line l;
    l.used = 0;
    ufs_put_str(&l, "ufslog rounds ");
    ufs_put_dec(&l, h->rounds);
    ufs_put_str(&l, " slots ");
    ufs_put_dec(&l, h->slots_written);
    ufs_put_str(&l, " errors ");
    ufs_put_dec(&l, h->write_errors);
    ufs_put_str(&l, " longest round ");
    ufs_put_dec(&l, w.stat_round_us);
    ufs_put_str(&l, " us (of the last ");
    ufs_put_dec(&l, UFS_LOG_CHECK_ROUNDS);
    ufs_put_str(&l, "), ever ");
    ufs_put_dec(&l, w.max_round_us);
    ufs_put_str(&l, " us; longest command ");
    ufs_put_dec(&l, w.stat_command_us);
    ufs_put_str(&l, " us, ever ");
    ufs_put_dec(&l, w.max_command_us);
    ufs_put_str(&l, " us; key presses ");
    ufs_put_dec(&l, w.presses);
    ramoops_log_line(RAMOOPS_KIND_TRACE, l.text);
    w.stat_round_us = w.stat_command_us = 0;
}

// One round; 0 when the writer stopped for good.
static int writer_round(uint64_t now)
{
    // No new round on the way out: it would fill the header that last_snapshot reads.
    if (__atomic_load_n(&last_state, __ATOMIC_ACQUIRE) != LAST_IDLE || __atomic_load_n(&reset_coming, __ATOMIC_ACQUIRE)) return 1;
    const struct ufs_log_header *current = __atomic_load_n(&live, __ATOMIC_ACQUIRE);
    struct ufs_log_header *h = current == &headers[0] ? &headers[1] : &headers[0];
    ufs_copy(h, current, sizeof(*h));
    const uint32_t round = h->rounds + 1;
    // Every UFS_LOG_CHECK_ROUNDS rounds, before this round writes: any change of the GPT, or a failure to read it, ends the log.
    if (!(round % UFS_LOG_CHECK_ROUNDS)) {
        h->gpt_last = native_ufs_store_gpt_check();
        if (h->gpt_last != h->gpt_open) {
            // The window is inside a partition this port rewrote, so a difference here is expected on a
            // bring-up image and must not end the log: the round still only writes inside the window it
            // opened. Recorded once in the header and in the ring, then the writer carries on.
            if (!w.gpt_noted) {
                struct ufs_line line;
                w.gpt_noted = 1;
                line.used = 0;
                ufs_put_str(&line, "ufslog GPT check differs (open ");
                ufs_put_dec(&line, h->gpt_open);
                ufs_put_str(&line, ", now ");
                ufs_put_dec(&line, h->gpt_last);
                ufs_put_str(&line, "): continuing");
                ramoops_log_line(RAMOOPS_KIND_TRACE, line.text);
            }
        }
    }
    struct ramoops_snapshot s;
    const unsigned copy = ramoops_log_snapshot(w.fresh, UFS_LOG_SLOT_BYTES, &s);
    if (copy < UFS_LOG_SLOT_BYTES) ufs_zero(w.fresh + copy, UFS_LOG_SLOT_BYTES - copy);
    const uint64_t began = ufs_log_platform_counter();
    int ok = 1;
    uint32_t target = w.current;
    int next = !w.have_slot;
    if (copy) {
        // The current slot holds everything since base. A snapshot past that goes to the next slot, which keeps the current one as it is.
        if (w.have_slot && (s.events_total - w.base_events > w.advance || s.console_total - w.base_console > w.advance)) {
            target = w.current + 1;
            next = 1;
        }
        ok = write_slot(slot_position(target), w.fresh, next || !w.shown_valid ? NULL : w.shown);
    }
    const int position = msgbufp ? msgbufp->msg_bufx : 0;
    if (ok && position != w.msg_position) {
        w.msg_bytes = copy_msgbuf(w.msg);
        ok = timed_write(UFS_LOG_MSG_FIRST, UFS_LOG_MSG_BLOCKS, w.msg);
    }
    // What the header will name is written already. A new blob goes to the medium before the header that names it (UfsLog.h).
    ufs_log_platform_lock(export_lock);
    h->export_generation = export_ready;
    h->export_bytes = export_ready_bytes;
    h->export_first = export_ready_first;
    ufs_log_platform_unlock(export_lock);
    if (ok && h->export_generation != w.export_seen) ok = timed_sync();
    if (ok) {
        h->msg_bytes = w.msg_bytes;
        h->slots_written = copy ? target + 1 : h->slots_written;
        h->uptime_s = (uint32_t)((now - w.start) / w.hz);
        h->rounds = round;
        h->write_errors = w.write_errors;
        h->ramoops_run = s.run;
        h->max_round_us = w.max_round_us;   // of the rounds before this one: this one ends with this header
        header_block(w.block, h);
        ok = timed_write(0, 1, w.block) && timed_sync();
    }
    note_longest(elapsed_us(began), &w.max_round_us, &w.stat_round_us);
    if (!ok) {
        // A round that failed changed nothing the next one relies on: it rewrites the slot in whole.
        w.shown_valid = 0;
        h->write_errors = ++w.write_errors;
        if (++w.errors >= UFS_LOG_MAX_ERRORS) {
            writer_stop(UFS_LOG_STATE_ERRORS, "writes failed in rounds in a row:", w.errors);
            return 0;
        }
        return 1;
    }
    w.errors = 0;
    if (copy) {
        if (next) {
            w.base_events = w.have_slot ? w.last_events : 0;
            w.base_console = w.have_slot ? w.last_console : 0;
        }
        w.current = target;
        w.have_slot = 1;
        w.last_events = s.events_total;
        w.last_console = s.console_total;
        w.last_sequence = s.sequence;
        uint8_t *swap = w.fresh;
        w.fresh = w.shown;
        w.shown = swap;
        w.shown_valid = 1;
    }
    w.msg_position = position;
    if (h->export_generation != w.export_seen) {
        struct ufs_line l;
        w.export_seen = h->export_generation;
        l.used = 0;
        ufs_put_str(&l, "ufslog export generation ");
        ufs_put_dec(&l, h->export_generation);
        ufs_put_str(&l, ": ");
        ufs_put_dec(&l, h->export_bytes);
        ufs_put_str(&l, " bytes at block ");
        ufs_put_dec(&l, h->export_first);
        ramoops_log_line(RAMOOPS_KIND_TRACE, l.text);
    }
    ufs_log_platform_lock(export_lock);
    export_named = h->export_generation;
    ufs_log_platform_unlock(export_lock);
    __atomic_store_n(&live, h, __ATOMIC_RELEASE);
    if (!(round % UFS_LOG_CHECK_ROUNDS)) writer_stats(h);
    return 1;
}

// One pass of the thread: a round when one is due and something is new. Returns the milliseconds to the next pass, 0 when it stopped.
unsigned ufs_log_writer_tick(void);
unsigned ufs_log_writer_tick(void)
{
    const uint64_t now = ufs_log_platform_counter();
    struct ramoops_snapshot s;
    (void)ramoops_log_snapshot(NULL, 0, &s);
    const uint64_t reset_ms = ramoops_log_reset_in_ms();
    const int last_seconds = reset_ms && reset_ms <= UFS_LOG_FINAL_MS;
    const uint64_t events = s.events_total - w.last_events, console = s.console_total - w.last_console;
    const int position = msgbufp ? msgbufp->msg_bufx : 0;
    ufs_log_platform_lock(export_lock);
    const uint32_t ready = export_ready;
    ufs_log_platform_unlock(export_lock);
    const int fresh = s.sequence != w.last_sequence || position != w.msg_position || ready != w.export_seen;
    // The Power key: one round now for what led to the press, then nothing until it comes up (UfsLog.h).
    const int held = ufs_log_platform_power_held();
    const int due = held ? !w.held : now >= w.next_round || last_seconds || events >= w.burst || console >= w.burst;
    if (held && !w.held) w.presses++;
    if (fresh && due) {
        w.next_round = now + w.hz * w.period_ms / 1000;
        if (!writer_round(now)) return 0;
    }
    w.held = held;
    if (now - w.start < w.hz * UFS_LOG_EARLY_MS / 1000) return UFS_LOG_EARLY_TICK_MS;
    return last_seconds ? UFS_LOG_FINAL_TICK_MS : UFS_LOG_TICK_MS;
}

#ifndef UFS_LOG_TEST
static void writer_thread(void *argument, wait_result_t result)
{
    (void)argument; (void)result;
    int opened;
    // The window exists once NativeUfs has read the root layout; until then the open is refused ("no root layout").
    while (!(opened = ufs_log_writer_open())) IOSleep(1000);
    if (opened > 0) {
        unsigned ms;
        while ((ms = ufs_log_writer_tick())) IOSleep(ms);
    }
    thread_terminate(current_thread());
}

static int ufs_log_platform_start(void)
{
    thread_t thread = THREAD_NULL;
    // An ordinary priority: kernel_thread_start's default is the kernel task's, above every user band, and a round spins on the transport.
    if (kernel_thread_start_priority(writer_thread, NULL, BASEPRI_DEFAULT, &thread) != KERN_SUCCESS) return 0;
    thread_set_thread_name(thread, "pagani-ufslog");
    thread_deallocate(thread);
    return 1;
}
#endif

static int writer_started, panic_flushed;

void ufs_log_start(void)
{
    uint32_t mode = 0;
    if (writer_started || !ufs_log_platform_arg("sheng-log", &mode) || mode != 1) return;
    writer_started = 1;
    if (!native_ufs_log_open || !native_ufs_log_write || !native_ufs_log_sync || !native_ufs_log_close || !native_ufs_export_blocks ||
        !native_ufs_store_gpt_check) {
        ramoops_log_line(RAMOOPS_KIND_RESET, "ufslog: sheng-log=1 but this kernel has no NativeUfs log window");
        return;
    }
    if (!ufs_log_platform_start()) ramoops_log_line(RAMOOPS_KIND_RESET, "ufslog: the writer thread did not start");
}

int boot_log_export_put(uint32_t generation, uint32_t total, uint32_t offset, const void *data, uint32_t size)
{
    if (!__atomic_load_n(&export_lock, __ATOMIC_ACQUIRE) || !generation || !total || !data || !size || offset > total || size > total - offset) return -1;
    ufs_log_platform_lock(export_lock);
    int status = 0;
    const uint32_t half = export_region / 2, stage = UFS_LOG_EXPORT_STAGE;
    if (!half || __atomic_load_n(&writer_stopped, __ATOMIC_ACQUIRE) || !native_ufs_export_write) status = -4;
    else if (total > (uint64_t)(half - 1) * UFS_LOG_BLOCK) status = -5;     // the last block of a half stays blank (the content gate)
    else if (!offset && (size < 8 || !ufs_same_magic(data, UFS_LOG_EXPORT_MAGIC))) status = -7;     // the content gate would close the log
    else if (!offset && export_ready != export_named) status = -8;          // this blob would go over the one the medium's header names
    if (!status && !export_stage) export_stage = ufs_log_platform_alloc(stage);
    if (!status && !export_stage) status = -2;
    if (!status && !offset) {
        export_generation = generation;
        export_total = total;
        export_filled = 0;
        export_target = export_ready && !export_ready_first ? half : 0;
    }
    if (!status && (generation != export_generation || total != export_total || offset != export_filled)) status = -3;   // only in order
    const uint8_t *from = data;
    while (!status && size) {
        const uint32_t at = export_filled % stage, n = size < stage - at ? size : stage - at;
        ufs_copy(export_stage + at, from, n);
        export_filled += n;
        from += n;
        size -= n;
        const int done = export_filled == export_total;
        if (at + n == stage || done) {
            const uint32_t used = at + n, blocks = (used + UFS_LOG_BLOCK - 1) / UFS_LOG_BLOCK;
            if (used % UFS_LOG_BLOCK) ufs_zero(export_stage + used, blocks * UFS_LOG_BLOCK - used);
            const int held = ufs_log_platform_power_held() || __atomic_load_n(&reset_coming, __ATOMIC_ACQUIRE);
            if (held || !native_ufs_export_write(export_target + (export_filled - used) / UFS_LOG_BLOCK, blocks, export_stage)) {
                status = held ? -9 : -6;
                export_generation = 0;              // the next generation starts over
            }
        }
        if (!status && done) {
            export_ready = export_generation;
            export_ready_bytes = export_total;
            export_ready_first = export_target;
        }
    }
    ufs_log_platform_unlock(export_lock);
    return status;
}

// What is left of the budget, in microseconds, and at most `step`.
static uint32_t panic_step(uint64_t started, uint32_t budget_us, uint32_t step)
{
    const uint32_t spent = elapsed_us(started), left = spent < budget_us ? budget_us - spent : 0;
    return left < step ? left : step;
}

// With the transport in hand: the next slot, the header, a flush. `live` is read only now: a round that was under way has had all its
// commands, and none starts any more. If that round has yet to publish its header this builds on the one before, which loses nothing: the
// same slot again, with a newer snapshot, and an export it no longer names is still whole, as nothing writes after the claim. The header
// names no blob that is not flushed, so one flush at the end is enough here too.
static void last_snapshot(uint64_t started, uint32_t budget, uint32_t state, const char *reason)
{
    const struct ufs_log_header *current = __atomic_load_n(&live, __ATOMIC_ACQUIRE);
    uint32_t timeout;
    if (reason) {
        struct ufs_line l;
        l.used = 0;
        ufs_put_str(&l, "ufslog last snapshot before the reset: ");
        ufs_put_str(&l, reason);
        ramoops_log_line(RAMOOPS_KIND_RESET, l.text);
    }
    struct ramoops_snapshot s;
    const unsigned copy = ramoops_log_snapshot(w.panic_slot, UFS_LOG_SLOT_BYTES, &s);
    if (copy) {
        if (copy < UFS_LOG_SLOT_BYTES) ufs_zero(w.panic_slot + copy, UFS_LOG_SLOT_BYTES - copy);
        struct ufs_log_header h;
        ufs_copy(&h, current, sizeof(h));
        const uint64_t base = (uint64_t)UFS_LOG_SLOT_FIRST + (uint64_t)slot_position(h.slots_written) * UFS_LOG_SLOT_BLOCKS;
        timeout = panic_step(started, budget, UFS_LOG_PANIC_WRITE_US);
        if (!timeout || !native_ufs_log_panic_write(base, UFS_LOG_SLOT_BLOCKS, w.panic_slot, timeout)) return;
        h.slots_written++;
        h.rounds++;
        h.state = state;
        h.ramoops_run = s.run;
        h.uptime_s = (uint32_t)((started - w.start) / w.hz);
        header_block(w.panic_block, &h);
        timeout = panic_step(started, budget, UFS_LOG_PANIC_WRITE_US);
        if (!timeout || !native_ufs_log_panic_write(0, 1, w.panic_block, timeout)) return;
    }
    // Also with nothing to add: the claim may have come between a round's writes and its flush.
    timeout = panic_step(started, budget, UFS_LOG_PANIC_SYNC_US);
    if (timeout) (void)native_ufs_log_panic_sync(timeout);
}

// Claims the transport and, with a `state`, takes the last snapshot. 0 when another CPU is at it. A claim that fails leaves it to the next
// caller: a panic's writer may be done by the time the reset comes.
static int last_flush(uint64_t started, uint32_t budget, uint32_t claim_us, uint32_t state, const char *reason)
{
    int expected = LAST_IDLE;
    if (!__atomic_compare_exchange_n(&last_state, &expected, LAST_BUSY, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) return 0;
    if (!claim_us || !native_ufs_log_panic(claim_us)) {
        __atomic_store_n(&last_state, LAST_IDLE, __ATOMIC_RELEASE);
        return 1;
    }
    if (state) last_snapshot(started, budget, state, reason);
    __atomic_store_n(&last_state, LAST_DONE, __ATOMIC_RELEASE);
    return 1;
}

static int last_snapshot_on(void)
{
    return __atomic_load_n(&live, __ATOMIC_ACQUIRE) && w.panic_on && native_ufs_log_panic_write && native_ufs_log_panic_sync;
}

int ufs_log_panic_pending(void)
{
    return !__atomic_load_n(&panic_flushed, __ATOMIC_ACQUIRE) && !__atomic_load_n(&writer_stopped, __ATOMIC_ACQUIRE) && native_ufs_log_panic &&
           last_snapshot_on();
}

void ufs_log_panic(uint64_t limit_ms)
{
    const int pending = ufs_log_panic_pending();
    if (__atomic_exchange_n(&panic_flushed, 1, __ATOMIC_ACQ_REL) || !pending) return;
    const uint64_t started = ufs_log_platform_counter();
    uint32_t budget = UFS_LOG_PANIC_BUDGET_US;
    if (limit_ms && limit_ms < budget / 1000 * 5 / 4) budget = (uint32_t)(limit_ms * 1000 / 5 * 4);
    (void)last_flush(started, budget, panic_step(started, budget, UFS_LOG_PANIC_CLAIM_US), UFS_LOG_STATE_PANIC, NULL);
}

void ufs_log_quiesce(const char *reason)
{
    // Nothing started, or a window closed for good: nothing is on the wire. No export either, as writer_stopped goes up under its lock.
    if (!writer_started || __atomic_load_n(&writer_stopped, __ATOMIC_ACQUIRE) || !native_ufs_log_panic) return;
    __atomic_store_n(&reset_coming, 1, __ATOMIC_RELEASE);
    const int snapshot = last_snapshot_on();
    if (snapshot) ufs_log_platform_stop_pets();
    const uint32_t state = __atomic_load_n(&panic_flushed, __ATOMIC_ACQUIRE) ? UFS_LOG_STATE_PANIC : UFS_LOG_STATE_RESET;
    const uint64_t began = ufs_log_platform_counter(), patience = ufs_log_platform_frequency() * UFS_LOG_QUIESCE_BUDGET_US / 1000000;
    for (;;) {
        const int phase = __atomic_load_n(&last_state, __ATOMIC_ACQUIRE);
        if (phase == LAST_DONE || ufs_log_platform_counter() - began >= patience) return;
        if (phase == LAST_IDLE && last_flush(began, UFS_LOG_QUIESCE_BUDGET_US, UFS_LOG_QUIESCE_CLAIM_US, snapshot ? state : 0, reason)) return;
        ufs_log_platform_pause();
    }
}
