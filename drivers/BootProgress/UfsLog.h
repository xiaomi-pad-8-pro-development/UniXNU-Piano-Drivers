// SPDX-License-Identifier: MIT
// The log on UFS: survives what the ramoops log does not (a reset that reaches the PMIC as HARD_RESET: DRAM loses bits, Linux drops both
// zones). sheng-log=1 starts a writer thread that keeps snapshots of the ramoops log (RamoopsLog.h) and the kernel's message buffer in
// NativeUfs's log window, the last 32 MiB of the unixnu partition, and takes the render server's export (boot_log_export_put) into the
// export region, the 256 MiB below it. Without the boot argument nothing here runs and nothing is allocated. 工具/gpu-test-loop/read-ufslog.py
// reads it back from Android.
//
// Window layout (4096-byte blocks; UFS_LOG_* below): block 0 the header (struct ufs_log_header), blocks 1..32 the message buffer (oldest
// first), from block 33 slots of 16 blocks. A slot holds one whole copy of the ramoops log (0xfffc bytes, the rest zero) as it was at one
// moment: every record in it carries its own CRC, so a reader only merges the slots by sequence number and gets a log longer than any ring.
// The first UFS_LOG_PINNED slots are kept for good (the boot), the others form a ring: logical slot n lies at position n below
// UFS_LOG_PINNED and at pinned + (n - pinned) % (count - pinned) above. Blocks past the last whole slot stay blank, the last one for the
// content gate (NativeUfs LogWindow.h).
//
// A round: the snapshot goes over the current slot (only the blocks that changed; a new slot in whole), the message buffer when it moved,
// the header, flush. No new record, nothing written, nothing flushed. The header goes last, but the device may put it on the medium first:
// that shows a reader older records or another run's, which it tells by the slot's copy header, never wrong ones, as every record carries
// its CRC. Only an export has no check of its own, so a round that names a new one flushes before the header too: a header never names a
// blob that is not whole on the medium. The current slot gives way to the next one when a ring has taken more than UFS_LOG_ADVANCE bytes
// since the last snapshot of the slot before it: a slot then always reaches back to where the slot before it ended, and neighbouring
// slots lose nothing unless a ring turned over between two rounds. So rounds also come early when a ring has taken half of that since the
// last one, and every UFS_LOG_FINAL_TICK_MS in the last UFS_LOG_FINAL_MS before a timed reset. The rings are looked at every UFS_LOG_TICK_MS,
// every UFS_LOG_EARLY_TICK_MS in the first UFS_LOG_EARLY_MS after the writer started, when the boot fills them fastest.
//
// Boot arguments: sheng-log=1 turns it on; pagani-ufslog-ms=<ms> the round period, default 3000, at least 1000; pagani-ufslog-panic=1 turns
// the last snapshot (below) on, 0 off; by default it is on when something can still reset a CPU stuck in it: a watchdog that is armed
// (pagani-wdog) or a timed reset (pagani-reset-after).
//
// It stops for good, closing the window, after three failed rounds in a row or when the GPT changes (checked every 30 rounds); the reason
// goes to the ramoops log. A refused window (NativeUfs's reasons) stops it the same way before anything is written.
//
// The last snapshot, in a panic and before a reset. One CPU, interrupts masked: it claims the transport with a bounded try-lock and keeps
// it, then a snapshot into the next slot, a header in state 5 (panic) or 6 (reset), a flush. Every step has a deadline, and it gives up on
// the first thing that fails. With the transport claimed no command is on the wire, whatever the writer was doing, and none can follow.
// - ufs_log_panic: called by RamoopsLog once the panic's records are in the log; it stays within limit_ms * 4/5 (limit_ms: the time to the
//   panic reset, 0 none) and UFS_LOG_PANIC_BUDGET_US. A watchdog that was armed stays armed through it (GunyahWdog.h).
// - ufs_log_quiesce: called by RamoopsLog first thing in every reset, before the bite, the steps or PSCI: a reset that lands on a write
//   is a power cut in the middle of it, the one way this log could touch anything outside its window (a logical unit without
//   bDataReliability can lose more than the blocks being written). It waits at most UFS_LOG_QUIESCE_CLAIM_US for the transport, then
//   the reset goes on regardless: a command already on the wire finishes by itself if it can. The pets stop first, so a bus that hangs
//   under the snapshot ends in the bite. All of it stays within UFS_LOG_QUIESCE_BUDGET_US, and a second resetter waits that long for the
//   first. With the last snapshot off the transport is still claimed, and nothing goes to the bus. A window closed for good has nothing
//   on the wire: nothing happens. From its first line no round and no piece of an export starts, also when the claim fails.
//
// The Power key is the one reset a test is sure to meet that no code sees coming: held down, with or without Volume Down, it ends in the
// PMIC's reset. So while it is down (PaganiInput's pagani_input_power_held) the wire stays idle: one round goes out for what led to the press,
// then none until the key comes up, and an export's next piece is refused (-9) and starts over. The reset comes seconds after the press.
// PaganiPanel's register probe holds the writer the same way around a read that may hang (PanelProbe.h, pagani_panel_probe_quiet), and so do
// PaganiBattery's USB probe, PaganiBt, PaganiCamera, PaganiSpeaker, PaganiAdsp and PaganiWifi's stretches: a word of bits, one each, held while it is not 0. The statistics
// line's key presses count those holds too.
// What it does not cover: a reset no code sees coming that gives no sign first (a watchdog bite, a firmware reset) can land on a write.
#ifndef PAGANI_UFS_LOG_H
#define PAGANI_UFS_LOG_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
enum {
    UFS_LOG_BLOCK = 4096,
    UFS_LOG_VERSION = 1,
    UFS_LOG_MSG_FIRST = 1,
    UFS_LOG_MSG_BLOCKS = 32,
    UFS_LOG_SLOT_FIRST = 33,
    UFS_LOG_SLOT_BLOCKS = 16,
    UFS_LOG_SLOT_BYTES = UFS_LOG_SLOT_BLOCKS * UFS_LOG_BLOCK,
    UFS_LOG_PINNED = 32,
    UFS_LOG_PERIOD_MS = 3000,
    UFS_LOG_PERIOD_MIN_MS = 1000,
    UFS_LOG_TICK_MS = 500,                  // how often the writer looks at the rings
    UFS_LOG_EARLY_MS = 30000,               // for this long after its start, the boot's busiest stretch, it looks
    UFS_LOG_EARLY_TICK_MS = 100,            // this often: a ring takes UFS_LOG_ADVANCE in well under 500 ms then, and what it turns over is gone
    UFS_LOG_FINAL_MS = 2000,
    UFS_LOG_FINAL_TICK_MS = 250,
    UFS_LOG_ADVANCE = 24 * 1024,            // or 7/8 of the smaller ring (pagani-ramoops-copy): a snapshot must still reach that far back
    // Bring-up: publishing the root windows holds the UFS transport for seconds, so three failed
    // rounds in a row happened during every boot and closed the log before user space started.
    UFS_LOG_MAX_ERRORS = 600,
    UFS_LOG_CHECK_ROUNDS = 30,              // the GPT check and the statistics record
    UFS_LOG_EXPORT_STAGE = 64 * UFS_LOG_BLOCK,
    UFS_LOG_PANIC_BUDGET_US = 1000000,
    UFS_LOG_PANIC_CLAIM_US = 100000,        // of the budget: the wait for the transport
    UFS_LOG_PANIC_WRITE_US = 200000,        // one command, at most
    UFS_LOG_PANIC_SYNC_US = 300000,
    UFS_LOG_QUIESCE_BUDGET_US = 2000000,    // before a reset
    UFS_LOG_QUIESCE_CLAIM_US = 1000000,     // of that budget: a command on the wire gets this long to end
};
// Header states.
enum {
    UFS_LOG_STATE_WRITING = 2,              // 0 off, 1 waiting for the window (neither reaches the medium)
    UFS_LOG_STATE_ERRORS = 3,               // stopped after write errors; the medium keeps the last header that went through
    UFS_LOG_STATE_GPT = 4,                  // stopped: the GPT changed
    UFS_LOG_STATE_PANIC = 5,                // the header of ufs_log_panic
    UFS_LOG_STATE_RESET = 6,                // the header of ufs_log_quiesce
};
#define UFS_LOG_MAGIC "PGLOG001"
#define UFS_LOG_EXPORT_MAGIC "SHEXR001"     // a blob's first bytes: the content gate takes nothing else at the export region's first block

// Block 0, little endian; the rest of the block is zero. read-ufslog.py and the CRC depend on this layout.
struct ufs_log_header {
    char magic[8];                          // UFS_LOG_MAGIC
    uint32_t version, header_bytes;         // sizeof
    uint64_t boot_id;                       // the counter when the writer started: tells one run from the next
    uint64_t first_lba;                     // absolute 4096-byte LBA of block 0 on LUN 0
    uint32_t ramoops_run;                   // the run id of the copies in the slots, 0 with the ramoops log off
    uint32_t window_blocks;
    uint32_t msg_first, msg_blocks, msg_bytes;
    uint32_t slot_first, slot_blocks, slot_count, slot_pinned;
    uint32_t slots_written;                 // logical slots 0 .. slots_written - 1 are on the medium
    uint32_t uptime_s;                      // since the writer started
    uint32_t rounds, write_errors;
    uint32_t gpt_open, gpt_last;            // native_ufs_store_gpt_check at the start and at the last check: 0 unchanged
    uint32_t state;
    // The newest export that is wholly on the medium, at block export_first of the export region, which is export_blocks long and ends
    // export_below blocks below block 0 of this window; export_bytes 0: none yet.
    uint32_t export_first, export_blocks, export_bytes, export_generation, export_below;
    uint32_t period_ms, max_round_us;       // the period in use; the longest round before this header's
    uint32_t crc;                           // CRC32 (zlib) of every byte before it
};

void ufs_log_start(void);
void ufs_log_panic(uint64_t limit_ms);
void ufs_log_quiesce(const char *reason);
// 1 while ufs_log_panic would go to the transport: the watchdog stays armed for it.
int ufs_log_panic_pending(void);
// Hands one in-order piece of a blob to the export region (Adreno830 NativeAdrenoPublish.cpp, a weak no-op until this strong definition):
// 0, or a negative reason. Any thread; it may block. A first piece is refused until the header on the medium names the blob before it (-8:
// the half that header names must stay whole, so try the blob again later) and unless it starts with UFS_LOG_EXPORT_MAGIC (-7).
int boot_log_export_put(uint32_t generation, uint32_t total, uint32_t offset, const void *data, uint32_t size);
#ifdef __cplusplus
}
#endif
#endif
