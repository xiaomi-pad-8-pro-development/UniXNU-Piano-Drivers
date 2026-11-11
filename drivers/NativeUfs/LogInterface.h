// SPDX-License-Identifier: BSD-2-Clause-Patent
// The log window's interface (NativeUfsLog.h) over the command path, included by NativeUfs.cpp after its root layout code; apart from it
// so that tests/LogInterface.cpp runs these functions, and UfsLog.c over them, on the host. Needs gHost and RootCommands.h, LogWindow.h,
// gStoreGeometry, reserveBlocks (), le32, rootCrc32, UFS_ROOT_PARTITION and the kernel's lock, clock and boot argument calls.
#pragma once
// ---- Diagnostic log window (sheng-log=1) --------------------------------------------------------------------------
// The board's log writer (BootProgress UfsLog.c, kits/sheng BootLog.c) gets the last kLogReserveBlocks of the root partition and the
// kExportBlocks below them, refused unless it is the unixnu partition (LogWindow.h), it holds them clear of the flashed root image plus a
// 4 MiB guard, and the blocks at their ends are blank or the log's own. The geometry comes from startRoot: the GPT entry named
// UFS_ROOT_PARTITION, checked against its CRCs, after the root manifest passed. Off without the boot argument.
static uint64_t gLogRegionFirst, gLogRegionBlocks;
static bool gLogClosed;        // the writer stopped for good: the window does not open again
static bool gLogPanicOwned;    // native_ufs_log_panic holds gHost.lock for good

static unsigned logMode (void)
{
  static int cached = -1;
  if (cached < 0) {
    uint32_t value = 0;
    if (!PE_parse_boot_argn ("sheng-log", &value, sizeof (value)) || value > 1) value = 0;
    cached = static_cast<int> (value);
  }
  return static_cast<unsigned> (cached);
}

// Returns 1 with the window's first absolute LBA and size in 4096-byte blocks, 0 on refusal (reason in *why, may be null).
extern "C" int native_ufs_log_open (uint64_t *firstLba, uint64_t *blocks, const char **why)
{
  const char *reason = "";
  int ok = 0;
  if (!logMode ()) reason = "off";
  else if (!gStoreGeometry.valid || !gHost.ready) reason = "no root layout";
  else if (gLogRegionBlocks) reason = "already open";
  else if (gLogClosed) reason = "closed";
  else {
    static uint8_t block[4096];
    uint64_t first = 0;
    // One write window covers the export region and the log: [first - kExportBlocks, first + kLogReserveBlocks).
    const char *refusal = ufsLogOpenWindow (UFS_ROOT_PARTITION, gStoreGeometry.first, gStoreGeometry.last, gStoreGeometry.imageBytes,
      reserveBlocks (), block, [] (uint64_t lba, uint8_t *to) { return hostRead4k (0, lba, 1, to) == kIOReturnSuccess; },
      [] (uint64_t at, uint64_t size) { return hostLogOpen (at, size); }, &first);
    if (refusal) reason = refusal;
    else {
      gLogRegionFirst = first;
      gLogRegionBlocks = kLogReserveBlocks;
      if (firstLba) *firstLba = first;
      if (blocks) *blocks = kLogReserveBlocks;
      ok = 1;
    }
  }
  if (why) *why = reason;
  return ok;
}

// Bring-up: the BootProgress writer can decide "closed" before the storage geometry exists (it
// tries once at start-up). Clearing that state lets a later attempt, made after the root windows
// are published, open the window instead of the whole boot staying without a storage log.
extern "C" void native_ufs_log_reset (void)
{
  gLogClosed = false;
  gLogRegionBlocks = 0;
  gLogRegionFirst = 0;
}

extern "C" int native_ufs_log_write (uint64_t rel, uint32_t blocks, const void *src)
{
  if (!gLogRegionBlocks || !src || !blocks || rel >= gLogRegionBlocks || blocks > gLogRegionBlocks - rel) return 0;
  while (blocks) {
    uint32_t chunk = blocks > kDataMax / 4096 ? static_cast<uint32_t> (kDataMax / 4096) : blocks;
    if (hostWrite4k (0, gLogRegionFirst + rel, chunk, src) != kIOReturnSuccess) return 0;
    src = static_cast<const uint8_t *> (src) + chunk * 4096ULL;
    rel += chunk;
    blocks -= chunk;
  }
  return 1;
}

// The export region below the log window: its size in blocks once the log is open, else 0; writes at a block offset inside it.
extern "C" uint64_t native_ufs_export_blocks (void) { return gLogRegionBlocks ? kExportBlocks : 0; }

extern "C" int native_ufs_export_write (uint64_t rel, uint32_t blocks, const void *src)
{
  if (!gLogRegionBlocks || !src || !blocks || rel >= kExportBlocks || blocks > kExportBlocks - rel) return 0;
  const uint64_t base = gLogRegionFirst - kExportBlocks;
  while (blocks) {
    uint32_t chunk = blocks > kDataMax / 4096 ? static_cast<uint32_t> (kDataMax / 4096) : blocks;
    if (hostWrite4k (0, base + rel, chunk, src) != kIOReturnSuccess) return 0;
    src = static_cast<const uint8_t *> (src) + chunk * 4096ULL;
    rel += chunk;
    blocks -= chunk;
  }
  return 1;
}

extern "C" int native_ufs_log_sync (void)
{
  return gLogRegionBlocks && hostSyncCache () == kIOReturnSuccess;
}

// The writer stopped for good: the window closes, and nothing can reopen it.
extern "C" void native_ufs_log_close (void)
{
  if (!gHost.lock) return;
  IOLockLock (gHost.lock);
  gLogClosed = true;
  gLogRegionBlocks = 0;
  hostLogClose ();
  IOLockUnlock (gHost.lock);
}

// ---- The log's way out: a panic, or a reset ------------------------------------------------------------------------------
// With interrupts masked and the other CPUs still running (boot_progress_panic comes first in panic_trap_to_debugger; a reset comes from any
// context). hostSubmit never sleeps: on SM8750 its pacing is a counter loop, its deadline the same counter, and neither the UTP slot nor
// the cache maintenance needs interrupts. Only gHost.lock can block, so native_ufs_log_panic takes it with a try-lock for at most waitUs
// and then keeps it: a reset is next, and an unlock here could run the mutex's wakeup path from a panic. Every command runs to its end under
// that lock, so with it in hand nothing is on the wire, and a reader or the log writer that comes after this waits on the lock until the
// reset. Returns 1 when the caller owns the transport. A transport in quarantine stays unused.
extern "C" int native_ufs_log_panic (uint32_t waitUs)
{
  if (!gLogRegionBlocks || !gHost.lock || !gHost.ready || gHost.poisoned) return 0;
  if (gLogPanicOwned) return 1;
  uint64_t deadline;
  clock_interval_to_deadline (waitUs, kMicrosecondScale, &deadline);
  const bool claimed = ufsLogClaim ([] { return IOLockTryLock (gHost.lock) != 0; }, [&] { return mach_absolute_time () >= deadline; },
                                    [] { __asm__ volatile ("yield"); });
  if (!claimed) return 0;
  gLogPanicOwned = true;
  return gHost.ready && !gHost.poisoned;
}

// Only after native_ufs_log_panic returned 1, on that CPU. The same window checks as hostWrite4k; the timeouts are the caller's.
extern "C" int native_ufs_log_panic_write (uint64_t rel, uint32_t blocks, const void *src, uint32_t timeoutUs)
{
  if (!gLogPanicOwned || !gLogRegionBlocks || rel >= gLogRegionBlocks || blocks > gLogRegionBlocks - rel) return 0;
  const uint64_t lba = gLogRegionFirst + rel;
  if (!hostWriteAllowed (0, lba, blocks, src)) return 0;
  return hostWrite4kHeld (lba, blocks, src, false, timeoutUs) == kIOReturnSuccess;
}

extern "C" int native_ufs_log_panic_sync (uint32_t timeoutUs)
{
  return gLogPanicOwned && gLogRegionBlocks && gHost.ready && hostSyncCacheHeld (timeoutUs) == kIOReturnSuccess;
}

// Re-read the GPT header and entry array and compare their CRCs with what was recorded at root start. 0 = unchanged.
extern "C" unsigned native_ufs_store_gpt_check (void)
{
  if (!gStoreGeometry.valid || !gHost.ready) return 1;
  static uint8_t block[4096];
  if (hostRead4k (0, 1, 1, block) != kIOReturnSuccess) return 2;
  uint32_t headerBytes = le32 (block + 12);
  if (headerBytes < 92 || headerBytes > 4096 || rootCrc32 (block, headerBytes, true) != le32 (block + 16) ||
      le32 (block + 16) != gStoreGeometry.headerCrc) return 3;
  uint32_t blocks = (gStoreGeometry.entryCount * 128 + 4095) / 4096;
  static uint8_t entries[4096 * 4];
  if (blocks > 4 || hostRead4k (0, gStoreGeometry.entriesLba, blocks, entries) != kIOReturnSuccess) return 4;
  return rootCrc32 (entries, gStoreGeometry.entryCount * 128) == gStoreGeometry.entriesCrc ? 0 : 5;
}
