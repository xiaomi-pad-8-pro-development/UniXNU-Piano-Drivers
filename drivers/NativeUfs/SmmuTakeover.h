// SPDX-License-Identifier: BSD-2-Clause-Patent
// Local override of kits/Sm8750Platform/SmmuTakeover.h for the piano (PKX110) bring-up.
//
// Difference from the shared header: this device's firmware leaves an EXCLUSIVE stream-match
// entry (SMR mask != 0) for the UFS host stream 0x60, mapped to a translating context bank.
// The shared planner treats any masked match as "conflicting UFS stream mapping" and refuses,
// which stops the storage root from ever being published. With native-ufs-smmu-takeover=1 the
// planner instead adopts that entry's context bank and reprograms it as the driver's bypass
// context (SCTLR=0, CBAR=S1_TRANS_S2_BYPASS), which is what the storage-only, read-only design
// needs. Without the boot argument the behaviour is byte-identical to the shared header.
#if defined(NATIVE_UFS_SM8750) && NATIVE_UFS_SM8750
static constexpr unsigned kUfsSmmuMapBytes = 0x100000; // PKX110 live.dtb: apps-smmu@15000000.
#else
static constexpr unsigned kUfsSmmuMapBytes = 0xd0000;
#endif

static bool ufsSmmuTakeoverRequested ()
{
  unsigned value = 0;
  return PE_parse_boot_argn ("native-ufs-smmu-takeover", &value, sizeof (value)) && value != 0;
}

struct UfsSmmuPlan {
  unsigned slot = 0xffffffffU, bank = 0, pages = 0;
  uint32_t smr = 0, s2cr = 0;
  bool reuse = false;
  bool takeover = false;
  const char *error = nullptr;
};
// ARM IHI 0062D.c sections 1.6 and 3.7: VA64=0 still rejects inputs above
// 32 bits with SCTLR.M=0. With VA64=1 the bypass range is bounded by all
// three hardware widths, not TCR2.PASize (which only limits translation).
static unsigned ufsSmmuAddressBits (unsigned code)
{
  switch (code) {
    case 0: return 32; case 1: return 36; case 2: return 40;
    case 3: return 42; case 4: return 44; case 5: return 48;
    default: return 0;
  }
}
// One attempt with a given context bank (counted from the top). The caller falls back to lower banks when this one is held by another stream.
template <class Read>
static UfsSmmuPlan planUfsSmmuBank (uint16_t stream, uint64_t dma, uint64_t bytes, Read read, unsigned bankFromTop,
                                    unsigned mapBytes = kUfsSmmuMapBytes)
{
  UfsSmmuPlan p;
  uint32_t idr0 = read (0x20), idr2 = read (0x28);
  unsigned count = idr0 & 255;
  if (!(idr0 & (1U << 30)) || !(idr2 & (1U << 12))) {
    p.error = "SMMU AArch64 stage 1 unsupported"; return p;
  }
  for (unsigned shift = 0; shift <= 8; shift += 4) {
    unsigned bits = ufsSmmuAddressBits ((idr2 >> shift) & 15);
    if (!bits || !bytes || dma >= (1ULL << bits) || bytes > (1ULL << bits) - dma) {
      p.error = "DMA exceeds SMMU address width"; return p;
    }
  }
  uint32_t idr1 = read (0x24);
  unsigned banks = idr1 & 255;
  p.pages = 1U << (((idr1 >> 28) & 7) + 1);
  if ((idr1 >> 31) || !count || p.pages > 128 || banks < 2 || banks > 128 ||
      (p.pages + banks) * 4096 > mapBytes) {
    p.error = "unsupported SMMU geometry"; return p;
  }
  // Qualcomm qcom_smmu_cfg_probe: extra groups have non-architectural behavior.
  if (count > 128) {
    count = 128;
  }
  if (bankFromTop >= banks - 1) {
    p.error = "no free context bank"; return p;
  }
  p.bank = banks - 1 - bankFromTop;
  bool takeover = ufsSmmuTakeoverRequested ();
  unsigned matches = 0;
  for (unsigned i = 0; i < count; ++i) {
    uint32_t smr = read (0x800 + 4 * i);
    if (!(smr & 0x80000000U)) {
      if (!matches) {
        p.slot = i;
      }
      continue;
    }
    uint32_t s2cr = read (0xc00 + 4 * i);
    uint32_t mask = (smr >> 16) & 0x7fff;
    bool match = ((smr ^ stream) & 0xffff & ~mask) == 0;
    bool ourBank = ((s2cr >> 16) & 3) == 0 && (s2cr & 255) == p.bank;
    if (match) {
      ++matches;
      bool usable = ourBank;
#if defined(NATIVE_UFS_SM8750) && NATIVE_UFS_SM8750
      // PKX110 firmware leaves SID 0x60 on bank 0; redirect only its exact entry.
      usable = usable || (stream == 0x60 && s2cr == 0);
#endif
      if (matches == 1 && takeover) {
        // Firmware entry for our stream (exact or masked): take over the bank it points at.
        // The piano firmware leaves SMR=0x80000060 (mask 0) with S2CR pointing at context
        // bank 1 and CBAR=0, i.e. a translating context with no bypass set up.
        unsigned firmwareBank = s2cr & 255;
        if (((s2cr >> 16) & 3) != 0 || firmwareBank >= banks) {
          p.slot = i; p.smr = smr; p.s2cr = s2cr;
          p.error = "conflicting UFS stream mapping"; return p;
        }
        p.bank = firmwareBank;
        p.slot = i;
        p.takeover = true;
        p.reuse = false;
        continue;
      }
      if (matches > 1 || mask || !usable) {
        p.slot = i; p.smr = smr; p.s2cr = s2cr;
        p.error = "conflicting UFS stream mapping"; return p;
      }
      p.reuse = ourBank; p.slot = i;
    } else if (ourBank) {
      p.slot = i; p.smr = smr; p.s2cr = s2cr;
      p.error = "reserved context bank already used"; return p;
    }
  }
  uint32_t sctlr = read ((p.pages + p.bank) * 4096);
  if ((sctlr & 1) && !p.takeover) {
    p.error = "reserved context bank not bypass"; return p;
  }
  if (p.reuse && (sctlr || read (0x1000 + 4 * p.bank) != 0x10000 ||
                  (read (0x1800 + 4 * p.bank) & ~1U))) {
    p.error = "reserved context bank not bypass"; return p;
  }
  if (p.slot == 0xffffffffU) {
    p.error = "no free stream-match group";
  }
  return p;
}

// The UFS stream's context bank: the top bank first. A stream mapping left behind by an earlier run (or by another OS that used the SMMU before a
// warm reset) can still point another stream at that bank; rather than refuse to boot, the next banks down are tried, each held to the same checks
// (no other valid stream maps it, translation off). Any other refusal (a conflicting mapping of our own stream, no free group, unsupported geometry)
// is final. If all attempts fail, prefer our stream's conflict over another stream's occupied bank.
template <class Read>
static UfsSmmuPlan planUfsSmmu (uint16_t stream, uint64_t dma, uint64_t bytes, Read read,
                                unsigned mapBytes = kUfsSmmuMapBytes)
{
  enum { kBankAttempts = 8 };
  UfsSmmuPlan first = planUfsSmmuBank (stream, dma, bytes, read, 0, mapBytes);
  if (!first.error) {
    return first;
  }
  // "conflicting" is retried too: our own stream may already sit on a lower bank from an earlier run that had to fall back.
  auto retryable = [](const char *e) {
    return !__builtin_strcmp (e, "reserved context bank already used") || !__builtin_strcmp (e, "reserved context bank not bypass") ||
           !__builtin_strcmp (e, "conflicting UFS stream mapping");
  };
  if (!retryable (first.error)) {
    return first;
  }
  UfsSmmuPlan failure = first;
  for (unsigned attempt = 1; attempt < kBankAttempts; ++attempt) {
    UfsSmmuPlan next = planUfsSmmuBank (stream, dma, bytes, read, attempt, mapBytes);
    if (!next.error) {
      return next;
    }
    // Report our stream's conflict rather than an unrelated occupied top bank.
    if (!__builtin_strcmp (next.error, "conflicting UFS stream mapping")) {
      failure = next;
    }
    if (!retryable (next.error)) {
      break;
    }
  }
  return failure;
}

// The caller must first quiesce the UFS queue. The planner refuses other
// streams sharing this bank and enabled translation; no live context is
// disabled here. Write callbacks must complete an MMIO barrier before return.
template <class Read, class Write>
static const char *configureUfsSmmu (const UfsSmmuPlan &p, uint16_t stream, Read read, Write write)
{
  if (p.error) {
    return p.error;
  }
  unsigned cb = (p.pages + p.bank) * 4096;
  if (!p.reuse) {
    write (cb, 0);
  }
  if (read (cb) != 0) {
    return "SMMU bypass disable readback";
  }
  // Set VA64 before exposing a newly allocated stream. Repair an exclusive
  // existing 32-bit bypass too; do not silently reuse the faulty state.
  if (read (0x1800 + 4 * p.bank) != 1) {
    write (0x1800 + 4 * p.bank, 1);
  }
  if (!p.reuse) {
    write (0x1000 + 4 * p.bank, 0x10000);
  }
  if (read (0x1800 + 4 * p.bank) != 1 || read (0x1000 + 4 * p.bank) != 0x10000 || read (cb) != 0) {
    return "SMMU VA64 bypass readback";
  }
  if (!p.reuse) {
    write (0xc00 + 4 * p.slot, p.bank);
    if (read (0xc00 + 4 * p.slot) != p.bank) {
      return "SMMU stream context readback";
    }
    write (0x800 + 4 * p.slot, 0x80000000U | stream);
  }
  if (read (0x800 + 4 * p.slot) != (0x80000000U | stream) ||
      read (0xc00 + 4 * p.slot) != p.bank) {
    return "SMMU stream mapping readback";
  }
  return nullptr;
}

// Complete firmware-era non-secure TLB invalidation before restarting UFS.
// Write callbacks include the MMIO barrier; the caller supplies a bounded deadline.
template <class Read, class Write, class Expired>
static bool syncUfsSmmu (Read read, Write write, Expired expired, uint32_t &status)
{
  // Qualcomm's trapped MMIO handling requires a non-WZR value (Linux QCOM_DUMMY_VAL).
  write (0x68, 0xffffffffU); // TLBIALLNSNH: preserve stream mappings and page tables.
  write (0x70, 0xffffffffU); // sTLBGSYNC
  do {
    status = read (0x74); // sTLBGSTATUS.GSACTIVE
    if (!(status & 1)) {
      return true;
    }
  } while (!expired ());
  return false;
}
