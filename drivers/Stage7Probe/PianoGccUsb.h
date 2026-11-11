// SM8150 GCC USB30_PRIM / QUSB2 PHY registers used by NativeDwc3.
//
// Transcribed from Linux drivers/clk/qcom/gcc-sm8150.c (torvalds/linux). Offsets
// are relative to the GCC block at 0x00100000 (piano board DT, clock-controller@100000).
// Kept numeric and commented so a wrong guess is visible, not buried in macros.

#ifndef PIANO_GCC_USB_H
#define PIANO_GCC_USB_H

#define GCC_BASE 0x00100000ULL

// --- USB30_PRIM power domain -------------------------------------------------
// usb30_prim_gdsc: .gdscr = 0x0f004, PWRSTS_OFF_ON, POLL_CFG_GDSCR.
// SW_COLLAPSE (bit 0): 1 = request off, 0 = request on. Power state is read-only
// in PWR_ON (bit 31); poll it after clearing SW_COLLAPSE.
#define GCC_USB30_PRIM_GDSCR      0x0f004
#define GCC_GDSCR_SW_COLLAPSE     (1u << 0)
#define GCC_GDSCR_PWR_ON          (1u << 31)

// --- branch clocks -----------------------------------------------------------
// enable register bit 0; CLK_OFF is bit 31 of the same (halt) register.
#define GCC_USB30_PRIM_MASTER_CLK     0x0f010
#define GCC_USB30_PRIM_SLEEP_CLK      0x0f014
#define GCC_USB30_PRIM_MOCK_UTMI_CLK  0x0f018
#define GCC_USB3_PRIM_PHY_AUX_CLK     0x0f050
#define GCC_USB3_PRIM_PHY_COM_AUX_CLK 0x0f054
#define GCC_USB3_PRIM_PHY_PIPE_CLK    0x0f058
#define GCC_AGGRE_USB3_PRIM_AXI_CLK   0x0f07c
#define GCC_CFG_NOC_USB3_PRIM_AXI_CLK 0x0f078
#define GCC_USB3_PRIM_CLKREF_CLK      0x8c008 // gcc_usb3_prim_clkref_clk
#define GCC_USB3_SEC_CLKREF_CLK       0x8c028 // gcc_usb3_sec_clkref_clk: what usb_1 uses
#define GCC_BRANCH_ENABLE_BIT         0x1u
#define GCC_BRANCH_CLK_OFF            (1u << 31)

// gcc_usb30_prim_master_clk_src: RCG2, cmd_rcgr = 0x0f01c, 200 MHz entry is
// P_GPLL0_OUT_MAIN / 3. parent_map_0: BI_TCXO = 0, GPLL0_OUT_MAIN = 1.
// cfg_rcgr = cmd_rcgr + 4: SRC_SEL<<8 | MODE<<12 | DIV<<16 (DIV mode, MODE = 0).
#define GCC_USB30_PRIM_MASTER_CLK_SRC 0x0f01c
#define GCC_RCG2_CMD		0x0
#define GCC_RCG2_CFG		0x4
#define GCC_RCG2_CMD_UPDATE	(1u << 0)
#define GCC_USB_MASTER_SRC_GPLL0_MAIN 1u
#define GCC_USB_MASTER_DIV_200MHZ     3u

// --- resets ------------------------------------------------------------------
// qcom_reset_map entries; write bit 0, the block self-clears.
#define GCC_USB30_PRIM_BCR            0x0f000
#define GCC_QUSB2PHY_PRIM_BCR         0x12000
#define GCC_USB3_PHY_PRIM_BCR         0x50000
#define GCC_USB3_DP_PHY_PRIM_BCR      0x50008
#define GCC_USB_PHY_CFG_AHB2PHY_BCR   0x6a000

#endif // PIANO_GCC_USB_H
