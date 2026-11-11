// PianoUsb: Synopsys DesignWare USB3 DRD controller for SM8150 (piano).
//
// Registers (from Android sm8150-usb.dtsi):
//   usb0: ssusb@a600000, reg = <0x0a600000 0x100000>, core_base
//   SMMU stream 0x140, pwr_event_irq 130
//   qcom,core-clk-rate = 200 MHz (GCC_USB30_PRIM_MASTER_CLK)
//
// DWC3 register map (drivers/usb/dwc3/core.h):
//   globals 0xc100-0xc6ff   device 0xc700-0xcbff   event buffers 0xc400
//
// Stage 1 (this file): map the block, read GSNPSID to prove the mapping and clocks
// work, then set device mode via GCTL.PRTCAPDIR and enable pull-ups via DCTL.
//
// Clocks: NativeUfs already drives GCC (GccUfs.h); the USB gate registers live in the
// same GCC block, only the bit offsets differ. Bring the gates up before touching DWC3.

#include <libkern/OSTypes.h>
#include <IOKit/IOService.h>

class PianoUsb : public IOService {
  OSDeclareDefaultStructors(PianoUsb)

 public:
  bool init(OSDictionary *dict = 0) override;
  void free(void) override;
  IOService *probe(IOService *provider, SInt32 *score) override;
  bool start(IOService *provider) override;
  void stop(IOService *provider) override;
};
