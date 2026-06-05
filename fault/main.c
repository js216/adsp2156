// SPDX-License-Identifier: MIT
// main.c --- Immediate-fault demo for the EV-21569-SOM
// Copyright (c) 2026 Jakob Kastelic

// Deliberately assert the processor fault output (SYS_FAULT, red
// LED1, scope C2) so the bench can confirm fault detection.

#include "regs.h"
#include <stdint.h>

// SEC0 fault-management registers (ADSP-2156x HRM chapter 6).
#define REG_SEC0_GCTL    0x31089000U  // Global Control
#define REG_SEC0_RAISE   0x31089008U  // Global Raise (write Source ID)
#define REG_SEC0_FCTL    0x31089010U  // Fault Control
#define REG_SEC0_FDLY    0x31089020U  // Fault Delay
#define REG_SEC0_SCTL(n) (0x31089800U + (n) * 8U)  // Source Control n

#define BITM_SEC_GCTL_EN   0x00000001U
#define BITM_SEC_FCTL_EN   0x00000001U
#define BITM_SEC_FCTL_FOEN 0x00000010U  // Fault Output Enable (drives SYS_FAULT)
#define BITM_SEC_SCTL_FEN  0x00000002U  // route source to fault, not interrupt
#define BITM_SEC_SCTL_SEN  0x00000004U  // source signal enable

// Software Interrupt 0: a source we can raise from software with no
// peripheral dependency (SEC source ID 29).
#define FAULT_SID 29U

int main(void)
{
   // Enable the SEC.
   MMR(REG_SEC0_GCTL) = BITM_SEC_GCTL_EN;

   // Make our chosen source a fault source and enable it.
   MMR(REG_SEC0_SCTL(FAULT_SID)) = BITM_SEC_SCTL_SEN | BITM_SEC_SCTL_FEN;

   // No fault-output delay: assert immediately.
   MMR(REG_SEC0_FDLY) = 0U;

   // Enable fault handling and drive the external SYS_FAULT pin.
   MMR(REG_SEC0_FCTL) = BITM_SEC_FCTL_EN | BITM_SEC_FCTL_FOEN;

   // Raise the source -> SEC takes the fault action -> SYS_FAULT
   // asserts. We never write SEC0_FEND, so it stays asserted.
   MMR(REG_SEC0_RAISE) = FAULT_SID;

   for (;;) {
      // Hold here with the fault latched (LED on).
   }
}
