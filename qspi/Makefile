# SPDX-License-Identifier: MIT
# Makefile --- Build the SPI2 slave demo for the ADSP-21569
# Copyright (c) 2026 Jakob Kastelic

OBJ = \
  startup.doj \
  main.doj \
  assert.doj \
  clocks.doj \
  dma.doj \
  timer.doj \
  pinmux.doj \
  uart.doj \
  printf.doj \
  spi.doj \
  twi.doj \
  mcp23017.doj \
  board.doj

include ../common/build.mk

# qspi-only generated artefact: blob inputs produced by
# plans/make_blobs.py.  quad_diag_exp_*.bin are tracked
# reference files (do not delete on clean).
clean: clean-qspi-extras
clean-qspi-extras:
	rm -rf blobs

.PHONY: clean-qspi-extras
