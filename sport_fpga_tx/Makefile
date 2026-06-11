# SPDX-License-Identifier: MIT
# Makefile --- sport_fpga_tx, SPORT4B PRBS receiver variants
# Copyright (c) 2026 Jakob Kastelic
OBJ = \
  startup.doj \
  main.doj \
  assert.doj \
  sport.doj \
  dma.doj \
  clocks.doj \
  gpio.doj \
  timer.doj \
  pinmux.doj \
  uart.doj \
  printf.doj \
  twi.doj \
  mcp23017.doj \
  board.doj

include ../common/build.mk

VARIANT_PRBS64_CFLAGS = -DTOTAL_BYTES=67108864ULL -DTOTAL_WORDS=16777216U
VARIANT_PRBS512_CFLAGS = -DTOTAL_BYTES=536870912ULL -DTOTAL_WORDS=134217728U
VARIANT_PRBS2048_CFLAGS = -DTOTAL_BYTES=2147483648ULL -DTOTAL_WORDS=536870912U

OBJS_PRBS64 = $(addprefix build/prbs64/,$(OBJ))
OBJS_PRBS512 = $(addprefix build/prbs512/,$(OBJ))
OBJS_PRBS2048 = $(addprefix build/prbs2048/,$(OBJ))

.DEFAULT_GOAL := variants

.PHONY: variants
variants: build/mainprbs64.ldr build/mainprbs512.ldr build/mainprbs2048.ldr

build/prbs64 build/prbs512 build/prbs2048:
	mkdir -p $@

build/mainprbs64.ldr: build/prbs64/main.dxe
	$(ELFL) $(ELFFLAGS) $< -o $@

build/mainprbs512.ldr: build/prbs512/main.dxe
	$(ELFL) $(ELFFLAGS) $< -o $@

build/mainprbs2048.ldr: build/prbs2048/main.dxe
	$(ELFL) $(ELFFLAGS) $< -o $@

build/prbs64/main.dxe: $(OBJS_PRBS64) ../common/link.ldf | build/prbs64
	$(CC) $(LDFLAGS) -o $@ $(OBJS_PRBS64)

build/prbs512/main.dxe: $(OBJS_PRBS512) ../common/link.ldf | build/prbs512
	$(CC) $(LDFLAGS) -o $@ $(OBJS_PRBS512)

build/prbs2048/main.dxe: $(OBJS_PRBS2048) ../common/link.ldf | build/prbs2048
	$(CC) $(LDFLAGS) -o $@ $(OBJS_PRBS2048)

build/prbs64/%.doj: ../common/%.s | build/prbs64
	$(ASM) $(ASFLAGS) -o $@ $<

build/prbs512/%.doj: ../common/%.s | build/prbs512
	$(ASM) $(ASFLAGS) -o $@ $<

build/prbs2048/%.doj: ../common/%.s | build/prbs2048
	$(ASM) $(ASFLAGS) -o $@ $<

build/prbs64/%.doj: ../common/%.c | build/prbs64
	$(CC) $(CFLAGS) $(VARIANT_PRBS64_CFLAGS) -c -o $@ $<

build/prbs512/%.doj: ../common/%.c | build/prbs512
	$(CC) $(CFLAGS) $(VARIANT_PRBS512_CFLAGS) -c -o $@ $<

build/prbs2048/%.doj: ../common/%.c | build/prbs2048
	$(CC) $(CFLAGS) $(VARIANT_PRBS2048_CFLAGS) -c -o $@ $<

build/prbs64/%.doj: $(LIBC_SRC)/stdio/%.c | build/prbs64
	$(CC) $(CFLAGS) $(VARIANT_PRBS64_CFLAGS) -c -o $@ $<

build/prbs512/%.doj: $(LIBC_SRC)/stdio/%.c | build/prbs512
	$(CC) $(CFLAGS) $(VARIANT_PRBS512_CFLAGS) -c -o $@ $<

build/prbs2048/%.doj: $(LIBC_SRC)/stdio/%.c | build/prbs2048
	$(CC) $(CFLAGS) $(VARIANT_PRBS2048_CFLAGS) -c -o $@ $<

build/prbs64/%.doj: %.c | build/prbs64
	$(CC) $(CFLAGS) $(VARIANT_PRBS64_CFLAGS) -c -o $@ $<

build/prbs512/%.doj: %.c | build/prbs512
	$(CC) $(CFLAGS) $(VARIANT_PRBS512_CFLAGS) -c -o $@ $<

build/prbs2048/%.doj: %.c | build/prbs2048
	$(CC) $(CFLAGS) $(VARIANT_PRBS2048_CFLAGS) -c -o $@ $<
