# SPDX-License-Identifier: MIT
# Makefile --- sport_fpga_lb dual-DMA, 64 / 512 / 2048 MiB variants
# Copyright (c) 2026 Jakob Kastelic
OBJ = \
  startup.doj \
  main.doj \
  assert.doj \
  sport.doj \
  dma.doj \
  clocks.doj \
  timer.doj \
  pinmux.doj \
  uart.doj \
  printf.doj \
  twi.doj \
  mcp23017.doj \
  board.doj

include ../common/build.mk

VARIANT_64_CFLAGS   = -DTOTAL_BYTES=67108864ULL   -DTOTAL_WORDS=16777216U
VARIANT_512_CFLAGS  = -DTOTAL_BYTES=536870912ULL  -DTOTAL_WORDS=134217728U
VARIANT_2048_CFLAGS = -DTOTAL_BYTES=2147483648ULL -DTOTAL_WORDS=536870912U

OBJS_64   = $(addprefix build/64/,$(OBJ))
OBJS_512  = $(addprefix build/512/,$(OBJ))
OBJS_2048 = $(addprefix build/2048/,$(OBJ))

.DEFAULT_GOAL := variants

.PHONY: variants
variants: build/main64.ldr build/main512.ldr build/main2048.ldr

build/64 build/512 build/2048:
	mkdir -p $@

build/main64.ldr:   build/64/main.dxe
	$(ELFL) $(ELFFLAGS) $< -o $@
build/main512.ldr:  build/512/main.dxe
	$(ELFL) $(ELFFLAGS) $< -o $@
build/main2048.ldr: build/2048/main.dxe
	$(ELFL) $(ELFFLAGS) $< -o $@

build/64/main.dxe:   $(OBJS_64)   ../common/link.ldf | build/64
	$(CC) $(LDFLAGS) -o $@ $(OBJS_64)
build/512/main.dxe:  $(OBJS_512)  ../common/link.ldf | build/512
	$(CC) $(LDFLAGS) -o $@ $(OBJS_512)
build/2048/main.dxe: $(OBJS_2048) ../common/link.ldf | build/2048
	$(CC) $(LDFLAGS) -o $@ $(OBJS_2048)

build/64/%.doj:   ../common/%.s | build/64
	$(ASM) $(ASFLAGS) -o $@ $<
build/512/%.doj:  ../common/%.s | build/512
	$(ASM) $(ASFLAGS) -o $@ $<
build/2048/%.doj: ../common/%.s | build/2048
	$(ASM) $(ASFLAGS) -o $@ $<

build/64/%.doj:   ../common/%.c | build/64
	$(CC) $(CFLAGS) $(VARIANT_64_CFLAGS)   -c -o $@ $<
build/512/%.doj:  ../common/%.c | build/512
	$(CC) $(CFLAGS) $(VARIANT_512_CFLAGS)  -c -o $@ $<
build/2048/%.doj: ../common/%.c | build/2048
	$(CC) $(CFLAGS) $(VARIANT_2048_CFLAGS) -c -o $@ $<

build/64/%.doj:   $(LIBC_SRC)/stdio/%.c | build/64
	$(CC) $(CFLAGS) $(VARIANT_64_CFLAGS)   -c -o $@ $<
build/512/%.doj:  $(LIBC_SRC)/stdio/%.c | build/512
	$(CC) $(CFLAGS) $(VARIANT_512_CFLAGS)  -c -o $@ $<
build/2048/%.doj: $(LIBC_SRC)/stdio/%.c | build/2048
	$(CC) $(CFLAGS) $(VARIANT_2048_CFLAGS) -c -o $@ $<

build/64/%.doj:   %.c | build/64
	$(CC) $(CFLAGS) $(VARIANT_64_CFLAGS)   -c -o $@ $<
build/512/%.doj:  %.c | build/512
	$(CC) $(CFLAGS) $(VARIANT_512_CFLAGS)  -c -o $@ $<
build/2048/%.doj: %.c | build/2048
	$(CC) $(CFLAGS) $(VARIANT_2048_CFLAGS) -c -o $@ $<
