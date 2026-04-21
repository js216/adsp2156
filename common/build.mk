# SPDX-License-Identifier: MIT
# build.mk --- Shared demo build recipe for the ADSP-2156x family
# Copyright (c) 2026 Jakob Kastelic

# The SHARC+ cross compiler binaries (cc21k, easm21k,
# elfloader) are invoked by name and must be on $PATH. Any
# install layout works: a CCES tree, a wrapper directory, or
# symlinks. Override individual tool names on the command line
# if necessary.
PROC ?= ADSP-21569

CC   = cc21k
ASM  = easm21k
ELFL = elfloader

# Stdlib headers (stdarg, stdbool, stdint, stdio, assert, ...)
# and their tiny implementations come from libsel, the
# freestanding C library shared with selache. common/ only
# holds drivers, project extensions, and board glue.
LIBSEL     = ../../selache/libsel
LIBSEL_INC = $(LIBSEL)/include
LIBSEL_SRC = $(LIBSEL)/src

ASFLAGS = \
  -proc $(PROC) -si-revision any \
  -char-size-8 -swc \
  -D__ASSEMBLY__ -I../common

CFLAGS = \
  -proc $(PROC) -si-revision any -O1 \
  -double-size-32 -char-size-8 -swc \
  -no-std-inc -I. -I$(LIBSEL_INC) -I../common

LDFLAGS  = -proc $(PROC) -si-revision any -T ../common/link.ldf -no-mem -no-std-lib
ELFFLAGS = -proc $(PROC) -b UARTHOST -f ASCII -Width 8 -verbose

main.ldr: main.dxe
	$(ELFL) $(ELFFLAGS) $< -o $@

main.dxe: $(OBJ) ../common/link.ldf
	$(CC) $(LDFLAGS) -o $@ $(OBJ)

%.doj: ../common/%.s
	$(ASM) $(ASFLAGS) -o $@ $<

%.doj: ../common/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.doj: $(LIBSEL_SRC)/stdio/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.doj: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJ) main.dxe main.ldr

.PHONY: clean
