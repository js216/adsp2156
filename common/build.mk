# SPDX-License-Identifier: MIT
# build.mk --- Shared demo build recipe for the ADSP-2156x family
# Copyright (c) 2026 Jakob Kastelic

# Toolchain root and target processor are overridable from the
# command line.  The default TC points at a CCES install, but
# any SHARC+ cross compiler with the same binary names (cc21k,
# easm21k, elfloader) can be used by overriding TC.
TC   ?= /opt/analog/cces/3.0.3
PROC ?= ADSP-21569

CC   = $(TC)/cc21k
ASM  = $(TC)/easm21k
ELFL = $(TC)/elfloader

ASFLAGS = \
  -proc $(PROC) -si-revision any \
  -char-size-8 -swc \
  -D__ASSEMBLY__ -I../common

CFLAGS = \
  -proc $(PROC) -si-revision any -O1 \
  -double-size-32 -char-size-8 -swc \
  -no-std-inc -I. -I../common -I../board

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

%.doj: ../board/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.doj: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJ) main.dxe main.ldr

.PHONY: clean
