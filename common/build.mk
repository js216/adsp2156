# SPDX-License-Identifier: MIT
# build.mk --- TODO: description
# Copyright (c) 2026 Jakob Kastelic
PROC ?= ADSP-21569
CC   = cc21k
ASM  = easm21k
ELFL = elfloader

LIBC     ?= ../../selache/libsel
LIBC_INC = $(LIBC)/include
LIBC_SRC = $(LIBC)/src

ASFLAGS = \
  -proc $(PROC) -si-revision any \
  -char-size-8 -swc \
  -D__ASSEMBLY__ -I../common

CFLAGS = \
  -proc $(PROC) -si-revision any -O1 \
  -double-size-32 -char-size-8 -swc \
  -no-std-inc -I. -I$(LIBC_INC) -I../common $(CFLAGS_EXTRA)

LDFLAGS  = -proc $(PROC) -si-revision any -T ../common/link.ldf -no-mem -no-std-lib
ELFFLAGS = -proc $(PROC) -b UARTHOST -f ASCII -Width 8 -verbose

OBJS = $(addprefix build/,$(OBJ))

all: build/main.ldr

build:
	mkdir -p build

build/main.ldr: build/main.dxe
	$(ELFL) $(ELFFLAGS) $< -o $@

build/main.dxe: $(OBJS) ../common/link.ldf | build
	$(CC) $(LDFLAGS) -o $@ $(OBJS)

build/%.doj: ../common/%.s | build
	$(ASM) $(ASFLAGS) -o $@ $<

build/%.doj: ../common/%.c | build
	$(CC) $(CFLAGS) -c -o $@ $<

build/%.doj: $(LIBC_SRC)/stdio/%.c | build
	$(CC) $(CFLAGS) -c -o $@ $<

build/%.doj: %.c | build
	$(CC) $(CFLAGS) -c -o $@ $<

# --- static analysis ---------------------------------------------------

EXAMPLE := $(notdir $(CURDIR))
SRCS_LOCAL = $(wildcard *.c) $(wildcard *.h)
SRCS_COMMON = $(wildcard ../common/*.c) $(wildcard ../common/*.h)
SRCS_ALL = $(SRCS_LOCAL) $(SRCS_COMMON)
INCL_SRC = $(addprefix ../$(EXAMPLE)/,$(SRCS_LOCAL)) $(SRCS_COMMON)
TIDY_SRCS = $(filter-out %/printf.c,$(filter %.c,$(SRCS_ALL)))

check: format-check cppcheck-check tidy inclusions done

format-check:
	@grep -lP '\r' $(SRCS_ALL) 2>/dev/null \
		&& { echo "CRLF line endings found (see above)"; exit 1; } \
		|| true
	@grep -lP '[^\x00-\x7F]' $(SRCS_ALL) 2>/dev/null \
		&& { echo "Non-ASCII characters found (see above)"; exit 1; } \
		|| true
	@clang-format --dry-run -Werror $(SRCS_ALL) > /dev/null 2>&1 \
		|| { clang-format --dry-run -Werror $(SRCS_ALL) 2>&1 | head -20; \
		     echo "clang-format violations (run: clang-format -i <files>)"; \
		     exit 1; }

cppcheck-check:
	python3 ../scripts/gen_compile_commands.py . > /dev/null
	cppcheck --enable=all --inconclusive --std=c99 --force --quiet \
		--inline-suppr --error-exitcode=1 --check-level=exhaustive \
		--project=build/compile_commands.json 2>&1 \
		| grep -v '\[checkersReport\]'

tidy:
	python3 ../scripts/gen_compile_commands.py . > /dev/null
	run-clang-tidy -j$$(nproc) -p build $(TIDY_SRCS) 2>&1 \
		| grep -E "warning:|error:" | head -30; \
		run-clang-tidy -j$$(nproc) -p build $(TIDY_SRCS) 2>&1 \
		| grep -qE "warning:|error:" \
		&& { echo "clang-tidy violations found (see above)"; exit 1; } \
		|| true

inclusions: | build
	rm -rf build/incl
	mkdir -p build/incl
	python3 ../scripts/inclusions.py -o build/incl $(INCL_SRC)
	for f in build/incl/*.dot; do \
		dot -Tpdf $$f -o $${f%.dot}.pdf; \
	done

done:
	@echo "\033[1;32mSUCCESS\033[0m"

clean:
	rm -rf build test_out

.PHONY: all clean check format-check cppcheck-check tidy inclusions done
