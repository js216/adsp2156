# SPDX-License-Identifier: MIT
# Makefile --- Top-level build and static-analysis entry point
# Copyright (c) 2026 Jakob Kastelic

DEMOS = blink uart gpio sport sport_dma
TC   ?= /opt/analog/cces/3.0.3

SOURCES = $(wildcard common/*.[ch] board/*.[ch] \
            blink/main.c uart/main.c gpio/main.c \
            sport/main.c sport_dma/main.c)

all:
	for d in $(DEMOS); do \
		$(MAKE) -C $$d TC=$(TC) -j$$(nproc); \
	done

clean:
	for d in $(DEMOS); do \
		$(MAKE) -C $$d clean; \
	done
	rm -rf build

check: format-check cppcheck-check tidy inclusions done

format-check:
	git ls-files -- '*.[chSs]' '*.py' '*.md' 'Makefile' '*.ldf' '*.mk' \
		| xargs grep -rlP '\r' 2>/dev/null \
		&& { echo "CRLF line endings found (see above)"; exit 1; } \
		|| true
	git ls-files -- '*.[chSs]' '*.py' '*.md' 'Makefile' '*.ldf' '*.mk' \
		| xargs grep -Pn '[^\x00-\x7F]' 2>/dev/null \
		&& { echo "Non-ASCII characters found (see above)"; exit 1; } \
		|| true
	clang-format --dry-run -Werror $(SOURCES) 2>&1 \
		| head -20; \
		clang-format --dry-run -Werror $(SOURCES) > /dev/null 2>&1 \
		|| { echo "clang-format violations found (run: clang-format -i <files>)"; exit 1; }

cppcheck-check:
	python3 scripts/gen_compile_commands.py > /dev/null
	cppcheck --enable=all --inconclusive --std=c99 --force --quiet \
		--inline-suppr --error-exitcode=1 --check-level=exhaustive \
		--project=build/compile_commands.json 2>&1 \
		| grep -v '\[checkersReport\]'

tidy:
	python3 scripts/gen_compile_commands.py > /dev/null
	run-clang-tidy -j$$(nproc) -p build \
		$(filter-out %/printf.c,$(filter %.c,$(SOURCES))) 2>&1 \
		| grep -E "warning:|error:" | head -30

inclusions:
	mkdir -p build
	python3 scripts/inclusions.py $(SOURCES) > build/incl.dot
	dot -Tpdf build/incl.dot -o build/incl.pdf

done:
	@echo "\033[1;32mSUCCESS\033[0m"

.PHONY: all clean check format-check cppcheck-check tidy inclusions done
