# SPDX-License-Identifier: MIT
# Makefile --- TODO: description
# Copyright (c) 2026 Jakob Kastelic
DEMOS := $(patsubst %/Makefile,%,$(wildcard */Makefile))

all clean check inclusions:
	@for d in $(DEMOS); do \
		$(MAKE) -C $$d $@ || exit 1; \
	done

.PHONY: all clean check inclusions
