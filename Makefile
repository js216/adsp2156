DEMOS := $(patsubst %/Makefile,%,$(wildcard */Makefile))

all clean check inclusions:
	@for d in $(DEMOS); do \
		$(MAKE) -C $$d $@ || exit 1; \
	done

.PHONY: all clean check inclusions
