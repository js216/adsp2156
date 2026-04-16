<!-- SPDX-License-Identifier: MIT -->
<!-- Copyright (c) 2026 Jakob Kastelic -->
everywhere
----------------

const qualify all function arguments that do not need to be mutable

be careful to validate all function arguments thoroughly! assertions are your
friends, doesn't C99 provide them?

header files should give a short description of each function argument (e.g. one
line per argument or so), what it means and what are the allowed values, and
likewise for return values

adsp2156/README.md
-------------

9 - **gpio**: three-channel GPIO loopback pre-check over P13 jumpers

rephrase: GPIO loopback check using hardware jumpers

adsp2156/blink/main.c
------------

5 // Toggles the SOM expander LEDs on U13 (driven via TWI2). The
6 // PORTC LEDs only exist on the EV-SOMCRR-EZKIT carrier board, so
7 // on a standalone SOM the U13 path is the only observable signal.

not true: the LEDs are actually on the SOM (you called it the "SOM expander"),
not the EZKIT. (true, EZKIT has another expander but that's not what we're
blinking here). Call it "GPIO expander", not "SOM expander".

14 static void delay_busy(uint32_t n)
15 {
16 for (volatile uint32_t i = 0U; i < n; i++)
17 ;
18 }

Can we now subsume delay_busy into timer.c/.h or some such file? Also, why not
just use delay_ms() which I think we've already implemented somehow?

adsp2156/common/somconfig.c and .h
-------------

the only SOM-related function concerns the MCP23017 GPIO expander, so why not
call these files mcp23017.c / .h?

6 // address 0x21. Port A bits 0..2 drive the on-SOM LEDs (LED4/LED6/
7 // LED7); bit 5 is the active-low UART0_EN line that gates UART0 TX
8 // to the carrier-board header. som_init() pinmuxes TWI2, brings up
9 // the controller, and pre-programs the expander direction registers
10 // once. som_set_leds() then only has to write GPIOA on each call.

this looks like board-specific information, so move it to board.h such that
mcp23017.h/.c can be generic MCP23017 control driver.

12 #include "regs.h"

MCP23017 is 100% programmed via I2C and we have an I2C driver (twi.h), so there
should be no register handling in mcp23017.c/.h.

13 #include "board.h"

again, MCP23017 driver is generic and should not depend on the specific board!

26 #define U13_ADDR 0x21U

`U13` is board specific --- just call it MCP23017_ADDR, no? are there multiple
possible addresses?

27 #define U13_PORTA_LEDS_ON 0x07U
28 #define U13_PORTA_LEDS_OFF 0x00U


this is board specific info

void som_init(void)

this function should be more general, so different boards/apps can configure the
gpio expander as they want to. maybe header defines some struct etc. for the
configuration? but stay lightweight please.

54 void som_set_leds(bool on)

which LEDs? that's board-specific info. MCP23017 driver just has GPIO, digital
write on/off kind of stuff.

adsp2156/common/build.mk
--------------

5 # Each demo Makefile is expected to define:
6 # TARGET -- short name (e.g. blink) -> TARGET.ldr is the final output
7 # OBJ -- list of .doj files to link in -> object stems are looked up
8 # first in the demo directory,
9 # then in ../common
10 # and then `include ../common/build.mk`.

unnecessary comment because the user can just look at the individual Makefiles
for each example.

12 CCES ?= /opt/analog/cces/3.0.3
13 CC = $(CCES)/cc21k
14 ASM = $(CCES)/easm21k
15 ELFL = $(CCES)/elfloader

let's not call it `CCES` but rather `TC` (tool chain, or some similar acronym if
you don't like `TC`), since CCES is just one of the toolchains that could be
used (selache being the other).

17 ASFLAGS = \
18 -proc ADSP-21569 -si-revision any \
19 -char-size-8 -swc \
20 -D__ASSEMBLY__ -I../common
21
22 CFLAGS = \
23 -proc ADSP-21569 -si-revision any -O1 \
24 -double-size-32 -char-size-8 -swc \
25 -no-std-inc -I. -I../common
26
27 LDFLAGS = -proc ADSP-21569 -si-revision any -T ../common/link.ldf -no-mem -no-std-lib
28
29 ELFFLAGS = -proc ADSP-21569 -b UARTHOST -f ASCII -Width 8 -verbose

all these flags are good except the processor should be defined in a make
variable that's easy to override on cmdline, since all code examples will
probably run just fine on another variant like ADSP-21565 with only minor
variations.

Also: we need a way to run static code analysis, for example:

```
# Static code analysis

check: format cppcheck tidy inclusions done

format:
	grep -rlP '\r' --include='*.[chSs]' --include='*.py' --include='*.md' \
		--include='Makefile' --include='*.ld' --include='*.tsv' . \
		&& { echo "CRLF line endings found (see above)"; exit 1; } || true
	git ls-files \
		| xargs grep -P -n '[^\x00-\x7F]' 2>/dev/null \
		&& { echo "Non-ASCII characters found (see above)"; exit 1; } || true
	clang-format --dry-run -Werror $(wildcard */*.[ch])

tidy:
	make clean
	mkdir -p build
	bear --output build/compile_commands.json -- make \
		 -j$(shell nproc) $(TARGET)
	run-clang-tidy -j$(shell nproc) -p build $(wildcard src/*.c) \
		-extra-arg=-I/usr/lib/arm-none-eabi/include

cppcheck:
	cppcheck --enable=all --inconclusive --std=c99 --force --quiet \
	--inline-suppr --error-exitcode=1 --suppress=missingInclude \
	'--suppress=*:drivers/*' src util drivers

inclusions: $(INC_SRCS) scripts/inclusions.py
	python3 scripts/inclusions.py $(wildcard src/*.[ch]) > build/incl.dot; \
	PYTHON_EXIT=$$? ; \
	dot -Tpdf build/incl.dot -o build/incl.pdf ; \
	exit $$PYTHON_EXIT

done:
	@echo "\033[1;32mSUCCESS\033[0m: All checks passed."
```

get the inclusions script from: https://github.com/js216/stm32mp135-bootloader/blob/main/scripts/inclusions.py
tidy: https://github.com/js216/stm32mp135-bootloader/blob/main/.clang-tidy
format: https://github.com/js216/stm32mp135-bootloader/blob/main/.clang-format

Also: is there such a thing as "reuse lint" which checks that all files have
proper licenses, that contributions are licensed correctly, etc.?

adsp2156/common/board.h
---------------------------

5 #ifndef BLINK_BOARD_H
6 #define BLINK_BOARD_H

this is a lot more than just a "blink board"!

12 // SCLK0 frequency the boot ROM leaves running on the EV-21569-SOM in
13 // UART-host boot mode. CLKIN is the on-board 25 MHz oscillator; the
14 // CGU defaults the boot ROM applies are MSEL = 15, SYSSEL = 2,
15 // S0SEL = 2 -- giving SCLK0 = 25 MHz x 15 / 2 / 2 = 93.75 MHz. See
16 // HRM 4-3 (Clock Generation Unit, equations for f_SYSCLK and
17 // f_SCLK0).
18 #define BOARD_SCLK_HZ 93750000U

We should do clock initialization explicitly rather than rely on defaults. Maybe
need to add/modify the common/clock.c/.h files?

29 // Done
30 // in the preprocessor so no runtime divide is needed at startup.

fluff, remove. Obviously preprocessor is great, we all know that.

33 // UART0 is wired to PA6 (TX) / PA7 (RX) on the EV-21569-SOM carrier;

isn't uart0 ALWAYS wired to these ports on all versions of all adsp21569?

34 // both pins are routed via the "b" alternate function = mux value 1.
35 // PORTA_FER bits select the pin (one bit per pin), PORTA_MUX bits
36 // pack two bits per pin starting at bit 0 for PA0.
37 #define BOARD_PA_UART0_TX_FER_BIT (1U << 6)
38 #define BOARD_PA_UART0_RX_FER_BIT (1U << 7)
39 #define BOARD_PA_UART0_TX_MUX_POS 12U
40 #define BOARD_PA_UART0_RX_MUX_POS 14U

if each driver will need to do pin muxing separately, isn't there danger that
one will overwrite the other? all BSPs I've seen have a separate pinmux file
where such details are all decided. then, board.c/.h and pinmux.c/.h do not
really belong in common/ but in board/ (add a line to README for the new
directory). commonly clocks.c/.h is also in board/ directory.

42 // ----------------------------------------------------------------------
43 // TWI2 pinmux
44 // ----------------------------------------------------------------------

same issue

54 // ----------------------------------------------------------------------
55 // Spin-loop delay
56 // ----------------------------------------------------------------------

as noted, we can cleanly implement delay_ms in some other way and using a
dedicated driver for it, as appropriate, and then spin-delay is not needed
anymore.

adsp2156/common/dma.c
--------------

11 // sizes, which is exactly what a SPORT half-channel wants
12 // for a continuous 32-bit data stream.

why mention SPORT in a general dma driver? don't.

23 #define OFF_DMA_YCNT 0x14
24 #define OFF_DMA_YMOD 0x18
25 #define OFF_DMA_DSCPTR_CUR 0x24
26 #define OFF_DMA_DSCPTR_PRV 0x28
27 #define OFF_DMA_ADDR_CUR 0x2C
28 #define OFF_DMA_STAT 0x30
29 #define OFF_DMA_XCNT_CUR 0x34
30 #define OFF_DMA_YCNT_CUR 0x38

missing HRM citation comments?

49 if (n <= 7U) {

what happens if n is negative?

50 return 0x31022000U + n * 0x80U;
51 }
52 // n is in [10..17] for the SPORT4..7 channels. DMA10 sits
53 // at 0x31024000 (the 0x31023xxx block holds MDMA0, the
54 // DMA8/9 pair). Channels are 0x80 apart as usual.
55 return 0x31024000U + (n - 10U) * 0x80U;

magic numbers should be #defined either in this file, the header file, or reg.h,
depending on how widely they need to be used.

50 return 0x31022000U + n * 0x80U;
51 }
52 // n is in [10..17] for the SPORT4..7 channels. DMA10 sits
53 // at 0x31024000 (the 0x31023xxx block holds MDMA0, the
54 // DMA8/9 pair). Channels are 0x80 apart as usual.
55 return 0x31024000U + (n - 10U) * 0x80U;

magic numbers!

68 if (a >= 0x00240000U && a <= 0x0039FFFFU) {
69 a += 0x28000000U;

magic numbers

adsp2156/common/dma.h
-----------

14 #ifndef BLINK_DMA_H

not just blink ...

adsp2156/common/gpio.h
-------------------

77 // PORT pins this sets DIR=1 and FER=0 (GPIO mode); for DAI
78 // pin buffers this sets the group-D source to LOW and the
79 // group-F enable to static HIGH.

that's an implementation detail, not needed for mere users of this function.
likewise in descriptions of other functions in this file.

79 // Calls with an unimplemented
80 // pin index are silently ignored.
81 void gpio_make_output(enum gpio_pin pin);

BAD IDEA! should FAIL LOUD AND CLEAR with an assertion first line of the
function

adsp2156/common/gpio.c
--------------

5 // Two distinct hardware flavors sit behind one API:
6 //
7 // PORT pins (PA, PB, PC). Each port has its own block of
8 // MMRs at PORT_BASE + 0x00..0x48: FER (function enable,
9 // clear for GPIO mode), DIR (direction, 1=output), DATA
10 // (driven value), INEN (input enable, set for input mode).
11 // One bit per pin inside each register.
12 //
13 // DAI pin buffers (DAI0_PB01..12,19,20 and DAI1_PB01..12,
14 // 19,20). Each pad has a 7-bit group-D source code (set to
15 // static HIGH or LOW to drive it from software) and a
16 // 6-bit group-F enable code (static HIGH turns the output
17 // driver on, LOW tri-states it so the pad reads external).
18 // The 20-bit DAI_PIN_STAT register returns the live signal
19 // level on every DAI pad regardless of direction.
20 //
21 // Pins not implemented on 21569 (PB13..PB18 on each DAI) are
22 // left out of gpio_pin_t entirely; calls to unknown enum
23 // values fall through the pin_info lookup table's size check
24 // and are silently ignored.

haven't you just explained all of this in .h files? things that are in .h files
don't need to be replicated in the corresponding .c file, since everyone reads
the header first and then implementation. If this is not explained in the
header, then maybe move information to the functions where it is used.

29 // ----- PORT block register offsets --------------------------

missing all HRM references?

41 // The three PORT blocks live 0x80 apart starting at PORTA.

same?

121 static const struct pin_info pin_info[GPIO_PIN_COUNT] = {

same?

adsp2156/common/link.ldf
-----------------

13 // ADSP-21569 memory map (byte addresses).
14 // The IVT must live in PM RAM at 0x90000 with width 48 (NW
15 // instructions). L1 SRAM blocks are byte-addressed; L2 sits
16 // at 0x20000000 and is naturally visible to peripheral DMA
17 // without the SHARC+ L1 MP alias dance.

some of these numbers in the comment appear to be duplicated below, in the code.
comments should never just blindly duplicate the code

73 // Code. cc21k -swc emits everything into seg_swco; the
74 // sections list also grabs seg_pmco as a fallback for any
75 // 48-bit assembly instructions we might add later.

don't want any toolchain-specific details polluting the source code! DO NOT
MENTION CC21K OR ANY OTHER PROPRIETARY OR NON-PROPRIETARY toolchain. We write
portable code here.

adsp2156/common/printf.h

8 // somewhere. The uart demo wires it to uart_putc.

who cares what the demo does!?

adsp2156/common/regs.h
-------------

I don't understand this file: sometimes these kinds of constants are declared in
individual driver .c or .h files, sometimes they're magic numbers dropped
randomly, sometimes they are in reg.h? Make things A LOT more systematic and
decide where they'll be placed (probably here I'd say? and then all drivers
include reg.h?).

17 #ifndef BLINK_REGS_H

more than just blink ...

20 // All numeric defines below are written without the `U` suffix
21 // so the SHARC+ assembler (easm21k), which doesn't accept C-style
22 // integer suffixes, can include this header alongside the C
23 // compiler. The C-only helpers (stdint pull-in, MMR macros) are
24 // gated behind __ASSEMBLY__ which the asm builds set in ASFLAGS.
25
26 #ifndef __ASSEMBLY__
27 #include <stdint.h>
28 #define MMR(addr) (*(volatile uint32_t *)(addr))
29 #define MMR16(addr) (*(volatile uint16_t *)(addr))
30 #endif

this is only two #defines. why not just move them to the startup assembly file
directly? but if they are used outside just assembly, I don't understand why
you're talking about some "U suffix" --- the code in this block has no U
suffices and no numbers, it's just a volatile cast! (I've seen a lot of MMR()
usage in .c files, so is this assembly only or not???)

adsp2156/common/sport.h
---------------

14 // The loopback demos in sport/main.c set those up themselves
15 // based on which SPORT and which routing they want.

don't mention demos here, this is a general driver!o

adsp2156/common/sport.c
------------------------

6 // Every bit the driver
7 // touches is named in regs.h with a page reference.

obviously! no need to mention it here.

adsp2156/common/startup.s
---------------

17 // ============================================================
18 // Interrupt Vector Table
19 // ============================================================

missing HRM references?

29 // slot 0 (0x00): emulator interrupt --- unused
30 NOP; NOP; NOP; NOP;
31
32 // slot 1 (0x04): reset --- absolute jump to start. NOP delay slot
33 // then JUMP, then NOPs to fill the 4-instruction slot.
34 NOP;
35 JUMP start;
36 NOP;
37 NOP;
38
39 // The remaining slots are unused. Fill them with RTIs so any
40 // spurious interrupt simply returns. The IVT region in the LDF
41 // stops at 0x900a7 = 168 bytes = 7 slots after reset, which is
42 // all we are going to populate.
43 RTI; RTI; RTI; RTI; // 0x08
44 RTI; RTI; RTI; RTI; // 0x0c
45 RTI; RTI; RTI; RTI; // 0x10
46 RTI; RTI; RTI; RTI; // 0x14
47 RTI; RTI; RTI; RTI; // 0x18

spurious interrupts SHOULD NOT HAPPEN! they should print a diagnostic
message and lockup the system so we catch early failure

adsp2156/gpio/main.c
-------------

14 // These are the same three pairs that the SPORT4 external
15 // loopback test will use for data / clock / frame sync, so
16 // this is the pre-check that validates the physical wiring
17 // before we trust the SPORT driver.

don't like unnecessary coupling between examples, remove this

23 // Note: DAI1_PB01 is tied to an on-SOM MEMS microphone's DATA
24 // output on this board, so writing it from GPIO contends with
25 // the mic driver. PB01 is still included in the test because
26 // we want to observe exactly that symptom -- it should show up
27 // as a steady FAIL on channel 1 while channels 2 and 3 pass.

is that true? I thought you said it's not actually true, and there was some
other issue---which you have fixed.

53 static void busy_delay_ms(uint32_t ms)
54 {
55 uint32_t iters = BOARD_DELAY_MS(ms);
56 for (volatile uint32_t i = 0; i < iters; i++) { }
57 }

replace with the common delay_ms implemented in some common/ file

adsp2156/gpio/Makefile
-----------

5 TARGET = gpio

this line can be eliminated from all demo makefiles if we just call all binaries
`main` or something like that

adsp2156/sport/main.c
------------

69 static void sru_loopback_sport0(void)
96 static void sru_enable_sport4_tx_pins(void)

these functions do what looks like raw register shuffling. sport.c/.h should
provide a more ergonomic interface to configure the sports.

adsp2156/sport_dma/main.c
---------------

100 static const struct sport_setup setup[N_SPORTS] = {
101 // SPORT0..2 on DAI0. Clock into CLK0 IN1/3/5, frame sync
102 // into FS0 IN1/3/5, data into DAT0/1/2 IN2/1/0.
103 { SPORT_ID_0, DMA_CH_SPORT0_A, DMA_CH_SPORT0_B,
104 0x310C90C0U, 5U, 0x14U,
105 0x310C9140U, 5U, 0x14U,
106 0x310C9100U, 12U, 0x14U },
107 { SPORT_ID_1, DMA_CH_SPORT1_A, DMA_CH_SPORT1_B,
108 0x310C90C0U, 15U, 0x16U,
109 0x310C9140U, 15U, 0x16U,
110 0x310C9104U, 6U, 0x18U },
111 { SPORT_ID_2, DMA_CH_SPORT2_A, DMA_CH_SPORT2_B,
112 0x310C90C0U, 25U, 0x18U,

what are all these numbers!? shouldn't sport and/or dma drivers provide a more
ergonomic solution?

239 // Both DAI pad input-enable registers reset to 0, which
240 // isolates the pin buffers and also breaks the SRU data
241 // crossbar for that DAI. Enable all 20 inputs on each DAI.
242 MMR(0x31004460U) = 0x000FFFFFU; // PADS0_DAI0_IE
243 MMR(0x31004464U) = 0x000FFFFFU; // PADS0_DAI1_IE

again, this looks like it belongs in some driver, not main() !
