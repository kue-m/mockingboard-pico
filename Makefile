# Mockingboard Pico - single build entry point.
#
# Host targets (fast, no cross toolchain):
#   make / make test   Build + run the host unit suite with the coverage gate.
#   make sim           Alias for the host build + tests.
#
# Firmware targets (need the Pico SDK + arm-none-eabi-gcc + cmake, PICO_SDK_PATH set):
#   make prod          Release image into dist/mockingboard_pico.uf2.
#   make clean         Remove dist/ and the host build artifacts.

FW      := firmware
DIST    := dist
BUILD   := $(DIST)/build
SIM     := sim

# Override the cmake generator with GEN=Ninja if preferred.
GEN ?=
ifneq ($(GEN),)
GEN_ARG := -G "$(GEN)"
endif

.PHONY: all test sim prod clean help

all: test

test sim:
	bash $(SIM)/build.sh

prod:
	cmake -S $(FW) -B $(BUILD)/prod $(GEN_ARG)
	cmake --build $(BUILD)/prod
	cp $(BUILD)/prod/mockingboard_pico.uf2 $(DIST)/mockingboard_pico.uf2

clean:
	rm -rf $(DIST)
	rm -rf $(SIM)/build $(SIM)/host $(SIM)/test $(SIM)/test_cov $(SIM)/test_ar $(SIM)/test_ar_cov $(SIM)/out.wav $(SIM)/out.mp3

help:
	@sed -n '3,9p' $(firstword $(MAKEFILE_LIST))

-include Makefile.local
