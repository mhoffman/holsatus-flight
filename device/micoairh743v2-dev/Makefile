# Makefile for MicoAir H743
# Default: flash via UART bootloader (stm32flash). USB-C DFU is also
# available as 'make flash-usb' / 'make flash-usb-release'.
#
# Usage (debug build, UART):
#   make flash BIN=flight_diag
#   make flash BIN=flight_diag PORT=/dev/cu.usbserial-XXXXXXXX
#
# Usage (release build, UART):
#   make flash-release BIN=flight
#   make flash-release BIN=flight PORT=/dev/cu.usbserial-XXXXXXXX
#
# Before running 'make flash': hold BOOT0, power-cycle the FC, release BOOT0.
#
# Usage (USB-C DFU):
#   make flash-usb BIN=flight_diag
#   make flash-usb-release BIN=flight
#
# Before running 'make flash-usb': hold BOOT0, power-cycle the FC over
# USB-C, release BOOT0. Requires dfu-util.

BIN    ?= motor_pwm
PORT   ?= /dev/cu.usbserial-A5069RR4
BAUD   ?= 115200
VERIFY ?= 0

# STM32 system-bootloader USB DFU VID:PID (fixed by the ROM bootloader,
# not board-specific).
DFU_VID_PID ?= 0483:df11

# Output directory for objcopy'd .bin artefacts. Gitignored; keeps the
# device root clean instead of accumulating <bin>.bin files alongside source.
OUT_DIR := firmware
BIN_OUT := $(OUT_DIR)/$(BIN).bin

ifeq ($(VERIFY),1)
  VERIFY_FLAG := -v
else
  VERIFY_FLAG :=
endif

.PHONY: build build-release flash flash-release flash-usb flash-usb-release probe probe-usb port-free git-clean

# Paths that, if dirty or untracked, would make the flashed binary
# unreproducible. Kept explicit rather than "entire repo" because the top
# level contains many legitimately-uncommitted artefacts (flight logs,
# test binaries, SD card images) that have no effect on firmware.
# Relative to the git repo root, not the Makefile's cwd.
GIT_CRITICAL_PATHS := \
  device/micoairh743v2-dev/src \
  device/micoairh743v2-dev/build.rs \
  device/micoairh743v2-dev/Cargo.toml \
  device/micoairh743v2-dev/Cargo.lock \
  device/micoairh743v2-dev/Makefile \
  device/micoairh743v2-dev/memory.x \
  device/micoairh743v2-dev/.cargo \
  common/src \
  common/Cargo.toml \
  common/Cargo.lock

# Refuse to flash unless everything the firmware depends on is committed.
# This makes every log's `[free] git=<sha>` line unambiguously point to an
# exact reproducible revision. Override with ALLOW_DIRTY=1 only for debug
# iteration that you promise won't be correlated to a log file.
git-clean:
	@if [ "$(ALLOW_DIRTY)" = "1" ]; then \
	  echo "WARNING: ALLOW_DIRTY=1 -- skipping git cleanliness check"; \
	else \
	  REPO_ROOT=$$(git rev-parse --show-toplevel); \
	  DIRTY=$$(cd "$$REPO_ROOT" && git status --porcelain -- $(GIT_CRITICAL_PATHS)); \
	  if [ -n "$$DIRTY" ]; then \
	    echo "ERROR: firmware-critical paths have uncommitted changes:"; \
	    echo "$$DIRTY"; \
	    echo ""; \
	    echo "  Commit or stash before flashing so log.git_sha traces to an"; \
	    echo "  exact revision. Bypass only if you don't care about the log:"; \
	    echo "    make $(MAKECMDGOALS) ALLOW_DIRTY=1"; \
	    exit 1; \
	  fi; \
	  SHA=$$(cd "$$REPO_ROOT" && git rev-parse --short=10 HEAD); \
	  echo "Git tree clean -- firmware will embed sha=$$SHA"; \
	fi

# Fail fast with a friendly message if another process (usually miniterm) holds
# the serial port. Without this, stm32flash's "already open" error is easy to
# miss in CI-style scroll.
port-free:
	@if lsof -t $(PORT) > /dev/null 2>&1; then \
	  PIDS=$$(lsof -t $(PORT) | tr '\n' ' '); \
	  CMDS=$$(ps -o comm= -p $$PIDS 2>/dev/null | sort -u | tr '\n' ' '); \
	  echo "ERROR: $(PORT) is busy (pid: $$PIDS cmd: $$CMDS)"; \
	  echo "  -> stop miniterm first (Ctrl+] in that terminal), or: kill $$PIDS"; \
	  exit 1; \
	fi

probe: port-free
	@echo "Probing bootloader on $(PORT) at $(BAUD) baud..."
	@stm32flash -b $(BAUD) $(PORT) > /dev/null || { echo "ERROR: no bootloader on $(PORT) -- hold BOOT0 and power-cycle"; exit 1; }
	@echo "Bootloader OK"

probe-usb:
	@echo "Probing USB DFU device ($(DFU_VID_PID))..."
	@dfu-util -l 2>/dev/null | grep -q "$(DFU_VID_PID)" || { echo "ERROR: no DFU device found ($(DFU_VID_PID)) -- hold BOOT0 and power-cycle over USB-C"; exit 1; }
	@echo "DFU device OK"

build:
	@mkdir -p $(OUT_DIR)
	cargo build --bin $(BIN)
	cargo objcopy --bin $(BIN) -- -O binary $(BIN_OUT)

build-release:
	@mkdir -p $(OUT_DIR)
	cargo build --release --bin $(BIN)
	cargo objcopy --release --bin $(BIN) -- -O binary $(BIN_OUT)

flash: git-clean probe build
	stm32flash -b $(BAUD) -w $(BIN_OUT) $(VERIFY_FLAG) -g 0x08000000 $(PORT)

flash-release: git-clean probe build-release
	stm32flash -b $(BAUD) -w $(BIN_OUT) $(VERIFY_FLAG) -g 0x08000000 $(PORT)

flash-usb: git-clean probe-usb build
	dfu-util -a 0 -d $(DFU_VID_PID) -s 0x08000000:leave -D $(BIN_OUT)

flash-usb-release: git-clean probe-usb build-release
	dfu-util -a 0 -d $(DFU_VID_PID) -s 0x08000000:leave -D $(BIN_OUT)
