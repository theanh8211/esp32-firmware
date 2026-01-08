PLATFORMIO_IMG := smartfarm-platformio:latest
PROJECT := $(CURDIR)
DEV := /dev/ttyUSB0
UPLOAD_PORT ?= $(DEV)
VENV_PY ?= $(abspath $(CURDIR)/../.venv/bin/python)
ESPLIB := $(VENV_PY) -m esptool
BOARD ?= esp32dev
TAG_PREFIX ?= SMFVN
DATE := $(shell date +%Y%m%d)
# GIT_SHORT := $(shell git rev-parse --short HEAD 2>/dev/null || echo nogit)

# This lets `make build upload-firmware` use the firmware's FIRMWARE_VERSION
VERSION ?= $(shell grep -E '#define[[:space:]]+FIRMWARE_VERSION' $(PROJECT)/src/config.h 2>/dev/null | sed -E 's/.*"([^\"]+)".*/\1/' | head -n1 || echo unknown)

BUILD_BIN := $(PROJECT)/.pio/build/$(BOARD)/firmware.bin
# use deferred expansion so VERSION is evaluated at use time (after parsing)
OUT_NAME = $(TAG_PREFIX)_$(BOARD)_$(VERSION)_$(DATE).bin
OUT := $(PROJECT)/binary/$(OUT_NAME)

# Docker build context for the platformio image
PLATFORMIO_DOCKER_CONTEXT := $(abspath $(CURDIR)/../docker/platformio)

# If OFFLINE=1 is passed to make, PlatformIO will run in offline mode
ifdef OFFLINE
PIO_OPTS += --offline
endif

# Host cache directories to speed up repeated runs (change if desired)
PLATFORMIO_CACHE ?= $(HOME)/.platformio
PIP_CACHE ?= $(HOME)/.cache/pip


.PHONY: build upload flash monitor clean

build:
	$(MAKE) docker-image
	sudo docker run --rm \
		-v $(PROJECT):/project/esp32-firmware \
		-v $(PLATFORMIO_CACHE):/root/.platformio \
		-v $(PIP_CACHE):/root/.cache/pip \
		-w /project/esp32-firmware $(PLATFORMIO_IMG) platformio run $(PIO_OPTS) -e esp32dev

upload:
	$(MAKE) docker-image
	sudo docker run --rm --device=$(UPLOAD_PORT) \
		-v $(PROJECT):/project/esp32-firmware \
		-v $(PLATFORMIO_CACHE):/root/.platformio \
		-v $(PIP_CACHE):/root/.cache/pip \
		-w /project/esp32-firmware $(PLATFORMIO_IMG) platformio run $(PIO_OPTS) -e esp32dev -t upload --upload-port $(UPLOAD_PORT)

flash: build
	$(MAKE) docker-image
	# Override container entrypoint to run esptool via python3 so the container
	# executes the esptool module directly (avoids PlatformIO entrypoint interpreting args)
	sudo docker run --rm --entrypoint python3 --device=$(UPLOAD_PORT) \
		-v $(PROJECT):/project/esp32-firmware \
		-v $(PLATFORMIO_CACHE):/root/.platformio \
		-v $(PIP_CACHE):/root/.cache/pip \
		-w /project/esp32-firmware $(PLATFORMIO_IMG) \
		/root/.platformio/packages/tool-esptoolpy/esptool.py --chip esp32 --port $(UPLOAD_PORT) --baud 460800 write_flash \
		0x1000 /project/esp32-firmware/.pio/build/$(BOARD)/bootloader.bin \
		0x8000 /project/esp32-firmware/.pio/build/$(BOARD)/partitions.bin \
		0x10000 /project/esp32-firmware/.pio/build/$(BOARD)/firmware.bin

.PHONY: package upload-firmware

package: build
	@echo "Packaging firmware for VERSION=$(VERSION)"
	@mkdir -p $(PROJECT)/binary
	# use top-level make variables DATE,GIT_SHORT,BUILD_BIN,OUT
	@echo "Looking for $(BUILD_BIN)"
	@if [ -f "$(BUILD_BIN)" ]; then \
		cp "$(BUILD_BIN)" "$(OUT)"; \
		echo "Created $(OUT)"; \
	else \
		echo "Firmware binary not found at $(BUILD_BIN)"; exit 1; \
	fi

upload-firmware: package
	@echo "Firmware placed at $(OUT)"

monitor:
	python3 scripts/monitor.py $(UPLOAD_PORT) 115200

clean:
	rm -rf .pio build .pioignore

.PHONY: docker-image
docker-image:
	@if ! docker image inspect $(PLATFORMIO_IMG) >/dev/null 2>&1; then \
		echo "Building $(PLATFORMIO_IMG) image..."; \
		docker build -t $(PLATFORMIO_IMG) $(PLATFORMIO_DOCKER_CONTEXT); \
	else \
		echo "$(PLATFORMIO_IMG) already exists, skipping build."; \
	fi
