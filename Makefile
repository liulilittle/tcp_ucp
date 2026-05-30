# ----------------------------------------------------------------------------
# Kernel Module Makefile - tcp_ucp
# Version: 1.0
# Supports: build, clean, install, uninstall, load, unload, modload, sign, status, dkms-help
# ----------------------------------------------------------------------------

MODULE_NAME := tcp_ucp
obj-m := $(MODULE_NAME).o
VERSION ?= 1.0

# --------------------------------------------------------------------------
# Configuration (may be overridden on command line or in environment)
# --------------------------------------------------------------------------
KERNELDIR       ?= /lib/modules/$(shell uname -r)/build
INSTALL_MOD_DIR ?= extra
INSTALL_MOD_PATH ?=

# Signing support for Secure Boot
SIGN_KEY  ?=
SIGN_CERT ?=
SIGN_ALG  ?= sha512
SIGN_FILE ?= $(KERNELDIR)/scripts/sign-file

# rmmod flags (use RMFLAGS=-f to force unload a busy module)
RMFLAGS ?=

# --------------------------------------------------------------------------
# Resolve kernel release safely
# --------------------------------------------------------------------------
KERNEL_RELEASE := $(shell \
    if [ -d "$(KERNELDIR)" ]; then \
        "$(MAKE)" -s -C "$(KERNELDIR)" kernelrelease 2>/dev/null; \
    fi)
ifeq ($(KERNEL_RELEASE),)
  KERNEL_RELEASE := $(shell uname -r)
  $(warning Kernel release from build tree empty or failed, using running kernel $(KERNEL_RELEASE))
endif

# --------------------------------------------------------------------------
# Helper: find the locally built module file (may be .ko, .ko.gz, .ko.xz, etc.)
# This macro sets the shell variable LOCAL_MOD to the first matching file.
# Used in load / sign targets.
# --------------------------------------------------------------------------
define find_local_module
	LOCAL_MOD=$$(set -- $(MODULE_NAME).ko*; \
		for f in "$$@"; do \
			if [ -f "$$f" ]; then \
				echo "$$f"; break; \
			fi; \
		done)
endef

# --------------------------------------------------------------------------
# Phony targets
# --------------------------------------------------------------------------
.PHONY: all build clean help install uninstall load unload modload sign status dkms-help

# --------------------------------------------------------------------------
# Default target – build the module
# --------------------------------------------------------------------------
all:
	@if [ ! -d "$(KERNELDIR)" ]; then \
		echo "Error: Kernel build tree '$(KERNELDIR)' does not exist."; \
		exit 1; \
	fi
	@$(MAKE) -C "$(KERNELDIR)" M="$(CURDIR)" modules

build: all

# --------------------------------------------------------------------------
# Clean
# --------------------------------------------------------------------------
clean:
	@if [ ! -d "$(KERNELDIR)" ]; then \
		echo "Warning: Kernel build tree '$(KERNELDIR)' missing, cleaning local artifacts only."; \
		rm -f *.o *.ko *.ko.gz *.ko.xz *.mod.c *.mod *.mod.o *.order \
		      Module.symvers modules.order .*.cmd .*.d; \
	else \
		$(MAKE) -C "$(KERNELDIR)" M="$(CURDIR)" clean; \
	fi
	@rm -f Module.symvers modules.order

# --------------------------------------------------------------------------
# Help
# --------------------------------------------------------------------------
help:
	@echo "Available targets:"
	@echo "  all               - Build the kernel module"
	@echo "  clean             - Remove all build artifacts"
	@echo "  install           - Build, install and (optionally) sign module (root)"
	@echo "  uninstall         - Remove installed module for current kernel (root)"
	@echo "  load              - Load module using insmod (dev, no dep resolution)"
	@echo "  unload            - Unload module using rmmod (root)"
	@echo "  modload           - Load module using modprobe (resolves deps, root)"
	@echo "  sign              - Sign the local module file (requires SIGN_KEY/CERT)"
	@echo "  status            - Show module info (loaded / installed)"
	@echo "  dkms-help         - Show hints for DKMS integration"
	@echo "  help              - This message"
	@echo ""
	@echo "Key variables (override with make VAR=value):"
	@echo "  VERSION           = $(VERSION)"
	@echo "  KERNELDIR         = $(KERNELDIR)"
	@echo "  KERNEL_RELEASE    = $(KERNEL_RELEASE)"
	@echo "  INSTALL_MOD_DIR   = $(INSTALL_MOD_DIR)"
	@echo "  INSTALL_MOD_PATH  = $(if $(INSTALL_MOD_PATH),$(INSTALL_MOD_PATH),<root>)"
	@echo "  SIGN_KEY          = $(if $(SIGN_KEY),$(SIGN_KEY),<not set>)"
	@echo "  SIGN_CERT         = $(if $(SIGN_CERT),$(SIGN_CERT),<not set>)"
	@echo "  SIGN_ALG          = $(SIGN_ALG)"
	@echo "  RMFLAGS           = $(if $(RMFLAGS),$(RMFLAGS),<none>)"

# --------------------------------------------------------------------------
# Installation (build, install, optionally sign, depmod)
# --------------------------------------------------------------------------
install: all
	@if [ "$$(id -u)" -ne 0 ]; then \
		echo "Error: 'make install' requires root privileges. Use sudo."; \
		exit 1; \
	fi
	@set -e; \
	echo "Installing module..."; \
	$(MAKE) -C "$(KERNELDIR)" M="$(CURDIR)" modules_install \
		INSTALL_MOD_DIR="$(INSTALL_MOD_DIR)" \
		INSTALL_MOD_PATH="$(INSTALL_MOD_PATH)"; \
	target_dir="$(INSTALL_MOD_PATH)/lib/modules/$(KERNEL_RELEASE)/$(INSTALL_MOD_DIR)"; \
	if [ ! -d "$$target_dir" ]; then \
		echo "Error: module install directory not found: $$target_dir"; exit 1; \
	fi; \
	if [ -n "$(SIGN_KEY)" ] && [ -n "$(SIGN_CERT)" ]; then \
		echo "Signing installed module(s) with custom key..."; \
		if [ ! -f "$(SIGN_FILE)" ]; then \
			echo "Error: sign-file tool not found at $(SIGN_FILE)."; exit 1; \
		fi; \
		if [ ! -f "$(SIGN_KEY)" ]; then \
			echo "Error: SIGN_KEY file '$(SIGN_KEY)' not found."; exit 1; \
		fi; \
		if [ ! -f "$(SIGN_CERT)" ]; then \
			echo "Error: SIGN_CERT file '$(SIGN_CERT)' not found."; exit 1; \
		fi; \
		signed=0; \
		for f in "$$target_dir/$(MODULE_NAME).ko"*; do \
			if [ -f "$$f" ]; then \
				echo "  Signing $$f"; \
				"$(SIGN_FILE)" "$(SIGN_ALG)" "$(SIGN_KEY)" "$(SIGN_CERT)" "$$f"; \
				signed=1; \
			fi; \
		done; \
		if [ "$$signed" -eq 0 ]; then \
			echo "Error: no installed module file found matching $(MODULE_NAME).ko* in $$target_dir"; \
			exit 1; \
		fi; \
		echo "Installed module(s) signed successfully with custom key."; \
	else \
		echo "SIGN_KEY or SIGN_CERT not set -- installed module will not be custom signed."; \
		if [ -d /sys/firmware/efi ]; then \
			echo "Hint: Secure Boot may prevent loading an unsigned module."; \
		fi; \
	fi; \
	depmod $(if $(INSTALL_MOD_PATH), -b "$(INSTALL_MOD_PATH)") $(KERNEL_RELEASE); \
	echo "Module installed for kernel $(KERNEL_RELEASE)."; \
	echo "You can now load it with: sudo modprobe $(MODULE_NAME)"

# --------------------------------------------------------------------------
# Uninstallation
# --------------------------------------------------------------------------
uninstall:
	@if [ "$$(id -u)" -ne 0 ]; then \
		echo "Error: 'make uninstall' requires root privileges. Use sudo."; \
		exit 1; \
	fi
	-rmmod $(RMFLAGS) "$(MODULE_NAME)" 2>/dev/null
	-find "$(INSTALL_MOD_PATH)/lib/modules/$(KERNEL_RELEASE)" \
		-type f -name "$(MODULE_NAME).ko*" -delete 2>/dev/null
	depmod $(if $(INSTALL_MOD_PATH), -b "$(INSTALL_MOD_PATH)") $(KERNEL_RELEASE)
	@echo "Module removed for kernel $(KERNEL_RELEASE)."

# --------------------------------------------------------------------------
# Load using insmod (development / no dependency resolution)
# --------------------------------------------------------------------------
load: all
	@if [ "$$(id -u)" -ne 0 ]; then \
		echo "Error: 'make load' requires root privileges. Use sudo."; \
		exit 1; \
	fi
	@set -e; \
	if lsmod | grep -qw "$(MODULE_NAME)"; then \
		echo "Module $(MODULE_NAME) is already loaded, removing..."; \
		rmmod $(RMFLAGS) "$(MODULE_NAME)" || { echo "Failed to remove existing module"; exit 1; }; \
	fi; \
	$(find_local_module); \
	if [ -z "$$LOCAL_MOD" ]; then \
		echo "Error: no built module file found ($(MODULE_NAME).ko*). Run 'make' first."; \
		exit 1; \
	fi; \
	echo "Found local module: $$LOCAL_MOD"; \
	if insmod "$$LOCAL_MOD"; then \
		echo "Module loaded via insmod."; \
	else \
		echo "If dependency errors occur, try 'make modload' which uses modprobe."; \
		exit 1; \
	fi

# --------------------------------------------------------------------------
# Unload using rmmod
# --------------------------------------------------------------------------
unload:
	@if [ "$$(id -u)" -ne 0 ]; then \
		echo "Error: 'make unload' requires root privileges. Use sudo."; \
		exit 1; \
	fi
	@if lsmod | grep -qw "$(MODULE_NAME)"; then \
		rmmod $(RMFLAGS) "$(MODULE_NAME)" && echo "Module unloaded." || { echo "Failed to unload module"; exit 1; }; \
	else \
		echo "Module $(MODULE_NAME) is not loaded."; \
	fi

# --------------------------------------------------------------------------
# Load using modprobe (respects dependencies, requires prior install)
# --------------------------------------------------------------------------
modload:
	@if [ "$$(id -u)" -ne 0 ]; then \
		echo "Error: 'make modload' requires root privileges. Use sudo."; \
		exit 1; \
	fi
	@if ! modprobe --dry-run "$(MODULE_NAME)" >/dev/null 2>&1; then \
		echo "Module $(MODULE_NAME) not found by modprobe. Did you run 'make install'?"; \
		exit 1; \
	fi
	-modprobe -r "$(MODULE_NAME)" 2>/dev/null
	modprobe "$(MODULE_NAME)" && echo "Module loaded via modprobe."

# --------------------------------------------------------------------------
# Sign the local module file(s) (for testing / manual signing)
# --------------------------------------------------------------------------
sign: all
	@if [ -z "$(SIGN_KEY)" ] || [ -z "$(SIGN_CERT)" ]; then \
		echo "Error: SIGN_KEY and SIGN_CERT must be set to sign the module."; \
		exit 1; \
	fi
	@if [ ! -f "$(SIGN_FILE)" ]; then \
		echo "Error: sign-file not found at $(SIGN_FILE). Check KERNELDIR."; \
		exit 1; \
	fi
	@if [ ! -f "$(SIGN_KEY)" ]; then \
		echo "Error: SIGN_KEY file '$(SIGN_KEY)' not found."; \
		exit 1; \
	fi
	@if [ ! -f "$(SIGN_CERT)" ]; then \
		echo "Error: SIGN_CERT file '$(SIGN_CERT)' not found."; \
		exit 1; \
	fi
	@set -e; \
	$(find_local_module); \
	if [ -z "$$LOCAL_MOD" ]; then \
		echo "Error: no built module file found ($(MODULE_NAME).ko*). Build first."; \
		exit 1; \
	fi; \
	for f in $(MODULE_NAME).ko*; do \
		if [ -f "$$f" ]; then \
			echo "  Signing $$f"; \
			"$(SIGN_FILE)" "$(SIGN_ALG)" "$(SIGN_KEY)" "$(SIGN_CERT)" "$$f"; \
		fi; \
	done; \
	echo "Local module file(s) signed."

# --------------------------------------------------------------------------
# Module status
# --------------------------------------------------------------------------
status:
	@if lsmod | grep -qw "$(MODULE_NAME)"; then \
		echo "Module $(MODULE_NAME) is loaded."; \
		if command -v modinfo >/dev/null 2>&1; then \
			modinfo "$(MODULE_NAME)" | head -n 10; \
		else \
			echo "modinfo not available."; \
		fi; \
	elif command -v modinfo >/dev/null 2>&1 && modinfo "$(MODULE_NAME)" >/dev/null 2>&1; then \
		echo "Module $(MODULE_NAME) is installed but not loaded."; \
		modinfo "$(MODULE_NAME)" | head -n 10; \
	else \
		echo "Module $(MODULE_NAME) is not installed."; \
	fi

# --------------------------------------------------------------------------
# DKMS integration hints
# --------------------------------------------------------------------------
dkms-help:
	@echo "To use this module with DKMS, create a dkms.conf file in the source directory:"
	@echo "  PACKAGE_VERSION=$(VERSION)"
	@echo "  PACKAGE_NAME=$(MODULE_NAME)"
	@echo "  BUILT_MODULE_NAME[0]=$(MODULE_NAME)"
	@echo "  DEST_MODULE_LOCATION[0]=/$(INSTALL_MOD_DIR)   # e.g. /extra or /updates"
	@echo "  AUTOINSTALL=yes"
	@echo "Then:"
	@echo "  sudo cp -r . /usr/src/$(MODULE_NAME)-$(VERSION)"
	@echo "  sudo dkms add -m $(MODULE_NAME) -v $(VERSION)"
	@echo "  sudo dkms build -m $(MODULE_NAME) -v $(VERSION)"
	@echo "  sudo dkms install -m $(MODULE_NAME) -v $(VERSION)"