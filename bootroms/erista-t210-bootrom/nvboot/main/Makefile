#!/usr/bin/make

#
# Copyright (c) 2007 - 2012 NVIDIA Corporation.  All rights reserved.
# 
# NVIDIA Corporation and its licensors retain all intellectual property
# and proprietary rights in and to this software and related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA Corporation is strictly prohibited.
#

DEPTH   := $(shell depth)
BR_ROOT := $(DEPTH)/drv/bootrom

# build PROJECTS
PROJLIST = $(PROJECTS)

define PROJDEFS

BR_IMAGES += boot_rom

BR_LOCATION__LIST := irom iram_plus_64k

ifndef BR_LOCATION
BR_LOCATION = irom
endif

ifeq (,$(filter $(BR_LOCATION),$(BR_LOCATION__LIST)))
  $(warning export BR_LOCATION= values: $(BR_LOCATION__LIST))
  $(error BOOTLOADER_MEMORY=$(BOOTLOADER_MEMORY) set to invalid value)
endif

BR_LOCATION_UC = $(shell echo $(BR_LOCATION) | tr a-z A-Z)
$(warning BR_LOCATION_UC is $(BR_LOCATION_UC))

# build objects; then build final binary images
TARGETS = $(OBJECTS) lib$(PROJECT)_nvboot_testmain$(ALIBEXT)
ifneq ($(TARGET_INST),arm)
  LTARGETS = $(BR_IMAGES:%=$(OBJDIR)/%.$(BR_LOCATION).bin)
endif

CFILES += boot0c.c
CFILES += boot0test.c
CFILES += nvboot_main.c

SSFILES += boot0.ss
SSFILES += init_clib.ss

SCATTER_SOURCE := ../scatter.irom.txt
SCATTER_FILE := scatter.$(BR_LOCATION).txt.e

LCFLAGS += -DBR_LOCATION_IS_$(BR_LOCATION_UC)

LLDFLAGS = --libpath $(ARMLIBPATH) --symbols --info totals --list $(@:.bin=).list
LLDFLAGS += --scatter $(SCATTER_FILE)
LLDFLAGS += --verbose --errors errors.txt

LASFLAGS += --apcs /interwork

include	$(BR_ROOT)/make/Makefile.localdefs

# Build the list of libraries needed during linking.
#BOOT_LIB_LIST := aes
BOOT_LIB_LIST := arc
BOOT_LIB_LIST += ahb
BOOT_LIB_LIST += bootloader
BOOT_LIB_LIST += clocks
BOOT_LIB_LIST += coldboot
BOOT_LIB_LIST += devmgr
BOOT_LIB_LIST += fuse
#BOOT_LIB_LIST += hash
#BOOT_LIB_LIST += i2c
BOOT_LIB_LIST += log
BOOT_LIB_LIST += pads
BOOT_LIB_LIST += pmc
BOOT_LIB_LIST += prod_uart
BOOT_LIB_LIST += rcm
BOOT_LIB_LIST += reset
BOOT_LIB_LIST += sata
BOOT_LIB_LIST += sdram
BOOT_LIB_LIST += sdmmc
BOOT_LIB_LIST += se
BOOT_LIB_LIST += sku
BOOT_LIB_LIST += snor
ifeq ($(NVENABLE_FOOS_SUPPORT),1)
BOOT_LIB_LIST += foos
endif
BOOT_LIB_LIST += ssk
ifeq ($(NVENABLE_XUSB_SUPPORT),1)
BOOT_LIB_LIST += usb3
endif
BOOT_LIB_LIST += spi_flash
BOOT_LIB_LIST += strap
BOOT_LIB_LIST += uart
BOOT_LIB_LIST += usbcharging
BOOT_LIB_LIST += usbf
BOOT_LIB_LIST += util
BOOT_LIB_LIST += warm_boot_0
BOOT_LIB_LIST += xusb_dev
BOOT_LIB_LIST += irom_patch
BOOT_LIB_LIST += wdt
endef

include $(DEPTH)/make/Common.make
include	$(BR_ROOT)/make/Makefile.localrules

define PROJRULES

AOBJ_1 = $(filter-out $(OBJDIR)/boot0c$(OBJEXT),$(OBJECTS))
AOBJECTS = $(filter-out $(OBJDIR)/init_clib$(OBJEXT),$(AOBJ_1))
BOBJECTS = $(filter-out $(OBJDIR)/boot0test$(OBJEXT),$(OBJECTS))

$(BR_IMAGES:%=$(OBJDIR)/%.$(BR_LOCATION).bin) : $(BOBJECTS)
	$(CC) $(LCINCPATH) $(CINCPATH) -DBR_LOCATION_IS_$(BR_LOCATION_UC) -E $(SCATTER_SOURCE) > $(SCATTER_FILE)
	$(LD) $(LDFLAGS) --no_remove $^ --entry __boot_start -o $(@:.bin=).axf
	$(BIN) $(@:.bin=).axf -o $@
	$(filter-out --bin,$(BIN)) $(@:.bin=).axf -c > $(@:.bin=).dis
	../region_chk $(@:.bin=).dis
	dd if=$@ bs=92k conv=sync of=$@.92k_padded
	$(INST) $(OBJDIR)/boot0$(OBJEXT) $(LIBDIR)
	$(INST) $(OBJDIR)/nvboot_main$(OBJEXT) $(LIBDIR)

#there used to be an installation of the scatter file 
#	$(INST) $(SCATTER_FILE) $(LIBDIR)

$(OBJDIR)/lib$(PROJECT)_nvboot_testmain$(ALIBEXT): Makefile $(AOBJECTS) $(LIBDIR)
	$(ALD) $(ALD_OUTPUT_OPTION) $(AOBJECTS) $(AFLAGSALL)
ifdef NV_BROWSE
	$(BSCMAKE) -o $(BSCNAME) $(patsubst %$(OBJEXT),%.sbr,$(AOBJECTS))
endif

#there used to be an installation of the scatter file 
#	@$(INSTMSG) "Installing $(SCATTER_FILE) -> $(LIBDIR)"; \

endef
