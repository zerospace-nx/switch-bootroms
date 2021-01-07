#
# Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
# 
# NVIDIA Corporation and its licensors retain all intellectual property
# and proprietary rights in and to this software and related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA Corporation is strictly prohibited.
#

DEPTH 	:= $(shell depth)
BR_ROOT	:= $(DEPTH)/drv/bootrom

include $(BR_ROOT)/make/Makefile.tools
include $(BR_ROOT)/make/Makefile.defs
include $(BR_ROOT)/make/Makefile.rules

# Get list of Core and IO libraries to link.
include core/Makefile.corelibs
include io/Makefile.iolibs

NVBOOT_AXF := boot_rom.irom.axf
NVBOOT_LD := $(BR_ROOT)/ldscripts/nvboot.ld

NVBOOT_LIBS := $(patsubst %, $(OUTLIBS)/lib$(PROJECT)_nvboot_%.a, $(CORELIB))
NVBOOT_LIBS += $(patsubst %, $(OUTLIBS)/lib$(PROJECT)_nvboot_%.a, $(IOLIB))

TARGETS=$(OUTBINS)/$(NVBOOT_AXF)
.PHONY: all clean core-libs io-libs make

$(TARGETS): core-libs io-libs
	@if [ ! -d $(OUTBINS) ]; then mkdir -p $(OUTBINS); fi
	@echo "  LINK  " $@
	@$(CC) $(NVBOOT_LDFLAGS) -T $(NVBOOT_LD) -Wl,-Map=$(@:axf=map) core/startup/$(PROJECT)/start.o \
		-Wl,--start-group $(NVBOOT_LIBS) -lgcc -Wl,--end-group \
		-o $@
	@echo "  GEN   " $(@:axf=axf.size)
	@$(NM) -S --size-sort $@ > $(@:axf=axf.size)
	@echo "  GEN   " $(@:axf=dis)
	@$(DMP) -Mreg-names-raw -d $@ > $(@:axf=dis)
	@echo "  SYM   " $(@:axf=sym)
	@$(DMP) -t $@ > $(@:axf=sym)
	@$(BR_ROOT)/nvboot/region_check_gcc $(@:axf=dis)
	@echo "  BIN   " $(@:axf=bin)
	@$(BIN) $@ $(@:axf=bin)
	dd if=$(@:axf=bin) bs=92k conv=sync of=$(@:axf=bin).92k_padded
	@$(SIZE) -t $(NVBOOT_LIBS) | sed -e 's:ex ./../../../drv/bootrom/libnvboot/::'
	@$(SIZE) $@

core-libs:
	$(MAKE) -C core

io-libs:
	$(MAKE) -C io

make: $(BR_ROOT)/make/Makefile.in
	$(MAKE) -C core make
	$(MAKE) -C io make

all: $(TARGETS)

clean:
	$(MAKE) -C core clean
	$(MAKE) -C io clean
	@rm -rf $(OUTLIBS) $(OUTBINS)

doxy:
	$(BR_ROOT)/tools/doxygen/bin/doxygen $(BR_ROOT)/tools/doxygen/br_config
