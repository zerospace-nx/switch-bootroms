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

# Add new Core modules here
include Makefile.corelibs

NVBOOT_LIBS := $(patsubst %, $(OUTLIBS)/lib$(PROJECT)_nvboot_%.a, $(CORELIB))

TARGETS=$(NVBOOT_LIBS)
.PHONY: all clean make

TARGETS: sdram_source
	@for i in $(CORELIB); do if [ ! -f $$i/Makefile ]; \
	    then echo "ERROR: Need 'make make' prior to 'make'."; exit 1; fi; done
	@for i in $(CORELIB); do $(MAKE) -C $$i || exit 1; done

make: $(BR_ROOT)/make/Makefile.in
	@for i in $(CORELIB); do printf "#####! DO NOT EDIT !#####\n\n" > $$i/Makefile; \
	    cat $< >>$$i/Makefile ; done

# Integrate MC code everytime we build.
sdram_source:

all: $(TARGETS)

clean:
	@for i in $(CORELIB); do if [ ! -f $$i/Makefile ]; \
	    then echo "ERROR: Need 'make make' prior to 'make'."; exit 1; fi; done
	@for i in $(CORELIB); do $(MAKE) -C $$i $@ || exit 1; \
	    rm -f $(OUTLIBS)/lib$(PROJECT)_nvboot_$$i.a; done

doxy:
	$(BR_ROOT)/tools/doxygen/bin/doxygen $(BR_ROOT)/tools/doxygen/br_config

