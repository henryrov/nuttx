############################################################################
# arch/risc-v/src/Makefile
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(TOPDIR)/tools/Make.defs
include chip/Make.defs
ifeq ($(CONFIG_OPENSBI),y)
include opensbi/Make.defs
else ifeq ($(CONFIG_NUTTSBI),y)
include nuttsbi/Make.defs
endif

ARCH_SRCDIR = $(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src

INCLUDES += ${INCDIR_PREFIX}$(ARCH_SRCDIR)$(DELIM)chip
INCLUDES += ${INCDIR_PREFIX}$(ARCH_SRCDIR)$(DELIM)common
INCLUDES += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)sched

CPPFLAGS += $(INCLUDES)
CFLAGS += $(INCLUDES)
CXXFLAGS += $(INCLUDES)
AFLAGS += $(INCLUDES)

NUTTX = $(call CONVERT_PATH,$(TOPDIR)$(DELIM)nuttx$(EXEEXT))

# Additional rules for system call wrapper

ifeq ($(CONFIG_SCHED_INSTRUMENTATION_SYSCALL),y)
  EXTRALINKCMDS += @$(TOPDIR)/syscall/syscall_wraps.ldcmd
endif

# The "head" object

HEAD_OBJ = $(HEAD_ASRC:.S=$(OBJEXT))
STARTUP_OBJS ?= $(HEAD_OBJ)

# Flat build or kernel-mode objects

ASRCS = $(CHIP_ASRCS) $(CMN_ASRCS) $(SBI_ASRCS)
AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS = $(CHIP_CSRCS) $(CMN_CSRCS) $(SBI_CSRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

# User-mode objects

UASRCS = $(CHIP_UASRCS) $(CMN_UASRCS)
UAOBJS = $(UASRCS:.S=$(OBJEXT))

UCSRCS = $(CHIP_UCSRCS) $(CMN_UCSRCS)
UCOBJS = $(UCSRCS:.c=$(OBJEXT))

USRCS = $(UASRCS) $(UCSRCS)
UOBJS = $(UAOBJS) $(UCOBJS)

KBIN = libkarch$(LIBEXT)
BIN  = libarch$(LIBEXT)

$(foreach lib,$(notdir $(wildcard $(APPDIR)$(DELIM)staging$(DELIM)*$(LIBEXT))), \
  $(foreach elib,$(EXTRA_LIBS), \
    $(if $(filter $(notdir $(elib)),$(lib)), \
      $(eval NAMEFULL_LIBS+=$(elib)), \
      $(if $(filter $(notdir $(elib)),$(patsubst lib%$(LIBEXT),-l%,$(lib))), \
        $(eval NAMESPEC_LIBS+=$(elib)) \
       ) \
     ) \
   ) \
 )

EXTRA_LIBS := $(filter-out $(NAMEFULL_LIBS) $(NAMESPEC_LIBS),$(EXTRA_LIBS))
EXTRA_LIBS += $(wildcard $(APPDIR)$(DELIM)staging$(DELIM)*$(LIBEXT))

ARCHSCRIPT := $(call CONVERT_PATH,$(ARCHSCRIPT))
LDFLAGS += $(addprefix -T,$(addsuffix .tmp,$(ARCHSCRIPT))) $(EXTRALINKCMDS)

ifeq ($(LD),$(CC))
  ifeq ($(CONFIG_STACK_CANARIES),y)
    # filter out ssp(Stack Smashing Protector) related flags:
    #  -fstack-protector
    #  -fstack-protector-all
    #  -fstack-protector-strong
    #  -fstack-protector-explicit
    STRIPCFLAGS = $(filter -fstack-protector%,$(CFLAGS))
  endif
  LDENTRY      ?= -Wl,--entry=__start
  LDSTARTGROUP ?= -Wl,--start-group
  LDENDGROUP   ?= -Wl,--end-group
  LDFLAGS      := $(addprefix -Xlinker ,$(LDFLAGS))
  LDFLAGS      += $(filter-out $(STRIPCFLAGS),$(CFLAGS))
else
  LDENTRY      ?= --entry=__start
  LDSTARTGROUP ?= --start-group
  LDENDGROUP   ?= --end-group
endif

BOARDMAKE = $(if $(wildcard board$(DELIM)Makefile),y,)

LIBPATHS += -L $(call CONVERT_PATH,$(TOPDIR)$(DELIM)staging)
ifeq ($(BOARDMAKE),y)
  LIBPATHS += -L $(call CONVERT_PATH,$(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src$(DELIM)board)
endif

LDLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(LINKLIBS)))
ifeq ($(BOARDMAKE),y)
  LDLIBS += -lboard
endif

VPATH += chip
VPATH += common
VPATH += common/espressif
VPATH += $(SBI_DIR)
VPATH += $(ARCH_SUBDIR)
VPATH += $(CHIP_DIR)

all: $(HEAD_OBJ) $(BIN)

.PHONY: board/libboard$(LIBEXT)

$(AOBJS) $(UAOBJS) $(HEAD_OBJ): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(UCOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

crt0$(OBJEXT): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

ifeq ($(CONFIG_BUILD_FLAT),y)
$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))
else
$(BIN): $(UOBJS)
	$(call ARCHIVE, $@, $(UOBJS))
endif

$(KBIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

board/libboard$(LIBEXT):
	$(Q) $(MAKE) -C board libboard$(LIBEXT) EXTRAFLAGS="$(EXTRAFLAGS)"

# When multiple linking, these two additional linking objects will be included

ifeq ($(CONFIG_MM_KASAN_GLOBAL),y)
EXTRA_LIBS += kasan_globals$(OBJEXT)
endif
ifeq ($(CONFIG_ALLSYMS),y)
EXTRA_LIBS += allsyms$(OBJEXT)
endif

define LINK_ALLSYMS_KASAN
	$(if $(CONFIG_ALLSYMS),
	$(Q) $(TOPDIR)/tools/mkallsyms.py $(NUTTX) allsyms.tmp --orderbyname $(CONFIG_SYMTAB_ORDEREDBYNAME)
	$(Q) $(call COMPILE, allsyms.tmp, allsyms$(OBJEXT), -x c)
	$(Q) $(call DELFILE, allsyms.tmp))
	$(if $(CONFIG_MM_KASAN_GLOBAL),
	$(Q) $(TOPDIR)/tools/kasan_global.py -e $(NUTTX) -o kasan_globals.tmp -a $(CONFIG_MM_KASAN_GLOBAL_ALIGN)
	$(Q) $(call COMPILE, kasan_globals.tmp, kasan_globals$(OBJEXT) -fno-sanitize=kernel-address, -x c)
	$(Q) $(call DELFILE, kasan_globals.tmp))
	$(Q) $(LD) $(LDENTRY) $(LDFLAGS) $(LIBPATHS) $(EXTRA_LIBPATHS) \
		-o $(NUTTX) $(HEAD_OBJ) $(EXTRA_OBJS) \
		$(LDSTARTGROUP) $(LDLIBS) $(EXTRA_LIBS) $(LDENDGROUP)
endef

$(addsuffix .tmp,$(ARCHSCRIPT)): $(ARCHSCRIPT)
	$(call PREPROCESS, $(patsubst %.tmp,%,$@), $@)

nuttx$(EXEEXT): $(HEAD_OBJ) board/libboard$(LIBEXT) $(addsuffix .tmp,$(ARCHSCRIPT))
	$(Q) echo "LD: nuttx"
ifeq ($(CONFIG_ALLSYMS)$(CONFIG_MM_KASAN_GLOBAL),)
	$(Q) $(LD) $(LDENTRY) $(LDFLAGS) $(LIBPATHS) $(EXTRA_LIBPATHS) \
		-o $(NUTTX) $(HEAD_OBJ) $(EXTRA_OBJS) \
		$(LDSTARTGROUP) $(LDLIBS) $(EXTRA_LIBS) $(LDENDGROUP)
else
	$(Q) $(call LINK_ALLSYMS_KASAN)
	$(Q) $(call LINK_ALLSYMS_KASAN)
	$(Q) $(call LINK_ALLSYMS_KASAN)
	$(Q) $(call LINK_ALLSYMS_KASAN)
endif
ifeq ($(CONFIG_MM_KASAN_GLOBAL),y)
	$(Q) $(OBJCOPY) -R .kasan.global $(NUTTX)
	$(Q) $(OBJCOPY) -R .kasan.unused $(NUTTX)
endif
ifneq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) $(NM) $(NUTTX) | \
	grep -v '\(compiled\)\|\(\$(OBJEXT)$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
	sort > $(TOPDIR)/System.map
endif
	$(Q) $(call DELFILE, $(addsuffix .tmp,$(ARCHSCRIPT)))

# This is part of the top-level export target
# Note that there may not be a head object if layout is handled
# by the linker configuration.

export_startup: $(STARTUP_OBJS)
ifneq ($(STARTUP_OBJS),)
	$(Q) if [ -d "$(EXPORT_DIR)/startup" ]; then \
		cp -f $(STARTUP_OBJS) "$(EXPORT_DIR)/startup/."; \
	 else \
		echo "$(EXPORT_DIR)/startup does not exist"; \
	exit 1; \
	fi
endif

# Dependencies

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds) $(HEAD_ASRC:.S=.dds)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board depend
endif
	$(Q) $(MAKE) makedepfile DEPPATH="$(patsubst %,--dep-path %,$(subst :, ,$(VPATH)))"
	$(Q) touch $@

depend: .depend

context::

clean:
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board clean
endif
	$(call DELFILE, $(addsuffix .tmp,$(ARCHSCRIPT)))
	$(call DELFILE, $(KBIN))
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(HEAD_OBJ))
	$(call CLEAN)

distclean:: clean
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board distclean
endif
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
