
ifneq ($(new)$(upd)$(upv),)
# make for new/upd sub module

SUB_NEW = $(addsuffix .new,$(new))
SUB_UPD = $(addsuffix .upd,$(upd))
SUB_UPV = $(addsuffix .upv,$(upv))

.PHONY: all all_32 $(SUB_NEW) $(SUB_UPD)
all all_32: $(SUB_NEW) $(SUB_UPD) $(SUB_UPV)

# call in this dir
$(SUB_NEW):
	$(Q)echo NEW $(basename $@) $(version)
	$(Q)./new $(basename $@) '$(version)' '$(force)'

# call in this dir
$(SUB_UPD):
	$(Q)[ -f "$(basename $@)/version.mk"  ] || (echo "$(basename $@) not exist for upd" && exit 1)
	$(Q)[ ! -z "$(version)"  ] || (echo "version need for upd" && exit 1)
	$(Q)echo UPD $(basename $@) $(version)
	$(Q)./new $(basename $@) '$(version)' '$(force)'

# call in sub dir
$(SUB_UPV):
	$(Q)[ -f "./version.mk"  ] || (echo "version.mk not exist for $(basename $@)" && exit 1)
	$(Q)[ ! -z "$(version)"  ] || (echo "version need for upv" && exit 1)
	$(Q)echo UPV $(basename $@) $(version)
	$(Q)cd ../; ./new $(basename $@) '$(version)' '$(force)'
else
# make for build
ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

ifeq ($(TARGET),)
# make for all sub list

SUB_IGNORE = _common
SUB_ALL = $(filter-out $(SUB_IGNORE),$(notdir $(shell find . -mindepth 1 -maxdepth 1 -type d)))
ifeq ($(SUB_USEALL),)
# filter notuse sub
SUB_NOTUSE = $(shell [ -f notuse ] && cat notuse |grep -v "^\#")
SUB_LIST = $(filter-out $(SUB_NOTUSE),$(SUB_ALL))
else
SUB_LIST = $(SUB_ALL)
endif
SUB_CLEAN  = $(addsuffix _clean,$(SUB_LIST))
SUB_INSTALL = $(addsuffix _install,$(SUB_LIST))
SUB_VER  = $(addsuffix _version,$(SUB_LIST))

# depend?
SUB_DEPEND = serial
SUB_NOMAL = $(filter-out $(SUB_DEPEND),$(SUB_LIST))

.PHONY: all all_32 clean install version $(SUB_LIST) $(SUB_CLEAN) $(SUB_VER)

all all_32: $(SUB_LIST)
	$(Q)[ -z "$(SUB_NOTUSE)" ] || echo "** NOTUSE: $(SUB_NOTUSE)"
	$(Q)[ -z "$(SUB_NOTUSE)" ] || echo "---> USED: $(SUB_LIST)"

clean: $(SUB_CLEAN)

install: $(SUB_INSTALL)

version: $(SUB_VER)

$(SUB_DEPEND):
	$(Q)echo MK mod: $@
	$(Q)$(MAKE) BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/$@ -C ./$@

$(SUB_NOMAL): $(SUB_DEPEND)
	$(Q)echo MK mod: $@
	$(Q)$(MAKE) BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/$@ -C ./$@

$(SUB_CLEAN):
	$(Q)echo CL mod: $(subst _clean,,$(notdir $@))
	$(Q)$(MAKE) BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/$(subst _clean,,$@) -C ./$(subst _clean,,$@) clean

$(SUB_INSTALL):
	$(Q)echo INS mod: $(subst _install,,$(notdir $@))
	$(Q)$(MAKE) BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/$(subst _install,,$@) -C ./$(subst _install,,$@) install

$(SUB_VER):
	$(Q)$(MAKE) BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/$(subst _version,,$@) -C ./$(subst _version,,$@) version

else

# make for one sub: version default
ifeq ($(SO_VERSION_MAJOR),)
	SO_VERSION_MAJOR := 1
	SO_VERSION_MINOR := 0
	SO_VERSION_PATCH := 0
endif

# source files
SO_SRCS = $(wildcard ./*.c)
SO_OBJS = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(SO_SRCS:.c=.o))

# builid params
INCS := -I../../inc/legacy/inc -I../../inc/legacy/utility -I../../inc/legacy/utility/sensor $(INCS)
INCS := -I. -I../../inc  -I../../inc/private -I../../inc/develop -I../../inc/legacy $(INCS)

CFLAGS += -fPIC
LIBS += -lcam -lpthread -lm -lrt
LDFLAGS := -L${BUILD_OUTPUT_PATH}/../../ -Wl,-rpath=/usr/lib/aarch64-linux-gnu/ $(LDFLAGS)

CFLAGS += -Wno-unused-variable
CFLAGS += -Wno-pointer-sign
#CFLAGS += -Wno-implicit-function-declaration
CFLAGS += -Wno-unused-function
CFLAGS += -Wno-unused-but-set-variable
CFLAGS += -Wno-sizeof-pointer-memaccess
CFLAGS += -fno-strict-aliasing

ifeq ($(TARGET), serial)
# serial modules as a.
SO_TARGET = lib$(TARGET).a
SO_VERSION = $(SO_VERSION_MAJOR).$(SO_VERSION_MINOR).$(SO_VERSION_PATCH)
SO_NAME = ${BUILD_OUTPUT_PATH}/$(SO_TARGET)
SO_NAMEV = $(SO_NAME).$(SO_VERSION_MAJOR)
SO_NAMEVER = $(SO_NAME).$(SO_VERSION)

# output files for clean
OUTS = $(SO_NAMEVER) $(SO_NAMEV) $(SO_NAME) $(SO_OBJS)

.PHONY: all all_32 clean install version upd

all all_32: $(SO_NAME)

$(SO_NAMEVER): $(SO_OBJS)
	$(Q)mkdir -p $(abspath $(dir $@))
	$(Q)echo AR $@
	$(Q)$(AR) $(CFLAGS_STATIC) $@ $(SO_OBJS)
else
# common modules as so.
# target files
SO_TARGET = lib$(TARGET).so
SO_VERSION = $(SO_VERSION_MAJOR).$(SO_VERSION_MINOR).$(SO_VERSION_PATCH)
SO_NAME = ${BUILD_OUTPUT_PATH}/$(SO_TARGET)
SO_NAMEV = $(SO_NAME).$(SO_VERSION_MAJOR)
SO_NAMEVER = $(SO_NAME).$(SO_VERSION)

# sub dirs: calibration
SUB_IGNORE = inc src
SUB_LIST = $(filter-out $(SUB_IGNORE),$(notdir $(shell find . -mindepth 1 -maxdepth 1 -type d)))
SUB_CLEAN  = $(addsuffix _clean,$(SUB_LIST))
SUB_INSTALL = $(addsuffix _install,$(SUB_LIST))
SUB_VER  = $(addsuffix _version,$(SUB_LIST))

# sub Makefile
SUB_MAKE = \
ifeq ($${BUILD_OUTPUT_PATH},)\n\
export BUILD_OUTPUT_PATH=$$(shell pwd)\n\
endif\n\
\n\
SUB_TYPE = $$(notdir $$(shell pwd))\n\
SUB_PARENT = $$(notdir $$(shell dirname `pwd`))\n\
\n\
include ../../Makefile_$$(SUB_TYPE)

# build params add.
LIBS += -lserial
LDFLAGS := -L${BUILD_OUTPUT_PATH}/../serial $(LDFLAGS)

# output files for clean
OUTS = $(SO_NAMEVER) $(SO_NAMEV) $(SO_NAME) $(SO_OBJS)

.PHONY: all all_32 clean version upd $(SUB_LIST) $(SUB_CLEAN) $(SUB_VER)

all all_32: $(SO_NAME) $(SUB_LIST)

$(SO_NAMEVER): $(SO_OBJS)
	$(Q)mkdir -p $(abspath $(dir $@))
	$(Q)echo CC $@
	$(Q)$(CC) $(CFLAGS_DYNAMIC) $^ $(LDFLAGS) $(LIBS) -o $@
endif

# target macro
CFLAGS_THIS = -DTARGET=\"${TARGET}\"
CFLAGS_THIS += -DVERSION=\"${SO_VERSION}\"
CFLAGS_THIS += -DSO_VERSION_MAJOR=$(SO_VERSION_MAJOR) -DSO_VERSION_MINOR=$(SO_VERSION_MINOR) -DSO_VERSION_PATCH=$(SO_VERSION_PATCH)

${BUILD_OUTPUT_PATH}/%.o: ./%.c
	$(Q)mkdir -p $(abspath $(dir $@))
	$(Q)echo CC $@
	$(Q)$(CC) $(CFLAGS) $(CFLAGS_THIS) $(INCS) -c $< -o $@

$(SO_NAMEV): $(SO_NAMEVER)
	$(Q)echo LN $@
	$(Q)ln -sf $(notdir $^) $@

$(SO_NAME): $(SO_NAMEV)
	$(Q)echo LN $@
	$(Q)ln -sf $(notdir $^) $@

clean: $(SUB_CLEAN)
	$(Q)echo CL $(notdir $(OUTS))
	$(Q)rm -fr ${OUTS}

install: $(SO_NAME) $(SUB_INSTALL)
ifeq ($(HB_PKG_LIBCAM_RUNTIME_DEB)$(suffix $(SO_TARGET)), y.so)
	$(Q)echo INS RUN $(notdir $(SO_NAMEVER) $(SO_NAMEV))
	$(Q)install -d ${MODULE_RUNTIME_DIR}/usr/hobot/lib/sensor
	$(Q)install $(SO_NAMEVER) ${MODULE_RUNTIME_DIR}/usr/hobot/lib/sensor
	$(Q)ln -srf ${MODULE_RUNTIME_DIR}/usr/hobot/lib/sensor/$(notdir $(SO_NAMEVER)) \
		${MODULE_RUNTIME_DIR}/usr/hobot/lib/sensor/$(notdir $(SO_NAMEV))
	$(Q)ln -srf ${MODULE_RUNTIME_DIR}/usr/hobot/lib/sensor/$(notdir $(SO_NAMEVER)) \
		${MODULE_RUNTIME_DIR}/usr/hobot/lib/sensor/$(notdir $(SO_NAME))
endif
ifeq ($(HB_PKG_LIBCAM_DEV_DEB), y)
ifeq ($(suffix $(SO_TARGET)), .so) # so add link.
	$(Q)echo INS DEV $(notdir $(SO_NAMEVER) $(SO_NAMEV) $(SO_NAME))
	$(Q)install -d ${MODULE_DEV_DIR}/usr/hobot/lib/sensor
	$(Q)install $(SO_NAMEVER) ${MODULE_DEV_DIR}/usr/hobot/lib/sensor
	$(Q)ln -srf ${MODULE_DEV_DIR}/usr/hobot/lib/sensor/$(notdir $(SO_NAMEVER)) \
		${MODULE_DEV_DIR}/usr/hobot/lib/sensor/$(notdir $(SO_NAMEV))
	$(Q)ln -srf ${MODULE_DEV_DIR}/usr/hobot/lib/sensor/$(notdir $(SO_NAMEVER)) \
		${MODULE_DEV_DIR}/usr/hobot/lib/sensor/$(notdir $(SO_NAME))
else # a no link.
	$(Q)echo INS DEV $(notdir $(SO_NAME))
	$(Q)install -d ${MODULE_DEV_DIR}/usr/hobot/lib/sensor
	$(Q)install $(SO_NAMEVER) ${MODULE_DEV_DIR}/usr/hobot/lib/sensor/$(SO_TARGET)
endif
endif

version: $(SUB_VER)
	$(Q)echo VER $(TARGET): $(SO_TARGET) $(SO_VERSION)

$(SUB_LIST):
	$(Q)echo MK $(TARGET) sub: $@
	$(Q)[ -f $@/Makefile ] || (echo "GEN $@/Makefile"; echo -e '$(SUB_MAKE)' |sed -e 's/^[  ]//' > $@/Makefile)
	$(Q)$(MAKE) BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/$@ -C ./$@

$(SUB_CLEAN):
	$(Q)echo CL $(TARGET) sub: $(subst _clean,,$(notdir $@))
	$(Q)$(MAKE) BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/$(subst _clean,,$@) -C ./$(subst _clean,,$@) clean

$(SUB_INSTALL):
	$(Q)echo INS $(TARGET) sub: $(subst _install,,$(notdir $@))
	$(Q)$(MAKE) BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/$(subst _install,,$@) -C ./$(subst _install,,$@) install

$(SUB_VER):
	$(Q)$(MAKE) BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/$(subst _version,,$@) -C ./$(subst _version,,$@) version

endif
endif
