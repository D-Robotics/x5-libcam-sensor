-include ./version.mk

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

TARGET = $(notdir $(shell pwd))

ifneq (${version},)
upv = $(TARGET)
endif

include ../Makefile

# hsmt_ns6607 dependency (only for drivers that need it)
INCS += -I../../deserial/hsmt_ns6607
LIBS += -lhsmt_ns6607
LDFLAGS += -L${BUILD_OUTPUT_PATH}/../../deserial/hsmt_ns6607
