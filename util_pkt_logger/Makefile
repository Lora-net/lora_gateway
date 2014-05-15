### Application-specific constants

APP_NAME := util_pkt_logger

### Environment constants 

LGW_PATH := ../libloragw
CROSS_COMPILE :=

### External constant definitions
# must get library build option to know if mpsse must be linked or not

include $(LGW_PATH)/library.cfg

### Constant symbols

CC := $(CROSS_COMPILE)gcc
AR := $(CROSS_COMPILE)ar

CFLAGS=-O2 -Wall -Wextra -std=c99 -Iinc -I.

### Constants for LoRa concentrator HAL library
# List the library sub-modules that are used by the application

LGW_INC = $(LGW_PATH)/inc/config.h
LGW_INC += $(LGW_PATH)/inc/loragw_hal.h

### Linking options

ifeq ($(CFG_SPI),native)
  LIBS := -lloragw -lrt
else ifeq ($(CFG_SPI),ftdi)
  LIBS := -lloragw -lrt -lmpsse
endif

### General build targets

all: $(APP_NAME) global_conf.json

clean:
	rm -f obj/*.o
	rm -f $(APP_NAME)
	find . -name global_conf.json -exec rm -i {} \;

### HAL library (do no force multiple library rebuild even with 'make -B')

$(LGW_PATH)/inc/config.h:
	@if test ! -f $@; then \
	$(MAKE) all -C $(LGW_PATH); \
	fi

$(LGW_PATH)/libloragw.a: $(LGW_INC)
	@if test ! -f $@; then \
	$(MAKE) all -C $(LGW_PATH); \
	fi

### Sub-modules compilation

obj/parson.o: src/parson.c inc/parson.h
	$(CC) -c $(CFLAGS) $< -o $@

### Select the proper configuration JSON for the program

ifeq ($(CFG_BAND),eu868)
global_conf.json: cfg/global_conf.eu868.json
	cp $< $@
else ifeq ($(CFG_BAND),us915)
global_conf.json: cfg/global_conf.us915.json
	cp $< $@
else ifeq ($(CFG_BAND),cn470)
global_conf.json: cfg/global_conf.cn470.json
	cp $< $@
else ifeq ($(CFG_BAND),eu433)
global_conf.json: cfg/global_conf.eu433.json
	cp $< $@
else
global_conf.json: cfg/global_conf.empty.json
	cp $< $@
endif

### Main program compilation and assembly

obj/$(APP_NAME).o: src/$(APP_NAME).c $(LGW_INC) inc/parson.h
	$(CC) -c $(CFLAGS) -I$(LGW_PATH)/inc $< -o $@

$(APP_NAME): obj/$(APP_NAME).o $(LGW_PATH)/libloragw.a obj/parson.o
	$(CC) -L$(LGW_PATH) $< obj/parson.o -o $@ $(LIBS)

### EOF