### Application-specific constants

APP_NAME := util_spi_stress

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
LGW_INC += $(LGW_PATH)/inc/loragw_reg.h

### Linking options

ifeq ($(CFG_SPI),native)
  LIBS := -lloragw -lrt
else ifeq ($(CFG_SPI),ftdi)
  LIBS := -lloragw -lrt -lmpsse
endif

### General build targets

all: $(APP_NAME)

clean:
	rm -f obj/*.o
	rm -f $(APP_NAME)

### HAL library (do no force multiple library rebuild even with 'make -B')

$(LGW_PATH)/inc/config.h:
	@if test ! -f $@; then \
	$(MAKE) all -C $(LGW_PATH); \
	fi

$(LGW_PATH)/libloragw.a: $(LGW_INC)
	@if test ! -f $@; then \
	$(MAKE) all -C $(LGW_PATH); \
	fi

### Main program compilation and assembly

obj/$(APP_NAME).o: src/$(APP_NAME).c $(LGW_INC)
	$(CC) -c $(CFLAGS) -I$(LGW_PATH)/inc $< -o $@

$(APP_NAME): obj/$(APP_NAME).o $(LGW_PATH)/libloragw.a
	$(CC) -L$(LGW_PATH) $< -o $@ $(LIBS)

### EOF