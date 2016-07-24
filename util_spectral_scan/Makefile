### Environment constants 

LGW_PATH ?= ../libloragw
ARCH ?=
CROSS_COMPILE ?=

### External constant definitions

include $(LGW_PATH)/library.cfg

### Constant symbols

CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
CFLAGS = -O2 -Wall -Wextra -std=c99 -I inc

OBJDIR = obj
INCLUDES = $(wildcard inc/*.h)

### Constants for LoRa concentrator HAL library
# List the library sub-modules that are used by the application

LGW_INC = $(LGW_PATH)/inc/config.h
LGW_INC += $(LGW_PATH)/inc/loragw_hal.h

### Linking options

LIBS := -lloragw -lrt

### General build targets

all: util_spectral_scan

clean:
	rm -f $(OBJDIR)/*.o
	rm -f util_spectral_scan

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

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: src/%.c $(INCLUDES) | $(OBJDIR)
	$(CC) -c $(CFLAGS) -I$(LGW_PATH)/inc $< -o $@

### Main program assembly

util_spectral_scan: $(OBJDIR)/util_spectral_scan.o
	$(CC) -L$(LGW_PATH) $^ $(LIBS) -o $@

### EOF
