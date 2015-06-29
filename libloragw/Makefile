### get external defined data

LIBLORAGW_VERSION := `cat ../VERSION`
include library.cfg

### constant symbols

ARCH ?=
CROSS_COMPILE ?=
CC := $(CROSS_COMPILE)gcc
AR := $(CROSS_COMPILE)ar

CFLAGS := -O2 -Wall -Wextra -std=c99 -Iinc -I.

### library.cfg configuration file processing

ifeq ($(CFG_SPI),native)
  CFG_SPI_MSG := Linux native SPI driver
  CFG_SPI_OPT := CFG_SPI_NATIVE
else ifeq ($(CFG_SPI),ftdi)
  CFG_SPI_MSG := FTDI SPI-over-USB bridge using libmpsse/libftdi/libusb
  CFG_SPI_OPT := CFG_SPI_FTDI
else
  $(error No SPI physical layer selected, check ../target.cfg file.)
endif

### linking options

ifeq ($(CFG_SPI),native)
  LIBS := -lloragw -lrt -lm
else ifeq ($(CFG_SPI),ftdi)
  LIBS := -lloragw -lrt -lmpsse -lm
endif

### general build targets

all: libloragw.a test_loragw_spi test_loragw_reg test_loragw_hal test_loragw_gps test_loragw_cal

clean:
	rm -f libloragw.a
	rm -f test_loragw_*
	rm -f obj/*.o
	rm -f inc/config.h

### transpose library.cfg into a C header file : config.h

inc/config.h: ../VERSION library.cfg
	@echo "*** Checking libloragw library configuration ***"
	@rm -f $@
  # File initialization
	@echo "#ifndef _LORAGW_CONFIGURATION_H" >> $@
	@echo "#define _LORAGW_CONFIGURATION_H" >> $@
  # Release version
	@echo "Release version   : $(LIBLORAGW_VERSION)"
	@echo "	#define LIBLORAGW_VERSION	"\"$(LIBLORAGW_VERSION)\""" >> $@
  # SPI interface
	@echo "SPI interface     : $(CFG_SPI_MSG)"
	@echo "	#define $(CFG_SPI_OPT)	1" >> $@
  # Debug options
	@echo "	#define DEBUG_AUX	$(DEBUG_AUX)" >> $@
	@echo "	#define DEBUG_SPI	$(DEBUG_SPI)" >> $@
	@echo "	#define DEBUG_REG	$(DEBUG_REG)" >> $@
	@echo "	#define DEBUG_HAL	$(DEBUG_HAL)" >> $@
	@echo "	#define DEBUG_GPS	$(DEBUG_GPS)" >> $@
	@echo "	#define DEBUG_GPIO	$(DEBUG_GPIO)" >> $@
  # end of file
	@echo "#endif" >> $@
	@echo "*** Configuration seems ok ***"

### library module target

obj/loragw_aux.o: src/loragw_aux.c inc/loragw_aux.h inc/config.h
	$(CC) -c $(CFLAGS) $< -o $@

ifeq ($(CFG_SPI),native)
obj/loragw_spi.o: src/loragw_spi.native.c inc/loragw_spi.h inc/config.h
	$(CC) -c $(CFLAGS) $< -o $@
else ifeq ($(CFG_SPI),ftdi)
obj/loragw_spi.o: src/loragw_spi.ftdi.c inc/loragw_spi.h inc/config.h
	$(CC) -c $(CFLAGS) $< -o $@
endif

obj/loragw_reg.o: src/loragw_reg.c inc/loragw_reg.h inc/loragw_spi.h inc/config.h
	$(CC) -c $(CFLAGS) $< -o $@

obj/loragw_hal.o: src/loragw_hal.c inc/loragw_hal.h inc/loragw_reg.h inc/loragw_aux.h src/arb_fw.var src/agc_fw.var src/cal_fw.var inc/config.h
	$(CC) -c $(CFLAGS) $< -o $@

obj/loragw_gps.o: src/loragw_gps.c inc/loragw_gps.h inc/config.h
	$(CC) -c $(CFLAGS) $< -o $@

### static library

ifeq ($(CFG_SPI),native)
libloragw.a: obj/loragw_hal.o obj/loragw_gps.o obj/loragw_reg.o obj/loragw_spi.o obj/loragw_aux.o
else ifeq ($(CFG_SPI),ftdi)
libloragw.a: obj/loragw_hal.o obj/loragw_gps.o obj/loragw_reg.o obj/loragw_spi.o obj/loragw_aux.o
endif
	$(AR) rcs $@ $^

### test programs

test_loragw_spi: tst/test_loragw_spi.c libloragw.a
	$(CC) $(CFLAGS) -L. $< -o $@ $(LIBS)

test_loragw_reg: tst/test_loragw_reg.c libloragw.a
	$(CC) $(CFLAGS) -L. $< -o $@ $(LIBS)

test_loragw_hal: tst/test_loragw_hal.c libloragw.a
	$(CC) $(CFLAGS) -L. $< -o $@ $(LIBS)

test_loragw_gps: tst/test_loragw_gps.c libloragw.a
	$(CC) $(CFLAGS) -L. $< -o $@ $(LIBS)

test_loragw_cal: tst/test_loragw_cal.c libloragw.a src/cal_fw.var
	$(CC) $(CFLAGS) -L. $< -o $@ $(LIBS)

### EOF
