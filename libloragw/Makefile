### get external defined data

LIBLORAGW_VERSION := `cat ../VERSION`
include library.cfg

### constant symbols

CROSS_COMPILE :=
CC := $(CROSS_COMPILE)gcc
AR := $(CROSS_COMPILE)ar

CFLAGS := -O2 -Wall -Wextra -std=c99 -Iinc -I.

### library.cfg configuration file processing

ifeq ($(CFG_SPI),native)
  CFG_SPI_MSG := Linux native SPI driver (/dev/spidev32766.0)
  CFG_SPI_OPT := CFG_SPI_NATIVE
else ifeq ($(CFG_SPI),ftdi)
  CFG_SPI_MSG := FTDI SPI-over-USB bridge using libmpsse/libftdi/libusb
  CFG_SPI_OPT := CFG_SPI_FTDI
else
  $(error No SPI physical layer selected, check ../target.cfg file.)
endif

ifeq ($(CFG_CHIP),sx1301)
  CFG_CHIP_MSG := Semtech SX1301 production chip
  CFG_CHIP_OPT := CFG_CHIP_1301
else ifeq ($(CFG_CHIP),fpga1301)
  CFG_CHIP_MSG := FPGA containing the SX1301 IP
  CFG_CHIP_OPT := CFG_CHIP_FPGA
else
  $(error No concentrator chip selected, check library.cfg file.)
endif

ifeq ($(CFG_RADIO),sx1257)
  CFG_RADIO_MSG := Dual SX1257 transceivers, covering 860-1000 MHz
  CFG_RADIO_OPT := CFG_RADIO_1257
else ifeq ($(CFG_RADIO),sx1255)
  CFG_RADIO_MSG := Dual SX1255 transceivers, covering 400-510 MHz
  CFG_RADIO_OPT := CFG_RADIO_1255
else
  $(error No radio chip selected, check library.cfg file.)
endif

ifeq ($(CFG_BAND),full)
  CFG_BAND_MSG := Full range supported by the radio(s)
  CFG_BAND_OPT := CFG_BAND_FULL
else ifeq ($(CFG_BAND),eu868)
  ifeq ($(CFG_RADIO),sx1257)
    CFG_BAND_MSG := ETSI 868 (866) MHz band
    CFG_BAND_OPT := CFG_BAND_868
  else
    $(error The selected radio(s) cannot work in the selected band.)
  endif
else ifeq ($(CFG_BAND),us915)
  ifeq ($(CFG_RADIO),sx1257)
    CFG_BAND_MSG := FCC 915 MHz band
    CFG_BAND_OPT := CFG_BAND_915
  else
    $(error The selected radio(s) cannot work in the selected band.)
  endif
else ifeq ($(CFG_BAND),cn470)
  ifeq ($(CFG_RADIO),sx1255)
    CFG_BAND_MSG := China 470 MHz band
    CFG_BAND_OPT := CFG_BAND_470
  else
    $(error The selected radio(s) cannot work in the selected band.)
  endif
else ifeq ($(CFG_BAND),eu433)
  ifeq ($(CFG_RADIO),sx1255)
    CFG_BAND_MSG := ETSI 433 MHz band
    CFG_BAND_OPT := CFG_BAND_433
  else
    $(error The selected radio(s) cannot work in the selected band.)
  endif
else ifeq ($(CFG_BAND),cn780)
  ifeq ($(CFG_RADIO),sx1257)
    CFG_BAND_MSG := China 780 MHz band
    CFG_BAND_OPT := CFG_BAND_780
  else
    $(error The selected radio(s) cannot work in the selected band.)
  endif
else
  $(warning [Warning] No frequency band selected, full radio capability assumed.)
  CFG_BAND_MSG := Full range supported by the radio(s)
  CFG_BAND_OPT := CFG_BAND_FULL
endif

ifeq ($(CFG_BRD),dev_nano_868)
  CFG_BRD_MSG := FPGA-based nano-concentrator, with 868 MHz SAW filter
  CFG_BRD_OPT := CFG_BRD_NANO868
else ifeq ($(CFG_BRD),ref_1301_868)
  CFG_BRD_MSG := SX1301 reference board with SX1257 radios, 868 MHz filters
  CFG_BRD_OPT := CFG_BRD_1301REF868
else ifeq ($(CFG_BRD),ref_1301_433)
  CFG_BRD_MSG := SX1301 reference board with SX1255 radios, 433 MHz filters
  CFG_BRD_OPT := CFG_BRD_1301REF433
else ifeq ($(CFG_BRD),kerlink_868)
  CFG_BRD_MSG := Kerlink LoRa 868MHz gateway
  CFG_BRD_OPT := CFG_BRD_KERLINK868
else ifeq ($(CFG_BRD),kerlink_433)
  CFG_BRD_MSG := Kerlink LoRa 433MHz gateway
  CFG_BRD_OPT := CFG_BRD_KERLINK433
else ifeq ($(CFG_BRD),cisco_433)
  CFG_BRD_MSG := Cisco LoRa 433MHz gateway
  CFG_BRD_OPT := CFG_BRD_CISCO433
else ifeq ($(CFG_BRD),cisco_470)
  CFG_BRD_MSG := Cisco LoRa 470MHz gateway
  CFG_BRD_OPT := CFG_BRD_CISCO470
else ifeq ($(CFG_BRD),cisco_780)
  CFG_BRD_MSG := Cisco LoRa 780MHz gateway
  CFG_BRD_OPT := CFG_BRD_CISCO780
else
  $(info [Info] No specific board selected.)
  CFG_BRD_MSG := None
  CFG_BRD_OPT := CFG_BRD_NONE
endif

ifeq ($(CFG_NET),private)
  CFG_NET_MSG := Private network
  CFG_NET_OPT := CFG_NET_PRIVATE
else ifeq ($(CFG_NET),lora_mac)
  CFG_NET_MSG := LoRa MAC network
  CFG_NET_OPT := CFG_NET_LORAMAC
else
  $(warning [Warning] No network type selected, assume private network by default.)
  CFG_NET_MSG := Private network
  CFG_NET_OPT := CFG_NET_PRIVATE
endif

### linking options

ifeq ($(CFG_SPI),native)
  LIBS := -lloragw -lrt
else ifeq ($(CFG_SPI),ftdi)
  LIBS := -lloragw -lrt -lmpsse
endif

### general build targets

all: libloragw.a test_loragw_spi test_loragw_reg test_loragw_hal test_loragw_gps

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
  # Concentrator chip
	@echo "Concentrator chip : $(CFG_CHIP_MSG)"
	@echo "	#define $(CFG_CHIP_OPT)	1" >> $@
  # Radio chip(s)
	@echo "Radio chip(s)     : $(CFG_RADIO_MSG)"
	@echo "	#define $(CFG_RADIO_OPT)	1" >> $@
  # Frequency band
	@echo "Frequency band    : $(CFG_BAND_MSG)"
	@echo "	#define $(CFG_BAND_OPT)	1" >> $@
  # Board misc. parameters
	@echo "Board misc. param : $(CFG_BRD_MSG)"
	@echo "	#define $(CFG_BRD_OPT)	1" >> $@
  # Network type
	@echo "Network type      : $(CFG_NET_MSG)"
	@echo "	#define $(CFG_NET_OPT)	1" >> $@
  # Debug options
	@echo "	#define DEBUG_AUX	$(DEBUG_AUX)" >> $@
	@echo "	#define DEBUG_SPI	$(DEBUG_SPI)" >> $@
	@echo "	#define DEBUG_REG	$(DEBUG_REG)" >> $@
	@echo "	#define DEBUG_HAL	$(DEBUG_HAL)" >> $@
	@echo "	#define DEBUG_GPS	$(DEBUG_GPS)" >> $@
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

libloragw.a: obj/loragw_hal.o obj/loragw_gps.o obj/loragw_reg.o obj/loragw_spi.o obj/loragw_aux.o
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

### EOF