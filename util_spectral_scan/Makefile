### Environment constants 

ARCH ?=
CROSS_COMPILE ?=

### Constant symbols

CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
CFLAGS = -O2 -Wall -Wextra -std=c99 -I inc -D DEBUG_SPI=0 -D DEBUG_REG=0

### General build targets

all: util_spectral_scan

clean:
	rm -f obj/*.o
	rm -f util_spectral_scan


### Sub-modules compilation

obj/loragw_fpga_aux.o: src/loragw_fpga_aux.c inc/loragw_fpga_aux.h
	$(CC) -c $(CFLAGS) $< -o $@

obj/loragw_fpga_spi.o: src/loragw_fpga_spi.c inc/loragw_fpga_spi.h
	$(CC) -c $(CFLAGS) $< -o $@

obj/loragw_fpga_reg.o: src/loragw_fpga_reg.c inc/loragw_fpga_reg.h inc/loragw_fpga_spi.h
	$(CC) -c $(CFLAGS) $< -o $@

### Main program compilation and assembly

obj/util_spectral_scan.o: src/util_spectral_scan.c inc/loragw_fpga_reg.h inc/loragw_fpga_spi.h inc/loragw_fpga_aux.h
	$(CC) -c $(CFLAGS) $< -o $@

util_spectral_scan: obj/util_spectral_scan.o obj/loragw_fpga_reg.o obj/loragw_fpga_spi.o obj/loragw_fpga_aux.o
	$(CC) $^ -lrt -o $@

### EOF
