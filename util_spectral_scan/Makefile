### Environment constants 

ARCH ?=
CROSS_COMPILE ?=

### Constant symbols

CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
CFLAGS = -O2 -Wall -Wextra -std=c99 -I inc -D DEBUG_SPI=0 -D DEBUG_REG=0

OBJDIR = obj
INCLUDES = $(wildcard inc/*.h)

### General build targets

all: util_spectral_scan

clean:
	rm -f $(OBJDIR)/*.o
	rm -f util_spectral_scan


### Sub-modules compilation

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: src/%.c $(INCLUDES) | $(OBJDIR)
	$(CC) -c $(CFLAGS) $< -o $@

### Main program assembly

util_spectral_scan: $(OBJDIR)/util_spectral_scan.o $(OBJDIR)/loragw_fpga_reg.o $(OBJDIR)/loragw_fpga_spi.o $(OBJDIR)/loragw_fpga_aux.o
	$(CC) $^ -lrt -o $@

### EOF
