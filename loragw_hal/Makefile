# putting the configuration in a separate file
include library.cfg

# constant symbols
CC=gcc
CFLAGS=-O2 -Iinc
C99FLAGS=-O2 -std=c99 -Iinc

# configuration-dependant symbols
ifeq ($(LGW_PHY),native)
LDFLAGS=-lrt
endif
ifeq ($(LGW_PHY),ftdi)
LDFLAGS=-lrt -lmpsse
endif


# general build targets

all: test_loragw_spi test_loragw_reg test_loragw_hal

clean:
	rm -f test_*
	rm -f obj/*.o
	rm -f .conf_ok

.conf_ok: library.cfg
	@echo "*** Checking Lora gateway HAL library config ***"
	@rm -f .conf_ok
ifeq ($(LGW_PHY),native)
	@echo "Selected SPI interface type: Linux native driver"
else
ifeq ($(LGW_PHY),ftdi)
	@echo "Selected SPI interface type: FTDI SPI-over-USB bridge"
else
	$(error No SPI physical layer selected)
endif
endif
	@echo "*** Config seems ok ***"
	@touch .conf_ok


# library module target

obj/loragw_aux.o: .conf_ok src/loragw_aux.c inc/loragw_aux.h
	$(CC) -c $(CFLAGS) src/loragw_aux.c -o obj/loragw_aux.o $(FLAG_AUX)

obj/loragw_spi.o: .conf_ok src/loragw_spi.native.c src/loragw_spi.ftdi.c inc/loragw_spi.h
ifeq ($(LGW_PHY),native)
	$(CC) -c $(C99FLAGS) src/loragw_spi.native.c -o obj/loragw_spi.o $(FLAG_SPI)
endif
ifeq ($(LGW_PHY),ftdi)
	$(CC) -c $(C99FLAGS) src/loragw_spi.ftdi.c -o obj/loragw_spi.o $(FLAG_SPI)
endif

obj/loragw_reg.o: .conf_ok src/loragw_reg.c inc/loragw_reg.h inc/loragw_spi.h
	$(CC) -c $(C99FLAGS) src/loragw_reg.c -o obj/loragw_reg.o $(FLAG_REG)

obj/loragw_hal.o: .conf_ok src/loragw_hal.c src/arb_fw.var src/agc_fw.var inc/loragw_hal.h inc/loragw_reg.h inc/loragw_spi.h inc/loragw_aux.h
	$(CC) -c $(C99FLAGS) src/loragw_hal.c -o obj/loragw_hal.o $(FLAG_HAL)


# test programs

obj/test_loragw_spi.o: test/test_loragw_spi.c inc/loragw_spi.h
	$(CC) -c $(C99FLAGS) -Iinc test/test_loragw_spi.c -o obj/test_loragw_spi.o

obj/test_loragw_reg.o: test/test_loragw_reg.c inc/loragw_reg.h
	$(CC) -c $(C99FLAGS) -Iinc test/test_loragw_reg.c -o obj/test_loragw_reg.o

obj/test_loragw_hal.o: test/test_loragw_hal.c inc/loragw_hal.h inc/loragw_aux.h
	$(CC) -c $(C99FLAGS) test/test_loragw_hal.c -o obj/test_loragw_hal.o

test_loragw_spi: .conf_ok obj/test_loragw_spi.o obj/loragw_spi.o
	$(CC) obj/test_loragw_spi.o obj/loragw_spi.o -o test_loragw_spi $(LDFLAGS)

test_loragw_reg: .conf_ok obj/test_loragw_reg.o obj/loragw_reg.o obj/loragw_spi.o
	$(CC) obj/test_loragw_reg.o obj/loragw_reg.o obj/loragw_spi.o -o test_loragw_reg $(LDFLAGS)
	
test_loragw_hal: .conf_ok obj/test_loragw_hal.o obj/loragw_hal.o obj/loragw_reg.o obj/loragw_spi.o obj/loragw_aux.o
	$(CC) obj/test_loragw_hal.o obj/loragw_hal.o obj/loragw_reg.o obj/loragw_spi.o obj/loragw_aux.o -o test_loragw_hal $(LDFLAGS)

