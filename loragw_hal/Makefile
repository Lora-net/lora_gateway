CC=gcc
CFLAGS=-O2 -Iinc
C99FLAGS=-O2 -std=c99 -Iinc
LDFLAGS=-lrt

all: test_loragw_spi test_loragw_reg test_loragw_hal

clean:
	rm -f test_*
	rm -f obj/*.o

test_loragw_spi: obj/test_loragw_spi.o obj/loragw_spi.o
	$(CC) obj/test_loragw_spi.o obj/loragw_spi.o -o test_loragw_spi $(LDFLAGS)

test_loragw_reg: obj/test_loragw_reg.o obj/loragw_reg.o obj/loragw_spi.o
	$(CC) obj/test_loragw_reg.o obj/loragw_reg.o obj/loragw_spi.o -o test_loragw_reg $(LDFLAGS)
	
test_loragw_hal: obj/test_loragw_hal.o obj/loragw_hal.o obj/loragw_reg.o obj/loragw_spi.o obj/loragw_aux.o
	$(CC) obj/test_loragw_hal.o obj/loragw_hal.o obj/loragw_reg.o obj/loragw_spi.o obj/loragw_aux.o -o test_loragw_hal $(LDFLAGS)

obj/loragw_aux.o: src/loragw_aux.c inc/loragw_aux.h
	$(CC) -c $(CFLAGS) src/loragw_aux.c -o obj/loragw_aux.o

obj/loragw_spi.o: src/loragw_spi.c inc/loragw_spi.h
	$(CC) -c $(C99FLAGS) src/loragw_spi.c -o obj/loragw_spi.o

obj/loragw_reg.o: src/loragw_reg.c inc/loragw_reg.h inc/loragw_spi.h
	$(CC) -c $(C99FLAGS) src/loragw_reg.c -o obj/loragw_reg.o

obj/loragw_hal.o: src/loragw_hal.c src/arb_fw.var src/agc_fw.var inc/loragw_hal.h inc/loragw_reg.h inc/loragw_spi.h inc/loragw_aux.h
	$(CC) -c $(C99FLAGS) src/loragw_hal.c -o obj/loragw_hal.o

obj/test_loragw_spi.o: test/test_loragw_spi.c inc/loragw_spi.h
	$(CC) -c $(C99FLAGS) -Iinc test/test_loragw_spi.c -o obj/test_loragw_spi.o

obj/test_loragw_reg.o: test/test_loragw_reg.c inc/loragw_reg.h
	$(CC) -c $(C99FLAGS) -Iinc test/test_loragw_reg.c -o obj/test_loragw_reg.o

obj/test_loragw_hal.o: test/test_loragw_hal.c inc/loragw_hal.h inc/loragw_aux.h
	$(CC) -c $(C99FLAGS) test/test_loragw_hal.c -o obj/test_loragw_hal.o

