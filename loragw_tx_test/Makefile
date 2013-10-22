### Application-specific constants

APP_NAME=loragw_tx_test

### constant symbols

CC=gcc
CFLAGS=-O2 -Wall -Wextra -Iinc
C99FLAGS=-O2 -Wall -Wextra -std=c99 -Iinc
FLAG_AUX=

### constants for Lora Gateway HAL library

LGW_PATH=../loragw_hal
LGW_INC=-I$(LGW_PATH)/inc
#LGW_LNK=-lloragw -lrt
LGW_LNK=-lloragw -lrt -lmpsse
# add libmpsse or not, depending on what option you compiled the libloragw with

### general build targets

all: $(APP_NAME)

clean:
	rm -f obj/*.o
	rm -f $(APP_NAME)

### main program compilation and assembly

obj/$(APP_NAME).o: src/$(APP_NAME).c 
	$(CC) -c $(C99FLAGS) -o obj/$(APP_NAME).o $(LGW_INC) src/$(APP_NAME).c $(FLAG_AUX)

$(APP_NAME): $(LGW_PATH)/libloragw.a obj/$(APP_NAME).o
	$(CC) -o $(APP_NAME) obj/$(APP_NAME).o -L$(LGW_PATH) $(LGW_LNK)
