CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
export CC
export AR

#comment this line for serial_posix use
USE_FTDI=1

UNAME_S := $(shell uname -s)

CCFLAGS = -Wall
ifeq ($(UNAME_S),Darwin)
CCFLAGS += -mmacosx-version-min=10.6 
endif

ifeq ($(UNAME_S),Linux)
ifneq ($(USE_FTDI),)
CCFLAGS :=  `pkg-config libudev libftdi1 libusb-1.0 --cflags`
LDFLAGS := `pkg-config libudev libftdi1 libusb-1.0 --libs`
FTDI_OPT := -DUSE_FTDI
endif
endif

all:
	$(MAKE) -C parsers
	$(CC) $(CCFLAGS) -g -o stm32flash -I./ \
		main.c \
		utils.c \
		stm32.c \
		serial_common.c \
		serial_platform.c \
		parsers/parsers.a \
		stm32/stmreset_binary.c \
		$(LDFLAGS) \
		$(FTDI_OPT) -DUSE_SERIAL_RESET

clean:
	$(MAKE) -C parsers clean
	rm -f stm32flash

install: all
	cp stm32flash /usr/local/bin
