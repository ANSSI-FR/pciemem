M=$(shell pwd)
KDIR ?= /lib/modules/`uname -r`/build

obj-m += pciemem.o

ifdef DEBUG
	CFLAGS_pciemem.o += -DDEBUG
endif

all: 
	make -C $(KDIR) M=$(M) modules
