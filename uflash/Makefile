CFLAGS		+=-O2 -W -Wall -std=gnu99 -pedantic
CFLAGS		+= -fPIE -fstack-protector --param=ssp-buffer-size=4 -Wformat -Wformat-security -Werror=format-security
ifdef DEBUG
	CFLAGS += -g -ggdb -O0
endif

CPPFLAGS	+= -D_FORTIFY_SOURCE=2

LDFLAGS		+= -lpci
LDFLAGS		+= -fPIE -pie -Wl,-z,relro -Wl,-z,now -Wl,--as-needed -Wl,-O1 -Wl,-z,defs

all: uflash

clean:
	-rm uflash
