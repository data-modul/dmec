
obj-m += dmec.o i2c-dmec.o gpio-dmec.o
dmec-objs = dmec-core.o

PWD := $(shell pwd)

ifeq (,$(KERNEL_SRC))
KERNEL_SRC := /lib/modules/$(shell uname -r)/build
endif

# ccflags-y += -DDEBUG

build:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

modules_install: install

test-install: build
	install -m 777 *.ko $(INSTALL_DIR)

help:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) help

clean:
	@rm -f *~
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
