TARGET = lepton3

obj-m	:= $(TARGET).o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build

all:	modules

install_headers:
	sudo apt-get install linux-headers-$(shell uname -r)

modules:
	$(MAKE) -C $(KERNELDIR) M=$$PWD modules

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$$PWD modules_install

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions modules.order Module.symvers $(TARGET) *.tmp *.log

$(TARGET): $(TARGET).c
