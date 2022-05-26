UNAME := $(shell uname -r)
DEVICE := sample
obj-m = $(DEVICE).o
PWD := $(CURDIR) 
KDIR = /lib/modules/$(shell uname -r)/build

all: 
	make -C $(KDIR) M=$(PWD) modules -B
	strip --strip-unneeded $(DEVICE).ko

debug:
	make -C $(KDIR) M=$(PWD) modules -B
 
clean: 
	make -C $(KDIR) M=$(PWD) clean

