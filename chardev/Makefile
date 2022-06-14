#CC = clang++
#CC = g++
CC = gcc
obj-m = chardev.o 
#CFLAGS = -O2 -Wall -DMODULE -D__KERNEL__ -DLINUX
CCFLAGS-y += -I/usr/src/linux-hwe-5.13-headers-5.13.0-48/include/
PWD := $(CURDIR) 
 
all: 
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	gcc -o ioctl ioctl.c
 
clean: 
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

remove:
	sudo rmmod chardev.ko

insert:
	sudo insmod chardev.ko

