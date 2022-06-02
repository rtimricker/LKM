#CC = clang++
#CC = g++
CC = gcc
obj-m = chardev.o 
#CFLAGS = -O2 -Wall -DMODULE -D__KERNEL__ -DLINUX
 
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

