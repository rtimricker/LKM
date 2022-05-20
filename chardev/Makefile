#CC = clang++
#CC = g++
CC = gcc
obj-m = chardev.o
 
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

node:
	sudo mknod /dev/char_dev c 100 0

