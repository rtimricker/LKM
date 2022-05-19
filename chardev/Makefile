#CC = clang++
CC = g++
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

