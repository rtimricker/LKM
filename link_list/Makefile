#CC = clang++
#CC = g++
CC = gcc
obj-m = link_list_del.o 
#CFLAGS = -O2 -Wall -DMODULE -D__KERNEL__ -DLINUX
 
PWD := $(CURDIR) 
 
all: 
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
 
clean: 
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

remove:
	sudo rmmod chardev.ko

insert:
	sudo insmod chardev.ko


