obj-m := blockio_dev.o
blockio_dev-objs	:= main_gpio.o gpio_v2.o

KDIR := /root/linux-dev/KERNEL
PWD := $(shell pwd)
ARCH := arm
default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
clean : 
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -rf *.ko
	rm -rf *.mod.*
	rm -rf *.o