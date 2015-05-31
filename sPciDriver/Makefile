sPciDriver := $(shell pwd)

KERNEL_DIR  := $(shell uname -r)

obj-m := sPciDriver.o

all:
	make -C /lib/modules/$(KERNEL_DIR)/build M=$(sPciDriver) modules

clean:
	make -C /lib/modules/$(KERNEL_DIR)/build M=$(sPciDriver) clean
