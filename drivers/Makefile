KERNELDIR := /home/qutao/RV1126/SDK_RockChip_2023/kernel
CURRENT_PATH := $(shell pwd)
obj-m := lsm6ds3trc.o

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules
clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean