obj-m := ch347.o
ch347-objs := core.o gpio-ch347.o i2c-ch347.o spi-ch347.o

all:
	make -C $(KDIR) M=$(PWD) modules

install:
	make -C $(KDIR) M=$(PWD) modules_install

clean:
	make -C $(KDIR) M=$(PWD) clean
