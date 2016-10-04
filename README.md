dmec is a linux driver for the embedded controllers found on the DMO CPU
modules.

# About dmec

The Linux driver provides at the moment the following functions:

- i2c
- gpio
- watchdog
- rtm (real time monitor)

# Building and installing

The kernel headers package has to be installed or the location of the kernel
build directory has to be set using the **KERNEL_SRC** variable.

~~~
export KERNEL_SRC=/usr/src/linux
make
make install
depmod -a
~~~

The drivers will be then loaded on the next boot.
