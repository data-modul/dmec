DMEC is a linux driver for the embedded controllers found on the DMO CPU
modules.

# About DMEC Driver

The dmec driver has support for the following DMO embedded controller(EC)
features:

- I2C
- GPIO
- Watchdog
- 2 x UART
- RTM (Real Time Monitor)
- ACPI-HWMON
- PWM (Pulse Width Modulation)

The features have to be enabled in BIOS. The I2C, UARTs and GPIO need an irq
number assigned in order to function properly. As an example: 

- I2C: IRQ 5
- GPIO: IRQ 7
- UART1: IRQ 10
- UART2: IRQ 15

For PWM0 and PWM1, GPIO4(COMe GPO0) and GPIO5(COMe GPO1) should be set 
respectively.

The UARTs are supported by the standard linux serial driver 8250/16550.

## I2C features
The missing i2c driver feature is:

- 10-bit addresses

## GPIO features

The GPIO driver provides digital inputs and outputs. The inputs can be
configured to generate an interrupt on different events. This can be
done either using sysfs or the new gpio api character device. For more
information check [[1]][[2]].

## Watchdog features

The watchdog can operate in two modes standard and window. The driver should be
loaded with the **win_mode** parameter set to one when window mode is used.
When the watchdog driver is loaded and the watchdog is active, the watchdog is
being automatically pinged. Please note that this does not work with kernel
versions below 4.6. The driver does not use the locking feature of the watchdog.

*NOTE: some features may not be supported. It depends on the board EC.*

## Driver compilation

### Prerequisites

1. GNU/GCC compiler and related tools such as make are needed to compile the DMEC 
driver. For this purpose, it is required to install the following package.

~~~bash
$ sudo apt-get update && apt-get upgrade
$ sudo apt-get install build-essential
~~~
The installation can be verified by the following syntax:

~~~bash
$ gcc -v
$ make -v
~~~

2. The linux kernel headers have to be installed in order to build the kernel
module. Most of the distribution provide packages for the kernel headers.

~~~bash
$ sudo apt-get update
$ apt-cache search linux-headers-$(uname -r)
$ sudo apt-get install linux-headers-$(uname -r)
~~~

3. The kernel build directory can also be set using the environment
variable **KERNEL_SRC**

~~~bash
$ export KERNEL_SRC=/usr/src/linux-headers-$(uname -r)
~~~
### Check out

The source of DMEC driver contains three branches for DMEC driver:

1. backport-for-v4.4  =>  DMEC driver for kernel 4.4
2. backport-for-v4.6  =>  DMEC driver for kernel 4.6
3. rya/backport-v4.14 =>  DMEC driver for kernel 4.14
4. kernel_4.15        =>  DMEC driver for kernel 4.15
5. master	            =>  DMEC driver for kernel 5.1x and above

### Compilation

All steps below have to be executed in the directory where the driver source
code was downloaded or checked out.

~~~bash
$ make
$ make install
$ depmod -a
~~~

### Loading

The driver should be auto-loaded on next boot. In case the I2C device nodes are 
missing, *i2c-dev* and *i2c-mux* are also should be loaded.

~~~bash
$ modprobe i2c-dev
$ modprove i2c-mux
~~~
To check whether the modules are loaded or not, use the below command:

~~~bash
$ lsmod |grep dmec
~~~

The output would be list of all dmec drivers: dmec, gpio_dmec, i2c_dmec, wdt_dmec, 
rtm_dmec, acpi_dmec, pwm_dmec
