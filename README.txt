########################
##### How to Build #####
########################

(1) Get Toolchain from Codesourcery site. (http://www.codesourcery.com)

    ex) Download : http://www.codesourcery.com/sgpp/lite/arm/portal/package7813/public/arm-none-eabi/arm-2010.09-51-arm-none-eabi-i686-pc-linux-gnu.tar.bz2
    
    recommand - Feature : ARM
                Target OS : "EABI"
                package : "IA32 GNU/Linux TAR"



(2) Edit Makefile for compile.

    edit "CROSS_COMPILE" to right toolchain path which you downloaded.

    ex) ARCH  ?= arm
        CROSS_COMPILE ?= ../toolchains/arm-2010.09/bin/arm-none-eabi-   //You have to modify this path!



(3) Compile as follow commands.

    $ cd kernel
    $ make clean
    $ make mrproper
    $ make SPH-M820_defconfig
    $ make



(4) Get the zImage on follow path.

    kernel/arch/arm/boot/zImage