# Makefile for the Samsung Paramter drivers.

# Each configuration option enables a list of files.

ifeq ($(CONFIG_ARM),y)
EXTRA_CFLAGS    += -Idrivers/fsr/Inc -Idrivers/fsr/PAM/MSM7k -D__CC_ARM
endif

# FSR base module
obj-$(CONFIG_MSM_PARAM)			+= sec_param.o

# For store accelerometer calibration data
#obj-$(CONFIG_MSM_PARAM)			+= acc_cal_param.o

# Should keep the build sequence. (fsr_base -> bml_block)
sec_param-objs	+= param.o
#acc_cal_param-objs += cal_param.o

