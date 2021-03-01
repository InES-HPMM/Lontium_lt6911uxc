# Lontium_lt6911uxc

This repository contains a driver for the lt6911uxc HDMI2.0 to MIPI CSI converter from Lontium. The driver was tested on a Nvidia Xavier AGX (Jetpack 4.4).

---

[![logo](https://github.com/InES-HPMM/FPD-LinkIII_Raspberry_HW/blob/master/images/ines_logo.png)](https://www.zhaw.ch/en/engineering/institutes-centres/ines/ "Homepage")

__The group High Performance Multimedia from the Institute of Embedded Systems associated with ZHAW School of Engineering proudly presents an open source driver for lonitum device LT6911UXC.__

> For recent news check out our [Blog](https://blog.zhaw.ch/high-performance/).

## Table of contents

[Insert Driver into L4T Sources](#insert-driver-into-your-l4t-sources): Instructions on how to add this driver to your Jetson Linux Driver Package (L4T).

---

## Insert Driver into your L4T Sources

The driver can be used with the Linux Driver Packages (L4T) for NVIDIA Jetson Modules. We have tested the driver with the NVIDIA Jetson AGX Xavier. In order to adapt the driver to your hardware setup use the detailed description regarding device tree parameters in the file `lt6911uxc.txt`. 

For geneneral information and instructions regarding the NVIDIA L4T release or how to manually build the linux kernel for your Jetson device consult: [L4T release package](https://developer.nvidia.com/embedded/linux-tegra-r32.4.3) , [NVIDIA Kernel Customization](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/kernel_custom.html#wwpID0E0WD0HA).

### Insert Driver into Kernel Sources 
 1) Locate the subdirectory of multimedia drivers for encoder, decoder and other helper chips as part of the NVIDIA overlay in (`{L4T-RELEASE}/kernel/nvidia/drivers/media/i2c/`).

 2) Add the driver source to the other present drivers 

 3) **Makefile:** Open the file `nvidia/drivers/media/i2c/Makefile` and add the following line:
    ```
    obj-$(CONFIG_VIDEO_LT6911UXC) += lt6911uxc.o
    ```
 4) **KConfig:** Open the file `nvidia/drivers/media/i2c/Kconfig` and add the following entry:

    ```
    config VIDEO_LT6911UXC
	    tristate "Lontium LT6911UXC decoder"
	    depends on VIDEO_V4L2 && I2C && VIDEO_V4L2_SUBDEV_API
	    help
	      Support for the Lontium LT6911UXC HDMI to MIPI CSI-2 bridge.

	      To compile this driver as a module, choose M here: the
	      module will be called lt6911uxc.
    ```

After those steps, the driver module can be enabled in the menuconfig and the custom kernel including this driver compiled.