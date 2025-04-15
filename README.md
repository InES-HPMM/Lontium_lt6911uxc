# Lontium_lt6911uxc

This repository contains a driver for the lt6911uxc HDMI2.0 to MIPI CSI converter from Lontium. The driver was tested on an Nvidia Jetson Orin NX running L4T 36.X (Jetpack 6).

---

[![logo](https://github.com/InES-HPMM/FPD-LinkIII_Raspberry_HW/blob/master/images/ines_logo.png)](https://www.zhaw.ch/en/engineering/institutes-centres/ines/ "Homepage")

__The group High Performance Multimedia from the Institute of Embedded Systems associated with ZHAW School of Engineering proudly presents the new version of the open source driver for lontium device LT6911UXC.__

> For recent news check out our [Blog](https://blog.zhaw.ch/high-performance/).

## Table of contents

[Insert Driver into L4T Sources](#insert-driver-into-your-l4t-sources): Instructions on how to add this driver to your Jetson Linux Driver Package (L4T).

[Information about the usage with LT6911](#information-about-the-usage-with-lt6911): Information on how to use the Lontium lt6911uxc in your Jetson Linux environment.

---

## Insert Driver into your L4T Sources

The driver can be used with the Linux Driver Packages (L4T) for NVIDIA Jetson Modules and was tested using a Jetson Orin NX 16GB on a custom carrier board. 
In order to adapt the driver to your hardware setup use the detailed description regarding device tree parameters in the file `lt6911uxc.txt`.

For geneneral information and instructions regarding the NVIDIA L4T release or how to manually build the linux kernel for your Jetson device consult: [L4T release package](https://developer.nvidia.com/embedded/jetson-linux-r3643) , [NVIDIA Kernel Customization](https://docs.nvidia.com/jetson/archives/r36.4.3/DeveloperGuide/SD/Kernel/KernelCustomization.html).

### Insert Driver into Kernel Sources 
 1) Locate the subdirectory of multimedia drivers for encoder, decoder and other helper chips as part of the NVIDIA overlay in (`{L4T_Workspace}/source/nvidia-oot/drivers/media/i2c/`).

 2) Add the driver source files to the other present drivers 

 3) **Makefile:** Open the file `nvidia/drivers/media/i2c/Makefile` and add the following line:
    ```
    obj-m += lt6911uxc_zhaw.o
    ```
 4) Rebuild your kernel according to NVIDIA's developer Guide mentioned above. With correct hardware description in the device-tree, the driver should automatically load on boot-up and should add a video device accordingly.

After those steps, the driver module can be enabled in the menuconfig and the custom kernel including this driver compiled.

## Information about the usage with LT6911
Even though this kernel driver implements lt6911uxc to be used within your Jetson Linux environment, there are a couple important points to mention at this point.
1. In Addition to implement this driver, the lt6911uxc IC has to be flashed with a working firmware by provided by Lontium directly.
2. Upon HDMI device connection or format change, the Lontium emits an interrupt over the specified interrupt line. Make sure that this interrupt is routed to a Jetson GPIO and configured accordingly in the device-tree. To apply formats detected by the lt6911uxc, you need to apply the timings to the VI using v4l2-ctl:
   ```bash
   v4l2-ctl --device=<video-device> --query-dv-timings
   v4l2-ctl --device=/dev/video0 --set-dv-bt-timings query
   ```
3. With a suiting firmware running on the IC, is capable streaming 4k60 HDMI input sources via 8-lane CSI. For this specific use case, feel free to [contact us](mailto:blog.zhaw.ch).