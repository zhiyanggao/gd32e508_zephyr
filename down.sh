#!/bin/bash

mkdir -p modules/hal/gigadevice/include/dt-bindings/pinctrl;
cp /GY/zephyrproject/modules/hal/gigadevice/include/dt-bindings/pinctrl/gd32e508xx-afio.h modules/hal/gigadevice/include/dt-bindings/pinctrl;
cp "/GY/zephyrproject/modules/hal/gigadevice/include/dt-bindings/pinctrl/gd32e508z(e-c)xx-pinctrl.h" modules/hal/gigadevice/include/dt-bindings/pinctrl;
cp /GY/zephyrproject/modules/hal/gigadevice/include/dt-bindings/pinctrl/gd32h8xx-afio.h modules/hal/gigadevice/include/dt-bindings/pinctrl;
cp /GY/zephyrproject/modules/hal/gigadevice/include/dt-bindings/pinctrl/gd32h8xx-pinctrl.h modules/hal/gigadevice/include/dt-bindings/pinctrl;

# new 2023-03-09
mkdir -p modules/hal/gigadevice/gd32e50x;
cp -rf /GY/zephyrproject/modules/hal/gigadevice/gd32e50x/GD32E50x_usbhs_library modules/hal/gigadevice/gd32e50x
# new 2023-03-09

mkdir -p modules/hal/gigadevice/pinconfigs
cp /GY/zephyrproject/modules/hal/gigadevice/pinconfigs/gd32e508xx.yml modules/hal/gigadevice/pinconfigs;

mkdir -p boards/arm;
cp -rf  /GY/zephyrproject/zephyr/boards/arm/gd32e508z_eval boards/arm;

mkdir -p soc/arm/gigadevice/gd32e50x;
cp /GY/zephyrproject/zephyr/soc/arm/gigadevice/gd32e50x/Kconfig.soc soc/arm/gigadevice/gd32e50x/Kconfig.soc;
cp /GY/zephyrproject/zephyr/soc/arm/gigadevice/gd32e50x/Kconfig.defconfig.gd32e508 soc/arm/gigadevice/gd32e50x/Kconfig.defconfig.gd32e508;

mkdir -p samples;
cp -rf /GY/zephyrproject/zephyr/samples/gd32e508_test_demo samples;

mkdir -p dts/arm/gigadevice/gd32e50x
cp /GY/zephyrproject/zephyr/dts/arm/gigadevice/gd32e50x/gd32e50x.dtsi dts/arm/gigadevice/gd32e50x;
cp /GY/zephyrproject/zephyr/dts/arm/gigadevice/gd32e50x/gd32e508xe.dtsi dts/arm/gigadevice/gd32e50x;

# new 2023-03-09
mkdir -p dts/bindings/usb
cp "/GY/zephyrproject/zephyr/dts/bindings/usb/gd,gd32-usbhs.yaml"    dts/bindings/usb;
# new 2023-03-09

mkdir -p modules/hal/gigadevice/gd32e50x/support;
cp /GY/zephyrproject/modules/hal/gigadevice/gd32e50x/support/GigaDevice.GD32E50x_DFP.1.3.3.pack modules/hal/gigadevice/gd32e50x/support;

# new 2023-03-09
mkdir -p drivers/usb/device;
cp /GY/zephyrproject/zephyr/drivers/usb/device/usb_dc_gd32.c    drivers/usb/device;
cp /GY/zephyrproject/zephyr/drivers/usb/device/CMakeLists.txt   drivers/usb/device;
cp /GY/zephyrproject/zephyr/drivers/usb/device/Kconfig          drivers/usb/device;

mkdir -p  zephyr/modules;
cp -rf /GY/zephyrproject/zephyr/modules/hal_gigadevice          zephyr/modules;
# new 2023-03-09

#new 2023-03-15
mkdir -p drivers/can;
cp /GY/zephyrproject/zephyr/drivers/can/can_gd32.c   drivers/can;
cp /GY/zephyrproject/zephyr/drivers/can/can_gd32.h   drivers/can;
cp /GY/zephyrproject/zephyr/drivers/can/CMakeLists.txt   drivers/can;
cp /GY/zephyrproject/zephyr/drivers/can/Kconfig          drivers/can;
cp /GY/zephyrproject/zephyr/drivers/can/Kconfig.gd32    drivers/can;

mkdir -p drivers/clock_control;
cp /GY/zephyrproject/zephyr/include/zephyr/drivers/clock_control/gd32.h     drivers/clock_control;

mkdir -p dts/bindings/can
cp "/GY/zephyrproject/zephyr/dts/bindings/can/gd,gd32-can.yaml"   dts/bindings/can;
#new 2023-03-15