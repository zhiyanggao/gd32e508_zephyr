#!/bin/bash

zephyr_path="/GY/zephyrproject"

cp modules/hal/gigadevice/include/dt-bindings/pinctrl/gd32e508xx-afio.h ${zephyr_path}/modules/hal/gigadevice/include/dt-bindings/pinctrl;
cp "modules/hal/gigadevice/include/dt-bindings/pinctrl/gd32e508z(e-c)xx-pinctrl.h" ${zephyr_path}/modules/hal/gigadevice/include/dt-bindings/pinctrl;

cp modules/hal/gigadevice/pinconfigs/gd32e508xx.yml ${zephyr_path}/modules/hal/gigadevice/pinconfigs;

cp -rf  boards/arm/gd32e508z_eval ${zephyr_path}/zephyr/boards/arm;

cp soc/arm/gigadevice/gd32e50x/Kconfig.soc ${zephyr_path}/zephyr/soc/arm/gigadevice/gd32e50x/Kconfig.soc;
cp soc/arm/gigadevice/gd32e50x/Kconfig.defconfig.gd32e508 ${zephyr_path}/zephyr/soc/arm/gigadevice/gd32e50x/Kconfig.defconfig.gd32e508;

cp -rf samples/gd32e508_test_demo ${zephyr_path}/zephyr/samples ;

cp dts/arm/gigadevice/gd32e50x/gd32e50x.dtsi ${zephyr_path}/zephyr/dts/arm/gigadevice/gd32e50x;
cp dts/arm/gigadevice/gd32e50x/gd32e508xe.dtsi ${zephyr_path}/zephyr/dts/arm/gigadevice/gd32e50x;

cp -rf modules/hal/gigadevice/gd32e50x/support ${zephyr_path}/modules/hal/gigadevice/gd32e50x;

# new 2023-03-09
cp  "dts/bindings/usb/gd,gd32-usbhs.yaml" ${zephyr_path}/zephyr/dts/bindings/usb;  
# new 2023-03-09


# new 2023-03-09
cp    drivers/usb/device/usb_dc_gd32.c     ${zephyr_path}/zephyr/drivers/usb/device;
cp    drivers/usb/device/CMakeLists.txt    ${zephyr_path}/zephyr/drivers/usb/device;
cp    drivers/usb/device/Kconfig           ${zephyr_path}/zephyr/drivers/usb/device;

cp -rf zephyr/modules/hal_gigadevice ${zephyr_path}/zephyr/modules;
# new 2023-03-09

# new 2023-03-09
cp -rf  modules/hal/gigadevice/gd32e50x/GD32E50x_usbhs_library  ${zephyr_path}/modules/hal/gigadevice/gd32e50x;
# new 2023-03-09

#new 2023-03-15
mkdir -p drivers/can;
cp  drivers/can/can_gd32.c ${zephyr_path}/zephyr/drivers/can;
cp  drivers/can/can_gd32.h ${zephyr_path}/zephyr/drivers/can;
cp  drivers/can/CMakeLists.txt ${zephyr_path}/zephyr/drivers/can;
cp  drivers/can/Kconfig ${zephyr_path}/zephyr/drivers/can;
cp  drivers/can/Kconfig.gd32 ${zephyr_path}/zephyr/drivers/can;

cp drivers/clock_control/gd32.h ${zephyr_path}/zephyr/include/zephyr/drivers/clock_control;


cp "dts/bindings/can/gd,gd32-can.yaml"  ${zephyr_path}/zephyr/dts/bindings/can;
#new 2023-03-15
