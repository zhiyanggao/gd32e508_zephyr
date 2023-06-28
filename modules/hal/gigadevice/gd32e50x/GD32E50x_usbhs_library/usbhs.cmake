# SPDX-License-Identifier: Apache-2.0

# set a minimum require version of CMake
cmake_minimum_required(VERSION 3.17)

set(PATH_DEF ${ZEPHYR_HAL_GIGADEVICE_MODULE_DIR}/${CONFIG_SOC_SERIES}/GD32E50x_usbhs_library)

 zephyr_include_directories(
    # ${PATH_DEF}/host/core/Include
    # ${PATH_DEF}/host/class/msc/Include
    # ${PATH_DEF}/host/class/hid/Include
    # ${PATH_DEF}/host/class/cdc/Include
    ${PATH_DEF}/driver/Include
    ${PATH_DEF}/ustd/common
    # ${PATH_DEF}/ustd/class/msc
    # ${PATH_DEF}/ustd/class/hid
    # ${PATH_DEF}/ustd/class/cdc
    ${PATH_DEF}/device/core/Include
    # ${PATH_DEF}/device/class/iap/Include
    # ${PATH_DEF}/device/class/dfu/Include/
    # ${PATH_DEF}/device/class/printer/Include
    # ${PATH_DEF}/device/class/msc/Include/
    # ${PATH_DEF}/device/class/audio/Include
    # ${PATH_DEF}/device/class/hid/Include
    # ${PATH_DEF}/device/class/cdc/Include
    ${PATH_DEF}/example
)

# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/host/core/Source/usbh_core.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/host/core/Source/usbh_transc.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/host/core/Source/usbh_enum.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/host/core/Source/usbh_pipe.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/host/class/msc/Source/usbh_msc_fatfs.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/host/class/msc/Source/usbh_msc_bbb.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/host/class/msc/Source/usbh_msc_core.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/host/class/msc/Source/usbh_msc_scsi.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/host/class/hid/Source/usbh_hid_core.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/host/class/hid/Source/usbh_standard_hid.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/host/class/cdc/Source/usbh_cdc_core.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/driver/Source/drv_usbd_int.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/driver/Source/drv_usb_dev.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/driver/Source/drv_usb_host.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/driver/Source/drv_usbh_int.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/driver/Source/drv_usb_core.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/core/Source/usbd_transc.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/core/Source/usbd_enum.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/core/Source/usbd_core.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/class/iap/Source/usb_iap_libusb.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/class/iap/Source/usb_iap_core.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/class/dfu/Source/dfu_core.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/class/dfu/Source/dfu_mem.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/class/printer/Source/printer_core.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/class/msc/Source/usbd_msc_scsi.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/class/msc/Source/usbd_msc_core.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/class/msc/Source/usbd_msc_bbb.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/class/msc/Source/usbd_msc_efs.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/class/audio/Source/audio_out_itf.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/class/audio/Source/audio_core.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/class/hid/Source/custom_hid_core.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/class/hid/Source/standard_hid_core.c)
# zephyr_library_sources_ifdef(CONFIG_USE_GD32_USBHS  ${PATH_DEF}/device/class/cdc/Source/cdc_acm_core.c)
