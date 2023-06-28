/*
 * Copyright (c) 2017 Christer Weinigel.
 * Copyright (c) 2017, I-SENSE group of ICCS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB device controller shim driver for GD32 devices
 *
 * This driver uses the GD32 std low level drivers to talk to the USB
 * device controller on the GD32 family of devices using the
 * GD32 HAL layer.
 */

#include <errno.h>
#include <string.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/interrupt_controller/gd32_exti.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>

#include "drv_usbd_int.h"
#include "drv_usb_dev.h"
#include "drv_usb_core.h"
#include "drv_usb_regs.h"
#include "usb_ch9_std.h"
#include "usbd_core.h"
#include "usbd_enum.h"
#include "usbd_transc.h"
#include "usb_conf.h"

#define LOG_LEVEL CONFIG_USB_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(usb_dc_gd32);


#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_usbfs) && DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_usbhs)
#error "Only one interface should be enabled at a time, USBFS or USBHS"
#endif

/*
 * Vbus sensing is determined based on the presence of the hardware detection
 * pin(s) in the device tree. E.g: pinctrl-0 = <&usb_otg_fs_vbus_pa9 ...>;
 *
 * The detection pins are dependent on the enabled USB driver and the physical
 * interface(s) offered by the hardware. These are mapped to PA9 and/or PB13
 * (subject to MCU), being the former the most widespread option.
 */
#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_usbhs)
    #define USE_USB_HS
    #define DT_DRV_COMPAT           gd_gd32_usbhs
    #define USB_IRQ_NAME            usbhs
    #define USB_VBUS_SENSING        (DT_NODE_EXISTS(DT_CHILD(DT_NODELABEL(pinctrl), usbhs_vbus_pa9)) || DT_NODE_EXISTS(DT_CHILD(DT_NODELABEL(pinctrl), usbhs_vbus_pb13)))
#elif DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_usbfs)
    #define USE_USB_FS
    #define DT_DRV_COMPAT           gd_gd32_usbfs
    #define USB_IRQ_NAME            usbfs
    #define USB_VBUS_SENSING        DT_NODE_EXISTS(DT_CHILD(DT_NODELABEL(pinctrl), usb_otg_fs_vbus_pa9))
#elif DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_usbd)
    #define USE_USBD
    #define DT_DRV_COMPAT           gd_gd32_usbd
    #define USB_IRQ_NAME            usbd
    #define USB_VBUS_SENSING        false
#endif

#define USB_BASE_ADDRESS            DT_INST_REG_ADDR(0)
#define USB_IRQ                     DT_INST_IRQ_BY_NAME(0, USB_IRQ_NAME, irq)
#define USB_IRQ_PRI                 DT_INST_IRQ_BY_NAME(0, USB_IRQ_NAME, priority)
#define USB_NUM_BIDIR_ENDPOINTS     DT_INST_PROP(0, num_bidir_endpoints)
#define USB_RAM_SIZE                DT_INST_PROP(0, ram_size)
#define USB_CLOCK_BITS              DT_INST_CLOCKS_CELL(0, id)
#if DT_INST_NODE_HAS_PROP(0, maximum_speed)
    #define USB_MAXIMUM_SPEED       DT_INST_PROP(0, maximum_speed)
#endif

PINCTRL_DT_INST_DEFINE(0);
#define USB_PINCTRL_CFG             PINCTRL_DT_INST_DEV_CONFIG_GET(0)

// #define USBHS_EMB_FS_PHY     (DT_HAS_COMPAT_STATUS_OKAY(usbhs_fs_phy) && DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_usbhs))
// #define USBHS_EMB_HS_PHY     (DT_HAS_COMPAT_STATUS_OKAY(usbhs_hs_phy) && DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_usbhs))
// #define USBHS_ULPI_PHY       (DT_HAS_COMPAT_STATUS_OKAY(usbhs_ulpi_phy) && DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_usbhs))

// #if DT_HAS_COMPAT_STATUS_OKAY(usbhs_fs_phy)
//     #define DT_DRV_COMPAT11           gd_gd32_usbhs
// #elif DT_HAS_COMPAT_STATUS_OKAY(usbhs_hs_phy)
//     #define DT_DRV_COMPAT12           gd_gd32_usbhs
// #endif

/*
 * USBD, USBFS and USBHS are defined in GD32 HAL and allows to distinguish between 
 * two kind of USB DC. GD32 F4 and H7 series support USBFS device controller. 
 * GD32 F1 and F3 series support either USBD or USBFS device controller.
 */
#define EP0_MPS                     64
#define EP_MPS                      64

/* Size of a USB SETUP packet */
#define SETUP_SIZE                  8

/* Helper macros to make it easier to work with endpoint numbers */
#define USB_EP0_IDX                 0
#define USB_EP0_IN                  (USB_EP0_IDX | USB_EP_DIR_IN)
#define USB_EP0_OUT                 (USB_EP0_IDX | USB_EP_DIR_OUT)

/* Endpoint state */
struct usb_dc_gd32_ep_state {
    uint16_t ep_mps;            /** Endpoint max packet size */
    uint16_t ep_pma_buf_len;    /** Previously allocated buffer size */
    uint8_t ep_type;            /** Endpoint type (GD32 HAL enum) */
    uint8_t ep_stalled;         /** Endpoint stall flag */
    usb_dc_ep_callback cb;      /** Endpoint callback function */
    uint32_t read_count;        /** Number of bytes in read buffer  */
    uint32_t read_offset;       /** Current offset in read buffer */
    struct k_sem write_sem;     /** Write boolean semaphore */
};

/* Clock configure */
struct gd32_usb_config {
    uint16_t clkid;
    const struct pinctrl_dev_config *pcfg;
};

/* Driver state */
struct usb_dc_gd32_state {
    usb_core_driver udev;
    usb_dc_status_callback status_cb; /* Status callback */
    struct usb_dc_gd32_ep_state out_ep_state[USB_NUM_BIDIR_ENDPOINTS];
    struct usb_dc_gd32_ep_state in_ep_state[USB_NUM_BIDIR_ENDPOINTS];
    uint8_t ep_buf[USB_NUM_BIDIR_ENDPOINTS][EP_MPS];
};

static struct usb_dc_gd32_state usb_dc_gd32_state;
static struct gd32_usb_config usb_dc_gd32_cfg = {.clkid = USB_CLOCK_BITS, .pcfg = USB_PINCTRL_CFG};

/* Internal functions */

static struct usb_dc_gd32_ep_state *usb_dc_gd32_get_ep_state(uint8_t ep)
{
    struct usb_dc_gd32_ep_state *ep_state_base;

    if (USB_EP_GET_IDX(ep) >= USB_NUM_BIDIR_ENDPOINTS) {
        return NULL;
    }

    if (USB_EP_DIR_IS_OUT(ep)) {
        ep_state_base = usb_dc_gd32_state.out_ep_state;
    } else {
        ep_state_base = usb_dc_gd32_state.in_ep_state;
    }

    return ep_state_base + USB_EP_GET_IDX(ep);
}

static void usb_dc_gd32_isr(const void *arg)
{
    usbd_isr(&usb_dc_gd32_state.udev);
}

static int usb_dc_gd32_clock_enable(void)
{
    uint32_t usb_prescaler = 0, system_clock = 0;

    system_clock = rcu_clock_freq_get(CK_SYS);

    if (system_clock == 48000000U) {
        usb_prescaler = RCU_CKUSB_CKPLL_DIV1;
    } else if (system_clock == 72000000U) {
        usb_prescaler = RCU_CKUSB_CKPLL_DIV1_5;
    } else if (system_clock == 120000000U) {
        usb_prescaler = RCU_CKUSB_CKPLL_DIV2_5;
    } else if (system_clock == 168000000U) {
        usb_prescaler = RCU_CKUSB_CKPLL_DIV3_5;
    } else {
    }

    rcu_usb_clock_config(usb_prescaler);

    if (!device_is_ready(GD32_CLOCK_CONTROLLER)) {
        LOG_ERR("clock control device not ready");
       return -ENODEV;
    }

    if (clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t *)&usb_dc_gd32_cfg.clkid) != 0) {
        LOG_ERR("Unable to enable USB clock");
       return -EIO;
    }

#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_usbhs)
    #if DT_HAS_COMPAT_STATUS_OKAY(usbhs_ulpi_phy)
        rcu_periph_clock_enable(RCU_ULPI);
    #endif
#endif

    return 0;
}

static int usb_dc_gd32_clock_disable(void)
{
    if (clock_control_off(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t *)&usb_dc_gd32_cfg.clkid) != 0) {
        LOG_ERR("Unable to disable USB clock");
       return -EIO;
    }

    return 0;
}

static int usb_dc_gd32_init(void)
{
    unsigned int i;
    int ret;
    usb_core_driver *udev;

    LOG_DBG("Pinctrl signals configuration");
    ret = pinctrl_apply_state(usb_dc_gd32_cfg.pcfg , PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("USB pinctrl setup failed (%d)", ret);
        return ret;
    }

    udev = &usb_dc_gd32_state.udev;

    LOG_DBG("init the USB device stack");

    /* configure USB capabilities */
    (void)usb_basic_init (&udev->bp, &udev->regs);

    /* initializes the USB core*/
    (void)usb_core_init (udev->bp, &udev->regs);

    /* set device disconnect */
    usbd_disconnect (udev);

    /* initializes device mode */
    (void)usb_devcore_init (udev);

    /* set device connect */
    usbd_connect (udev);

    usb_dc_gd32_state.out_ep_state[USB_EP0_IDX].ep_mps = EP0_MPS;
    usb_dc_gd32_state.out_ep_state[USB_EP0_IDX].ep_type = USB_DC_EP_CONTROL;
    usb_dc_gd32_state.in_ep_state[USB_EP0_IDX].ep_mps = EP0_MPS;
    usb_dc_gd32_state.in_ep_state[USB_EP0_IDX].ep_type = USB_DC_EP_CONTROL;

    /* TODO: make this dynamic (depending usage) */
    for (i = 0U; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
        k_sem_init(&usb_dc_gd32_state.in_ep_state[i].write_sem, 1, 1);
    }

    IRQ_CONNECT(USB_IRQ, USB_IRQ_PRI, usb_dc_gd32_isr, 0, 0);
    irq_enable(USB_IRQ);

    return 0;
}




/* Zephyr USB device controller API implementation */

int usb_dc_attach(void)
{
    int ret;

    LOG_DBG("");

    ret = usb_dc_gd32_clock_enable();
    if (ret) {
        return ret;
    }

    ret = usb_dc_gd32_init();
    if (ret) {
        return ret;
    }

    return 0;
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb)
{
    struct usb_dc_gd32_ep_state *ep_state = usb_dc_gd32_get_ep_state(ep);

    LOG_DBG("ep 0x%02x", ep);

    if (!ep_state) {
        return -EINVAL;
    }

    ep_state->cb = cb;

    return 0;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
    LOG_DBG("");

    usb_dc_gd32_state.status_cb = cb;
}

int usb_dc_set_address(const uint8_t addr)
{
    LOG_DBG("addr %u (0x%02x)", addr, addr);

    /* set usb device address */
    usbd_addr_set(&usb_dc_gd32_state.udev, addr);

    return 0;
}

int usb_dc_ep_start_read(uint8_t ep, uint8_t *data, uint32_t max_data_len)
{
    LOG_DBG("ep 0x%02x, len %u", ep, max_data_len);

    /* we flush USB_EP0_IN by doing a 0 length receive on it */
    if (!USB_EP_DIR_IS_OUT(ep) && (ep != USB_EP0_IN || max_data_len)) {
        LOG_ERR("invalid ep 0x%02x", ep);
        return -EINVAL;
    }

    if (max_data_len > EP_MPS) {
        max_data_len = EP_MPS;
    }

    usbd_ep_recev (&usb_dc_gd32_state.udev, ep,
                    usb_dc_gd32_state.ep_buf[USB_EP_GET_IDX(ep)],
                    max_data_len);

    return 0;
}

int usb_dc_ep_get_read_count(uint8_t ep, uint32_t *read_bytes)
{
    if (!USB_EP_DIR_IS_OUT(ep) || !read_bytes) {
        LOG_ERR("invalid ep 0x%02x", ep);
        return -EINVAL;
    }

    *read_bytes = usbd_rxcount_get(&usb_dc_gd32_state.udev, USB_EP_GET_IDX(ep));

    return 0;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data * const cfg)
{
    uint8_t ep_idx = USB_EP_GET_IDX(cfg->ep_addr);

    LOG_DBG("ep %x, mps %d, type %d", cfg->ep_addr, cfg->ep_mps, cfg->ep_type);

    if ((cfg->ep_type == USB_DC_EP_CONTROL) && ep_idx) {
        LOG_ERR("invalid endpoint configuration");
        return -1;
    }

    if (ep_idx > (USB_NUM_BIDIR_ENDPOINTS - 1)) {
        LOG_ERR("endpoint index/address out of range");
        return -1;
    }

    return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data * const ep_cfg)
{
    uint8_t ep = ep_cfg->ep_addr;
    struct usb_dc_gd32_ep_state *ep_state = usb_dc_gd32_get_ep_state(ep);

    if (!ep_state) {
        return -EINVAL;
    }

    LOG_DBG("ep 0x%02x, previous ep_mps %u, ep_mps %u, ep_type %u",
        ep_cfg->ep_addr, ep_state->ep_mps, ep_cfg->ep_mps,
        ep_cfg->ep_type);

    ep_state->ep_mps = ep_cfg->ep_mps;

    switch (ep_cfg->ep_type) {
    case USB_DC_EP_CONTROL:
        ep_state->ep_type = USB_DC_EP_CONTROL;
        break;
    case USB_DC_EP_ISOCHRONOUS:
        ep_state->ep_type = USB_DC_EP_ISOCHRONOUS;
        break;
    case USB_DC_EP_BULK:
        ep_state->ep_type = USB_DC_EP_BULK;
        break;
    case USB_DC_EP_INTERRUPT:
        ep_state->ep_type = USB_DC_EP_INTERRUPT;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

int usb_dc_ep_set_stall(const uint8_t ep)
{
    struct usb_dc_gd32_ep_state *ep_state = usb_dc_gd32_get_ep_state(ep);

    LOG_DBG("ep 0x%02x", ep);

    if (!ep_state) {
        return -EINVAL;
    }

    usbd_ep_stall(&usb_dc_gd32_state.udev, ep);

    ep_state->ep_stalled = 1U;

    return 0;
}

int usb_dc_ep_clear_stall(const uint8_t ep)
{
    struct usb_dc_gd32_ep_state *ep_state = usb_dc_gd32_get_ep_state(ep);

    LOG_DBG("ep 0x%02x", ep);

    if (!ep_state) {
        return -EINVAL;
    }

    usbd_ep_stall_clear(&usb_dc_gd32_state.udev, ep);

    ep_state->ep_stalled = 0U;
    ep_state->read_count = 0U;

    return 0;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled)
{
    struct usb_dc_gd32_ep_state *ep_state = usb_dc_gd32_get_ep_state(ep);

    LOG_DBG("ep 0x%02x", ep);

    if (!ep_state || !stalled) {
        return -EINVAL;
    }

    *stalled = ep_state->ep_stalled;

    return 0;
}

int usb_dc_ep_enable(const uint8_t ep)
{
    usb_desc_ep ep_desc;
    struct usb_dc_gd32_ep_state *ep_state = usb_dc_gd32_get_ep_state(ep);

    LOG_DBG("ep 0x%02x", ep);

    if (!ep_state) {
        return -EINVAL;
    }

    LOG_DBG("HAL_PCD_EP_Open(0x%02x, %u, %u)", ep, ep_state->ep_mps, ep_state->ep_type);

    ep_desc.bEndpointAddress = ep;
    ep_desc.wMaxPacketSize = ep_state->ep_mps;
    ep_desc.bmAttributes = ep_state->ep_type;

    /* initialize the data TX endpoint */
    usbd_ep_setup(&usb_dc_gd32_state.udev, &ep_desc);

    if (USB_EP_DIR_IS_OUT(ep) && ep != USB_EP0_OUT) {
        return usb_dc_ep_start_read(ep, usb_dc_gd32_state.ep_buf[USB_EP_GET_IDX(ep)], EP_MPS);
    }

    return 0;
}

int usb_dc_ep_disable(const uint8_t ep)
{
    struct usb_dc_gd32_ep_state *ep_state = usb_dc_gd32_get_ep_state(ep);

    LOG_DBG("ep 0x%02x", ep);

    if (!ep_state) {
        return -EINVAL;
    }

    usbd_ep_clear(&usb_dc_gd32_state.udev, ep);

    return 0;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data, const uint32_t data_len, uint32_t * const ret_bytes)
{
    struct usb_dc_gd32_ep_state *ep_state = usb_dc_gd32_get_ep_state(ep);
    uint32_t len = data_len;
    int ret = 0;

    LOG_DBG("ep 0x%02x, len %u", ep, data_len);

    if (!ep_state || !USB_EP_DIR_IS_IN(ep)) {
        LOG_ERR("invalid ep 0x%02x", ep);
        return -EINVAL;
    }

    ret = k_sem_take(&ep_state->write_sem, K_NO_WAIT);
    if (ret) {
        LOG_ERR("Unable to get write lock (%d)", ret);
        return -EAGAIN;
    }

    if (!k_is_in_isr()) {
        irq_disable(USB_IRQ);
    }

    if (ep == USB_EP0_IN && len > USB_MAX_CTRL_MPS) {
        len = USB_MAX_CTRL_MPS;
    }

    usbd_ep_send(&usb_dc_gd32_state.udev, ep, (void *)data, len);

    if (!ret && ep == USB_EP0_IN && len > 0) {
        /* Wait for an empty package as from the host.
         * This also flushes the TX FIFO to the host.
         */
        usb_dc_ep_start_read(ep, NULL, 0);
    }

    if (!k_is_in_isr()) {
        irq_enable(USB_IRQ);
    }

    if (!ret && ret_bytes) {
        *ret_bytes = len;
    }

    return ret;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data, uint32_t max_data_len, uint32_t *read_bytes)
{
    struct usb_dc_gd32_ep_state *ep_state = usb_dc_gd32_get_ep_state(ep);
    uint32_t read_count;

    if (!ep_state) {
        LOG_ERR("Invalid Endpoint %x", ep);
        return -EINVAL;
    }

    read_count = ep_state->read_count;

    LOG_DBG("ep 0x%02x, %u bytes, %u+%u, %p", ep, max_data_len,
        ep_state->read_offset, read_count, data);

    if (!USB_EP_DIR_IS_OUT(ep)) { /* check if OUT ep */
        LOG_ERR("Wrong endpoint direction: 0x%02x", ep);
        return -EINVAL;
    }

    /* When both buffer and max data to read are zero, just ignore reading
     * and return available data in buffer. Otherwise, return data
     * previously stored in the buffer.
     */
    if (data) {
        read_count = MIN(read_count, max_data_len);
        memcpy(data, usb_dc_gd32_state.ep_buf[USB_EP_GET_IDX(ep)] + ep_state->read_offset, read_count);
        ep_state->read_count -= read_count;
        ep_state->read_offset += read_count;
    } else if (max_data_len) {
        LOG_ERR("Wrong arguments");
    }

    if (read_bytes) {
        *read_bytes = read_count;
    }

    return 0;
}

int usb_dc_ep_read_continue(uint8_t ep)
{
    struct usb_dc_gd32_ep_state *ep_state = usb_dc_gd32_get_ep_state(ep);

    if (!ep_state || !USB_EP_DIR_IS_OUT(ep)) { /* Check if OUT ep */
        LOG_ERR("Not valid endpoint: %02x", ep);
        return -EINVAL;
    }

    /* If no more data in the buffer, start a new read transaction.
     * DataOutStageCallback will called on transaction complete.
     */
    if (!ep_state->read_count) {
        usb_dc_ep_start_read(ep, usb_dc_gd32_state.ep_buf[USB_EP_GET_IDX(ep)], EP_MPS);
    }

    return 0;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data, const uint32_t max_data_len, uint32_t * const read_bytes)
{
    if (usb_dc_ep_read_wait(ep, data, max_data_len, read_bytes) != 0) {
        return -EINVAL;
    }

    if (usb_dc_ep_read_continue(ep) != 0) {
        return -EINVAL;
    }

    return 0;
}

int usb_dc_ep_halt(const uint8_t ep)
{
    return usb_dc_ep_set_stall(ep);
}

int usb_dc_ep_flush(const uint8_t ep)
{
    struct usb_dc_gd32_ep_state *ep_state = usb_dc_gd32_get_ep_state(ep);

    if (!ep_state) {
        return -EINVAL;
    }

    LOG_ERR("Not implemented");

    return 0;
}

int usb_dc_ep_mps(const uint8_t ep)
{
    struct usb_dc_gd32_ep_state *ep_state = usb_dc_gd32_get_ep_state(ep);

    if (!ep_state) {
        return -EINVAL;
    }

    return ep_state->ep_mps;
}

int usb_dc_wakeup_request(void)
{
    /* set remote wakeup signaling */
    usb_rwkup_set(&usb_dc_gd32_state.udev);

    /* Must be active from 1ms to 15ms as per reference manual. */
    k_sleep(K_MSEC(2));

    /* reset remote wakeup signaling */
    usb_rwkup_set(&usb_dc_gd32_state.udev);

    return 0;
}

int usb_dc_detach(void)
{
    int ret;

    LOG_DBG("USB DeInit");

    /* disable the global interrupts */
    usb_globalint_disable(&usb_dc_gd32_state.udev.regs);

    /* stop the device and clean up FIFOs */
    usb_dev_stop(&usb_dc_gd32_state.udev);

    /* set device disconnect */
    usbd_disconnect (&usb_dc_gd32_state.udev);

    ret = usb_dc_gd32_clock_disable();
    if (ret) {
        return ret;
    }

    if (irq_is_enabled(USB_IRQ)) {
        irq_disable(USB_IRQ);
    }

    return 0;
}

int usb_dc_reset(void)
{
    LOG_ERR("Not implemented");

    return 0;
}





/* Callbacks from the GD32 HAL code */
void zephyr_usbd_int_reset(usb_core_driver *udev)
{
    int i;

    /* The DataInCallback will never be called at this point for any pending
     * transactions. Reset the IN semaphores to prevent perpetual locked state.
     * */
    for (i = 0; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
        k_sem_give(&usb_dc_gd32_state.in_ep_state[i].write_sem);
    }

    if (usb_dc_gd32_state.status_cb) {
        usb_dc_gd32_state.status_cb(USB_DC_RESET, NULL);
    }
}

void zephyr_usbd_int_suspend (usb_core_driver *udev)
{
    if (usb_dc_gd32_state.status_cb) {
        usb_dc_gd32_state.status_cb(USB_DC_SUSPEND, NULL);
    }
}

void zephyr_usbd_int_wakeup (usb_core_driver *udev)
{
    if (usb_dc_gd32_state.status_cb) {
        usb_dc_gd32_state.status_cb(USB_DC_RESUME, NULL);
    }
}

void zephyr_usbd_int_sof(usb_core_driver *udev)
{
    usb_dc_gd32_state.status_cb(USB_DC_SOF, NULL);
}

void zephyr_usbd_setup_transc (usb_core_driver *udev)
{
    usb_req *gd_req = &usb_dc_gd32_state.udev.dev.control.req;

    struct usb_setup_packet setup = {
        .bmRequestType = gd_req->bmRequestType,
        .bRequest = gd_req->bRequest,
        .wIndex = gd_req->wIndex,
        .wLength = gd_req->wLength,
        .wValue = gd_req->wValue,
    } ;

    struct usb_dc_gd32_ep_state *ep_state;

    ep_state = usb_dc_gd32_get_ep_state(USB_EP0_OUT); /* can't fail for ep0 */
    __ASSERT(ep_state, "No corresponding ep_state for EP0");

    ep_state->read_count = SETUP_SIZE;
    ep_state->read_offset = 0U;
    memcpy(&usb_dc_gd32_state.ep_buf[USB_EP0_IDX], &usb_dc_gd32_state.udev.dev.control.req, ep_state->read_count);

    if (ep_state->cb) {
        ep_state->cb(USB_EP0_OUT, USB_DC_EP_SETUP);

        if (!(setup.wLength == 0U) && usb_reqtype_is_to_device(&setup)) {
            usb_dc_ep_start_read(USB_EP0_OUT, usb_dc_gd32_state.ep_buf[USB_EP0_IDX], setup.wLength);
        }
    }
}

void zephyr_usbd_out_transc (usb_core_driver *udev, uint8_t epnum)
{
    uint8_t ep_idx = USB_EP_GET_IDX(epnum);
    uint8_t ep = ep_idx | USB_EP_DIR_OUT;
    struct usb_dc_gd32_ep_state *ep_state = usb_dc_gd32_get_ep_state(ep);

    /* Transaction complete, data is now stored in the buffer and ready
     * for the upper stack (usb_dc_ep_read to retrieve).
     */
    usb_dc_ep_get_read_count(ep, &ep_state->read_count);
    ep_state->read_offset = 0U;

    if (ep_state->cb) {
        ep_state->cb(ep, USB_DC_EP_DATA_OUT);
    }
}

void zephyr_usbd_in_transc (usb_core_driver *udev, uint8_t epnum)
{
    uint8_t ep_idx = USB_EP_GET_IDX(epnum);
    uint8_t ep = ep_idx | USB_EP_DIR_IN;
    struct usb_dc_gd32_ep_state *ep_state = usb_dc_gd32_get_ep_state(ep);

    __ASSERT(ep_state, "No corresponding ep_state for ep");

    k_sem_give(&ep_state->write_sem);

    if (ep_state->cb) {
        ep_state->cb(ep, USB_DC_EP_DATA_IN);
    }
}
