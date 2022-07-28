/*
The MIT License (MIT)

Copyright (c) 2021 Lancaster University.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

// Adapted from
// https://github.com/raspberrypi/pico-examples/blob/master/usb/device/dev_lowlevel/dev_lowlevel.c
// Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
// SPDX-License-Identifier: BSD-3-Clause

#include "CodalUSB.h"

#if CONFIG_ENABLED(DEVICE_USB)

#include "MessageBus.h"
#include "Event.h"
#include "CodalFiber.h"
#include "CodalDmesg.h"

#include "hardware/structs/usb.h"
#include "hardware/irq.h"
#include "hardware/resets.h"

#include "ram.h"

#define usb_hw_set ((usb_hw_t *)hw_set_alias_untyped(usb_hw))
#define usb_hw_clear ((usb_hw_t *)hw_clear_alias_untyped(usb_hw))

static uint8_t usb_num_endpoints;

// usb_assert(this != NULL) gets optimized away
#define ASSERT_THIS_NOT_NULL() usb_assert(((uint32_t)this >> 8) != 0)

void usb_configure(uint8_t numEndpoints)
{
    usb_assert(usb_num_endpoints == 0);
    usb_assert(numEndpoints > 0);

    usb_num_endpoints = numEndpoints;

    // Reset usb controller
    reset_block(RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

// Clear any previous state in dpram just in case
#pragma GCC diagnostic ignored "-Wclass-memaccess"
    memset(usb_hw, 0, sizeof(*usb_hw));
    memset(usb_dpram, 0, sizeof(*usb_dpram));

    ram_irq_set_priority(USBCTRL_IRQ, 1 << 6);
    // Enable USB interrupt at processor
    ram_irq_set_enabled(USBCTRL_IRQ, true);

    // Mux the controller to the onboard usb phy
    usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;

    // Force VBUS detect so the device thinks it is plugged into a host
    usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;

    // Enable the USB controller in device mode.
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;

    // Enable interrupts for when a buffer is done, when the bus is reset,
    // and when a setup packet is received
    usb_hw->inte = USB_INTS_BUFF_STATUS_BITS | USB_INTS_BUS_RESET_BITS | USB_INTS_SETUP_REQ_BITS;

    CodalUSB *cusb = CodalUSB::usbInstance;
    cusb->initEndpoints();

    // Present full speed device by enabling pull up on DP
    usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

extern "C" void isr_usbctrl(void)
{
    // USB interrupt handler
    CodalUSB *cusb = CodalUSB::usbInstance;

    uint32_t status = usb_hw->ints;

    // Bus is reset
    if (status & USB_INTS_BUS_RESET_BITS)
    {
        DMESG("USB EORST");
        usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
        usb_hw->dev_addr_ctrl = 0;
    }

    // Setup packet received
    if (status & USB_INTS_SETUP_REQ_BITS)
    {
        usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
        USBSetup setup;
        memcpy(&setup, (void *)usb_dpram->setup_packet, sizeof(setup));
        if (cusb && cusb->ctrlIn)
        {
            cusb->ctrlIn->userdata |= 1; // next_pid==1
            cusb->setupRequest(setup);
        }
    }

    if (cusb)
        cusb->interruptHandler();

    // Buffer status, one or more buffers have completed
    if (status & USB_INTS_BUFF_STATUS_BITS)
    {
        if (cusb && cusb->ctrlIn)
            cusb->ctrlOut->read(NULL, 0); // flush any ZLP on EP0
        usb_hw->buf_status = usb_hw->buf_status;
        usb_hw_clear->sie_status = USB_INTS_BUFF_STATUS_BITS;
    }
}

void usb_set_address(uint16_t wValue)
{
    usb_hw->dev_addr_ctrl = wValue & 0xff;
}

void usb_set_address_pre(uint16_t wValue)
{
    // do nothing
}

int UsbEndpointIn::clearStall()
{
    DMESG("clear stall IN %d", ep);

    usb_dpram->ep_buf_ctrl[ep].in &= ~USB_BUF_CTRL_STALL;
    if (ep == 0)
        usb_hw_clear->ep_stall_arm = USB_EP_STALL_ARM_EP0_IN_BITS;

    wLength = 0;
    return DEVICE_OK;
}

int UsbEndpointIn::reset()
{
    DMESG("reset IN %d", ep);
    clearStall(); // ???
    // USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
    // USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
    wLength = 0;
    return DEVICE_OK;
}

int UsbEndpointIn::stall()
{
    DMESG("stall IN %d", ep);
    usb_dpram->ep_buf_ctrl[ep].in |= USB_BUF_CTRL_STALL;
    if (ep == 0)
        usb_hw_set->ep_stall_arm = USB_EP_STALL_ARM_EP0_IN_BITS;
    wLength = 0;
    return DEVICE_OK;
}

int UsbEndpointOut::clearStall()
{
    DMESG("clear stall OUT %d", ep);
    usb_dpram->ep_buf_ctrl[ep].out &= ~USB_BUF_CTRL_STALL;
    if (ep == 0)
        usb_hw_clear->ep_stall_arm = USB_EP_STALL_ARM_EP0_OUT_BITS;
    return DEVICE_OK;
}

int UsbEndpointOut::reset()
{
    DMESG("reset OUT %d", ep);
    clearStall(); // ???
    // USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
    // USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;
    return DEVICE_OK;
}

int UsbEndpointOut::stall()
{
    DMESG("stall OUT %d", ep);
    usb_dpram->ep_buf_ctrl[ep].out |= USB_BUF_CTRL_STALL;
    if (ep == 0)
        usb_hw_set->ep_stall_arm = USB_EP_STALL_ARM_EP0_OUT_BITS;
    return DEVICE_OK;
}

static inline uint32_t usb_buffer_offset(volatile uint8_t *buf)
{
    return (uint32_t)buf ^ (uint32_t)usb_dpram;
}

static inline volatile uint8_t *in_buffer(int ep)
{
    if (ep == 0)
        return usb_dpram->ep0_buf_a; // EP0 in&out share buffer
    return &usb_dpram->epx_data[(ep - 1) * 2 * 64];
}

static inline volatile uint8_t *out_buffer(int ep)
{
    if (ep == 0)
        return usb_dpram->ep0_buf_a; // EP0 in&out share buffer
    return &usb_dpram->epx_data[((ep - 1) * 2 + 1) * 64];
}

UsbEndpointIn::UsbEndpointIn(uint8_t idx, uint8_t type, uint8_t size)
{
    usb_assert(size == 64);
    usb_assert(type <= USB_EP_TYPE_INTERRUPT);
    ep = idx;
    flags = 0;

    if (type == USB_EP_TYPE_INTERRUPT)
        flags = USB_EP_FLAG_NO_AUTO_ZLP;

    // EP0 doesn't have config
    if (ep != 0)
    {
        // Get the data buffer as an offset of the USB controller's DPRAM
        uint32_t dpram_offset = usb_buffer_offset(in_buffer(ep));
        uint32_t reg = EP_CTRL_ENABLE_BITS | (type << EP_CTRL_BUFFER_TYPE_LSB) | dpram_offset;

        usb_dpram->ep_ctrl[ep - 1].in = reg;
    }
}

UsbEndpointOut::UsbEndpointOut(uint8_t idx, uint8_t type, uint8_t size)
{
    usb_assert(size == 64);
    usb_assert(type <= USB_EP_TYPE_INTERRUPT);
    ep = idx;

    if (ep != 0)
    {
        uint32_t dpram_offset = usb_buffer_offset(out_buffer(ep));
        uint32_t reg = EP_CTRL_ENABLE_BITS | (type << EP_CTRL_BUFFER_TYPE_LSB) | dpram_offset;
        usb_dpram->ep_ctrl[ep - 1].out = reg;
    }

    enableIRQ();
    startRead();
}

int UsbEndpointOut::disableIRQ()
{
    if (ep == 0)
        usb_hw_clear->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS;
    else
        usb_dpram->ep_ctrl[ep - 1].out &= ~EP_CTRL_INTERRUPT_PER_BUFFER;
    return DEVICE_OK;
}

int UsbEndpointOut::enableIRQ()
{
    if (ep == 0)
        usb_hw_set->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS;
    else
        usb_dpram->ep_ctrl[ep - 1].out |= EP_CTRL_INTERRUPT_PER_BUFFER;
    return DEVICE_OK;
}

void UsbEndpointOut::startRead()
{
    // Prepare buffer control register value
    uint32_t val = USB_MAX_PACKET_SIZE | USB_BUF_CTRL_AVAIL;

    // Set pid and flip for next transfer
    val |= userdata & 1 ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
    userdata ^= 1u;

    usb_dpram->ep_buf_ctrl[ep].out = val;
}

int UsbEndpointOut::read(void *dst, int maxlen)
{
    ASSERT_THIS_NOT_NULL();

    uint32_t mask = 1 << (ep * 2 + 1);

    if (usb_hw->buf_status & mask)
    {
        usb_hw->buf_status = mask;
        uint16_t len = usb_dpram->ep_buf_ctrl[ep].out & USB_BUF_CTRL_LEN_MASK;
        if (len > maxlen)
            len = maxlen; // too bad - we drop excessive data
        memcpy(dst, (void *)out_buffer(ep), len);
        startRead();
        return len;
    }

    return 0;
}

static int writeEP(UsbEndpointIn *t, const void *buf, int len)
{
    if (usb_dpram->ep_buf_ctrl[t->ep].in & USB_BUF_CTRL_FULL)
        return DEVICE_BUSY;

    // DMESG("wr ep=%d l=%d", t->ep, len);
    //  Prepare buffer control register value
    uint32_t val = len | USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_FULL;

    // Need to copy the data from the user buffer to the usb memory
    memcpy((void *)in_buffer(t->ep), (void *)buf, len);

    // Set pid and flip for next transfer
    val |= t->userdata & 1 ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
    t->userdata ^= 1u;

    usb_dpram->ep_buf_ctrl[t->ep].in = val;

    if (t->flags & USB_EP_FLAG_ASYNC)
        return 0;

    // Wait for transfer to complete
    while (usb_dpram->ep_buf_ctrl[t->ep].in & USB_BUF_CTRL_FULL)
    {
    }
    // DMESG("wr done");

    return 0;
}

int UsbEndpointIn::write(const void *src, int len)
{
    // this happens when someone tries to write before USB is initialized
    ASSERT_THIS_NOT_NULL();

    int epSize = USB_MAX_PACKET_SIZE;
    int zlp = !(flags & USB_EP_FLAG_NO_AUTO_ZLP);

    if (flags & USB_EP_FLAG_ASYNC)
    {
        usb_dpram->ep_ctrl[ep - 1].in |= EP_CTRL_INTERRUPT_PER_BUFFER;
        zlp = 0;
    }

    if (wLength)
    {
        if (len >= wLength)
        {
            len = wLength;
            // see
            // https://stackoverflow.com/questions/3739901/when-do-usb-hosts-require-a-zero-length-in-packet-at-the-end-of-a-control-read-t
            zlp = 0;
        }
        wLength = 0;
    }

    if (len == 0)
        zlp = 1;

    for (int p = 0; p < len; p += epSize)
    {
        int chunk = len - p;
        if (chunk > epSize)
            chunk = epSize;
        int r = writeEP(this, (const uint8_t *)src + p, chunk);
        if (r)
            return r;
    }

    // We just send ZLP manually if needed.
    if (zlp && (len & (epSize - 1)) == 0)
        return writeEP(this, "", 0);

    return DEVICE_OK;
}

bool UsbEndpointIn::canWrite()
{
    return (usb_dpram->ep_buf_ctrl[ep].in & USB_BUF_CTRL_FULL) == 0;
}

#endif
