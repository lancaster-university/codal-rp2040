#include "CodalConfig.h"
#include "CodalDmesg.h"
#include "ZSingleWireSerial.h"
#include "Event.h"

#include "stdio.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "jacdac.pio.h"
#include "dma.h"

using namespace codal;

#define STATUS_IDLE 0
#define STATUS_TX 0x10
#define STATUS_RX 0x20

#define PIO_BREAK_IRQ 0x2
// #define DEBUG_PIN

int dmachTx = -1;
int dmachRx = -1;
extern "C"
{
    void rx_handler(void *p)
    {
        ZSingleWireSerial* inst = (ZSingleWireSerial *)p;
        if (inst && inst->cb)
            inst->cb(SWS_EVT_DATA_RECEIVED);
    }

    void tx_handler(void *p)
    {
        ZSingleWireSerial* inst = (ZSingleWireSerial *)p;
        if (inst && inst->cb)
            inst->cb(SWS_EVT_DATA_SENT);
    }

    static ZSingleWireSerial *inst;
    void isr_pio0_0()
    {
        uint32_t n = pio0->irq;
        pio0->irq = n;
        if (n & PIO_BREAK_IRQ)
        {
            inst->cb(SWS_EVT_DATA_RECEIVED);
        }
    }
}

static void jd_tx_arm_pin(PIO pio, uint sm, uint pin)
{
    pio_sm_set_pins_with_mask(pio, sm, 1u << pin, 1u << pin);
    pio_sm_set_pindirs_with_mask(pio, sm, 1u << pin, 1u << pin);
    pio_gpio_init(pio, pin);
}

static void jd_tx_program_init(PIO pio, uint sm, uint offset, uint pin, uint baud)
{
    jd_tx_arm_pin(pio, sm, pin);
    pio_sm_config c = jd_tx_program_get_default_config(offset);
    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_out_pins(&c, pin, 1);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    float div = (float)125000000 / (8 * baud);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, false); // enable when need
}

static void jd_rx_arm_pin(PIO pio, uint sm, uint pin)
{
#ifdef DEBUG_PIN
    // for debug pin 29
    pio_sm_set_pins_with_mask(pio, sm, 1u << 29, 1u << 29); // init high
    pio_sm_set_pindirs_with_mask(pio, sm, (0u << pin) | (1u << 29), (1u << pin) | (1u << 29));
    pio_gpio_init(pio, 29);
    gpio_set_outover(29, GPIO_OVERRIDE_NORMAL);
#else
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);
#endif

    pio_gpio_init(pio, pin);
    gpio_pull_up(pin);
}

static void jd_rx_program_init(PIO pio, uint sm, uint offset, uint pin, uint baud)
{
    jd_rx_arm_pin(pio, sm, pin);
    pio_sm_config c = jd_rx_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_jmp_pin(&c, pin);
    sm_config_set_in_shift(&c, true, true, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
#ifdef DEBUG_PIN
    sm_config_set_sideset_pins(&c, 29);
#endif
    float div = (float)125000000 / (8 * baud);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, false); // enable when need
}

ZSingleWireSerial::ZSingleWireSerial(Pin &p) : DMASingleWireSerial(p)
{
    inst = this;
    txprog = pio_add_program(pio0, &jd_tx_program);
    rxprog = pio_add_program(pio0, &jd_rx_program);

    jd_tx_program_init(pio0, smtx, txprog, p.name, baudrate);
    jd_rx_program_init(pio0, smrx, rxprog, p.name, baudrate);

    // fixed dma channels
    dmachRx = dma_claim_unused_channel(true);
    dmachTx = dma_claim_unused_channel(true);

    // init dma

    dma_channel_config c = dma_channel_get_default_config(dmachRx);
    channel_config_set_bswap(&c, 1);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, DREQ_PIO0_RX0 + smrx);
    uint8_t *rxPtr = (uint8_t *)&pio0->rxf[smrx] + 3;
    dma_channel_configure(dmachRx, &c,
                          NULL,  // dest
                          rxPtr, // src
                          0, false);

    c = dma_channel_get_default_config(dmachTx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, pio_get_dreq(pio0, smtx, true));
    dma_channel_configure(dmachTx, &c, &pio0->txf[smtx], NULL, 0, false);
    // pio irq for dma rx break handling
    irq_set_enabled(PIO0_IRQ_0, true);
}

int ZSingleWireSerial::setMode(SingleWireMode sw)
{
    // either enable rx or tx program
    if (sw == SingleWireRx)
    {
        status = STATUS_RX;
        jd_rx_arm_pin(pio0, smrx, p.name);
        jd_rx_program_init(pio0, smrx, rxprog, p.name, baudrate);
        pio_sm_set_enabled(pio0, smrx, true);
    }
    else if (sw == SingleWireTx)
    {
        status = STATUS_TX;
        jd_tx_arm_pin(pio0, smtx, p.name);
        jd_tx_program_init(pio0, smtx, txprog, p.name, baudrate);
        pio_sm_set_enabled(pio0, smtx, true);
    }
    else
    {
        status = STATUS_IDLE;
        gpio_set_function(p.name, GPIO_FUNC_SIO); // release gpio
        pio_sm_set_enabled(pio0, smtx, false);
        pio_sm_set_enabled(pio0, smrx, false);
    }

    return DEVICE_OK;
}

void ZSingleWireSerial::configureRxInterrupt(int enable) {}

int ZSingleWireSerial::configureTx(int enable)
{
    return setMode(enable ? SingleWireTx : SingleWireDisconnected);
}

int ZSingleWireSerial::configureRx(int enable)
{
    return setMode(enable ? SingleWireRx : SingleWireDisconnected);
}

int ZSingleWireSerial::putc(char c)
{
    return DEVICE_NOT_IMPLEMENTED;
}

int ZSingleWireSerial::getc()
{
    return DEVICE_NOT_IMPLEMENTED;
}

int ZSingleWireSerial::send(uint8_t *data, int len)
{
    return DEVICE_NOT_IMPLEMENTED;
}

int ZSingleWireSerial::receive(uint8_t *data, int len)
{
    return DEVICE_NOT_IMPLEMENTED;
}

int ZSingleWireSerial::setBaud(uint32_t baud)
{
    baudrate = baud;
    return DEVICE_OK;
}

uint32_t ZSingleWireSerial::getBaud()
{
    return baudrate;
}

int ZSingleWireSerial::sendDMA(uint8_t *data, int len)
{
    if (status != STATUS_TX)
        setMode(SingleWireTx);

    DMA_SetChannelCallback(dmachTx, tx_handler, this);
    dma_channel_transfer_from_buffer_now(dmachTx, data, len);

    return DEVICE_OK;
}

int ZSingleWireSerial::receiveDMA(uint8_t *data, int len)
{
    if (status != STATUS_RX)
        setMode(SingleWireRx);

    pio0->irq = PIO_BREAK_IRQ; // clear irq
    DMA_SetChannelCallback(dmachRx, rx_handler, this);
    dma_channel_transfer_to_buffer_now(dmachRx, data, len);
    pio_set_irq0_source_enabled(pio0, pis_interrupt1, true);

    return DEVICE_OK;
}

int ZSingleWireSerial::abortDMA()
{
    if (!(status & (STATUS_RX | STATUS_TX)))
        return DEVICE_INVALID_PARAMETER;

    setMode(SingleWireDisconnected);
    pio_set_irq0_source_enabled(pio0, pis_interrupt1, false);

    return DEVICE_OK;
}

int ZSingleWireSerial::sendBreak()
{
    return DEVICE_NOT_IMPLEMENTED;
}

int ZSingleWireSerial::getBytesReceived()
{
    return DEVICE_NOT_IMPLEMENTED;
}

int ZSingleWireSerial::getBytesTransmitted()
{
    return DEVICE_NOT_IMPLEMENTED;
}
