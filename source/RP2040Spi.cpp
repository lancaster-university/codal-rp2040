#include "stdio.h"

#include "CodalConfig.h"
#include "CodalDmesg.h"

#include "MessageBus.h"
#include "Event.h"
#include "CodalFiber.h"

#include "RP2040Spi.h"
#include "dma.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

namespace codal
{

static spi_inst_t *spi_inst;
static int rxCh, txCh, channel_irq;

RP2040SPI::RP2040SPI(Pin &mosi, Pin &miso, Pin &sclk) : codal::SPI()
{
    this->mosi = (RP2040Pin *)&mosi;
    this->miso = (RP2040Pin *)&miso;
    this->sclk = (RP2040Pin *)&sclk;
    this->baudrate = 1000000;
    rxCh = txCh = -1;

    this->transferCompleteEventCode = codal::allocateNotifyEvent();

    // TODO: set instance by check pin config
    spi_inst = spi0;

    gpio_set_function(this->sclk->name, GPIO_FUNC_SPI);
    gpio_set_function(this->miso->name, GPIO_FUNC_SPI);
    gpio_set_function(this->mosi->name, GPIO_FUNC_SPI);

    spi_init(spi_inst, this->baudrate);
}

int RP2040SPI::setFrequency(uint32_t frequency)
{
    spi_set_baudrate(spi_inst, frequency);
    return DEVICE_OK;
}

int RP2040SPI::setMode(int mode, int bits)
{
    return DEVICE_OK;
}

int RP2040SPI::write(int data)
{
    return DEVICE_OK;
}

int RP2040SPI::transfer(const uint8_t *txBuffer, uint32_t txSize, uint8_t *rxBuffer,
                        uint32_t rxSize)
{
    fiber_wake_on_event(DEVICE_ID_NOTIFY, transferCompleteEventCode);
    auto res = startTransfer(txBuffer, txSize, rxBuffer, rxSize, NULL, NULL);
    schedule();
    return res;
}

int RP2040SPI::startTransfer(const uint8_t *txBuffer, uint32_t txSize, uint8_t *rxBuffer,
                             uint32_t rxSize, PVoidCallback doneHandler, void *arg)
{
    int res;
    // make sure buffers are not on the stack
    uint8_t getSP = 0;
    CODAL_ASSERT(txBuffer < &getSP && rxBuffer < &getSP, DEVICE_HARDWARE_CONFIGURATION_ERROR);

    int len = max(txSize, rxSize);
    if (len >= 32)
    {
        return startTransferDma(txBuffer, txSize, rxBuffer, rxSize, doneHandler, arg);
    }

    if (rxBuffer && txBuffer)
    {
        res = spi_write_read_blocking(spi_inst, txBuffer, rxBuffer, txSize);
    }
    else if (txBuffer)
    {
        res = spi_write_blocking(spi_inst, txBuffer, txSize);
    }
    else
    {
        res = spi_read_blocking(spi_inst, 0, rxBuffer, rxSize);
    }
    Event(DEVICE_ID_NOTIFY_ONE, transferCompleteEventCode);
    if (doneHandler)
    {
        doneHandler(arg);
    }
    return DEVICE_OK;
}

void RP2040SPI::_complete(void)
{
    if (txCh >= 0)
    {
        dma_channel_unclaim(txCh);
    }
    if (rxCh >= 0)
    {
        dma_channel_unclaim(rxCh);
    }
    if (doneHandler)
    {
        PVoidCallback done = doneHandler;
        doneHandler = NULL;
        done(doneHandlerArg);
    }
    Event(DEVICE_ID_NOTIFY_ONE, transferCompleteEventCode);
}

extern "C" void _irqDone(void *p)
{
    RP2040SPI *_this = (RP2040SPI *)p;
    _this->_complete();
}

int RP2040SPI::startTransferDma(const uint8_t *txBuffer, uint32_t txSize, uint8_t *rxBuffer,
                                uint32_t rxSize, PVoidCallback doneHandler, void *arg)
{
    if (txCh >= 0)
        dma_channel_unclaim(txCh);
    if (rxCh >= 0)
        dma_channel_unclaim(rxCh);
    txCh = rxCh = -1;
    uint32_t channel_mask = 0;

    if (txBuffer)
    {
        txCh = dma_claim_unused_channel(false);
        if (txCh >= 0)
        {
            dma_channel_config c = dma_channel_get_default_config(txCh);
            channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
            channel_config_set_dreq(&c, spi_get_index(spi_inst) ? DREQ_SPI1_TX : DREQ_SPI0_TX);
            dma_channel_configure(
                txCh, &c,
                &spi_get_hw(spi_inst)->dr, // txbuff > spi
                txBuffer,
                txSize, // element count (each element is of size transfer_data_size)
                false); // don't start yet
            channel_mask += (1u << txCh);
            channel_irq = txCh;
        }
    }
    if (rxBuffer)
    {
        rxCh = dma_claim_unused_channel(false);
        if (txCh >= 0)
        {
            dma_channel_config c = dma_channel_get_default_config(rxCh);
            channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
            channel_config_set_dreq(&c, spi_get_index(spi_inst) ? DREQ_SPI1_RX : DREQ_SPI0_RX);
            dma_channel_configure(
                rxCh, &c,
                rxBuffer, // spi > rx buff
                &spi_get_hw(spi_inst)->dr,
                rxSize, // element count (each element is of size transfer_data_size)
                false); // don't start yet
            channel_mask += (1u << rxCh);
            channel_irq = rxCh;
        }
    }
    this->doneHandler = doneHandler;
    this->doneHandlerArg = arg;

    DMA_SetChannelCallback(channel_irq, _irqDone, this);
    dma_start_channel_mask(channel_mask);

    return DEVICE_OK;
}

} // namespace codal
