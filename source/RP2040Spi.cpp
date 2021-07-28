#include "stdio.h"

#include "CodalConfig.h"
#include "CodalDmesg.h"

#include "MessageBus.h"
#include "Event.h"
#include "CodalFiber.h"

#include "RP2040Spi.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/dma.h"

namespace codal
{

static spi_inst_t *spi_inst;

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

int RP2040SPI::transfer(const uint8_t *txBuffer, uint32_t txSize, uint8_t *rxBuffer, uint32_t rxSize)
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
  // TODO: add dma transceiver
  // txCh = dma_claim_unused_channel(false);
  // rxCh = dma_claim_unused_channel(false);

  if (rxBuffer && txBuffer){
    res = spi_write_read_blocking(spi_inst, txBuffer, rxBuffer, txSize);
  } else if (txBuffer){
    res = spi_write_blocking(spi_inst, txBuffer, txSize);
  } else {
    res = spi_read_blocking(spi_inst, 0, rxBuffer, rxSize);
  }
  Event(DEVICE_ID_NOTIFY_ONE, transferCompleteEventCode);
  return DEVICE_OK;
}

}
