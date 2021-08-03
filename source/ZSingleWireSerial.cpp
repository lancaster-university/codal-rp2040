#include "CodalConfig.h"
#include "CodalDmesg.h"
#include "ZSingleWireSerial.h"
#include "Event.h"

#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "jacdac.pio.h" 

using namespace codal;

#define STATUS_IDLE 0
#define STATUS_TX   0x10
#define STATUS_RX   0x20

static void jd_tx_program_init(PIO pio, uint sm, uint offset, uint pin, uint baud) {
  pio_sm_set_pins_with_mask(pio, sm, 1u << pin, 1u << pin);
  pio_sm_set_pindirs_with_mask(pio, sm, 1u << pin, 1u << pin);
  pio_gpio_init(pio, pin);
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

static void jd_rx_program_init(PIO pio, uint sm, uint offset, uint pin, uint baud) {
  pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);
  pio_gpio_init(pio, pin);
  gpio_pull_up(pin);
  pio_sm_config c = jd_rx_program_get_default_config(offset);
  sm_config_set_in_pins(&c, pin);
  sm_config_set_in_shift(&c, true, true, 8);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
  float div = (float)125000000 / (8 * baud);
  sm_config_set_clkdiv(&c, div);
  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_enabled(pio, sm, false); // enable when need
}

ZSingleWireSerial::ZSingleWireSerial(Pin& p) : DMASingleWireSerial(p)
{
  txprog = pio_add_program(pio0, &jd_tx_program);
  rxprog = pio_add_program(pio0, &jd_rx_program);

  jd_rx_program_init(pio0, smrx, rxprog, p.name, 1000000);
  jd_tx_program_init(pio0, smtx, txprog, p.name, 1000000);

}

int ZSingleWireSerial::setMode(SingleWireMode sw)
{
  // either enable rx or tx program
  if (sw == SingleWireRx){
    pio_sm_set_enabled(pio0, smtx, false);
    pio_sm_set_enabled(pio0, smrx, true);
    // jd_tx_program_init(pio0, smrx, txprog, p.name, baudrate);
    status = STATUS_RX;
  } else if (sw == SingleWireTx){
    pio_sm_set_enabled(pio0, smrx, false);
    pio_sm_set_enabled(pio0, smtx, true);
    // jd_tx_program_init(pio0, smtx, txprog, p.name, baudrate);
    status = STATUS_TX;
  } else {
    pio_sm_set_enabled(pio0, smtx, false);
    pio_sm_set_enabled(pio0, smrx, false);
    status = STATUS_IDLE;
  }
  return DEVICE_OK;
}

void ZSingleWireSerial::configureRxInterrupt(int enable)
{
}

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

int ZSingleWireSerial::send(uint8_t* data, int len)
{
  return DEVICE_NOT_IMPLEMENTED;
}

int ZSingleWireSerial::receive(uint8_t* data, int len)
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


int ZSingleWireSerial::sendDMA(uint8_t* data, int len)
{
  if (status != STATUS_TX)
    setMode(SingleWireTx);
  if (dmachTx != -1){
    dma_channel_unclaim(dmachTx);
  }
  
  gpio_set_function(p.name, GPIO_FUNC_PIO0);
  
  dmachTx = dma_claim_unused_channel(false);
  dma_channel_config c = dma_channel_get_default_config(dmachTx);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_dreq(&c, pio_get_dreq(pio0, smtx, true));
  dma_channel_configure(dmachTx,
                        &c,
                        &pio0->txf[smtx],
                        data,
                        len,
                        false);

  dma_channel_start(dmachTx);
  return DEVICE_OK;
}

int ZSingleWireSerial::receiveDMA(uint8_t* data, int len)
{
  if (status != STATUS_RX)
    setMode(SingleWireRx);
  if (dmachRx != -1){
    dma_channel_unclaim(dmachRx);
  }

  gpio_set_function(p.name, GPIO_FUNC_PIO0);

  dmachRx = dma_claim_unused_channel(false);
  dma_channel_config c = dma_channel_get_default_config(dmachRx);
  channel_config_set_bswap(&c, 1);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_dreq(&c, DREQ_PIO0_RX0 + smrx);
  uint8_t * rxPtr = (uint8_t*)&pio0->rxf[smrx] + 3;
  dma_channel_configure(
          dmachRx,
          &c,
          data,         // dest
          rxPtr,        // src
          len,
          false
  );
  dma_channel_start(dmachRx);

  return DEVICE_OK;
}


int ZSingleWireSerial::abortDMA()
{
  if (!(status & (STATUS_RX | STATUS_TX)))
    return DEVICE_INVALID_PARAMETER;
  
  setMode(SingleWireDisconnected);
  gpio_set_function(p.name, GPIO_FUNC_SIO); // release gpio

  if (dmachRx != -1){
    dma_channel_unclaim(dmachRx);
    dmachRx = -1;
  }
  if (dmachTx != -1){
    dma_channel_unclaim(dmachTx);
    dmachTx = -1;
  }
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
 

