#include "CodalConfig.h"
#include "CodalDmesg.h"
#include "ZSingleWireSerial.h"
#include "Event.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "jacdac.pio.h" 

using namespace codal;

#define STATUS_IDLE 0
#define STATUS_TX   0x10
#define STATUS_RX   0x20

ZSingleWireSerial::ZSingleWireSerial(Pin& p) : DMASingleWireSerial(p)
{
  txprog = pio_add_program(pio0, &jd_tx_program);
  rxprog = pio_add_program(pio0, &jd_rx_program);
  jd_tx_program_init(pio0, smtx, txprog, p.name, 1000000);
  jd_rx_program_init(pio0, smrx, rxprog, p.name, 1000000);
}

int ZSingleWireSerial::setMode(SingleWireMode sw)
{
  // either enable rx or tx program
  if (sw == SingleWireRx){
    pio_sm_set_enabled(pio0, smtx, false);
    pio_sm_set_enabled(pio0, smrx, true);
    status = STATUS_RX;
  } else if (sw == SingleWireTx){
    pio_sm_set_enabled(pio0, smtx, true);
    pio_sm_set_enabled(pio0, smrx, false);
    status = STATUS_TX;
  } else {
    pio_sm_set_enabled(pio0, smtx, false);
    pio_sm_set_enabled(pio0, smrx, false);
    status = STATUS_IDLE;
  }
  return DEVICE_OK;
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

  return DEVICE_OK;
}

