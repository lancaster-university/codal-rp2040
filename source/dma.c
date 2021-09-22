#include "dma.h"
#include "CodalDmesg.h"
#include "ErrorNo.h"

#include "hardware/dma.h"
#include "hardware/irq.h"

static DMAHandler dmaHandler[12]; // max channel 12

REAL_TIME_FUNC
void isr_dma_0 (){
  uint mask = dma_hw->ints0;
  // write to clear
  dma_hw->ints0 = mask;
  for (uint8_t ch=0;ch<12;ch++){
    if ((1<<ch) & mask){
      if (dmaHandler[ch].handler){
        dma_channel_set_irq0_enabled(ch, false);
        dmaHandler[ch].handler(dmaHandler[ch].context);
      }
    }
  }
}

REAL_TIME_FUNC
void DMA_SetChannelCallback(uint8_t channel, DMAChannelCallback handler, void * context){
  if (channel >= 12) return;
  dmaHandler[channel].handler = handler;
  dmaHandler[channel].context = context;
  dma_channel_set_irq0_enabled(channel, true);
  irq_set_enabled(DMA_IRQ_0, true);
}

