#include "dma.h"
#include "ram.h"
#include "CodalDmesg.h"
#include "ErrorNo.h"

#include "hardware/dma.h"
#include "hardware/irq.h"

static DMAHandler dmaHandler[12]; // max channel 12
static DMAHandler dmaHandler_low[12]; // max channel 12

REAL_TIME_FUNC
void isr_dma_0()
{
    uint mask = dma_hw->ints0;
    // write to clear
    dma_hw->ints0 = mask;
    for (uint8_t ch = 0; ch < 12; ch++)
    {
        if ((1 << ch) & mask)
        {
            if (dmaHandler[ch].handler)
            {
                dma_channel_set_irq0_enabled(ch, false);
                dmaHandler[ch].handler(dmaHandler[ch].context);
            }
        }
    }
}

REAL_TIME_FUNC
void isr_dma_1()
{
    uint mask = dma_hw->ints1;
    // write to clear
    dma_hw->ints1 = mask;
    for (uint8_t ch = 0; ch < 12; ch++)
    {
        if ((1 << ch) & mask)
        {
            if (dmaHandler_low[ch].handler)
            {
                dma_channel_set_irq1_enabled(ch, false);
                dmaHandler_low[ch].handler(dmaHandler_low[ch].context);
            }
        }
    }
}

REAL_TIME_FUNC
void DMA_SetChannelCallback(uint8_t channel, DMAChannelCallback handler, void *context)
{
    if (channel >= 12)
        return;
    dmaHandler[channel].handler = handler;
    dmaHandler[channel].context = context;
    dma_hw->ints0 = (1 << DMA_IRQ_0); // clear any pending irq
    dma_channel_set_irq0_enabled(channel, true);
    ram_irq_set_enabled(DMA_IRQ_0, true);
}


REAL_TIME_FUNC
void DMA_SetChannelCallback_lowpri(uint8_t channel, DMAChannelCallback handler, void *context)
{
    if (channel >= 12)
        return;
    dmaHandler_low[channel].handler = handler;
    dmaHandler_low[channel].context = context;
    dma_hw->ints1 = (1 << DMA_IRQ_1); // clear any pending irq
    dma_channel_set_irq1_enabled(channel, true);
    ram_irq_set_enabled(DMA_IRQ_1, true);
}
