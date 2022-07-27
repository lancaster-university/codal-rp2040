#include "stdio.h"
#include "RP2040PWM.h"
#include "CodalDmesg.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "dma.h"
#include "hardware/dma.h"

namespace codal
{

// 10bit and 256 byte per chunk
#define BIT_LEN (1 << 10)
#define CHUNKSZ (256 * 2)

static void dma0_cb(void *inst)
{
    ((RP2040PWM *)inst)->dmaCB(0);
}
static void dma1_cb(void *inst)
{
    ((RP2040PWM *)inst)->dmaCB(1);
}

RP2040PWM::RP2040PWM(Pin &pin, DataSource &source, int sampleRate, uint16_t id) : upstream(source)
{
    this->pin = (RP2040Pin *)&pin;
    this->en = false;

    // gpio_set_drive_strength(pin.name, GPIO_DRIVE_STRENGTH_12MA);
    pin.setDigitalValue(0);

    slice = pwm_gpio_to_slice_num(pin.name);
    gpio_set_function(pin.name, GPIO_FUNC_PWM);

    for (int i = 0; i < 2; ++i)
    {
        dmaCh[i] = dma_claim_unused_channel(true);
        buf[i] = new uint16_t[CHUNKSZ / 2];
    }

    setSampleRate(sampleRate);
    upstream.connect(*this);
}

void RP2040PWM::dump()
{
#if 0
    uint32_t c0 = dma_get_channel_config(dmaCh[0]).ctrl;
    uint32_t c1 = dma_get_channel_config(dmaCh[1]).ctrl;

    DMESG("DMA ST %x/%d %x/%d", c0, (c0 >> 11) & 0xf, c1, (c1 >> 11) & 0xf);
#endif
}

void RP2040PWM::dmaCB(int ch)
{
    dump();

    // dmaCh[ch] finished - refill it's buffer
    ManagedBuffer output = upstream.pull();
    bool isMute = true;
    if (output.length())
    {
        if (output.length() != CHUNKSZ)
            target_panic(DEVICE_HARDWARE_CONFIGURATION_ERROR);

        auto dp = (uint16_t *)output.getBytes();
        for (int i = 0; i < 16; i++)
        {
            if (dp[i] != BIT_LEN / 2)
            {
                isMute = false;
                break;
            }
        }
    }

    if (!isMute)
    {
        memcpy(buf[ch], output.getBytes(), CHUNKSZ);
    }
    else
    {
        memset(buf[ch], 0, CHUNKSZ);
    }

    volatile uint16_t *dst = (volatile uint16_t *)&pwm_hw->slice[slice].cc;
    if (pin->name & 1)
        dst++;

    bool start = false;
    dma_channel_config c = dma_channel_get_default_config(dmaCh[ch]);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_dreq(&c, DREQ_PWM_WRAP0 + slice);
    channel_config_set_chain_to(&c, dmaCh[1 - ch]);
    dma_channel_configure(dmaCh[ch], &c, dst, buf[ch], CHUNKSZ / 2, start);

    dump();

    // at 44kHz the 256 samples is around 5ms - no need for any high priority here
    DMA_SetChannelCallback_lowpri(dmaCh[ch], ch == 0 ? dma0_cb : dma1_cb, this);
}

int RP2040PWM::setSampleRate(int frequency)
{
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, (float)clock_get_hz(clk_sys) / (frequency * BIT_LEN));
    pwm_config_set_wrap(&config, BIT_LEN);
    pwm_init(slice, &config, false);

    sampleRate = frequency;
    return DEVICE_OK;
}

int RP2040PWM::pullRequest()
{
    // just enable; we keep pulling at our own pace anyways
    enable();
    return DEVICE_OK;
}

int RP2040PWM::getSampleRate()
{
    return sampleRate;
}

void RP2040PWM::enable()
{
    if (en)
        return;
    en = true;

    // refill both buffers
    dmaCB(0);
    dmaCB(1);

    pwm_set_enabled(slice, true);
    dma_channel_start(dmaCh[0]);
}

void RP2040PWM::disable()
{
    if (!en)
        return;
    en = false;

    pwm_set_enabled(slice, false);
    dma_channel_abort(dmaCh[0]);
    dma_channel_abort(dmaCh[1]);
    pin->disconnect();
    pin->setDigitalValue(0);
}

} // namespace codal