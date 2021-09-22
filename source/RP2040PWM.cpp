#include "stdio.h"
#include "RP2040PWM.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

namespace codal
{

// 10bit and 256 byte per chunk
#define BIT_LEN (1 << 10)
#define CHUNKSZ (256)

RP2040PWM *inst;

extern "C" void isr_pwm_wrap()
{
    uint16_t n = pwm_hw->ints;
    pwm_hw->intr = n;
    inst->irq(n);
}

// TODO: add dual buffer for this
void RP2040PWM::irq(uint16_t slice_mask)
{
    if (playPos < CHUNKSZ)
    {
        pwm_set_gpio_level(pin->name, buf0[playPos++]);
        if (playPos == CHUNKSZ)
        {
            pwm_set_enabled(slice, false);
            pwm_set_irq_enabled(slice, false);
            inst->playing = false;
            inst->fillBuffer();
        }
    }
}

RP2040PWM::RP2040PWM(Pin &pin, DataSource &source, int sampleRate, uint16_t id) : upstream(source)
{
    this->pin = (RP2040Pin *)&pin;
    this->playing = false;
    buf0 = new uint16_t[CHUNKSZ];

    gpio_set_function(pin.name, GPIO_FUNC_PWM);

    slice = pwm_gpio_to_slice_num(pin.name);
    pwm_clear_irq(slice);
    irq_set_enabled(PWM_IRQ_WRAP, true);
    setSampleRate(sampleRate);
    inst = this;
    upstream.connect(*this);
}

int RP2040PWM::setSampleRate(int frequency)
{
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0 * MHZ / (frequency * BIT_LEN));
    pwm_config_set_wrap(&config, BIT_LEN);
    pwm_init(slice, &config, false);

    sampleRate = frequency;
    return DEVICE_OK;
}

void RP2040PWM::fillBuffer()
{
    playing = 1;
    output = upstream.pull();
    if (output.length())
    {
        auto dp = (uint16_t *)output.getBytes();
        bool isMute = true;
        for (int i = 0; i < 16; i++)
        {
            if (dp[i] != 512)
            {
                isMute = false;
                break;
            }
        }
        // mute the channel but still keep the irq running.
        if (!isMute)
        {
            memcpy(buf0, output.getBytes(), output.length());
        }
        else
        {
            memset(buf0, 0, output.length());
        }
        playPos = 0;
        pwm_set_enabled(slice, true);
        pwm_set_irq_enabled(slice, true);
    }
}

int RP2040PWM::pullRequest()
{
    if (!playing)
    {
        fillBuffer();
    }
    return DEVICE_OK;
}

int RP2040PWM::getSampleRate()
{
    return sampleRate;
}

void RP2040PWM::enable()
{
    pwm_set_enabled(slice, true);
}

void RP2040PWM::disable()
{
    pwm_set_enabled(slice, false);
}

} // namespace codal