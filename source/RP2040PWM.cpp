#include "stdio.h"
#include "RP2040PWM.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

namespace codal
{

RP2040PWM::RP2040PWM(Pin &pin, DataSource &source, int sampleRate, uint16_t id) : upstream(source)
{
  this->pin = (RP2040Pin *)&pin;

  // TODO: share pwm config with pin analog write
  gpio_set_function(pin.name, GPIO_FUNC_PWM);
  
  slice = pwm_gpio_to_slice_num(pin.name);
  setSampleRate(sampleRate);
}


int RP2040PWM::setSampleRate(int frequency)
{
  pwm_set_clkdiv(slice, 125*MHZ/(frequency*256));
  pwm_set_wrap(slice, 254); // N+1: cc value from 0~255
  pwm_set_chan_level(slice, pin->name & 0x1 ? PWM_CHAN_B : PWM_CHAN_A, 128);
  pwm_set_enabled(slice, true);
  sampleRate = frequency;
  return DEVICE_OK;
}

int RP2040PWM::pullRequest()
{
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



}