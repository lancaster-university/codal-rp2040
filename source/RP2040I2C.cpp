
#include "codal_target_hal.h"
#include "CodalDmesg.h"

#include "RP2040I2C.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"


namespace codal
{

static i2c_inst_t *i2c_inst;

RP2040I2C::RP2040I2C(Pin &sda, Pin &scl) : codal::I2C(sda, scl){
  this->sda = (RP2040Pin *)&sda;
  this->scl = (RP2040Pin *)&scl;

  i2c_inst = i2c0;

  gpio_set_function(scl.name, GPIO_FUNC_I2C);
  gpio_set_function(sda.name, GPIO_FUNC_I2C);
  gpio_pull_up(scl.name);
  gpio_pull_up(sda.name);

  i2c_init(i2c_inst, 400000);

}

int RP2040I2C::setFrequency(uint32_t frequency)
{
  i2c_init(i2c_inst, frequency);
  return DEVICE_OK;
}

int RP2040I2C::write(uint16_t address, uint8_t *data, int len, bool repeated)
{
  int ret = i2c_write_blocking(i2c_inst, address>>1, data, len, repeated);
  return ret == len ? DEVICE_OK : ret;
}

int RP2040I2C::read(uint16_t address, uint8_t *data, int len, bool repeated)
{
  int ret = i2c_read_blocking(i2c_inst, address>>1, data, len, repeated);
  return ret == len ? DEVICE_OK : ret;
}

int RP2040I2C::readRegister(uint16_t address, uint8_t reg, uint8_t *data, int length, bool repeated)
{
  int result;
  result = write(address, &reg, 1, repeated);

  if (result != DEVICE_OK)
      return result;

  result = read(address, data, length);
  if (result != DEVICE_OK)
      return result;
  return DEVICE_OK;
}


}