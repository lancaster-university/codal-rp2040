#ifndef CODAL_RP2040_I2C_H
#define CODAL_RP2040_I2C_H

#include "CodalConfig.h"
#include "codal-core/inc/driver-models/I2C.h"
#include "RP2040Pin.h"

namespace codal
{

class RP2040I2C : public I2C, public CodalComponent
{
protected:
  RP2040Pin *sda, *scl;

public: 
  /**
   * Constructor.
   */
  RP2040I2C(Pin &sda, Pin &scl);

  /** Set the frequency of the I2C interface
   *
   * @param frequency The bus frequency in hertz
   */
  virtual int setFrequency(uint32_t frequency);

  /**
  * Issues a standard, I2C command write to the I2C bus.
  * This consists of:
  *  - Asserting a Start condition on the bus
  *  - Selecting the Slave address (as an 8 bit address)
  *  - Writing a number of raw data bytes provided
  *  - Asserting a Stop condition on the bus
  *
  * The CPU will busy wait until the transmission is complete.
  *
  * @param address The 8bit I2C address of the device to write to
  * @param data pointer to the bytes to write
  * @param len the number of bytes to write
  * @param repeated Suppresses the generation of a STOP condition if set. Default: false;
  *
  * @return DEVICE_OK on success, DEVICE_I2C_ERROR if the the write request failed.
  */
  virtual int write(uint16_t address, uint8_t *data, int len, bool repeated = false);

  /**
    * Issues a standard, 2 byte I2C command read to the I2C bus.
    * This consists of:
    *  - Asserting a Start condition on the bus
    *  - Selecting the Slave address (as an 8 bit address)
    *  - reading "len" bytes of raw 8 bit data into the buffer provided
    *  - Asserting a Stop condition on the bus
    *
    * The CPU will busy wait until the transmission is complete.
    *
    * @param address The 8bit I2C address of the device to read from
    * @param data pointer to store the the bytes read
    * @param len the number of bytes to read into the buffer
    * @param repeated Suppresses the generation of a STOP condition if set. Default: false;
    *
    * @return DEVICE_OK on success, DEVICE_I2C_ERROR if the the read request failed.
    */
  virtual int read(uint16_t address, uint8_t *data, int len, bool repeated = false);

  /**
    * Performs a typical register read operation to the I2C slave device provided.
    * This consists of:
    *  - Asserting a Start condition on the bus
    *  - Selecting the Slave address (as an 8 bit address, I2C WRITE)
    *  - Selecting a RAM register address in the slave
    *  - Asserting a Stop condition on the bus
    *  - Asserting a Start condition on the bus
    *  - Selecting the Slave address (as an 8 bit address, I2C READ)
    *  - Performing an 8 bit read operation (of the requested register)
    *  - Asserting a Stop condition on the bus
    *
    * The CPU will busy wait until the transmission is complete..
    *
    * @param address 8bit I2C address of the device to read from
    * @param reg The 8bit register address of the to read.
    * @param data A pointer to a memory location to store the result of the read operation
    * @param length The number of mytes to read
    * @param repeated Use a repeated START/START/STOP transaction if true, or independent START/STOP/START/STOP transactions if fasle. Default: true
    *
    * @return DEVICE_OK or DEVICE_I2C_ERROR if the the read request failed.
    */
  virtual int readRegister(uint16_t address, uint8_t reg, uint8_t *data, int length, bool repeated = true);

};

}

#endif
