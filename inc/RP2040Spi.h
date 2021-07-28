#ifndef CODAL_RP2040_SPI_H
#define CODAL_RP2040_SPI_H

#include "CodalConfig.h"
#include "codal-core/inc/driver-models/SPI.h"
#include "RP2040Pin.h"

// from micropython pico
#define IS_VALID_PERIPH(spi, pin)   ((((pin) & 8) >> 3) == (spi))
#define IS_VALID_SCK(spi, pin)      (((pin) & 3) == 2 && IS_VALID_PERIPH(spi, pin))
#define IS_VALID_MOSI(spi, pin)     (((pin) & 3) == 3 && IS_VALID_PERIPH(spi, pin))
#define IS_VALID_MISO(spi, pin)     (((pin) & 3) == 0 && IS_VALID_PERIPH(spi, pin))

namespace codal
{

class RP2040SPI : public SPI, public CodalComponent
{
protected:
  RP2040Pin *mosi, *miso, *sclk;
  uint8_t spi_id;
  uint32_t baudrate;

  // for dma
  int rxCh, txCh;
  uint16_t transferCompleteEventCode;


public:

  /**
   * Initialize SPI instance with given pins.
   *
   * Default setup is 1 MHz, 8 bit, mode 0.
   */
  RP2040SPI(Pin &mosi, Pin &miso, Pin &sclk);

  
  /** Set the frequency of the SPI interface
   *
   * @param frequency The bus frequency in hertz
   */
  virtual int setFrequency(uint32_t frequency);

  /** Set the mode of the SPI interface
   *
   * @param mode Clock polarity and phase mode (0 - 3)
   * @param bits Number of bits per SPI frame (4 - 16)
   *
   * @code
   * mode | POL PHA
   * -----+--------
   *   0  |  0   0
   *   1  |  0   1
   *   2  |  1   0
   *   3  |  1   1
   * @endcode
   */
  virtual int setMode(int mode, int bits = 8);

  /**
   * Writes the given byte to the SPI bus.
   *
   * The CPU will wait until the transmission is complete.
   *
   * @param data The data to write.
   * @return Response from the SPI slave or DEVICE_SPI_ERROR if the the write request failed.
   */
  virtual int write(int data);

  /**
   * Writes and reads from the SPI bus concurrently. Waits un-scheduled for transfer to finish.
   *
   * Either buffer can be NULL.
   */
  virtual int transfer(const uint8_t *txBuffer, uint32_t txSize, uint8_t *rxBuffer,
                        uint32_t rxSize);

  virtual int startTransfer(const uint8_t *txBuffer, uint32_t txSize, uint8_t *rxBuffer,
                            uint32_t rxSize, PVoidCallback doneHandler, void *arg);


};

}

#endif 
