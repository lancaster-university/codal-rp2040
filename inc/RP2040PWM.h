#ifndef CODAL_RP2040_PWM_H
#define CODAL_RP2040_PWM_H

#include "CodalConfig.h"
#include "codal-core/inc/types/Event.h"
#include "Timer.h"
#include "DataStream.h"
#include "RP2040Pin.h"
#include "Mixer.h"

#ifndef RP2040PWM_DEFAULT_FREQUENCY
#define RP2040PWM_DEFAULT_FREQUENCY 16000
#endif

namespace codal
{
class RP2040PWM : public CodalComponent, public DataSink
{
private:
  uint8_t slice;
  int sampleRate;
  RP2040Pin *pin;

public:
  // The stream component that is serving our data
  DataSource &upstream;
  ManagedBuffer output;

  /**
   * Constructor for an instance of a DAC.
   *
   * @param source The DataSource that will provide data.
   * @param sampleRate The frequency (in Hz) that data will be presented.
   * @param id The id to use for the message bus when transmitting events.
   */
  RP2040PWM(Pin &pin, DataSource &source, int sampleRate = RP2040PWM_DEFAULT_FREQUENCY,
        uint16_t id = DEVICE_ID_SYSTEM_DAC);

  /**
   * Callback provided when data is ready.
   */
  virtual int pullRequest();

  /**
   * Determine the DAC playback sample rate to the given frequency.
   * @return the current sample rate.
   */
  int getSampleRate();

  /**
   * Change the DAC playback sample rate to the given frequency.
   * @param frequency The new sample playback frequency.
   */
  int setSampleRate(int frequency);

  /**
   * Enable this component
   */
  void enable();

  /**
   * Disable this component
   */
  void disable();

};

}

#endif