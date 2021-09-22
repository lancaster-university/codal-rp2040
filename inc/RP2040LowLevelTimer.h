#ifndef RP2040_LOW_LEVEL_TIMER_H
#define RP2040_LOW_LEVEL_TIMER_H

#include "LowLevelTimer.h"
#include "RP2040.h"
#include "hardware/structs/timer.h"

#define TIMER_CHANNEL_COUNT 4
namespace codal
{
class RP2040LowLevelTimer : public LowLevelTimer
{
    uint32_t irq_msk;

public:
    timer_hw_t *timer;

    RP2040LowLevelTimer();

    virtual int setIRQPriority(int priority) override;

    virtual int enable();

    virtual int enableIRQ();

    virtual int disable();

    virtual int disableIRQ();

    virtual int reset();

    virtual int setMode(TimerMode t);

    virtual int setCompare(uint8_t channel, uint32_t value);

    virtual int offsetCompare(uint8_t channel, uint32_t value);

    virtual int clearCompare(uint8_t channel);

    virtual uint32_t captureCounter();

    virtual int setClockSpeed(uint32_t speedKHz);

    virtual int setBitMode(TimerBitMode t);

    virtual int setSleep(bool doSleep) override;
};
} // namespace codal

#endif