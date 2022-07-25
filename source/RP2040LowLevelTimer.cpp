#include "RP2040LowLevelTimer.h"
#include "CodalDmesg.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/structs/scb.h"
#include "RP2040.h"
#include "ram.h"

#define PRESCALE_VALUE_MAX 9

using namespace codal;

// NUM_TIMERS defines in structs/timer.h

static RP2040LowLevelTimer *instance = NULL;

static inline irq_handler_t *get_vtable(void)
{
    return (irq_handler_t *)scb_hw->vtor;
}

inline uint alarm_irq_number(uint8_t num)
{
    return TIMER_IRQ_0 + num;
}

inline int alarms_enabled()
{
    int count = 0;
    for (int i = 0; i < NUM_TIMERS; i++)
        count += NVIC_GetEnableIRQ((IRQn_Type)alarm_irq_number(i));
    return count;
}

void timer_handler(uint8_t instance_number)
{
    if (instance == NULL)
        return;

    uint8_t channel_bitmsk = timer_hw->intr;
    timer_hw->intr = channel_bitmsk;

    if (instance->timer_pointer)
        instance->timer_pointer(channel_bitmsk);
}

extern "C" void isr_timer_0()
{
    timer_handler(0);
}
extern "C" void isr_timer_1()
{
    timer_handler(1);
}
extern "C" void isr_timer_2()
{
    timer_handler(2);
}
extern "C" void isr_timer_3()
{
    timer_handler(3);
}

RP2040LowLevelTimer::RP2040LowLevelTimer() : LowLevelTimer(3)
{
    if (!instance)
    {
        irq_msk = 0;
        instance = this;
        this->timer = timer_hw;
        disable();
        setIRQPriority(2);
        setBitMode(BitMode32);
        reset();
    }

    irq_msk = timer_hw->inte;
}

int RP2040LowLevelTimer::setIRQPriority(int priority)
{
    for (int i = 0; i < NUM_TIMERS; i++)
        NVIC_SetPriority((IRQn_Type)alarm_irq_number(i), priority);
    return DEVICE_OK;
}

int RP2040LowLevelTimer::enable()
{
    for (int i = 0; i < NUM_TIMERS; i++)
        NVIC_ClearPendingIRQ((IRQn_Type)alarm_irq_number(i));

    timer_hw->pause = 0;
    return DEVICE_OK;
}

int RP2040LowLevelTimer::enableIRQ()
{
    timer_hw->inte = irq_msk;
    return DEVICE_OK;
}

int RP2040LowLevelTimer::disable()
{
    disableIRQ();
    timer_hw->pause = 1;
    return DEVICE_OK;
}

int RP2040LowLevelTimer::disableIRQ()
{
    irq_msk = timer_hw->inte;
    timer_hw->inte = 0;
    return DEVICE_OK;
}

int RP2040LowLevelTimer::reset()
{
    int wasEnabled = alarms_enabled();
    disableIRQ();

    timer_hw->timelw = 0;
    timer_hw->timehw = 0;

    if (wasEnabled > 0)
        enableIRQ();

    return DEVICE_OK;
}

int RP2040LowLevelTimer::setMode(TimerMode t)
{
    return DEVICE_NOT_IMPLEMENTED;
}

REAL_TIME_FUNC
int RP2040LowLevelTimer::setCompare(uint8_t channel, uint32_t value)
{
    if (channel > getChannelCount() - 1)
        return DEVICE_INVALID_PARAMETER;

    timer_hw->alarm[channel] = value;
    timer_hw->inte = (1 << channel);
    ram_irq_set_priority(alarm_irq_number(channel), 2 << 6);
    ram_irq_set_enabled(alarm_irq_number(channel), true);

    return DEVICE_OK;
}

int RP2040LowLevelTimer::offsetCompare(uint8_t channel, uint32_t value)
{
    if (channel > getChannelCount() - 1)
        return DEVICE_INVALID_PARAMETER;

    timer_hw->alarm[channel] = timer_hw->alarm[channel] + value;
    timer_hw->inte = (1 << channel);

    return DEVICE_OK;
}

int RP2040LowLevelTimer::clearCompare(uint8_t channel)
{
    if (channel > getChannelCount() - 1)
        return DEVICE_INVALID_PARAMETER;

    timer_hw->inte = (1 << channel);
    return DEVICE_OK;
}

REAL_TIME_FUNC
uint32_t RP2040LowLevelTimer::captureCounter()
{
    return timer_hw->timerawl;
}

int RP2040LowLevelTimer::setClockSpeed(uint32_t speedKHz)
{
    return DEVICE_NOT_IMPLEMENTED;
}

int RP2040LowLevelTimer::setBitMode(TimerBitMode t)
{
    return DEVICE_NOT_IMPLEMENTED;
}

int RP2040LowLevelTimer::setSleep(bool doSleep)
{
    return DEVICE_NOT_IMPLEMENTED;
}