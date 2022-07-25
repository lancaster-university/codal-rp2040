#pragma once

#include "CodalConfig.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

void ram_irq_set_enabled(unsigned int num, bool enabled);
void ram_irq_set_priority(unsigned num, uint8_t hardware_priority);

#ifdef __cplusplus
}
#endif
