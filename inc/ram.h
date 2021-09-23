#pragma once

#include "CodalConfig.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

void ram_irq_set_enabled(unsigned int num, bool enabled);

#ifdef __cplusplus
}
#endif
