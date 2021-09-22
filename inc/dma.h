#pragma once

#include "CodalConfig.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef void (*DMAChannelCallback)(void *);

    typedef struct __DMAHandlerDef
    {
        void *context;
        DMAChannelCallback handler;
    } DMAHandler;

    void DMA_SetChannelCallback(uint8_t channel, DMAChannelCallback handler, void *context);

#ifdef __cplusplus
}
#endif
