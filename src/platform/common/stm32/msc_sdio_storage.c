#include "platform.h"

#ifdef USE_SDCARD

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/sdmmc_sdio.h"

#include "pg/sdio.h"

bool mscSdioInitDma(void)
{
#ifdef USE_DMA_SPEC
    const dmaChannelSpec_t *dmaChannelSpec =
        dmaGetChannelSpecByPeripheral(DMA_PERIPH_SDIO, 0, sdioConfig()->dmaopt);
    dmaResource_t *dmaRef = dmaChannelSpec ? dmaChannelSpec->ref : NULL;
#else
    dmaResource_t *dmaRef = NULL;
#endif
    if (!dmaRef) {
        return false;
    }
    return SD_InitialiseHardware(dmaRef);
}

#endif
