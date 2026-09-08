#include "platform.h"

#ifdef USE_SDCARD

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/sdmmc_sdio.h"

#include "pg/sdio.h"

bool mscSdioInitDma(void)
{
#if ENABLE_SDIO_EXTERNAL_DMA
    const dmaChannelSpec_t *dmaChannelSpec =
        dmaGetChannelSpecByPeripheral(DMA_PERIPH_SDIO, 0, sdioConfig()->dmaopt);
    dmaResource_t *dmaRef = dmaChannelSpec ? dmaChannelSpec->ref : NULL;
    if (!dmaRef) {
        return false;
    }
#else
    dmaResource_t *dmaRef = NULL;
#endif
    return SD_InitialiseHardware(dmaRef);
}

#endif
