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
    if (!dmaChannelSpec) return false;
    return SD_Initialize_LL((DMA_ARCH_TYPE *)dmaChannelSpec->ref);
#elif defined(PLATFORM_TRAIT_SDIO_INIT)
    return SD_Initialize_LL(0);
#else
    return SD_Initialize_LL(SDCARD_SDIO_DMA_OPT);
#endif
}

#endif
