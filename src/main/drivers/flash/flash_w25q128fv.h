
#pragma once

#ifdef USE_FLASH_W25Q128FV

bool w25q128fv_identify(flashDevice_t *fdevice, uint32_t jedecID);
#endif
