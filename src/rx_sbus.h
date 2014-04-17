#pragma once

void sbusInit(rcReadRawDataPtr *callback, rxConfig_t *initialRxConfig);
bool sbusFrameComplete(void);
