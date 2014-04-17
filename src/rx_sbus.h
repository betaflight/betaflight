#pragma once

void sbusInit(rxConfig_t *initialRxConfig, rcReadRawDataPtr *callback);
bool sbusFrameComplete(void);
