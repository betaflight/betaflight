#pragma once

#include "pg/rx.h"

#include "rx/rx.h"

#include <stdint.h>
#include <stdbool.h>

struct sbuf_s;

bool srxlv2RxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);
bool srxlv2RxIsActive(void);
void srxlv2RxWriteData(const void *data, int len);
bool srxlv2TelemetryRequested(void);
void srxlv2InitializeFrame(struct sbuf_s *dst);
void srxlv2FinalizeFrame(struct sbuf_s *dst);
void srxlv2Bind(void);
