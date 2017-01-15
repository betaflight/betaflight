#pragma once

bool trampInit();
void trampProcess(uint32_t currentTimeUs);

#ifdef CMS
#include "cms/cms.h"
#include "cms/cms_types.h"
extern CMS_Menu cmsx_menuVtxTramp;
#endif
