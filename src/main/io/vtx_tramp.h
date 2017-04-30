#pragma once

#if defined(VTX_TRAMP) && defined(VTX_CONTROL)

bool vtxTrampInit();

#ifdef CMS
#include "cms/cms.h"
#include "cms/cms_types.h"
extern CMS_Menu cmsx_menuVtxTramp;
#endif

#endif
