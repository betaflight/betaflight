#pragma once

#include "pg/pg.h"
#include "common/time.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "rx/rx.h"   
//rcData[THROTTLE]

void alt_ctrl_run(uint32_t z_ref);