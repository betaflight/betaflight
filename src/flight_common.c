#include "board.h"
#include "mw.h"

#include "runtime_config.h"

#include "flight_common.h"


void mwDisarm(void)
{
    if (f.ARMED)
        f.ARMED = 0;
}
