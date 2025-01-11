/* There is core_cm3.h wrapper just to avoid warnings from CMSIS headers */
/* if you want use original file add to make file:
    INC += \
        $(TOP)/$(CH32F20X_SDK_SRC)/CMSIS
*/
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-prototypes"

#include <../../CMSIS/core_cm3.h>

#pragma GCC diagnostic pop
