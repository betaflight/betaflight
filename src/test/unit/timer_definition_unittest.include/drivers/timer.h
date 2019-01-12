#include <mock_enums.h>

typedef struct timerHardware_s {
    enum TestTimerEnum timer;
    enum TestChannelEnum channel;
    enum TestPinEnum pin;
    enum TestTimUseEnum purpose;
    unsigned int def_tim_counter;
} timerHardware_t;

// F7 and F4 have 6 arguments, F3 and F1 have 5 arguments.
#define DEF_TIM(timer_, channel_, pin_, purpose_, ...)  \
{                                                       \
    .timer = timer_,                                    \
    .channel = channel_,                                \
    .pin = pin_,                                        \
    .purpose = purpose_,                                \
    .def_tim_counter = __COUNTER__,                     \
}

#define TIM_N(n) (1 << (n))
