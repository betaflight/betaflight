#ifndef NRFX_CONFIG_H__
#define NRFX_CONFIG_H__

#define NRFX_POWER_ENABLED   1
#define NRFX_POWER_DEFAULT_CONFIG_IRQ_PRIORITY  7

#define NRFX_CLOCK_ENABLED   0
#define NRFX_GPIOTE_ENABLED  1
#define NRFX_GPIOTE0_ENABLED 1

#define NRFX_UARTE_ENABLED   1
#define NRFX_UARTE0_ENABLED  1

#define NRFX_SPIM_ENABLED    1
#define NRFX_SPIM1_ENABLED   1 // use SPI1 since nrf5340 share uart with spi

#define NRFX_PRS_ENABLED     0
#define NRFX_USBREG_ENABLED  1

#if defined(NRF51)
#include <templates/nrfx_config_nrf51.h>
#elif defined(NRF52805_XXAA)
#include <templates/nrfx_config_nrf52805.h>
#elif defined(NRF52810_XXAA)
#include <templates/nrfx_config_nrf52810.h>
#elif defined(NRF52811_XXAA)
#include <templates/nrfx_config_nrf52811.h>
#elif defined(NRF52820_XXAA)
#include <templates/nrfx_config_nrf52820.h>
#elif defined(NRF52832_XXAA) || defined (NRF52832_XXAB)
#include <templates/nrfx_config_nrf52832.h>
#elif defined(NRF52833_XXAA)
#include <templates/nrfx_config_nrf52833.h>
#elif defined(NRF52840_XXAA)
#include <templates/nrfx_config_nrf52840.h>
#elif defined(NRF5340_XXAA_APPLICATION)
#include <templates/nrfx_config_nrf5340_application.h>
#elif defined(NRF5340_XXAA_NETWORK)
    #include <templates/nrfx_config_nrf5340_network.h>
#elif defined(NRF9120_XXAA) || defined(NRF9160_XXAA)
    #include <templates/nrfx_config_nrf91.h>
#else
    #error "Unknown device."
#endif

#endif // NRFX_CONFIG_H__
