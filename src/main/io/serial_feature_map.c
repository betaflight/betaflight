/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "io/serial.h"
#include "io/serial_feature_map.h"

#ifdef USE_GPS
#include "pg/gps.h"
#endif
#if defined(USE_RX_PWM) || defined(USE_RX_PPM) || defined(USE_SERIALRX) || defined(USE_RX_MSP) || defined(USE_RX_SPI)
#include "pg/rx.h"
#endif
#ifdef USE_BLACKBOX
#include "blackbox/blackbox.h"
#endif
#ifdef USE_ESC_SENSOR
#include "sensors/esc_sensor.h"
#endif
#ifdef USE_RCDEVICE
#include "pg/rcdevice.h"
#endif
#ifdef USE_GIMBAL
#include "pg/gimbal.h"
#endif
#ifdef USE_VTX_COMMON
#include "drivers/vtx_common.h"
#include "io/vtx.h"
#endif
#ifdef USE_RANGEFINDER
#include "sensors/rangefinder.h"
#endif
#ifdef USE_OSD
#include "osd/osd.h"
#endif

uint32_t serialSynthesizeFunctionMask(serialPortIdentifier_e identifier)
{
    if (identifier == SERIAL_PORT_NONE) {
        return 0;
    }

    uint32_t mask = 0;

#ifdef USE_GPS
    if (gpsConfig()->gps_uart == identifier) {
        mask |= FUNCTION_GPS;
    }
#endif
#if defined(USE_RX_PWM) || defined(USE_RX_PPM) || defined(USE_SERIALRX) || defined(USE_RX_MSP) || defined(USE_RX_SPI)
    if (rxConfig()->rx_uart == identifier) {
        mask |= FUNCTION_RX_SERIAL;
    }
#endif
#ifdef USE_BLACKBOX
    if (blackboxConfig()->blackbox_uart == identifier) {
        mask |= FUNCTION_BLACKBOX;
    }
#endif
#ifdef USE_ESC_SENSOR
    if (escSensorConfig()->esc_sensor_uart == identifier) {
        mask |= FUNCTION_ESC_SENSOR;
    }
#endif
#ifdef USE_RCDEVICE
    if (rcdeviceConfig()->rcdevice_uart == identifier) {
        mask |= FUNCTION_RCDEVICE;
    }
#endif
#ifdef USE_GIMBAL
    if (gimbalTrackConfig()->gimbal_uart == identifier) {
        mask |= FUNCTION_GIMBAL;
    }
#endif
#ifdef USE_VTX_COMMON
    if (vtxSettingsConfig()->vtx_uart == identifier) {
        switch (vtxSettingsConfig()->vtx_type) {
#ifdef USE_VTX_SMARTAUDIO
        case VTXDEV_SMARTAUDIO:
            mask |= FUNCTION_VTX_SMARTAUDIO;
            break;
#endif
#ifdef USE_VTX_TRAMP
        case VTXDEV_TRAMP:
            mask |= FUNCTION_VTX_TRAMP;
            break;
#endif
#ifdef USE_VTX_MSP
        case VTXDEV_MSP:
            mask |= FUNCTION_VTX_MSP;
            break;
#endif
        default:
            break;
        }
    }
#endif
#ifdef USE_RANGEFINDER
    if (rangefinderConfig()->rangefinder_uart == identifier) {
        switch (rangefinderConfig()->rangefinder_hardware) {
        case RANGEFINDER_TFMINI:
        case RANGEFINDER_TF02:
        case RANGEFINDER_TFNOVA:
        case RANGEFINDER_UPT1:
            mask |= FUNCTION_LIDAR_TF;
            break;
        case RANGEFINDER_NOOPLOOP_F2:
        case RANGEFINDER_NOOPLOOP_F2P:
        case RANGEFINDER_NOOPLOOP_F2PH:
        case RANGEFINDER_NOOPLOOP_F:
        case RANGEFINDER_NOOPLOOP_FP:
        case RANGEFINDER_NOOPLOOP_F2MINI:
            mask |= FUNCTION_LIDAR_NL;
            break;
        default:
            break;
        }
    }
#endif
#ifdef USE_OSD
    if (osdConfig()->osd_uart == identifier && osdConfig()->displayPortDevice == OSD_DISPLAYPORT_DEVICE_FRSKYOSD) {
        mask |= FUNCTION_FRSKY_OSD;
    }
    if (osdConfig()->osd_custom_text_uart == identifier) {
        mask |= FUNCTION_OSD_CUSTOM_TEXT;
    }
#endif

    return mask;
}

bool serialApplyFunctionMask(serialPortIdentifier_e identifier, uint32_t mask)
{
    // Decomposer wired in a later task; for now behavior is unchanged.
    (void)identifier;
    (void)mask;
    return true;
}
