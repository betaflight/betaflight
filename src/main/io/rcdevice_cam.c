/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "drivers/time.h"

#include "cms/cms.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/rcdevice_cam.h"

#include "rx/rx.h"

#ifdef USE_RCDEVICE

#define IS_HI(X) (rcData[X] > FIVE_KEY_CABLE_JOYSTICK_MAX)
#define IS_LO(X) (rcData[X] < FIVE_KEY_CABLE_JOYSTICK_MIN)
#define IS_MID(X) (rcData[X] > FIVE_KEY_CABLE_JOYSTICK_MID_START && rcData[X] < FIVE_KEY_CABLE_JOYSTICK_MID_END)

static runcamDevice_t runcamDevice;
runcamDevice_t *camDevice = &runcamDevice;
rcdeviceSwitchState_t switchStates[BOXCAMERA3 - BOXCAMERA1 + 1];
bool rcdeviceInMenu;
bool needRelease = false;

static bool isFeatureSupported(uint8_t feature)
{
    if (camDevice->info.features & feature) {
        return true;
    }

    return false;
}

static bool rcdeviceIsCameraControlEnabled(void)
{
    bool isPowerSimulationSupported = isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_SIMULATE_POWER_BUTTON);
    bool isWiFiSimulationSupported = isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_SIMULATE_WIFI_BUTTON);
    bool isChangeModeSupported = isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_CHANGE_MODE);
    bool isStartRecordingSupported = isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_START_RECORDING);
    bool isStopRecordingSupported = isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_STOP_RECORDING);

    if (camDevice->serialPort != NULL && (isPowerSimulationSupported || isWiFiSimulationSupported || isChangeModeSupported || isStartRecordingSupported || isStopRecordingSupported)) {
        return true;
    }

    return false;
}

bool rcdeviceIsEnabled(void)
{
    return findSerialPortConfig(FUNCTION_RCDEVICE) != NULL;
}

static bool rcdeviceIs5KeyEnabled(void)
{
    if (camDevice->serialPort != NULL && isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_SIMULATE_5_KEY_OSD_CABLE)) {
        return true;
    }

    return false;
}

static bool reInitializeDevice() {
#define MAX_RETRY_COUNT  4
#define RETRY_INTERVAL_MS  500
    static timeMs_t lastInitializeTime = 0;
    static int tryInitCount = 0;
    bool result = false;

    if (ARMING_FLAG(ARMED)) {
        return false;
    }

    if (tryInitCount < MAX_RETRY_COUNT) {
        timeMs_t nextRetryTime = lastInitializeTime + RETRY_INTERVAL_MS;
        if (millis() >= nextRetryTime) {
            result = rcdeviceInit();
            tryInitCount++;
            lastInitializeTime = millis();
        }
    }

    return result;
}

static void rcdeviceCameraControlProcess(void)
{
    for (boxId_e i = BOXCAMERA1; i <= BOXCAMERA3; i++) {
        uint8_t switchIndex = i - BOXCAMERA1;

        if (IS_RC_MODE_ACTIVE(i)) {
            if (!rcdeviceIsCameraControlEnabled()) {
                reInitializeDevice();
            }

            // check last state of this mode, if it's true, then ignore it.
            // Here is a logic to make a toggle control for this mode
            if (switchStates[switchIndex].isActivated) {
                continue;
            }

            uint8_t behavior = RCDEVICE_PROTOCOL_CAM_CTRL_UNKNOWN_CAMERA_OPERATION;
            switch (i) {
            case BOXCAMERA1:
                if (isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_SIMULATE_WIFI_BUTTON)) {
                    behavior = RCDEVICE_PROTOCOL_CAM_CTRL_SIMULATE_WIFI_BTN;
                }
                break;
            case BOXCAMERA2:
                if (isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_SIMULATE_POWER_BUTTON)) {
                    behavior = RCDEVICE_PROTOCOL_CAM_CTRL_SIMULATE_POWER_BTN;
                }
                break;
            case BOXCAMERA3:
                if (isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_CHANGE_MODE)) {
                    behavior = RCDEVICE_PROTOCOL_CAM_CTRL_CHANGE_MODE;
                }
                break;
            default:
                break;
            }
            if (behavior != RCDEVICE_PROTOCOL_CAM_CTRL_UNKNOWN_CAMERA_OPERATION) {
                runcamDeviceSimulateCameraButton(camDevice, behavior);
                switchStates[switchIndex].isActivated = true;
            }
        } else {
            switchStates[switchIndex].isActivated = false;
        }
    }
}

static bool rcdeviceCamSimulate5KeyCablePress(rcdeviceCamSimulationKeyEvent_e key)
{
    uint8_t operation = RCDEVICE_PROTOCOL_5KEY_SIMULATION_NONE;
    switch (key) {
    case RCDEVICE_CAM_KEY_LEFT:
        operation = RCDEVICE_PROTOCOL_5KEY_SIMULATION_LEFT;
        break;
    case RCDEVICE_CAM_KEY_UP:
        operation = RCDEVICE_PROTOCOL_5KEY_SIMULATION_UP;
        break;
    case RCDEVICE_CAM_KEY_RIGHT:
        operation = RCDEVICE_PROTOCOL_5KEY_SIMULATION_RIGHT;
        break;
    case RCDEVICE_CAM_KEY_DOWN:
        operation = RCDEVICE_PROTOCOL_5KEY_SIMULATION_DOWN;
        break;
    case RCDEVICE_CAM_KEY_ENTER:
        operation = RCDEVICE_PROTOCOL_5KEY_SIMULATION_SET;
        break;
    default:
        operation = RCDEVICE_PROTOCOL_5KEY_SIMULATION_NONE;
        break;
    }

    return runcamDeviceSimulate5KeyOSDCableButtonPress(camDevice, operation);
}

static bool rcdeviceSend5KeyOSDCableSimualtionEvent(rcdeviceCamSimulationKeyEvent_e key)
{
    bool reqResult = false;
    switch (key) {
    case RCDEVICE_CAM_KEY_CONNECTION_OPEN:
        reqResult = runcamDeviceOpen5KeyOSDCableConnection(camDevice);
        if (reqResult) {
            rcdeviceInMenu = true;
            beeper(BEEPER_CAM_CONNECTION_OPEN);
        }
        break;
    case RCDEVICE_CAM_KEY_CONNECTION_CLOSE: {
        uint8_t resultCode = 0;
        reqResult = runcamDeviceClose5KeyOSDCableConnection(camDevice, &resultCode);
        if (resultCode == 1) {
            rcdeviceInMenu = false;
            beeper(BEEPER_CAM_CONNECTION_CLOSE);
        }
    }
        break;
    case RCDEVICE_CAM_KEY_ENTER:
    case RCDEVICE_CAM_KEY_LEFT:
    case RCDEVICE_CAM_KEY_UP:
    case RCDEVICE_CAM_KEY_RIGHT:
    case RCDEVICE_CAM_KEY_DOWN:
        reqResult = rcdeviceCamSimulate5KeyCablePress(key);
        break;
    case RCDEVICE_CAM_KEY_RELEASE:
        reqResult = runcamDeviceSimulate5KeyOSDCableButtonRelease(camDevice);
        break;
    default:
        reqResult = false;
        break;
    }

    return reqResult;
}

static void rcdevice5KeySimulationProcess(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

#ifdef USE_CMS
    if (cmsInMenu) {
        return;
    }
#endif

    if (ARMING_FLAG(ARMED) || getArmingDisableFlags() & ARMING_DISABLED_RUNAWAY_TAKEOFF) {
        return;
    }

    rcdeviceCamSimulationKeyEvent_e key = RCDEVICE_CAM_KEY_NONE;

    if (needRelease) {
        if (IS_MID(YAW) && IS_MID(PITCH) && IS_MID(ROLL)) {
            if ((camDevice->serialPort != NULL || reInitializeDevice()) && rcdeviceIs5KeyEnabled()) {
                key = RCDEVICE_CAM_KEY_RELEASE;
                if (rcdeviceSend5KeyOSDCableSimualtionEvent(key)) {
                    needRelease = false;
                } else {
                    rcdeviceInMenu = false;
                }
            }
        }
        return;
    } else {
        if (IS_MID(THROTTLE) && IS_MID(ROLL) && IS_MID(PITCH) && IS_LO(YAW)) { // Disconnect HI YAW
            if (rcdeviceInMenu) {
                key = RCDEVICE_CAM_KEY_CONNECTION_CLOSE;
            }
        } else {
            if (rcdeviceInMenu) {
                if (IS_LO(ROLL)) { // Left LO ROLL
                    key = RCDEVICE_CAM_KEY_LEFT;
                } else if (IS_HI(PITCH)) { // Up HI PITCH
                    key = RCDEVICE_CAM_KEY_UP;
                } else if (IS_HI(ROLL)) { // Right HI ROLL
                    key = RCDEVICE_CAM_KEY_RIGHT;
                } else if (IS_LO(PITCH)) { // Down LO PITCH
                    key = RCDEVICE_CAM_KEY_DOWN;
                } else if (IS_MID(THROTTLE) && IS_MID(ROLL) && IS_MID(PITCH) && IS_HI(YAW)) { // Enter HI YAW
                    key = RCDEVICE_CAM_KEY_ENTER;
                }
            } else {
                if (IS_MID(THROTTLE) && IS_MID(ROLL) && IS_MID(PITCH) && IS_HI(YAW)) { // Enter HI YAW
                    key = RCDEVICE_CAM_KEY_CONNECTION_OPEN;
                }
            }
        }
    }

    if (key != RCDEVICE_CAM_KEY_NONE) {
        if ((camDevice->serialPort != NULL || reInitializeDevice()) && rcdeviceIs5KeyEnabled()) {
            if (rcdeviceSend5KeyOSDCableSimualtionEvent(key)) {
                needRelease = true;
            } else {
                rcdeviceInMenu = false;
            }
        }
    }
}

void rcdeviceUpdate(timeUs_t currentTimeUs)
{
    rcdeviceCameraControlProcess();
    rcdevice5KeySimulationProcess(currentTimeUs);
}

bool rcdeviceInit(void)
{
    // open serial port
    if (!runcamDeviceInit(camDevice)) {
        return false;
    }

    for (boxId_e i = BOXCAMERA1; i <= BOXCAMERA3; i++) {
        uint8_t switchIndex = i - BOXCAMERA1;
        switchStates[switchIndex].isActivated = true;
    }

    return true;
}

#endif
