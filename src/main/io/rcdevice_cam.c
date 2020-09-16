/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "pg/rx.h"

#include "common/time.h"

#include "cms/cms.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/rcdevice_cam.h"

#include "rx/rx.h"

#include "pg/rcdevice.h"

#ifdef USE_RCDEVICE

#define IS_HI(X) (rcData[X] > FIVE_KEY_CABLE_JOYSTICK_MAX)
#define IS_LO(X) (rcData[X] < FIVE_KEY_CABLE_JOYSTICK_MIN)
#define IS_MID(X) (rcData[X] > FIVE_KEY_CABLE_JOYSTICK_MID_START && rcData[X] < FIVE_KEY_CABLE_JOYSTICK_MID_END)


static runcamDevice_t runcamDevice;
runcamDevice_t *camDevice = &runcamDevice;
rcdeviceSwitchState_t switchStates[BOXCAMERA3 - BOXCAMERA1 + 1];
bool rcdeviceInMenu = false;
bool isButtonPressed = false;
bool waitingDeviceResponse = false;


static bool isFeatureSupported(uint16_t feature)
{
    if (camDevice->info.features & feature || rcdeviceConfig()->feature & feature) {
        return true;
    }

    return false;
}

bool rcdeviceIsEnabled(void)
{
    return camDevice->serialPort != NULL;
}

static void rcdeviceCameraControlProcess(void)
{
    for (boxId_e i = BOXCAMERA1; i <= BOXCAMERA3; i++) {
        uint8_t switchIndex = i - BOXCAMERA1;

        if (IS_RC_MODE_ACTIVE(i)) {

            // check last state of this mode, if it's true, then ignore it.
            // Here is a logic to make a toggle control for this mode
            if (switchStates[switchIndex].isActivated) {
                continue;
            }

            uint8_t behavior = RCDEVICE_PROTOCOL_CAM_CTRL_UNKNOWN_CAMERA_OPERATION;
            switch (i) {
            case BOXCAMERA1:
                if (isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_SIMULATE_WIFI_BUTTON)) {
                    // avoid display wifi page when arming, in the next firmware(>2.0) of rcsplit we have change the wifi page logic:
                    // when the wifi was turn on it won't turn off the analog video output,
                    // and just put a wifi indicator on the right top of the video output. here is for the old split firmware
                    if (!ARMING_FLAG(ARMED) && !(getArmingDisableFlags() & (ARMING_DISABLED_RUNAWAY_TAKEOFF | ARMING_DISABLED_CRASH_DETECTED))) {
                        behavior = RCDEVICE_PROTOCOL_CAM_CTRL_SIMULATE_WIFI_BTN;
                    }
                }
                break;
            case BOXCAMERA2:
                if (isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_SIMULATE_POWER_BUTTON)) {
                    behavior = RCDEVICE_PROTOCOL_CAM_CTRL_SIMULATE_POWER_BTN;
                }
                break;
            case BOXCAMERA3:
                if (isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_CHANGE_MODE)) {
                    // avoid change camera mode when arming
                    if (!ARMING_FLAG(ARMED) && !(getArmingDisableFlags() & (ARMING_DISABLED_RUNAWAY_TAKEOFF | ARMING_DISABLED_CRASH_DETECTED))) {
                        behavior = RCDEVICE_PROTOCOL_CAM_CTRL_CHANGE_MODE;
                    }
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

static void rcdeviceSimulationOSDCableFailed(rcdeviceResponseParseContext_t *ctx)
{
    waitingDeviceResponse = false;
    if (ctx->command == RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION) {
        uint8_t operationID = ctx->paramData[0];
        if (operationID == RCDEVICE_PROTOCOL_5KEY_CONNECTION_CLOSE) {
            return;
        }
    }
}

static void rcdeviceSimulationRespHandle(rcdeviceResponseParseContext_t *ctx)
{
    if (ctx->result != RCDEVICE_RESP_SUCCESS) {
        rcdeviceSimulationOSDCableFailed(ctx);
        waitingDeviceResponse = false;
        return;
    }

    switch (ctx->command) {
    case RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_RELEASE:
        isButtonPressed = false;
        break;
    case RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION:
    {
        // the high 4 bits is the operationID that we sent
        // the low 4 bits is the result code
        isButtonPressed = true;
        uint8_t operationID = ctx->paramData[0];
        bool errorCode = (ctx->recvBuf[1] & 0x0F);
        if (operationID == RCDEVICE_PROTOCOL_5KEY_CONNECTION_OPEN) {
            if (errorCode == 1) {
                rcdeviceInMenu = true;
                beeper(BEEPER_CAM_CONNECTION_OPEN);
            } else {
                beeper(BEEPER_CAM_CONNECTION_CLOSE);
            }
        } else if (operationID == RCDEVICE_PROTOCOL_5KEY_CONNECTION_CLOSE) {
            if (errorCode == 1) {
                rcdeviceInMenu = false;
                beeper(BEEPER_CAM_CONNECTION_CLOSE);
            }
        }
    }
        break;
    case RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_PRESS:
        isButtonPressed = true;
        break;
    }

    waitingDeviceResponse = false;
}

static void rcdeviceCamSimulate5KeyCablePress(rcdeviceCamSimulationKeyEvent_e key)
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
    case RCDEVICE_CAM_KEY_NONE:
    default:
        operation = RCDEVICE_PROTOCOL_5KEY_SIMULATION_NONE;
        break;
    }

    runcamDeviceSimulate5KeyOSDCableButtonPress(camDevice, operation, rcdeviceSimulationRespHandle);
}

void rcdeviceSend5KeyOSDCableSimualtionEvent(rcdeviceCamSimulationKeyEvent_e key)
{
    switch (key) {
    case RCDEVICE_CAM_KEY_CONNECTION_OPEN:
        runcamDeviceOpen5KeyOSDCableConnection(camDevice, rcdeviceSimulationRespHandle);
        break;
    case RCDEVICE_CAM_KEY_CONNECTION_CLOSE:
        runcamDeviceClose5KeyOSDCableConnection(camDevice, rcdeviceSimulationRespHandle);
        break;
    case RCDEVICE_CAM_KEY_ENTER:
    case RCDEVICE_CAM_KEY_LEFT:
    case RCDEVICE_CAM_KEY_UP:
    case RCDEVICE_CAM_KEY_RIGHT:
    case RCDEVICE_CAM_KEY_DOWN:
        rcdeviceCamSimulate5KeyCablePress(key);
        break;
    case RCDEVICE_CAM_KEY_RELEASE:
        runcamDeviceSimulate5KeyOSDCableButtonRelease(camDevice, rcdeviceSimulationRespHandle);
        break;
    case RCDEVICE_CAM_KEY_NONE:
    default:
        break;
    }
}

static void rcdevice5KeySimulationProcess(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

#ifdef USE_CMS
    if (cmsInMenu) {
        return;
    }
#endif

    if (ARMING_FLAG(ARMED) || IS_RC_MODE_ACTIVE(BOXSTICKCOMMANDDISABLE) || (getArmingDisableFlags() & (ARMING_DISABLED_RUNAWAY_TAKEOFF | ARMING_DISABLED_CRASH_DETECTED))) {
        return;
    }

    if (isButtonPressed) {
        if (IS_MID(YAW) && IS_MID(PITCH) && IS_MID(ROLL)) {
            rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_RELEASE);
            waitingDeviceResponse = true;
        }
    } else {
        if (waitingDeviceResponse) {
            return;
        }

        rcdeviceCamSimulationKeyEvent_e key = RCDEVICE_CAM_KEY_NONE;

        if (IS_MID(THROTTLE) && IS_MID(ROLL) && IS_MID(PITCH) && IS_LO(YAW)) { // Disconnect Lo YAW
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

        if (key != RCDEVICE_CAM_KEY_NONE) {
            rcdeviceSend5KeyOSDCableSimualtionEvent(key);
            isButtonPressed = true;
            waitingDeviceResponse = true;
        }
    }
}

static void rcdeviceProcessDeviceRequest(runcamDeviceRequest_t *request)
{
    switch (request->command) {
        case RCDEVICE_PROTOCOL_COMMAND_REQUEST_FC_ATTITUDE:
            runcamDeviceSendAttitude(camDevice);
            break;
    }
}

void rcdeviceUpdate(timeUs_t currentTimeUs)
{
    rcdeviceReceive(currentTimeUs);

    rcdeviceCameraControlProcess();

    rcdevice5KeySimulationProcess(currentTimeUs);

    if (isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_FC_ATTITUDE)) {
        runcamDeviceRequest_t *request = rcdeviceGetRequest();
        if (request) {
            rcdeviceProcessDeviceRequest(request);
        }
    }
}

void rcdeviceInit(void)
{
    // open serial port
    runcamDeviceInit(camDevice);

    for (boxId_e i = BOXCAMERA1; i <= BOXCAMERA3; i++) {
        uint8_t switchIndex = i - BOXCAMERA1;
        switchStates[switchIndex].isActivated = true;
    }
}

#endif
