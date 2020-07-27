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

#include "platform.h"

#if defined(USE_RX_MSP_OVERRIDE)

#include "rx/msp_override.h"
#include "rx/msp.h"
#include "fc/rc_modes.h"
#include "common/maths.h"


uint16_t rxMspOverrideReadRawRc(const rxRuntimeState_t *rxRuntimeState, const rxConfig_t *rxConfig, uint8_t chan)
{
    uint16_t rxSample = (rxRuntimeState->rcReadRawFn)(rxRuntimeState, chan);

    uint16_t overrideSample = rxMspReadRawRC(rxRuntimeState, chan);
    bool override = (1 << chan) & rxConfig->msp_override_channels_mask;

    if (IS_RC_MODE_ACTIVE(BOXMSPOVERRIDE) && override) {
        return overrideSample;
    } else {
        return rxSample;
    }
}
#endif
