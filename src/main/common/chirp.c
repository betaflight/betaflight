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

#include <math.h>

#include "platform.h"

#include "maths.h"

#include "chirp.h"

// initialize the chirp signal generator
// f0: start frequency in Hz
// f1: end frequency in Hz
// t1: signal length in seconds
// looptimeUs: loop time in microseconds
void chirpInit(chirp_t *chirp, const float f0, const float f1, const float t1, const uint32_t looptimeUs)
{
    chirp->f0 = f0;
    chirp->Ts = looptimeUs * 1e-6f;
    chirp->N = (uint32_t)(t1 / chirp->Ts);
    chirp->beta = pow_approx(f1 / f0, 1.0f / t1);
    chirp->k0 = 2.0f * M_PIf / log_approx(chirp->beta);
    chirp->k1 = chirp->k0 * f0;
    chirpReset(chirp);
}

// reset the chirp signal generator fully
void chirpReset(chirp_t *chirp)
{
    chirp->count = 0;
    chirp->isFinished = false;
    chirpResetSignals(chirp);
}

// reset the chirp signal generator signals
void chirpResetSignals(chirp_t *chirp)
{
    chirp->exc = 0.0f;
    chirp->fchirp = 0.0f;
    chirp->sinarg = 0.0f;
}

// update the chirp signal generator
bool chirpUpdate(chirp_t *chirp)
{
    if (chirp->isFinished) {

        return false;

    } else if (chirp->count == chirp->N) {

        chirp->isFinished = true;
        chirpResetSignals(chirp);
        return false;

    } else {

        chirp->fchirp = chirp->f0 * pow_approx(chirp->beta, (float)(chirp->count) * chirp->Ts);
        chirp->sinarg = chirp->k0 * chirp->fchirp - chirp->k1;

        // wrap sinarg to 0...2*pi
        chirp->sinarg = fmodf(chirp->sinarg, 2.0f * M_PIf);

        // use cosine so that the angle will oscillate around 0 (integral of gyro)
        chirp->exc = cos_approx(chirp->sinarg);

        // frequencies below 1 Hz will lead to the same angle magnitude as at 1 Hz (integral of gyro)
        if (chirp->fchirp < 1.0f) {
            chirp->exc = chirp->fchirp * chirp->exc;
        }
        chirp->count++;

        return true;
    }
}
