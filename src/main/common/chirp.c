#include "chirp.h"
#include "maths.h"

#include <math.h>

void chirpInit(chirp_t *chirp, const float f0, const float f1, const float t1, const uint32_t looptimeUs)
{
    chirp->f0 = f0;
    chirp->Ts = looptimeUs * 1e-6f;
    chirp->N = (uint32_t)(t1 / chirp->Ts);
    chirp->beta = pow_approx(f1 / chirp->f0, 1.0f / t1);
    chirp->k0 = 2.0f * M_PIf / log_approx(chirp->beta);
    chirp->k1 = chirp->k0 * chirp->f0;
    chirpReset(chirp);
}

void chirpReset(chirp_t *chirp)
{
    chirp->count = 0;
    chirp->isFinished = false;
    chirpResetSignals(chirp);
}

void chirpResetSignals(chirp_t *chirp)
{
    chirp->exc = 0.0f;
    chirp->fchirp = 0.0f;
    chirp->sinarg = 0.0f;
}

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