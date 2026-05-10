/*
 * OTP interface stub.
 *
 * ST's CubeN6 reference OBL ships an OTP fuse-access path via the BSEC
 * peripheral HAL. We don't enable BSEC HAL (no security/lifecycle work
 * needed for our recovery + BF-loader use case), so the upstream
 * otp_interface.c won't compile in our build. This stub provides the
 * Init/DeInit symbols app_openbootloader.c calls and stubs Read/Write so
 * any host attempt at OTP commands no-ops cleanly rather than wedging
 * the device. If a manufacturer needs OTP commissioning later, replace
 * this file with the real implementation behind a per-config knob.
 */

#include <string.h>

#include "main.h"
#include "otp_interface.h"

void OPENBL_OTP_Init(void)
{
}

void OPENBL_OTP_DeInit(void)
{
}

Otp_Partition_t OPENBL_OTP_Read(void)
{
    Otp_Partition_t result;
    memset(&result, 0, sizeof(result));
    result.Version     = OPENBL_OTP_VERSION;
    result.GlobalState = BSEC_SEC_OTP_INVALID;
    return result;
}

int OPENBL_OTP_Write(Otp_Partition_t Otp)
{
    (void)Otp;
    return OTP_ERROR;
}
