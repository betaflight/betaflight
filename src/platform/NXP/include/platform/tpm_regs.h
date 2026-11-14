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

/*
 * NXP TPM (Timer/PWM Module) registers, as found on the i.MX 95.
 */

#pragma once

#include <stdint.h>

typedef struct {
    volatile uint32_t CnSC;    /* Channel n Status and Control */
    volatile uint32_t CnV;     /* Channel n Value (compare)    */
} TPM_Channel_Type;

typedef struct {
    volatile uint32_t VERID;        /* 0x00 Version ID            */
    volatile uint32_t PARAM;        /* 0x04 Parameter             */
    volatile uint32_t GLOBAL;       /* 0x08 Global (software reset)*/
    volatile uint32_t RESV0;        /* 0x0C                       */
    volatile uint32_t SC;           /* 0x10 Status and Control    */
    volatile uint32_t CNT;          /* 0x14 Counter               */
    volatile uint32_t MOD;          /* 0x18 Modulo (period)       */
    volatile uint32_t STATUS;       /* 0x1C Capture/Compare Status*/
    TPM_Channel_Type  CONTROLS[6];  /* 0x20.. CnSC/CnV pairs      */
    volatile uint32_t RESV1[5];     /* 0x50..0x60                 */
    volatile uint32_t COMBINE;      /* 0x64                       */
    volatile uint32_t RESV2;        /* 0x68                       */
    volatile uint32_t TRIG;         /* 0x6C                       */
    volatile uint32_t POL;          /* 0x70 Channel polarity      */
    volatile uint32_t RESV3;        /* 0x74                       */
    volatile uint32_t FILTER;       /* 0x78                       */
    volatile uint32_t RESV4;        /* 0x7C                       */
    volatile uint32_t QDCTRL;       /* 0x80                       */
    volatile uint32_t CONF;         /* 0x84 Configuration         */
} TPM_Type;

// GLOBAL
#define TPM_GLOBAL_RST          (1u << 1)   /* software reset of all logic   */

// SC
#define TPM_SC_PS_MASK          (7u << 0)   /* prescale = 2^PS               */
#define TPM_SC_PS(n)            (((n) & 7u) << 0)
#define TPM_SC_CMOD_MASK        (3u << 3)   /* clock mode                    */
#define TPM_SC_CMOD(n)          (((n) & 3u) << 3)   /* 1 = count on TPM clk  */
#define TPM_SC_CPWMS            (1u << 5)   /* center-aligned PWM            */
#define TPM_SC_TOIE             (1u << 6)   /* timer overflow IRQ enable     */
#define TPM_SC_TOF              (1u << 7)   /* timer overflow flag (w1c)     */
#define TPM_SC_DMA              (1u << 8)   /* DMA enable                    */

// CnSC
#define TPM_CnSC_DMA            (1u << 0)
#define TPM_CnSC_ELSA           (1u << 2)
#define TPM_CnSC_ELSB           (1u << 3)
#define TPM_CnSC_MSA            (1u << 4)
#define TPM_CnSC_MSB            (1u << 5)   /* MSB:MSA = 10 -> edge-aligned PWM */
#define TPM_CnSC_CHIE           (1u << 6)
#define TPM_CnSC_CHF            (1u << 7)   /* channel flag (w1c)            */

// Edge-aligned PWM, output high from CNT = 0 until CNT == CnV.
#define TPM_CnSC_PWM_HIGH       (TPM_CnSC_MSB | TPM_CnSC_ELSB)
