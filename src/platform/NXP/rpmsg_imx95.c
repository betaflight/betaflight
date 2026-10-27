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
 * RPMsg link to Linux on the A55 (imx_rproc attach mode); the resource table and vrings sit in reserved DDR.
 * MU7 side B channel 1 is the doorbell both ways, virtqueue id in bits 31:16. Virtqueue work runs from rpmsgPoll().
 * DDR survives any reset: a doorbell Linux takes at boot resumes its table, else a clean table is published.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_RPMSG

#include <openamp/remoteproc.h>
#include <openamp/remoteproc_virtio.h>
#include <openamp/rpmsg_virtio.h>

#include "common/utils.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#include "rpmsg_imx95.h"

#define RPMSG_MU                MU7_MUB
#define RPMSG_MU_IRQ            MU7_B_IRQn
#define RPMSG_MU_PRIORITY       NVIC_BUILD_PRIORITY(0x0f, 0x0f)
#define RPMSG_MU_CHANNEL        1
#define RPMSG_MU_VQID_SHIFT     16

#define RPMSG_VDEV_NOTIFYID     2
#define RPMSG_VRING_ALIGN       0x1000
#define RPMSG_VRING_ENTRIES     256
#define RPMSG_VRING_ALIGN_MIN   4
#define RPMSG_MAX_SERVICES      4
#define RPMSG_PROBE_TIMEOUT_MS  100

extern uint8_t __rpmsg_vring0[];
extern uint8_t __rpmsg_vring1[];
extern uint8_t __rpmsg_shm_end[];
extern uint8_t __rpmsg_rsc_table[];

typedef struct {
    struct resource_table header;
    uint32_t offset[1];
    struct fw_rsc_vdev vdev;
    struct fw_rsc_vdev_vring vring[2];
} __attribute__((packed)) rpmsgResourceTable_t;

#define RPMSG_RSC_TABLE ((rpmsgResourceTable_t *)__rpmsg_rsc_table)

static bool rpmsgStarted;
static bool rpmsgTableReady;
static timeMs_t rpmsgProbeStart;
static metal_phys_addr_t rpmsgShmPhys;
static metal_phys_addr_t rpmsgRscPhys;
static struct metal_io_region rpmsgShmIo;
static struct metal_io_region rpmsgRscIo;
static struct virtio_device *rpmsgVdev;
static struct rpmsg_virtio_device rpmsgRvdev;
static struct rpmsg_device *rpmsgDevice;
static rpmsgServiceFn *rpmsgServices[RPMSG_MAX_SERVICES];

// libmetal's generic system maps nothing: physical and virtual addresses match.
void *metal_machine_io_mem_map(void *va, metal_phys_addr_t pa, size_t size, unsigned int flags)
{
    UNUSED(pa);
    UNUSED(size);
    UNUSED(flags);
    return va;
}

static bool rpmsgTableValid(void)
{
    const rpmsgResourceTable_t *rsc = RPMSG_RSC_TABLE;

    return rsc->header.ver == 1 && rsc->header.num == 1 && rsc->offset[0] == offsetof(rpmsgResourceTable_t, vdev)
        && rsc->vdev.type == RSC_VDEV && rsc->vdev.id == VIRTIO_ID_RPMSG;
}

// Status 0 makes Linux set the link up from the start when it attaches.
static void rpmsgResourceTableInit(void)
{
    rpmsgResourceTable_t *rsc = RPMSG_RSC_TABLE;

    memset(rsc, 0, sizeof(*rsc));
    rsc->header.ver = 1;
    rsc->header.num = 1;
    rsc->offset[0] = offsetof(rpmsgResourceTable_t, vdev);
    rsc->vdev.type = RSC_VDEV;
    rsc->vdev.id = VIRTIO_ID_RPMSG;
    rsc->vdev.notifyid = RPMSG_VDEV_NOTIFYID;
    rsc->vdev.dfeatures = 1u << VIRTIO_RPMSG_F_NS;
    rsc->vdev.num_of_vrings = 2;
    rsc->vring[0].da = (uint32_t)(uintptr_t)__rpmsg_vring0;
    rsc->vring[1].da = (uint32_t)(uintptr_t)__rpmsg_vring1;
    for (unsigned i = 0; i < 2; i++) {
        rsc->vring[i].align = RPMSG_VRING_ALIGN;
        rsc->vring[i].num = RPMSG_VRING_ENTRIES;
        rsc->vring[i].notifyid = i;
    }
    memset(__rpmsg_vring0, 0, vring_size(RPMSG_VRING_ENTRIES, RPMSG_VRING_ALIGN));
    memset(__rpmsg_vring1, 0, vring_size(RPMSG_VRING_ENTRIES, RPMSG_VRING_ALIGN));
}

static bool rpmsgDoorbellFree(void)
{
    return RPMSG_MU->TSR & (1u << RPMSG_MU_CHANNEL);
}

// An attached Linux takes a doorbell at once; the table and vrings are left alone until the answer is known.
static bool rpmsgTableSettled(void)
{
    if (rpmsgTableReady) {
        return true;
    }
    if (rpmsgDoorbellFree() && rpmsgTableValid()) {
        rpmsgTableReady = true;
        return true;
    }
    if (cmpTimeMs(millis(), rpmsgProbeStart) < RPMSG_PROBE_TIMEOUT_MS) {
        return false;
    }
    rpmsgResourceTableInit();
    rpmsgTableReady = true;
    return true;
}

void rpmsgInit(void)
{
    if (rpmsgStarted) {
        return;
    }
    rpmsgStarted = true;

    // A doorbell still pending at boot was not taken by Linux and serves as the probe.
    rpmsgProbeStart = millis();
    if (rpmsgDoorbellFree()) {
        RPMSG_MU->TR[RPMSG_MU_CHANNEL] = 0;
    }

    // Page shift -1 maps each region as one page; 31 would mask bit 31 off the DDR address.
    rpmsgShmPhys = (metal_phys_addr_t)(uintptr_t)__rpmsg_vring0;
    rpmsgRscPhys = (metal_phys_addr_t)(uintptr_t)__rpmsg_rsc_table;
    metal_io_init(&rpmsgShmIo, __rpmsg_vring0, &rpmsgShmPhys, __rpmsg_shm_end - __rpmsg_vring0, -1, 0, NULL);
    metal_io_init(&rpmsgRscIo, __rpmsg_rsc_table, &rpmsgRscPhys, sizeof(rpmsgResourceTable_t), -1, 0, NULL);

    RPMSG_MU->RCR |= 1u << RPMSG_MU_CHANNEL;
    NVIC_SetPriority(RPMSG_MU_IRQ, NVIC_PRIORITY_LEVEL(RPMSG_MU_PRIORITY));
    NVIC_EnableIRQ(RPMSG_MU_IRQ);
}

typedef uint32_t rpmsgWord_t __attribute__((aligned(1)));

// newlib-nano memcpy makes one uncached DDR access per byte; volatile keeps this word loop from becoming memcpy.
void rpmsgCopy(void *dst, const void *src, uint32_t len)
{
    uint8_t *d = dst;
    const uint8_t *s = src;

    for (; len >= sizeof(uint32_t); len -= sizeof(uint32_t)) {
        *(volatile rpmsgWord_t *)d = *(const volatile rpmsgWord_t *)s;
        d += sizeof(uint32_t);
        s += sizeof(uint32_t);
    }
    while (len--) {
        *d++ = *s++;
    }
}

bool rpmsgAddService(rpmsgServiceFn *service)
{
    for (unsigned i = 0; i < ARRAYLEN(rpmsgServices); i++) {
        if (rpmsgServices[i] == service) {
            return true;
        }
        if (!rpmsgServices[i]) {
            rpmsgServices[i] = service;
            return true;
        }
    }
    return false;
}

// Linux serves every vring on any doorbell, so a pending doorbell covers this one.
static int rpmsgNotify(void *priv, uint32_t id)
{
    UNUSED(priv);
    if (RPMSG_MU->TSR & (1u << RPMSG_MU_CHANNEL)) {
        // The vring writes must reach DDR before Linux takes the doorbell.
        __DSB();
        RPMSG_MU->TR[RPMSG_MU_CHANNEL] = id << RPMSG_MU_VQID_SHIFT;
    }
    return 0;
}

void MU7_B_IRQHandler(void)
{
    if (RPMSG_MU->RSR & (1u << RPMSG_MU_CHANNEL)) {
        (void)RPMSG_MU->RR[RPMSG_MU_CHANNEL];
    }
}

// A name service message with no free buffer fails at once; the service retries on the next poll.
static int rpmsgNoWait(struct rpmsg_device *rdev, uint32_t id)
{
    UNUSED(rdev);
    UNUSED(id);
    return RPMSG_ERR_NO_BUFF;
}

static bool isPowerOfTwo(uint32_t value)
{
    return value && !(value & (value - 1));
}

// Linux writes the table, so a vring must fit the shared memory and the buffer count this image was built for.
static bool rpmsgVringValid(const struct fw_rsc_vdev_vring *vring)
{
    const uint32_t shmSize = __rpmsg_shm_end - __rpmsg_vring0;
    const uint32_t offset = vring->da - (uint32_t)(uintptr_t)__rpmsg_vring0;

    if (!isPowerOfTwo(vring->num) || vring->num > RPMSG_VRING_ENTRIES || !isPowerOfTwo(vring->align)
        || vring->align < RPMSG_VRING_ALIGN_MIN || vring->align > RPMSG_VRING_ALIGN) {
        return false;
    }
    return offset < shmSize && (uint32_t)vring_size(vring->num, vring->align) <= shmSize - offset;
}

static struct rpmsg_device *rpmsgAttach(void)
{
    rpmsgResourceTable_t *rsc = RPMSG_RSC_TABLE;

    if (!(*(volatile uint8_t *)&rsc->vdev.status & VIRTIO_CONFIG_STATUS_DRIVER_OK)) {
        return NULL;
    }

    struct fw_rsc_vdev_vring vrings[ARRAYLEN(rsc->vring)];
    rpmsgCopy(vrings, rsc->vring, sizeof(vrings));
    if (rsc->vdev.num_of_vrings != ARRAYLEN(vrings) || !rpmsgVringValid(&vrings[0]) || !rpmsgVringValid(&vrings[1])) {
        return NULL;
    }

    struct virtio_device *vdev = rproc_virtio_create_vdev(VIRTIO_DEV_DEVICE, RPMSG_VDEV_NOTIFYID, &rsc->vdev,
                                                          &rpmsgRscIo, NULL, rpmsgNotify, NULL);
    if (!vdev) {
        return NULL;
    }

    for (unsigned i = 0; i < ARRAYLEN(vrings); i++) {
        const struct fw_rsc_vdev_vring *vring = &vrings[i];
        if (rproc_virtio_init_vring(vdev, i, vring->notifyid, metal_io_phys_to_virt(&rpmsgShmIo, vring->da),
                                    &rpmsgShmIo, vring->num, vring->align) != 0) {
            rproc_virtio_remove_vdev(vdev);
            return NULL;
        }
    }

    if (rpmsg_init_vdev(&rpmsgRvdev, vdev, NULL, &rpmsgShmIo, NULL) != 0) {
        rproc_virtio_remove_vdev(vdev);
        return NULL;
    }
    rpmsg_virtio_set_wait_cb(&rpmsgRvdev, rpmsgNoWait);

    // Take back buffers held but not returned before this image restarted.
    struct virtqueue *vqs[] = { rpmsgRvdev.rvq, rpmsgRvdev.svq };
    for (unsigned i = 0; i < ARRAYLEN(vqs); i++) {
        vqs[i]->vq_available_idx = vqs[i]->vq_ring.used->idx;
    }

    rpmsgVdev = vdev;
    return rpmsg_virtio_get_rpmsg_device(&rpmsgRvdev);
}

// Linux clears the status on detach or reattach; a vring set up anew in between shows more buffers than it has.
static bool rpmsgLinkLost(void)
{
    if (!(*(volatile uint8_t *)&RPMSG_RSC_TABLE->vdev.status & VIRTIO_CONFIG_STATUS_DRIVER_OK)) {
        return true;
    }
    struct virtqueue *vqs[] = { rpmsgRvdev.rvq, rpmsgRvdev.svq };
    for (unsigned i = 0; i < ARRAYLEN(vqs); i++) {
        const uint16_t availIdx = *(volatile uint16_t *)&vqs[i]->vq_ring.avail->idx;
        if ((uint16_t)(availIdx - vqs[i]->vq_available_idx) > vqs[i]->vq_nentries) {
            return true;
        }
    }
    return false;
}

// Drops the endpoints without writing to the vrings, which are Linux's now.
static void rpmsgDetach(void)
{
    rpmsgRvdev.rdev.support_ns = false;
    rpmsg_deinit_vdev(&rpmsgRvdev);
    rproc_virtio_remove_vdev(rpmsgVdev);
    rpmsgVdev = NULL;
    rpmsgDevice = NULL;
}

struct rpmsg_device *rpmsgPoll(void)
{
    if (!rpmsgStarted) {
        return NULL;
    }

    if (rpmsgDevice && rpmsgLinkLost()) {
        rpmsgDetach();
    } else if (rpmsgDevice) {
        rproc_virtio_notified(rpmsgVdev, RSC_NOTIFY_ID_ANY);
    } else if (rpmsgTableSettled()) {
        rpmsgDevice = rpmsgAttach();
    }

    for (unsigned i = 0; i < ARRAYLEN(rpmsgServices) && rpmsgServices[i]; i++) {
        rpmsgServices[i](rpmsgDevice);
    }

    return rpmsgDevice;
}

#endif // USE_RPMSG
