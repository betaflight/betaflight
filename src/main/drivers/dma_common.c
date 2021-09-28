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

#ifdef USE_DMA

#include "drivers/dma_impl.h"

#include "dma.h"

dmaIdentifier_e dmaAllocate(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex)
{
    if (dmaGetOwner(identifier)->owner != OWNER_FREE) {
        return DMA_NONE;
    }

    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    dmaDescriptors[index].owner.owner = owner;
    dmaDescriptors[index].owner.resourceIndex = resourceIndex;

    return identifier;
}

const resourceOwner_t *dmaGetOwner(dmaIdentifier_e identifier)
{
    return &dmaDescriptors[DMA_IDENTIFIER_TO_INDEX(identifier)].owner;
}

dmaIdentifier_e dmaGetIdentifier(const dmaResource_t* channel)
{
    for (int i = 0; i < DMA_LAST_HANDLER; i++) {
        if (dmaDescriptors[i].ref == channel) {
            return i + 1;
        }
    }

    return 0;
}

dmaChannelDescriptor_t* dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier)
{
    return &dmaDescriptors[DMA_IDENTIFIER_TO_INDEX(identifier)];
}

uint32_t dmaGetChannel(const uint8_t channel)
{
    return ((uint32_t)channel*2)<<24;
}
#endif
