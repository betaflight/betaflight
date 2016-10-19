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
#include <stdint.h>
#include <stdlib.h>

#include <limits.h>

extern "C" {
#include <platform.h>

#include "build/build_config.h"

#include "common/utils.h"

    
#include "drivers/nvic.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(NVIC, priorityMacros) {
    struct {
        int grouping;
        int base, sub;
        int expect_prio, expect_base, expect_sub;
    } test[] = {
        // grouping            base   sub  ->  prio  base  sub
        {NVIC_PriorityGroup_2,    0,    0,     0x00,    0,   0},
        {NVIC_PriorityGroup_2,    3,    0,     0xc0,    3,   0},  // base is not truncated
        {NVIC_PriorityGroup_2,    0,    3,     0x30,    0,   3},  // sub is not truncated
        {NVIC_PriorityGroup_2, 0xff,    0,     0xc0,    3,   0},  // 2 bits for base, truncated
        {NVIC_PriorityGroup_2,    0, 0xff,     0x30,    0,   3},  // 2 bits for sub, truncated
        {NVIC_PriorityGroup_0, 0xff,    0,     0x00,    0,   0},  // 0 bits for base
        {NVIC_PriorityGroup_0,    0, 0xff,     0xf0,    0, 0xf},  // 4 bits for sub, truncated
        {NVIC_PriorityGroup_4, 0xff,    0,     0xf0,  0xf,   0},  // 4 bits for base, truncated
        {NVIC_PriorityGroup_4,    0, 0xff,     0x00,    0,   0},  // 0 bits for sub
    };

    // replace macro with local variable
    #undef NVIC_PRIORITY_GROUPING
    int NVIC_PRIORITY_GROUPING;

    for(unsigned i = 0; i < ARRAYLEN(test); i++) {
        NVIC_PRIORITY_GROUPING = test[i].grouping;
        int prio = NVIC_BUILD_PRIORITY(test[i].base, test[i].sub);
        EXPECT_EQ(test[i].expect_prio, prio);
        EXPECT_EQ(test[i].expect_base, NVIC_PRIORITY_BASE(prio));
        EXPECT_EQ(test[i].expect_sub, NVIC_PRIORITY_SUB(prio));
    }
}
