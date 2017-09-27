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
#include <stdbool.h>

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
#include "build/atomic.h"
}

TEST(AtomicUnittest, TestAtomicBlock)
{
    atomic_BASEPRI = 0;    // reset BASEPRI

    ATOMIC_BLOCK(10) {
        EXPECT_EQ(atomic_BASEPRI, 10);     // locked
        ATOMIC_BLOCK(5) {
            EXPECT_EQ(atomic_BASEPRI, 5);  // priority increase
        }
        EXPECT_EQ(atomic_BASEPRI, 10);     // restore priority
        ATOMIC_BLOCK(20) {
            EXPECT_EQ(atomic_BASEPRI, 10); // lower priority, no change to value
        }
        EXPECT_EQ(atomic_BASEPRI, 10);     // restore priority
    }
    EXPECT_EQ(atomic_BASEPRI, 0);          // restore priority to unlocked
}

TEST(AtomicUnittest, TestAtomicBlockNB)
{
    atomic_BASEPRI = 0;    // reset BASEPRI

    ATOMIC_BLOCK_NB(10) {
        EXPECT_EQ(atomic_BASEPRI, 10);     // locked
        ATOMIC_BLOCK_NB(5) {
            EXPECT_EQ(atomic_BASEPRI, 5);  // priority increase
        }
        EXPECT_EQ(atomic_BASEPRI, 10);     // restore priority
        ATOMIC_BLOCK_NB(20) {
            EXPECT_EQ(atomic_BASEPRI, 10); // lower priority, no change to value
        }
        EXPECT_EQ(atomic_BASEPRI, 10);     // restore priority
    }
    EXPECT_EQ(atomic_BASEPRI, 0);          // restore priority to unlocked
}

struct barrierTrace {
    int enter, leave;
};

extern "C" {
    int testAtomicBarrier_C(struct barrierTrace *b0, struct barrierTrace *b1, struct barrierTrace sample[][2]);
}

TEST(AtomicUnittest, TestAtomicBarrier)
{
    barrierTrace b0,b1,sample[10][2];

    int sampled = testAtomicBarrier_C(&b0,&b1,sample);
    int sIdx = 0;
    EXPECT_EQ(sample[sIdx][0].enter, 0);
    EXPECT_EQ(sample[sIdx][0].leave, 0);
    EXPECT_EQ(sample[sIdx][1].enter, 0);
    EXPECT_EQ(sample[sIdx][1].leave, 0);
    sIdx++;
    // do {
    //   ATOMIC_BARRIER(*b0);
    //   ATOMIC_BARRIER(*b1);
    EXPECT_EQ(sample[sIdx][0].enter, 1);
    EXPECT_EQ(sample[sIdx][0].leave, 0);
    EXPECT_EQ(sample[sIdx][1].enter, 1);
    EXPECT_EQ(sample[sIdx][1].leave, 0);
    sIdx++;
    //    do {
    //        ATOMIC_BARRIER(*b0);
    EXPECT_EQ(sample[sIdx][0].enter, 2);
    EXPECT_EQ(sample[sIdx][0].leave, 0);
    EXPECT_EQ(sample[sIdx][1].enter, 1);
    EXPECT_EQ(sample[sIdx][1].leave, 0);
    sIdx++;
    //    } while(0);
    EXPECT_EQ(sample[sIdx][0].enter, 2);
    EXPECT_EQ(sample[sIdx][0].leave, 1);
    EXPECT_EQ(sample[sIdx][1].enter, 1);
    EXPECT_EQ(sample[sIdx][1].leave, 0);
    sIdx++;
    //} while(0);
    EXPECT_EQ(sample[sIdx][0].enter, 2);
    EXPECT_EQ(sample[sIdx][0].leave, 2);
    EXPECT_EQ(sample[sIdx][1].enter, 1);
    EXPECT_EQ(sample[sIdx][1].leave, 1);
    sIdx++;
    //return sIdx;
    EXPECT_EQ(sIdx, sampled);
}
