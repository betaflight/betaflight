# Looptime / Control loop frequency

Looptime / Control loop frequency (how often PID controller is executed and data is passed to motors) in INAV depends on multiple settings.

More, with asynchronous gyro processing, not all tasks are executed in the same time. Depending on `async_mode` setting, there are following cases:

## `async_mode = NONE`

Gyroscope sampling and filtering, Accelerometer sampling and filtering, Attitude computation and PID control loop are executed as single tasks and processed one after another.

| **gyro_sync**     | **gyro_lpf**      | **Control Loop Rate** (actual looptime) [us] |
| ----              | ----              | -----                     |   
| OFF               | any               | `looptime`                |   
| ON                | != 256HZ          | 1000 * `gyro_sync_denom`  |
| ON                | = 256HZ           | 125 * `gyro_sync_denom`   |

## `async_mode = GYRO`

In this mode, gyro sampling and filtering is decoupled from PID main loop and executed separately. In this mode, `gyro_sync` is forced and is always **ON**

| **gyro_lpf**      | **Control Loop Rate** [us] | Gyro looptime [us] |
| ----              | -----                                        | ---- |   
| != 256HZ          | `looptime`                                   | 1000 * `gyro_sync_denom`  |
| = 256HZ           | `looptime`                                   | 125 * `gyro_sync_denom`   |

## `async_mode = ALL`

In this mode, Gyroscope sampling and filtering, Accelerometer sampling and filtering, Attitude computation and PID control loop are decoupled and run as separate tasks.

| **gyro_lpf**      | **Control Loop Rate** [us] | Gyro looptime [us] | Accelerometer looptime [us] | Attitude looptime [us] |
| ----              | -----                                        | ---- |  ---- |  ---- |
| != 256HZ          | `looptime`                                   | 1000 * `gyro_sync_denom`  | 1000000 / `acc_task_frequency` | 1000000 / `attitude_task_frequency` |
| = 256HZ           | `looptime`                                   | 125 * `gyro_sync_denom`   | 1000000 / `acc_task_frequency` | 1000000 / `attitude_task_frequency` |
