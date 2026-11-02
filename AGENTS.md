# Guidelines for AI agents and automated contributors

These rules apply to AI agents, LLM-assisted tools and the persons who use them. Read these rules before you change code or open a pull request. Also obey [CONTRIBUTING.md](CONTRIBUTING.md) and the [pull request template](.github/pull_request_template.md).

Betaflight is flight controller firmware. Errors can cause crashes, injuries and damage. Do not submit code that you cannot explain.

## 1. Before you start

- **Search first.** Search the open issues and pull requests for the same change. Do not open a duplicate.
- **Tell us about AI use.** If an AI tool wrote all or part of the change, say so in the pull request description.
- **One change per pull request.** Change only the files that the problem needs. Do not add refactoring, reformatting or unrelated fixes.
- **Use the correct repository and branch.** Open pull requests against `master`. Do not open them from the `master` branch of your fork. Board configurations go to [betaflight/config](https://github.com/betaflight/config).
- **Do not change submodules.** Do not commit changes to `src/config` or to the libraries in `lib/` unless the pull request is about them.

## 2. Use the existing infrastructure

Betaflight has a framework for most tasks. Use it. Do not write a second solution. The table gives examples only. It is not a full list. Before you write new code, find how a similar feature does it and do the same.

| Task | Use | Location |
|------|-----|----------|
| Store a setting | Parameter groups (`PG_REGISTER*`, `PG_DECLARE`) | `src/main/pg/`, IDs in `pg_ids.h` |
| Show a setting in the CLI | The value table and lookup tables | `src/main/cli/settings.c` |
| Send data to the app | MSP handlers | `src/main/msp/msp.c`, `msp_protocol.h` |
| Run code periodically | Scheduler tasks (`DEFINE_TASK`) | `src/main/fc/tasks.c` |
| Use a pin | IO tags and resource owners | `src/main/drivers/io.h`, `resource.h` |
| Use a timer or DMA | Timer and DMA allocation | `src/main/drivers/timer.h`, `dma_reqmap.h` |
| Talk to a sensor | Bus devices (SPI, I2C) | `src/main/drivers/bus.h`, `bus_spi.h`, `bus_i2c.h` |
| Filter a signal | Filter library | `src/main/common/filter.h` |
| Do maths | Maths and vector helpers | `src/main/common/maths.h`, `vector.h` |
| Add a flight mode or switch | Box IDs and modes | `src/main/msp/msp_box.c`, `src/main/fc/rc_modes.h` |
| Block arming | Arming disable flags | `src/main/fc/runtime_config.h` |
| Turn a feature on or off | Features | `src/main/config/feature.h` |
| Show debug data | Debug modes and `DEBUG_SET()`. Do not write to `debug[]` directly. | `src/main/build/debug.h`, `debug.c` |
| Measure time | `timeUs_t` and `cmpTimeUs()`. Do not subtract timestamps directly, because they wrap. | `src/main/common/time.h` |
| Limit a value | `constrain()`, `constrainf()`, `MIN()`, `MAX()` | `src/main/common/maths.h` |
| Common macros | `ARRAYLEN()`, `UNUSED()`, `STATIC_ASSERT()` | `src/main/common/utils.h` |
| Share data with an interrupt | `volatile` and `ATOMIC_BLOCK()` | `src/main/build/atomic.h` |
| Test a static function | `STATIC_UNIT_TESTED` | `src/main/build/build_config.h` |

## 3. Code rules

- **Obey the coding style.** See <https://betaflight.com/docs/development/CodingStyle>. Do not reformat lines that you do not change.
- **Keep it simple.** Do not add abstractions, libraries or dependencies that the change does not need.
- **Fix the root cause.** A bug fix must tell why the bug occurred. Do not add a guard that only hides the symptom.
- **Refactors do not change behaviour.** If a refactor or cleanup changes behaviour, say so in the pull request. Better, put the behaviour change in a different pull request.
- **No dead code.** Do not add commented-out code, `#if 0` blocks, unused variables or unused functions.
- **Check all external input.** Check the length of MSP payloads with `sbufBytesRemaining()` before you read them. Add new MSP fields at the end of the payload. Check the range of CLI arguments and of data from serial devices.
- **Target independence.** Do not use MCU or board names (for example `STM32F7`) in `src/main`. Use `USE_*` feature defines. Put MCU-specific code in `src/platform/`. `make checks` finds some of these errors.
- **Real time.** Do not use dynamic memory allocation. Do not block or wait in interrupt handlers or scheduler tasks.
- **Hot path.** Do not add work to the gyro or PID task without a measurement of the cycle cost. Use the `tasks` CLI command or a debug mode. Tell the result in the pull request.
- **Flash wear.** Do not write the configuration to flash on a frequent event, for example on each arm or disarm. If a feature must save automatically, do it only when the user enabled that feature. See `src/main/fc/stats.c` for an example.
- **Flash and RAM.** Many targets are near their flash limit. Put new features behind a `USE_*` define. Keep the size small.

### Build system

- **All warnings are errors.** The build uses `-Werror` and `-Wdouble-promotion`. Use `float` and the `f` suffix (`1.0f`). Use `sqrtf()` and `sinf()`, or the fast `sin_approx()` and `atan2_approx()`. Do not use `double`.
- **New source files.** Add each new `.c` file to `mk/source.mk`. Put code that runs in the gyro or PID loop in `SPEED_OPTIMISED_SRC`. Put other code in `SIZE_OPTIMISED_SRC`.
- **Feature dependencies.** Resolve dependencies between `USE_*` features in `src/main/target/common_post.h`, not in `common_pre.h`.
- **Memory placement.** Use `FAST_CODE` and `FAST_DATA` only for hot-path code and data. Use `FAST_DATA_ZERO_INIT` for data that starts at zero. `make checks` finds incorrect use.
- **Hardware values.** Do not guess register addresses, timings or pin functions. Get them from the datasheet or reference manual. Tell which document and section you used.
- **Flight behaviour.** Do not change default values, PID, mixer, failsafe or arming behaviour without a reason in the pull request.

### Compatibility

The CLI, MSP, blackbox logs and saved configurations must stay compatible. Obey these rules:

- **Parameter groups.** If you change the layout of a PG struct, increment the version in `PG_REGISTER*`. Do not reuse a PG ID.
- **Enumerations.** Add new values at the end, before `*_COUNT` or before the value that a comment marks as last. Do not reorder or remove values that the CLI, MSP or blackbox use. For example: `debugType_e`, OSD items and `armingDisableFlags_e`.
- **Box IDs.** Do not change or reuse a `permanentId` in `msp_box.c`.
- **MSP API version.** Increment `API_VERSION_MINOR` in `src/main/msp/msp_protocol.h` one time per release only. Compare with the last release tag:
  `git show <last release tag>:src/main/msp/msp_protocol.h | grep API_VERSION_MINOR`
  If `master` already has a higher value, do not change it again.
- **Other repositories.** If the change needs work in the app, blackbox log viewer or Lua scripts, tell this in the pull request and link the matching pull request. For example, a new MSP message, debug mode or OSD item.

## 4. Verification

Do these checks before you open a pull request. All checks must pass with no new warnings.

| Check | Command |
|-------|---------|
| Unit tests | `make test` |
| Build an MCU target | `make TARGET=<MCU>`, for example `make TARGET=STM32F405` |
| Build a board | `make CONFIG=<board>`, for example `make CONFIG=MATEKF411`. The board comes from the `src/config` submodule. Run `make configs` first if the submodule is empty. |
| Representative CI subset | `make preview` |
| Static checks | `make checks` |
| Simulator | `make TARGET=SITL` |

- **Build what you change.** Build at least one target for each MCU family that the change affects.
- **Add unit tests for logic.** Put tests in `src/test/unit/` and add them to `src/test/Makefile`. For a bug fix, add a test that fails without the fix and passes with it, where this is possible.
- **Hardware tests.** Unit tests cannot test drivers, sensors and timing. For these changes, tell which hardware you used and how you tested it.

### Blackbox logs and flight tests

If the change can have an effect on flight, a maintainer will ask for a Blackbox log from a real flight. The log must show that the change has no bad effect on flight. We do not merge the pull request without it.

- **Code that needs a flight log.** This includes, for example:
  - Gyro, accelerometer, barometer, magnetometer and GPS drivers.
  - Filters, PID, feedforward, anti-gravity, mixer and RC smoothing.
  - Motor protocols (DShot, bitbang, bidirectional DShot) and ESC telemetry.
  - Scheduler, task timing, DMA and interrupt changes.
  - Flight modes, failsafe, GPS rescue, position hold and autopilot.
  - Changes to default values for these.
- **Simulated flights are not flight tests.** A SITL log, a bench test or unit tests do not replace a real flight. You can attach them as extra data, but do not call them a flight test.
- **Log content.** Use a `debug_mode` that shows the changed behaviour, if one is available. In the comment, tell:
  - The commit or cloud build key that you flew.
  - The board, MCU, gyro, gyro and PID loop rates, motor protocol, `debug_mode` and blackbox rate.
  - What you did in the flight and what the log shows.
- **Attach the file.** GitHub does not accept `.bbl` or `.bfl` files. Rename the file to `.log` before you attach it.
- **If you cannot fly.** Say so in the pull request. Ask for testers. Do not mark the pull request as ready until a flight log is attached.

## 5. Writing

- **Use ASD-STE100 Simplified Technical English** in commit messages, pull request descriptions, code comments and replies to reviewers. Use short sentences, the active voice and one instruction in each sentence.
- **Commit message.** Use one commit with a short summary line in the imperative, for example `Fix gyro alignment on BMI270`. In the body, tell why you made the change.
- **Pull request description.** Include:
  1. A summary: why you made the change and how it works.
  2. The commands that you used to test it, their results and the hardware tests. Attach a Blackbox log if the change can have an effect on flight.
  3. The tools that you used, if you used AI.
  4. `Fixes #<number>` if the pull request fixes an issue.
- **Replies to reviewers.** Read the review comment and the code before you reply. Do not paste replies from an AI tool without checking them.

## 6. Pull requests that we close

We close pull requests without a full review if they:

- Change files outside the scope of the change.
- Duplicate existing code, or do not use the existing infrastructure in section 2.
- Break compatibility (section 3).
- Reformat files that the change does not need.
- Fail CI, or do not have tests for logic that can be tested.
- Have an author who cannot explain the change.
