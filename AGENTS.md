# Guidelines for AI agents and automated contributors

These rules apply to AI agents, LLM-assisted tools and the persons who use them. Read these rules before you change code or open a pull request. Also obey [CONTRIBUTING.md](CONTRIBUTING.md) and the [pull request template](.github/pull_request_template.md).

Betaflight is flight controller firmware. Errors can cause crashes, injuries and damage.

The [Betaflight AI Contribution Policy](https://github.com/betaflight/.github/blob/main/AI_POLICY.md) applies to all repositories. You are the author of what you submit. Read every line, test it, and be able to explain it without help from a tool. We close pull requests that show no human judgement.

## 1. Before you start

- **Search first.** Search the open issues and pull requests for the same change. Do not open a duplicate.
- **One change per pull request.** Change only the files that the problem needs. Do not add refactoring, reformatting or unrelated fixes.
- **Use the correct repository and branch.** Open pull requests against `master`. Do not open them from the `master` branch of your fork. Board configurations go to [betaflight/config](https://github.com/betaflight/config).
- **Do not change submodules.** Do not commit changes to `src/config` or to the libraries in `lib/` unless the pull request needs them.
- **No new libraries in the tree.** Do not copy vendor or third-party libraries into the repository. A new library must be a hosted submodule, and a maintainer must agree first.
- **Release timing.** After the release candidate, we merge only bug fixes until the release. A feature pull request waits for the next release cycle.

## 2. Use the existing infrastructure

Betaflight has a framework for most tasks. Use it. Do not write a second solution. The table gives examples only. It is not a full list. Before you write new code, find how a similar feature does it and do the same. If an existing mechanism (for example a curve, a timer or the PID controller) almost does what you need, extend it. Do not add a parallel system.

| Task | Use | Location |
|------|-----|----------|
| Store a setting | Parameter groups (`PG_REGISTER*`, `PG_DECLARE`) | `src/main/pg/`, IDs in `pg_ids.h` |
| Show a setting in the CLI | The value table and lookup tables | `src/main/cli/settings.c` |
| Name a setting | `PARAM_NAME_*` defines. Do not use string literals. | `src/main/fc/parameter_names.h` |
| Make a curve | Piecewise linear interpolation | `src/main/common/pwl.h` |
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
| Show debug data | Debug modes and `DEBUG_SET()` with a `//!<` field annotation. Do not write to `debug[]` directly. | `src/main/build/debug.h`, `debug.c` |
| Measure time | `timeUs_t` and `cmpTimeUs()`. Do not subtract timestamps directly, because they wrap. | `src/main/common/time.h` |
| Limit a value | `constrain()`, `constrainf()`, `MIN()`, `MAX()` | `src/main/common/maths.h` |
| Common macros | `ARRAYLEN()`, `UNUSED()`, `STATIC_ASSERT()`, `FALLTHROUGH` | `src/main/common/utils.h` |
| Share data with an interrupt | `volatile` and `ATOMIC_BLOCK()` | `src/main/build/atomic.h` |
| Test a static function | `STATIC_UNIT_TESTED` | `src/main/build/build_config.h` |

## 3. Code rules

- **Obey the coding style.** See <https://betaflight.com/docs/development/CodingStyle>. Do not reformat lines that you do not change.
- **Keep it simple.** Do not add abstractions, libraries or dependencies that the change does not need.
- **Fix the root cause.** A bug fix must tell why the bug occurred. Do not add a guard that only hides the symptom.
- **Make sure that the bug can occur.** Before you report or fix a bug, make sure that the firmware can get into that state in real use. Tell how it gets there, for example with a configuration, a sequence of actions or bad data from a device. If the state cannot occur in real use, do not call the change a bug fix.
- **Refactors do not change behaviour.** If a refactor or cleanup changes behaviour, say so in the pull request. Better, put the behaviour change in a different pull request.
- **No duplicate code.** If the same code is in two or more places, put it in one function.
- **No dead code.** Do not add commented-out code, `#if 0` blocks, unused variables or unused functions.
- **Check all external input.** Check the length of MSP payloads with `sbufBytesRemaining()` before you read them. Add new MSP fields at the end of the payload. Check the range of CLI arguments and of data from serial devices.
- **One range for each setting.** The minimum and maximum in `settings.c` apply to all inputs. MSP and CMS must use the same limits. Use one shared function or constant, so the limits cannot be different.
- **Safe float maths.** Prevent division by zero, `NaN` and values out of range. Limit the input before `powf()`, `logf()`, `sqrtf()` or `pwlInterpolate()`. A `NaN` in a filter or controller state stays there.
- **Check failures.** Check the return values of init functions and register reads. If a sensor gives no value (for example `0` for "no reading"), handle that case. Do not use it as real data.
- **Gate all parts of a feature together.** Put the enum value, the CLI lookup entry, the settings, the MSP fields and the init code behind the same `ENABLE_*` flag. A user must not be able to select an option that is not compiled in. Make each `#endif` comment name the correct flag.
- **No MCU defines in `src/main`.** Do not use MCU, MCU family or board names in `src/main`, for example `#ifdef STM32F7`, `defined(STM32H7)` or `defined(AT32F435)`. All MCU-specific code is in `src/platform/<VENDOR>/`. If the behaviour must be different for an MCU, do this:
  - Define a capability flag in `src/platform/<VENDOR>/target/<MCU>/target.h`. New flags obey the *Feature flags* rules below.
  - Test that flag in `src/main`. For example, `USE_PID_DENOM_CHECK` is defined in the APM32F405 `target.h` and tested in `src/main/config/config.c`.
  - Put the MCU-specific implementation in `src/platform/<VENDOR>/`, behind a common function that `src/main` calls.
  - For the simulator, use `ENABLE_SIMULATOR`, not `SIMULATOR_BUILD`.
  - `make checks` finds only full target names (for example `STM32F405`). It does not find family names such as `STM32F7`. Check your change yourself.
- **Real time.** Do not use dynamic memory allocation. Do not block or wait in interrupt handlers or scheduler tasks.
- **Hot path.** Do not add work to the gyro or PID task without a measurement of the cycle cost. Use the `tasks` CLI command or a debug mode. Tell the result in the pull request.
- **Flash wear.** Do not write the configuration to flash on a frequent event, for example on each arm or disarm. If a feature must save automatically, do it only when the user enabled that feature. See `src/main/fc/stats.c` for an example.
- **Flash and RAM.** Many targets are near their flash limit. Put new features behind an `ENABLE_*` flag. Keep the size small.
- **Hardware values.** Do not guess register addresses, timings or pin functions. Get them from the datasheet or reference manual. Tell which document and section you used.
- **Flight behaviour.** Do not change default values, PID, mixer, failsafe or arming behaviour without a reason in the pull request.

### Build system

- **All warnings are errors.** The build uses `-Werror` and `-Wdouble-promotion`. Use `float` and the `f` suffix (`1.0f`). Use `sqrtf()` and `sinf()`, or the fast `sin_approx()` and `atan2_approx()`. Do not use `double`.
- **Unused parameters.** `-Wextra` makes an unused parameter an error. If a build flag removes all the code that uses a parameter, put `UNUSED(param);` at the start of the function, outside the `#if`.
- **New files.** Start each new `.c` and `.h` file with the licence header from `DEFAULT_LICENSE.md`. Do not use a different header. Each `.c` file in `src/main` must include `platform.h`. `make checks` finds a missing include.
- **Generated files.** Do not edit these by hand: `src/main/msp/msp_build_info.c` and `.h` (from `src/utils/make-build-info.py`) and `src/main/target/serial_post.h` (from `src/utils/gen-serial-j2.py`). Change the generator or its input, and generate them again.
- **New source files.** Add each new `.c` file to `mk/source.mk`. Put code that runs in the gyro or PID loop in `SPEED_OPTIMISED_SRC`. Put other code in `SIZE_OPTIMISED_SRC`.
- **Feature flags.** Betaflight changes from `USE_*` to `ENABLE_*` build guards. Obey these rules:
  - Do not add new `USE_*` flags. Add an `ENABLE_*` flag.
  - Set the default (`0` or `1`) in `src/main/target/common_post.h`. If an old `USE_*` name exists, set `ENABLE_*` to `1` when the `USE_*` name is defined and `ENABLE_*` is not already defined. Do not override an explicit `ENABLE_X=0`. The comment block in `common_post.h` shows the pattern: `#if !defined(ENABLE_X) && defined(USE_X)`.
  - A new `ENABLE_*` flag always has a value (`0` or `1`). Test it with `#if ENABLE_X`, not `#ifdef ENABLE_X`.
  - Some older `ENABLE_*` names have no value, for example `ENABLE_STDIO_PREINCLUDE` and `ENABLE_MULTICORE_INIT`. Keep `#ifdef` for these. `#if` on a name without a value does not compile.
  - If you change code that has a `USE_*` guard, you can change that guard to `ENABLE_*`. Do not change guards in code that you do not otherwise change.
  - Resolve dependencies between features in `common_post.h`, not in `common_pre.h`.
- **Memory placement.** Use `FAST_CODE` and `FAST_DATA` only for hot-path code and data. Use `FAST_DATA_ZERO_INIT` for data that starts at zero. `make checks` finds incorrect use.

### Compatibility

The CLI, MSP, blackbox logs and saved configurations must stay compatible. Obey these rules:

- **Parameter groups.** A version change in `PG_REGISTER*` resets the settings of all users in that PG to the defaults. `pgLoad()` keeps the stored values only if the version is the same. Obey these rules:
  - Increment the version only if a stored field moves, changes size or changes meaning.
  - Do not increment the version if you add fields or array items at the end. The new items get their reset values automatically.
  - Set new defaults in the reset template or the reset function.
  - Do not reuse a PG ID.
- **Enumerations.** Add new values at the end, before `*_COUNT` or before the value that a comment marks as last. Do not reorder or remove values that the CLI, MSP or blackbox use. For example: `debugType_e`, OSD items and `armingDisableFlags_e`.
- **Box IDs.** Do not change or reuse a `permanentId` in `msp_box.c`.
- **MSP API version.** Increment `API_VERSION_MINOR` in `src/main/msp/msp_protocol.h` one time per release only. Compare with the last release tag:
  `git show <last release tag>:src/main/msp/msp_protocol.h | grep API_VERSION_MINOR`
  If `master` already has a higher value, do not change it again.
- **Debug fields.** Each `DEBUG_SET()` call needs a `//!<` annotation that tells the label and unit of the field. The app and the blackbox log viewer read these annotations. The grammar is in `src/main/build/debug.h`. See [Debug Field Annotations](https://betaflight.com/docs/development/debugging/Debug-Field-Annotations).
- **Other repositories.** If the change needs work in the app, blackbox log viewer or Lua scripts, tell this in the pull request and link the matching pull request. For example, a new MSP message, debug mode or OSD item.

## 4. Verification

Do these checks before you open a pull request. All checks must pass with no new warnings.

| Check | Command |
|-------|---------|
| Unit tests | `make test`. CI runs `make EXTRA_FLAGS=-Werror test-all`, which also runs the tests for each target. |
| Build an MCU target | `make TARGET=<MCU>`, for example `make TARGET=STM32F405` |
| Build a board | `make CONFIG=<board>`, for example `make CONFIG=MATEKF411`. The board comes from the `src/config` submodule. Run `make configs` first if the submodule is empty. |
| Representative CI subset | `make preview`. It builds the targets in `PREVIEW_TARGETS` in the `Makefile` and runs `make test`. `make targets` shows the list. If you change a platform that is not in the list, build it with `make TARGET=<MCU>`. |
| Static checks | `make checks`. CI runs `make EXTRA_FLAGS=-Werror checks`. |
| Simulator | `make TARGET=SITL` |

- **Build what you change.** Build at least one target for each MCU family that the change affects.
- **Add unit tests for logic.** Put tests in `src/test/unit/` and add them to `src/test/Makefile`. For a bug fix, add a test that fails without the fix and passes with it, where this is possible.
- **Stubs in other tests.** Unit tests replace the functions that the code under test calls with stubs. If you add a call to tested code, other tests can fail to link because they do not have a stub for it. Run all unit tests, not only the test for your file.
- **Hardware tests.** Unit tests cannot test drivers, sensors and timing. For these changes, tell which hardware you used and how you tested it. Also tell which paths you did not test.

### Blackbox logs and flight tests

If the change can have an effect on flight, a maintainer will ask for a Blackbox log from a real flight. The log must show that the change has no bad effect on flight. We do not merge the pull request without it.

- **Code that needs a flight log.** This includes, for example:
  - Gyro, accelerometer, barometer, magnetometer and GPS drivers.
  - Filters, PID, feedforward, anti-gravity, mixer and RC smoothing.
  - Motor protocols (DShot, bitbang, bidirectional DShot) and ESC telemetry.
  - Scheduler, task timing, DMA and interrupt changes.
  - Flight modes, failsafe, GPS rescue, position hold and autopilot.
  - Changes to default values for these.
- **New features and new hardware.** Show that the feature improves flight, for example with logs from flights with and without it. For a new sensor or MCU, tell which hardware maintainers can use to test it.
- **Simulated flights are not flight tests.** A SITL log, a bench test or unit tests do not replace a real flight. You can attach them as extra data, but do not call them a flight test.
- **Log content.** Use a `debug_mode` that shows the changed behaviour, if one is available. In the comment, tell:
  - The commit or cloud build key that you flew.
  - The board, the MCU and the gyro model.
  - The gyro and PID loop rates, the motor protocol, the `debug_mode` and the blackbox rate.
  - What you did in the flight and what the log shows.
- **Attach the file.** GitHub does not accept `.bbl` or `.bfl` files. Rename the file to `.log` before you attach it.
- **If you cannot fly.** Say so in the pull request. Ask for testers. Do not mark the pull request as ready until a flight log is attached.

## 5. Writing

- **Use ASD-STE100 Simplified Technical English** in commit messages, pull request descriptions, code comments and replies to reviewers. Use short sentences, the active voice and one instruction in each sentence.
- **Pull request title and commits.** We squash each pull request into one commit when we merge it. The pull request title becomes the summary line of that commit. Write the title as a short summary in the imperative, for example `Fix gyro alignment on BMI270`. Give each commit a clear message that tells why you made the change.
- **Pull request description.** Include:
  1. A summary: why you made the change and how it works.
  2. The commands that you used to test it, their results and the hardware tests. Attach a Blackbox log if the change can have an effect on flight.
  3. `Fixes #<number>` if the pull request fixes an issue.
- **Replies to reviewers.** You can use a tool to write a draft of a reply. Before you send it, read the review comment, the code and the draft. Send the reply only if it is correct and you agree with it. You are responsible for the reply. You must be able to answer the next questions yourself, without the tool.
- **Keep the pull request up to date.** Resolve conflicts with `master`. You can merge `master` into your branch or rebase on `master`. A pull request needs two approvals before we merge it.

### Review findings

Review tools (for example CodeRabbit) and reviewers find problems. A finding is a proposal, not an instruction. Do these steps for each finding:

1. **Make sure that the finding is correct.** Find it in the code. Make sure that it can occur in real use.
2. **Give it a severity.** Use one of three levels:
   - *Actionable*: fix it.
   - *Nit*: fix it if the fix is small. If not, give a reason in one line.
   - *No value*: give the reason only.
3. **Security and data loss.** Do not give a security or data-loss finding the level *No value* until you examine it again.
4. **Make the smallest fix.** It is better to remove or narrow code than to add code.
5. **Stay in scope.** Fix a correct finding in the same code path now. For a problem in other code, write one line about it. Do not fix it in this pull request.
6. **Do not agree only to stop a discussion.** If you disagree, give evidence: a `file:line`, a test or the reference document.
7. **Give a reason for each finding that you do not fix.** Do not mark the thread of a review tool as resolved yourself.
8. **Let a person decide.** If a finding is still disputed after two replies, write both positions in two or three lines. A maintainer decides.
9. **Push one time.** Fix all the findings, then push one time. Each push starts a new review, so many small pushes make more review rounds.

Source: [discussion #15961](https://github.com/betaflight/betaflight/discussions/15961).

## 6. Pull requests that we close

We close pull requests without a full review if they:

- Change files outside the scope of the change.
- Duplicate existing code, or do not use the existing infrastructure in section 2.
- Break compatibility (section 3).
- Reformat files that the change does not need.
- Fail CI, or do not have tests for logic that can be tested.
- Have an author who cannot explain the change.
