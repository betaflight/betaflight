# ADRC: ревизия и план исправлений

Этот файл — основной рабочий tracker для ревизии и исправления ADRC в
Betaflight. Все последующие изменения ADRC должны быть привязаны к одному из
пунктов ниже либо сначала добавлены сюда отдельным пунктом.

## Базовая точка

- Дата ревизии: 2026-07-11.
- Основная ветка: `adrc-gate-fix`.
- Проверенный head: `a138a5dd19fe`.
- Официальный PR: [betaflight/betaflight#15400](https://github.com/betaflight/betaflight/pull/15400).
- Merge base на момент ревизии: `b6bf0f5ab8`.
- Состояние PR на момент ревизии: `OPEN`, `DRAFT`, `REVIEW_REQUIRED`.
- Итог ревизии: **REQUEST CHANGES**.

Последний commit `a138a5dd19fe` меняет только комментарии. Функционально
текущему head соответствует `0079d685a2`, на котором основан release
`adrc-pr15400-b2`.

## Как вести этот файл

Статусы:

- `TODO` — работа не начата.
- `IN PROGRESS` — пункт взят в работу.
- `BLOCKED` — продолжение требует отдельного решения или внешнего evidence.
- `DONE` — acceptance criteria выполнены и указан commit реализации.
- `DEFERRED` — осознанно отложено с записанной причиной.

Правила обновления:

1. Перед изменением кода поставить пункту статус `IN PROGRESS`.
2. Для каждого исправления сначала получить воспроизводящий тест или другой
   проверяемый критерий.
3. Исправление кода и его тесты фиксировать отдельным содержательным commit.
4. После commit обновить здесь статус, фактические проверки и поле
   `Implementation commit(s)`. Обновление tracker идёт следующим commit,
   поскольку commit не может надёжно содержать собственный SHA.
5. `DONE` ставить только после прохождения всех acceptance criteria пункта.
6. Если commit был rebased или squashed, до push заменить устаревший SHA.
7. Не смешивать в одном implementation commit независимые пункты без
   технической необходимости. Если смешивание неизбежно, один SHA указывается
   во всех затронутых пунктах.
8. После каждого изменения ADRC выполнять минимум:

   `make EXTRA_FLAGS=-Werror test_adrc_unittest test_pid_unittest`

9. Перед commit проверять `git diff --check`, `git status` и GitNexus
   `detect_changes`. Для изменяемых code symbols заранее выполнять GitNexus
   impact analysis.
10. Изменения не считать flight-validated, пока не указан точный firmware SHA,
    target, конфигурация и ссылка на лог.

## Карта веток и артефактов

| Линия | SHA на ревизии | Роль | Использование |
|---|---:|---|---|
| `repo/master` | `4028af2aab` | Legacy inline ADRC и старые flight logs | Только evidence/reference |
| `adrc-toggle-fixes` | `d45d6b0c9c` | Ранний модульный вариант | История fixes |
| `adrc-review-fixes-2` | `71bd38ff95` | Промежуточные CLI/safety fixes | История fixes |
| `adrc-gate-fix` | `a138a5dd19` | Точный head PR #15400 | Основная линия исправлений |
| `adrc-dterm-lpf` | `665bbb1dc5` | Экспериментальный D-term LPF | Не выпускать до ADRC-006 |
| `pr15400-builds` | `69053ba918` | Release workflow поверх `0079d685` | Источник release b2 |

Remotes в этом workspace:

- `origin` — `Boyyt357/ADRC-betaflight`, исходный PoC.
- `fork` — `danusha2345/ADRC-betaflight`, рабочий fork.
- `bvandevliet` — fork автора официального PR.
- `upstream` — `betaflight/betaflight`.

Важно: `repo/master` настроен отслеживать `origin/master`, хотя совпадает с
`fork/master`. Поэтому отображаемое `ahead 49` не означает 49 новых commits
относительно рабочего fork.

## Подтверждённое состояние до исправлений

### Unit tests и сборки

На чистых временных worktree с `EXTRA_FLAGS=-Werror`:

| Ветка | ADRC tests | PID tests | Результат |
|---|---:|---:|---|
| `adrc-toggle-fixes` | 20/20 | 17/17 | PASS |
| `adrc-review-fixes-2` | 28/28 | 17/17 | PASS |
| `adrc-gate-fix` | 29/29 | 17/17 | PASS |
| `adrc-dterm-lpf` | 33/33 | 17/17 | PASS |
| `pr15400-builds` | 29/29 | 17/17 | PASS |

Ветка `adrc-dterm-lpf` также собирает target `STM32F405`. Успешная сборка не
устраняет проблему persisted layout из ADRC-006.

### GitHub CI

- Текущий head PR имеет 53/53 зелёных checks.
- [Основной run 29113914811](https://github.com/betaflight/betaflight/actions/runs/29113914811)
  завершил 51/51 jobs, включая полный `test-all` и firmware matrix.
- `CodeRabbit / Review = success` не является независимым review: review был
  пропущен из-за draft-состояния.
- Формальных reviews и inline review threads нет.
- Ветки `adrc-dterm-lpf` и `adrc-toggle-fixes` не имеют собственных GitHub
  Actions runs.
- Release workflow не завершает shard ненулевым exit при единичной ошибке
  board build. Для b2 полнота отдельно подтверждена набором 623/623 assets.

### Flight evidence

- [Issue #1](https://github.com/danusha2345/ADRC-betaflight/issues/1)
  содержит основную историю полётов legacy inline ADRC. Эти логи нельзя
  напрямую считать валидацией текущего `adrc.c`.
- [Issue #2](https://github.com/danusha2345/ADRC-betaflight/issues/2)
  в основном содержит release/PR coordination и не добавляет независимых
  тестеров текущего head.
- PR-архитектура летала на pre-fix build `c1db438203`. Логи выявили false
  mid-air re-arm, Airmode limit cycle, слишком большое `b0` scaling и
  gate-open transient.
- Исправления `4ec9b282de`, `4b51833842` и `0079d685a2` после этих логов
  покрыты unit tests, но точный post-fix head ещё не перелётан.
- D-term LPF в полётном build был выставлен в `0`. Эффект включённого LPF
  flight-validated не был.
- Dedicated legacy fix-#8 A/B невалиден: обе стороны логов содержали один и
  тот же firmware revision с reverted gate fix.

### Артефакты

Локальный `ADRC_PR15400_DTERMLPF_MAMBAF722_I2C.hex`:

- SHA-256:
  `0c93140a5d0e21e14a19720974b27470178b232f71efb4bfab2bc5531e32365c`;
- embedded revision: `c1db43820`;
- persisted schema: PG13;
- старые defaults: re-arm `500 ms`, `b0 scale max=9`.

Этот hex устарел и не должен использоваться для проверки текущих fixes.

[Release b2](https://github.com/danusha2345/ADRC-betaflight/releases/tag/adrc-pr15400-b2)
основан на функциональном `0079d685a2` и предпочтительнее legacy/D-term
артефакта для контролируемых тестов. Он всё ещё содержит нерешённый
gate-open transition из ADRC-001.

## Сводный план

| ID | Приоритет | Кратко | Ветка | Статус | Implementation commit(s) |
|---|---|---|---|---|---|
| ADRC-001 | P0 | Bumpless liftoff gate open | `adrc-gate-fix` | IN PROGRESS | — |
| ADRC-002 | P0 | Crash detector без зависимости от classic D | `adrc-gate-fix` | IN PROGRESS | — |
| ADRC-003 | P0 | ADRC I/z3 во время crash recovery | `adrc-gate-fix` | IN PROGRESS | — |
| ADRC-004 | P0 | Устойчивая TD discretization | `adrc-gate-fix` | IN PROGRESS | — |
| ADRC-005 | P0 | Loop-rate safe ESO | `adrc-gate-fix` | IN PROGRESS | — |
| ADRC-006 | P0 | Persisted schema для D-term LPF | `adrc-dterm-lpf` | TODO | — |
| ADRC-007 | P1 | Фактически приложенный mixer feedback | `adrc-gate-fix` | IN PROGRESS | — |
| ADRC-008 | P1 | Post-override throttle для ADRC | `adrc-gate-fix` | IN PROGRESS | — |
| ADRC-009 | P1 | State limits и finite-value defenses | `adrc-gate-fix` | IN PROGRESS | — |
| ADRC-010 | P1 | Семантика CLASSIC↔ADRC handover | `adrc-gate-fix` | IN PROGRESS | — |
| ADRC-011 | P1 | Исправить vacuous gate tests | `adrc-gate-fix` | IN PROGRESS | — |
| ADRC-012 | P1 | End-to-end tests и F411 cycle budget | `adrc-gate-fix` | TODO | — |
| ADRC-013 | P1 | Rebase, CI, release b3 и точный re-flight | `adrc-gate-fix` | BLOCKED | Нужны ADRC-001…012 |

## Детальные пункты

### ADRC-001 — Bumpless liftoff gate open

- Приоритет: **P0**.
- Статус: `IN PROGRESS`.
- Implementation commit(s): —.
- Затронутые места:
  - `src/main/flight/adrc.c`: gate transition, `lastOutput` и ESO update;
  - `src/test/unit/adrc_unittest.cc`;
  - при необходимости `src/test/unit/pid_unittest.cc`.

Finding:

При `liftoff=false → true` код в том же loop подключает
`b0 * lastOutput` к состоянию ESO без отдельного transition/priming/blend.
На pre-fix `btfl_001-AIR` первый gate open сопровождался примерно 1,3 с
осцилляции, motor saturation до 89% и gyro около ±145 deg/s. Уменьшение
`b0 scale max` снижает возможную амплитуду, но не устраняет сам разрыв.

Acceptance criteria:

- [ ] Выбран и кратко документирован invariant перехода `closed → open`.
- [ ] Добавлен regression test, который воспроизводит старый discontinuity на
      baseline `a138a5dd19` и проходит после исправления.
- [ ] На первом открытом loop нет скачка только из-за подключения `b0u`.
- [ ] `z1/z2/z3`, gyro filter и `lastOutput` переходят в согласованное
      состояние либо `b0u` вводится ограниченным blend.
- [ ] Повторное открытие при opt-in re-arm покрыто тем же правилом.
- [ ] ADRC/PID unit tests проходят с `-Werror`.
- [ ] После firmware build выполнен контролируемый Airmode takeoff test на
      точном SHA; ссылка на лог записана ниже.

Flight evidence после исправления: —.

### ADRC-002 — Crash detector без зависимости от classic D

- Приоритет: **P0**.
- Статус: `IN PROGRESS`.
- Implementation commit(s): —.
- Затронутые места:
  - `src/main/flight/pid.c`;
  - `src/test/unit/pid_unittest.cc`.

Finding:

`detectAndSetCrashRecovery()` вызывается только внутри `if (Kd > 0)`.
В режиме ADRC classic D затем перезаписывается, но при `d_roll=0` или
`d_pitch=0` вместе с ним исчезает crash detection, включая путь
`GPS_RESCUE_MODE`.

Acceptance criteria:

- [ ] При `pid_type=ADRC` crash detection не зависит от значения classic
      `Kd`.
- [ ] Тест ADRC + `D=0` подтверждает вход в crash recovery.
- [ ] Покрыт GPS Rescue crash-detection path.
- [ ] Поведение classic PID не изменилось.
- [ ] ADRC/PID unit tests проходят с `-Werror`.

### ADRC-003 — ADRC I/z3 во время crash recovery

- Приоритет: **P0**.
- Статус: `IN PROGRESS`.
- Implementation commit(s): —.
- Затронутые места:
  - `src/main/flight/pid.c`;
  - `src/main/flight/adrc.c` и `adrc.h` при необходимости;
  - `src/test/unit/pid_unittest.cc`.

Finding:

Crash recovery обнуляет `pidData.I`, но позднее в том же loop
`applyAdrcControl()` безусловно возвращает `I=-z3/b0`. При насыщенном `z3`
это может вернуть до полного `pidSumLimit`, который recovery пыталась убрать.

Acceptance criteria:

- [ ] Определена политика ADRC state на входе, во время и при выходе из crash
      recovery.
- [ ] Пока recovery требует нулевой I, ADRC не восстанавливает его позднее в
      том же loop.
- [ ] Нет скачка I при выходе из recovery.
- [ ] Unit test начинает с ненулевого/насыщенного `z3` и проверяет полный
      порядок вызовов одного PID loop.
- [ ] Покрыты normal crash recovery и GPS Rescue.

### ADRC-004 — Устойчивая TD discretization и reset semantics

- Приоритет: **P0** при включённом `adrc_td_hz`.
- Статус: `IN PROGRESS`.
- Implementation commit(s): —.
- Затронутые места:
  - `src/main/flight/adrc.c`;
  - `src/main/cli/settings.c`;
  - `src/test/unit/adrc_unittest.cc`;
  - reset callers в `pid.c`/`mixer.c`.

Finding:

TD использует forward Euler:

`vRef += dT * 2πf * (setpoint - vRef)`.

Разрешённая комбинация PID loop `1.6 kHz` и `td=1000 Hz` имеет pole около
`-2.927` и доходит до `Inf` примерно за 48 ms. Кроме того,
`adrcResetState()` ставит `vRef=0`; повторный reset при 3D reversal не даёт
reference tracker развиваться и способен создавать saturated opposite output.

Acceptance criteria:

- [ ] TD использует устойчивую для любого положительного `dT` discretization
      либо cutoff жёстко ограничен относительно loop rate.
- [ ] Sweep всех разрешённых loop-rate/cutoff комбинаций остаётся finite.
- [ ] Reset seed’ит `vRef` из физически согласованного текущего значения.
- [ ] Повторные resets 3D reversal не создают opposite saturated command.
- [ ] Добавлена защита/recovery от non-finite state.
- [ ] `adrc_td_hz=0` остаётся точным bypass.

### ADRC-005 — Loop-rate safe ESO

- Приоритет: **P0** для разрешённых экстремальных конфигураций.
- Статус: `IN PROGRESS`.
- Implementation commit(s): —.
- Затронутые места:
  - `src/main/flight/adrc.c`;
  - `src/main/cli/settings.c`;
  - `src/test/unit/adrc_unittest.cc`.

Finding:

ESO использует forward Euler, `wo` разрешён до `600`. Для BMI270
`3.2 kHz` и `pid_process_denom=16` PID loop равен `200 Hz`. При `wo=600`
observer имеет spectral radius больше `2` и чередуется между state clamps.

Acceptance criteria:

- [ ] Для каждого runtime `dT` существует проверяемая граница устойчивости.
- [ ] Невозможна настройка `wo`, которая отправляет observer в rail-to-rail
      limit cycle в пределах поддерживаемых loop rates.
- [ ] Unit sweep покрывает минимум 200 Hz, 1.6 kHz, 4 kHz и 8 kHz.
- [ ] Проверены gated и airborne observer paths.
- [ ] Default tune не изменён без отдельного обоснования.

### ADRC-006 — Persisted schema для D-term LPF

- Приоритет: **P0**, блокирует использование `adrc-dterm-lpf`.
- Статус: `TODO`.
- Implementation commit(s): —.
- Ветка: `adrc-dterm-lpf` после переноса остальных обязательных fixes.
- Затронутые места:
  - `src/main/flight/pid.h`;
  - `src/main/pg/pg_ids.h`/регистрация PID profile;
  - PG load/migration tests;
  - CLI и ADRC unit tests.

Finding:

`uint16_t dtermFilterHz` добавлен в середину persisted `adrcProfile_t`, но
`PG_PID_PROFILE` остаётся version 14. Размер `pidProfile_t` меняется
`266 → 268`, а массива профилей `1064 → 1072`. Loader при совпавшей version
raw-copy’ит старую запись, тихо смещая поля и границы профилей.

Динамическая проверка старой PG14 записи на новой структуре получила, среди
прочего, `dterm=5160`, `b0ThrottleScaleMax=100` и
`motorOutputLimit=0` при `LOAD_OK=1`.

Acceptance criteria:

- [ ] Старые PG14 данные не могут быть тихо интерпретированы как новый layout.
- [ ] Есть автоматический test старого serialized PG14 blob.
- [ ] Явно выбрана стратегия: reset, migration или отдельный ADRC PG.
- [ ] Учтено, что PG version занимает 4 бита, а version 15 — последнее
      доступное значение.
- [ ] CLI save/reboot/load сохраняет новый cutoff без изменения соседних
      полей и последующих профилей.
- [ ] Новый отдельно маркированный hex собран только после прохождения теста.
- [ ] Старый `c1db43820` hex не распространяется как актуальный.

Предпочтительное направление: отдельный ADRC parameter group вместо
дальнейшего расходования version основного PID profile.

### ADRC-007 — Фактически приложенный mixer feedback

- Приоритет: **P1**.
- Статус: `IN PROGRESS`.
- Implementation commit(s): —.
- Затронутые места:
  - `src/main/flight/pid.c`;
  - `src/main/flight/mixer.c`;
  - ADRC/PID/mixer tests.

Finding:

`lastOutput` сохраняется после per-axis clamp, но до mixer normalization.
При `motorMixRange>1`, linear/legacy normalization и low-throttle attenuation
реальный actuator input меньше сохранённого. ESO принимает эту разницу за
disturbance и накапливает её в `z3`.

Acceptance criteria:

- [ ] Observer получает фактически приложенный axis command либо доказанно
      эквивалентный scale.
- [ ] Покрыты legacy mixer, linear mixer и no-Airmode attenuation.
- [ ] Saturation test не вызывает ложный устойчивый drift `z3`.
- [ ] Не добавлена циклическая зависимость PID↔mixer или лишняя задержка без
      документированного анализа.

### ADRC-008 — Post-override throttle для ADRC

- Приоритет: **P1**.
- Статус: `IN PROGRESS`.
- Implementation commit(s): —.
- Затронутые места:
  - `src/main/flight/adrc.c`;
  - `src/main/flight/mixer.c`;
  - tests автоматических режимов.

Finding:

`adrcUpdatePerLoopState()` читает `mixerGetThrottle()` до
`ALT_HOLD`/`GPS_RESCUE` throttle overrides. Gate и `b0` scheduling поэтому
моделируют pilot/pre-override throttle, а не реально приложенную тягу.

Acceptance criteria:

- [ ] ADRC использует определённое post-override throttle значение.
- [ ] Normal/manual mode сохраняет прежнюю семантику.
- [ ] ALT_HOLD и GPS_RESCUE покрыты тестами.
- [ ] Порядок вычислений явно задокументирован рядом с API.

### ADRC-009 — State limits и finite-value defenses

- Приоритет: **P1**.
- Статус: `IN PROGRESS`.
- Implementation commit(s): —.
- Затронутые места:
  - `src/main/flight/adrc.c`;
  - profile validation/init;
  - `src/test/unit/adrc_unittest.cc`.

Findings:

- `ADRC_Z1_LIMIT=2000` почти равен максимальному setpoint и ниже
  поддерживаемого gyro range ±4000 dps.
- Corrupt/internal `b0ThrottleScaleMax=0` может дать `b0=0` и `0/0 → NaN`,
  несмотря на CLI minimum.
- Комментарий о одинаковых units `z1/z2/z3` неверен: это соответственно
  deg/s, deg/s² и deg/s³ для выбранной модели.

Acceptance criteria:

- [ ] State bounds согласованы с реальным gyro FSR и command range.
- [ ] Invalid persisted/internal parameters санитизируются до деления.
- [ ] P/I/D/Sum и observer state не покидают finite domain.
- [ ] Тесты покрывают high-FSR gyro, zero/invalid `b0` и corrupt scale max.
- [ ] Units в комментариях исправлены без изменения runtime semantics.

### ADRC-010 — Семантика CLASSIC↔ADRC handover

- Приоритет: **P1**, latent для stock runtime.
- Статус: `IN PROGRESS`.
- Implementation commit(s): —.
- Затронутые места:
  - `src/main/flight/pid_init.c`;
  - `src/main/flight/adrc.c`;
  - `src/test/unit/pid_unittest.cc`;
  - публичная документация/PR wording.

Finding:

Старый PT2 zero-state kick уже исправлен: обе ступени filter seed’ятся и
`z1` согласован с gyro. Однако CLASSIC→ADRC transition обнуляет `z3`,
`lastOutput` и `vRef` и не переводит gate в airborne state. При спокойном
hover ниже liftoff threshold gate может оставаться закрытым неограниченно.

В stock Betaflight stick/MSP profile switch заблокирован при `ARMED`, поэтому
это не обычный полётный путь. Текущий тест и wording тем не менее создают
ожидание mid-air bumpless handover.

Acceptance criteria:

- [ ] Принято явное решение: поддерживать runtime handover или удалить такое
      обещание из tests/docs.
- [ ] Если handover поддерживается, тест проверяет gate, первый полный control
      loop, I-equivalent state, `lastOutput` и TD reference, а не только `z1`.
- [ ] Если handover не поддерживается, API не создаёт ложной гарантии.
- [ ] CLASSIC path после ADRC не получает скрытый ADRC I без принятой политики.

### ADRC-011 — Исправить vacuous gate tests

- Приоритет: **P1**, выполнять вместе с первым gate-related fix.
- Статус: `IN PROGRESS`.
- Implementation commit(s): —.
- Затронуто: `src/test/unit/adrc_unittest.cc`.

Finding:

`GateDoesNotChatterWithInvertedThresholds` явно ставит
`liftoffIdleHoldMs=0`, а `GateReArmStillnessIsCappedIndependently` наследует
default `0`. Это полностью выключает re-arm path, который тесты должны
проверять.

Acceptance criteria:

- [ ] Оба теста используют ненулевой hold и реально входят в re-arm branch.
- [ ] Mutation/revert проверка показывает, что регрессия cross-clamp или
      stillness cap ломает соответствующий тест.
- [ ] Отдельно сохранён test, подтверждающий, что hold `0` отключает re-arm.

### ADRC-012 — End-to-end tests и F411 cycle budget

- Приоритет: **P1**.
- Статус: `TODO`.
- Implementation commit(s): —.

Обязательное покрытие:

- [ ] Полный `pidController` ADRC path, а не только isolated `adrc.c`.
- [ ] Первый gate open и opt-in re-arm.
- [ ] Crash recovery и GPS Rescue.
- [ ] Mixer normalization/saturation feedback.
- [ ] ALT_HOLD/GPS_RESCUE throttle.
- [ ] TD/ESO sweep по loop rates и граничным настройкам.
- [ ] High-FSR gyro и non-finite recovery.
- [ ] PG migration/reset для каждого нового persisted layout.
- [ ] Cycle benchmark на F411 при 8 kHz с ADRC enabled/disabled.
- [ ] Зафиксирован запас до deadline, stack/RAM/flash delta.

### ADRC-013 — Rebase, CI, release b3 и точный re-flight

- Приоритет: **P1**, финальный integration gate.
- Статус: `BLOCKED` до завершения ADRC-001…012.
- Implementation/release commit(s): —.

Acceptance criteria:

- [ ] Ветка rebased на актуальный `upstream/master`.
- [ ] `src/config` обновлён до совместимой актуальной revision.
- [ ] `git diff --check` чист.
- [ ] Полный `make EXTRA_FLAGS=-Werror test-all` проходит.
- [ ] Официальная firmware matrix проходит.
- [ ] Release workflow либо падает при любом board failure, либо полнота
      artifacts проверяется отдельным обязательным job.
- [ ] Создан release b3 для точного reviewed SHA.
- [ ] Проверены Airmode first takeoff, zero-throttle catch, throttle punches,
      sustained saturation, crash recovery/GPS Rescue и несколько loop rates.
- [ ] Для каждого лога записаны SHA, target, diff конфигурации и verdict.
- [ ] Только после этого PR снимается с draft.

## Порядок выполнения

Рекомендуемая последовательность:

1. ADRC-001 и ADRC-011 — gate transition и реальные regression tests.
2. ADRC-002 и ADRC-003 — crash recovery.
3. ADRC-004 и ADRC-005 — TD/ESO stability.
4. ADRC-009 — finite/state defenses.
5. ADRC-007 и ADRC-008 — фактический actuator/throttle feedback.
6. ADRC-010 — зафиксировать поддерживаемую handover semantics.
7. ADRC-012 — end-to-end coverage и cycle budget.
8. ADRC-006 — перенос fixes в D-term branch и безопасная PG schema.
9. ADRC-013 — rebase, полный CI, b3 и flight validation.

ADRC-006 ведётся отдельно от основного PR, пока D-term LPF не принят в scope
PR #15400.

## Журнал завершённых работ

Добавлять строку после каждого implementation commit.

| Дата | ID | Статус | Implementation commit | Проверки | Примечание |
|---|---|---|---|---|---|
| — | — | — | — | — | — |

## Открытые решения

Эти вопросы нельзя закрывать молча внутри реализации:

1. Gate open: state transform/priming или ограниченный blend `b0u`.
2. Crash recovery: reset, freeze или controlled decay `z3`.
3. ESO: ограничение `wo*dT` или другая discretization.
4. D-term LPF config: PG15 reset/migration или отдельный ADRC PG.
5. Observer feedback: effective mixer scale в текущем loop или delayed applied
   command предыдущего loop.
6. Runtime CLASSIC↔ADRC handover: официально поддерживаем или явно запрещаем.

Принятое решение нужно записывать непосредственно в соответствующий пункт до
или вместе с implementation commit.
