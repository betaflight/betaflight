# ADRC: ревизия и план исправлений

Этот файл — основной рабочий tracker для ревизии и исправления ADRC в
Betaflight. Все последующие изменения ADRC должны быть привязаны к одному из
пунктов ниже либо сначала добавлены сюда отдельным пунктом.

## Базовая точка

- Дата ревизии: 2026-07-11.
- Основная ветка: `adrc-gate-fix`.
- Проверенный head исходной ревизии: `a138a5dd19fe`.
- Локальный implementation/test head основной линии: `2a8ed176cf2c`.
- Локальный implementation/test head D-term линии: `2ddd4c4222`.
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
- `IMPLEMENTED` — код и доступные локальные проверки готовы, но остался
  внешний acceptance criterion: полёт, CI/matrix, release либо измерение на
  реальном контроллере.
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

| Линия | SHA | Роль | Использование |
|---|---:|---|---|
| `repo/master` | `4028af2aab` | Legacy inline ADRC и старые flight logs | Только evidence/reference |
| `adrc-toggle-fixes` | `d45d6b0c9c` | Ранний модульный вариант | История fixes |
| `adrc-review-fixes-2` | `71bd38ff95` | Промежуточные CLI/safety fixes | История fixes |
| `adrc-gate-fix` | `2a8ed176cf` | Локальная основная линия исправлений и mixer E2E tests поверх head PR #15400 | Не push; требует ADRC-012/013 |
| `codex/adrc-dterm-remediation` | `2ddd4c4222` | Исправленный экспериментальный D-term LPF + PG15 + tests | Не push; перед прошивкой сохранить `diff all` |
| `adrc-dterm-lpf` | `665bbb1dc5` | Исходный небезопасный экспериментальный D-term LPF | Только reference, не прошивать |
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

## Локальное состояние после исправлений

Основная линия `adrc-gate-fix`:

- `7d69539a63` — feedback фактически доступной mixer authority и
  post-override throttle;
- `5d862e6a73` — bumpless gate-open/re-open и невакуумные gate tests;
- `11fd579d92` — disarmed-only semantics перехода CLASSIC↔ADRC;
- `73c5bc34a2` — независимые от classic D crash detection и подавление
  `I/z3` на полном recovery loop;
- `538931a201` — стабильный TD, loop-rate cap ESO, физические state limits и
  finite recovery, включая `-ffast-math`;
- `6ea7c11aac` — feedback фактического yaw limit во время yaw-spin recovery;
- `2a8ed176cf` — восемь real-`mixTable()` regression tests для mixer feedback
  и автоматических throttle override.

Отдельная D-term линия `codex/adrc-dterm-remediation`:

- `ed747bbb0c` — opt-in `adrc_dterm_lpf_hz`, debug и blackbox metadata;
- `a0d8dbc8d0` — PG14 rejection, PG15 round-trip и перенос нового поля в
  конец `adrcProfile_t`;
- `2ca9126ac2` — синхронизация yaw-spin fix с основной линией;
- `6447e513fe` — runtime cutoff transitions, filter corruption recovery и
  multi-axis debug tests;
- `2ddd4c4222` — тот же real-`mixTable()` regression harness, что в main.

Ни одна ветка не отправлена в remote. Код собран и протестирован локально;
полётной валидации этих SHA пока нет.

Финальная локальная матрица:

| Линия | Mixer | ADRC | PID | `-ffast-math` | `test-all` | F405 | F411 |
|---|---:|---:|---:|---|---|---|---|
| main `2a8ed176cf` | 8/8 | 39/39 | 24/24 | PASS | PASS | PASS | PASS |
| D-term `2ddd4c4222` | 8/8 | 46/46 | 26/26 | PASS | PASS | PASS | PASS |

Оба `test-all` запускались после `make test_clean` с `EXTRA_FLAGS=-Werror`.
Целевые Mixer/ADRC/PID suites дополнительно прошли с
`EXTRA_FLAGS='-Werror -ffast-math'`.

Финальный GitNexus compare с `a138a5dd19` видит 12 файлов/139 symbols и 8
затронутых realtime flows (`pidController`/`mixTable`), формальный risk
`HIGH`. Это ожидаемый blast radius для изменений control/mixer hot path;
поэтому дополнительно выполнен независимый source-order review. Он нашёл
только yaw-spin limit gap, исправленный в `6ea7c11aac`; повторный review
default path и compile guards новых blocker не нашёл.

## Сводный план

| ID | Приоритет | Кратко | Ветка | Статус | Implementation commit(s) |
|---|---|---|---|---|---|
| ADRC-001 | P0 | Bumpless liftoff gate open | `adrc-gate-fix` | IMPLEMENTED | `5d862e6a73` |
| ADRC-002 | P0 | Crash detector без зависимости от classic D | `adrc-gate-fix` | DONE | `73c5bc34a2` |
| ADRC-003 | P0 | ADRC I/z3 во время crash recovery | `adrc-gate-fix` | DONE | `73c5bc34a2` |
| ADRC-004 | P0 | Устойчивая TD discretization | `adrc-gate-fix` | DONE | `538931a201` |
| ADRC-005 | P0 | Loop-rate safe ESO | `adrc-gate-fix` | DONE | `538931a201` |
| ADRC-006 | P0 | Persisted schema для D-term LPF | `codex/adrc-dterm-remediation` | IMPLEMENTED | `ed747bbb0c`, `a0d8dbc8d0`, `6447e513fe` |
| ADRC-007 | P1 | Фактически приложенный mixer feedback | `adrc-gate-fix` | IMPLEMENTED | `7d69539a63`, `6ea7c11aac`, `2a8ed176cf` |
| ADRC-008 | P1 | Post-override throttle для ADRC | `adrc-gate-fix` | DONE | `7d69539a63`, `2a8ed176cf` |
| ADRC-009 | P1 | State limits и finite-value defenses | `adrc-gate-fix` | DONE | `538931a201` |
| ADRC-010 | P1 | Семантика CLASSIC↔ADRC handover | `adrc-gate-fix` | DONE | `11fd579d92` |
| ADRC-011 | P1 | Исправить vacuous gate tests | `adrc-gate-fix` | DONE | `5d862e6a73` |
| ADRC-012 | P1 | End-to-end tests и F411 cycle budget | `adrc-gate-fix` | BLOCKED | F411 timing, dynamic/closed-loop E2E |
| ADRC-013 | P1 | Rebase, CI, release b3 и точный re-flight | `adrc-gate-fix` | BLOCKED | Нужны ADRC-001…012 |

## Детальные пункты

### ADRC-001 — Bumpless liftoff gate open

- Приоритет: **P0**.
- Статус: `IMPLEMENTED` — локально закрыто, flight criterion остаётся.
- Implementation commit(s): `5d862e6a73`.
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

- [x] Выбран и кратко документирован invariant перехода `closed → open`.
- [x] Добавлен regression test, который воспроизводит старый discontinuity на
      baseline `a138a5dd19` и проходит после исправления.
- [x] На первом открытом loop нет скачка только из-за подключения `b0u`.
- [x] `z1/z2/z3`, gyro filter и `lastOutput` переходят в согласованное
      состояние либо `b0u` вводится ограниченным blend.
- [x] Повторное открытие при opt-in re-arm покрыто тем же правилом.
- [x] ADRC/PID unit tests проходят с `-Werror`.
- [ ] После firmware build выполнен контролируемый Airmode takeoff test на
      точном SHA; ссылка на лог записана ниже.

Flight evidence после исправления: —.

Принятое правило: при `closed → open` сохраняются `z1/z2/z3`, TD и gyro
filter, но `lastOutput` предыдущей grounded-эпохи обнуляется. Первый открытый
loop поэтому не подключает к ESO команду, которую зажатая землёй модель не
могла реализовать; mixer публикует уже новую airborne-команду для следующего
loop. Mutation/revert старого поведения ломает новые gate tests.

### ADRC-002 — Crash detector без зависимости от classic D

- Приоритет: **P0**.
- Статус: `DONE`.
- Implementation commit(s): `73c5bc34a2`.
- Затронутые места:
  - `src/main/flight/pid.c`;
  - `src/test/unit/pid_unittest.cc`.

Finding:

`detectAndSetCrashRecovery()` вызывается только внутри `if (Kd > 0)`.
В режиме ADRC classic D затем перезаписывается, но при `d_roll=0` или
`d_pitch=0` вместе с ним исчезает crash detection, включая путь
`GPS_RESCUE_MODE`.

Acceptance criteria:

- [x] При `pid_type=ADRC` crash detection не зависит от значения classic
      `Kd`.
- [x] Тест ADRC + `D=0` подтверждает вход в crash recovery.
- [x] Покрыт GPS Rescue crash-detection path.
- [x] Поведение classic PID не изменилось.
- [x] ADRC/PID unit tests проходят с `-Werror`.

### ADRC-003 — ADRC I/z3 во время crash recovery

- Приоритет: **P0**.
- Статус: `DONE`.
- Implementation commit(s): `73c5bc34a2`.
- Затронутые места:
  - `src/main/flight/pid.c`;
  - `src/main/flight/adrc.c` и `adrc.h` при необходимости;
  - `src/test/unit/pid_unittest.cc`.

Finding:

Crash recovery обнуляет `pidData.I`, но позднее в том же loop
`applyAdrcControl()` безусловно возвращает `I=-z3/b0`. При насыщенном `z3`
это может вернуть до полного `pidSumLimit`, который recovery пыталась убрать.

Acceptance criteria:

- [x] Определена политика ADRC state на входе, во время и при выходе из crash
      recovery.
- [x] Пока recovery требует нулевой I, ADRC не восстанавливает его позднее в
      том же loop.
- [x] Нет скачка I при выходе из recovery.
- [x] Unit test начинает с ненулевого/насыщенного `z3` и проверяет полный
      порядок вызовов одного PID loop.
- [x] Покрыты normal crash recovery и GPS Rescue.

Принятая политика: recovery-state latch действует на весь PID loop, `z3` и
выведенный из него `I` удерживаются в нуле, включая оси, вычисленные до поздно
обнаружившей crash оси. Первый чистый post-recovery loop тоже начинается без
старого disturbance trim.

### ADRC-004 — Устойчивая TD discretization и reset semantics

- Приоритет: **P0** при включённом `adrc_td_hz`.
- Статус: `DONE`.
- Implementation commit(s): `538931a201`.
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

- [x] TD использует устойчивую для любого положительного `dT` discretization
      либо cutoff жёстко ограничен относительно loop rate.
- [x] Sweep всех разрешённых loop-rate/cutoff комбинаций остаётся finite.
- [x] Reset seed’ит `vRef` из физически согласованного текущего значения.
- [x] Повторные resets 3D reversal не создают opposite saturated command.
- [x] Добавлена защита/recovery от non-finite state.
- [x] `adrc_td_hz=0` остаётся точным bypass.

Решение: raw Euler заменён на монотонный PT1 gain
`omega*dT/(1 + omega*dT)`, а reset seed’ит `vRef` текущим gyro rate.

### ADRC-005 — Loop-rate safe ESO

- Приоритет: **P0** для разрешённых экстремальных конфигураций.
- Статус: `DONE`.
- Implementation commit(s): `538931a201`.
- Затронутые места:
  - `src/main/flight/adrc.c`;
  - `src/main/cli/settings.c`;
  - `src/test/unit/adrc_unittest.cc`.

Finding:

ESO использует forward Euler, `wo` разрешён до `600`. Для BMI270
`3.2 kHz` и `pid_process_denom=16` PID loop равен `200 Hz`. При `wo=600`
observer имеет spectral radius больше `2` и чередуется между state clamps.

Acceptance criteria:

- [x] Для каждого runtime `dT` существует проверяемая граница устойчивости.
- [x] Невозможна настройка `wo`, которая отправляет observer в rail-to-rail
      limit cycle в пределах поддерживаемых loop rates.
- [x] Unit sweep покрывает минимум 200 Hz, 1.6 kHz, 4 kHz и 8 kHz.
- [x] Проверены gated и airborne observer paths.
- [x] Default tune не изменён без отдельного обоснования.

Решение: effective observer bandwidth ограничен инвариантом
`wo*dT <= 0.5`; profile/default при этом не переписывается.

### ADRC-006 — Persisted schema для D-term LPF

- Приоритет: **P0**, блокирует использование `adrc-dterm-lpf`.
- Статус: `IMPLEMENTED` — schema локально безопасна, flight/release отсутствуют.
- Implementation commit(s): `ed747bbb0c`, `a0d8dbc8d0`, `6447e513fe`.
- Ветка: `codex/adrc-dterm-remediation`.
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

- [x] Старые PG14 данные не могут быть тихо интерпретированы как новый layout.
- [x] Есть автоматический test старого serialized PG14 blob.
- [x] Явно выбрана стратегия: reset, migration или отдельный ADRC PG.
- [x] Учтено, что PG version занимает 4 бита, а version 15 — последнее
      доступное значение.
- [x] `pgStore`/reset/`pgLoad` round-trip сохраняет новый cutoff и соседние
      поля без смещения последующих профилей.
- [ ] CLI save/reboot/load проверен на реальном EEPROM/config storage path.
- [ ] Новый отдельно маркированный hex собран только после прохождения теста.
- [x] Старый `c1db43820` hex не распространяется как актуальный.

Предпочтительное направление: отдельный ADRC parameter group вместо
дальнейшего расходования version основного PID profile.

Локально выбрана безопасная краткосрочная стратегия PG15 reset: новое поле
перенесено в конец `adrcProfile_t`, registry version поднята `14 → 15`.
`pgLoad()` отклоняет blob PG14 размером 1064 bytes до копирования и сбрасывает
PID profiles в defaults; PG15 размером 1072 bytes проходит round-trip.

Критически важно: в штатном boot path один version mismatch делает весь
`readEEPROM()` неуспешным, после чего Betaflight выполняет полный config
reset/rewrite, а не только reset PID profiles. Поэтому до прошивки этой
экспериментальной ветки обязательно сохранить `diff all`. Отдельный ADRC PG
остаётся предпочтительной архитектурой перед включением функции в основной PR.

Независимый повторный review не нашёл code-correctness blocker. Остаточный
risk: даже при cutoff `0` две PT1-ступени остаются в hot path, а при включении
фильтр добавляет фазу в D/control path. На F411 D-term ветка добавляет около
1404 bytes text и 48 bytes BSS к main и занимает примерно 96.36% FLASH1.
Функция остаётся `off by default` до DWT 8 kHz и A/B blackbox/flight sweep.

### ADRC-007 — Фактически приложенный mixer feedback

- Приоритет: **P1**.
- Статус: `IMPLEMENTED` — основные uniform-scale paths и mixer E2E готовы;
  точная модель `MIXER_DYNAMIC` и closed-loop z3 drift остаются в ADRC-012.
- Implementation commit(s): `7d69539a63`, `6ea7c11aac`, `2a8ed176cf`.
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

- [x] Для legacy/linear/EZ Landing observer получает приложенный uniform
      normalization/attenuation scale, включая effective yaw-spin limit.
- [ ] Для `MIXER_DYNAMIC` observer получает фактически приложенный axis command либо доказанно
      эквивалентный scale.
- [x] Real-`mixTable()` tests покрывают legacy mixer, linear mixer и
      no-Airmode attenuation.
- [ ] Saturation test не вызывает ложный устойчивый drift `z3`.
- [x] Не добавлена циклическая зависимость PID↔mixer или лишняя задержка без
      документированного анализа.

Mixer публикует нормализованный command после constraint/normalization; ESO
использует его на следующем PID loop, что совпадает с прежней one-loop
семантикой `lastOutput`. При yaw-spin передаётся временный
`PIDSUM_LIMIT_MAX`, а не обычный `pidSumLimitYaw`. Для `MIXER_DYNAMIC`
сохранён documented approximation: uniform normalization учитывается, а
преднамеренная per-motor redistribution остаётся частью lumped plant
disturbance. Точная обратная реконструкция axis torque в hot path пока не
обоснована по cycle budget.

### ADRC-008 — Post-override throttle для ADRC

- Приоритет: **P1**.
- Статус: `DONE`.
- Implementation commit(s): `7d69539a63`, `2a8ed176cf`.
- Затронутые места:
  - `src/main/flight/adrc.c`;
  - `src/main/flight/mixer.c`;
  - tests автоматических режимов.

Finding:

`adrcUpdatePerLoopState()` читает `mixerGetThrottle()` до
`ALT_HOLD`/`GPS_RESCUE` throttle overrides. Gate и `b0` scheduling поэтому
моделируют pilot/pre-override throttle, а не реально приложенную тягу.

Acceptance criteria:

- [x] ADRC использует определённое post-override throttle значение.
- [x] Normal/manual mode сохраняет прежнюю семантику.
- [x] ALT_HOLD и GPS_RESCUE покрыты real-`mixTable()` tests.
- [x] Порядок вычислений явно задокументирован рядом с API.

`mixerThrottle` оставлен без изменения для blackbox/TPA. Отдельный
`mixerAdrcThrottle` публикуется после yaw-spin/launch/ALT_HOLD/GPS_RESCUE
override и mixer constraints; crash-flip/motor-stop публикуют нулевую
authority.

### ADRC-009 — State limits и finite-value defenses

- Приоритет: **P1**.
- Статус: `DONE`.
- Implementation commit(s): `538931a201`.
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

- [x] State bounds согласованы с реальным gyro FSR и command range.
- [x] Invalid persisted/internal parameters санитизируются до деления.
- [x] P/I/D и observer state восстанавливаются в finite domain; mixer clamp
      ограничивает итоговый `Sum`.
- [x] Тесты покрывают high-FSR gyro, zero/invalid `b0` и corrupt scale max.
- [x] Units в комментариях исправлены без изменения runtime semantics.

`z1` bound поднят до 8000 deg/s, profile coefficients имеют верхние и нижние
runtime bounds, а finite check читает IEEE-754 exponent напрямую и поэтому не
оптимизируется прочь при `-ffast-math`.

### ADRC-010 — Семантика CLASSIC↔ADRC handover

- Приоритет: **P1**, latent для stock runtime.
- Статус: `DONE`.
- Implementation commit(s): `11fd579d92`.
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

- [x] Принято явное решение: поддерживать runtime handover или удалить такое
      обещание из tests/docs.
- [x] Ветка «если handover поддерживается» неприменима: in-flight handover не
      поддерживается; disarmed switch test проверяет reset state и первый
      подготовленный control epoch вместо ложного mid-air обещания.
- [x] Если handover не поддерживается, API не создаёт ложной гарантии.
- [x] CLASSIC path после ADRC не получает скрытый ADRC I без принятой политики.

Принятое решение: stock Betaflight разрешает смену PID profile/controller law
только disarmed. На такой смене `pidResetIterm()` и ADRC gate/state reset
начинают новый arm epoch; same-type in-flight adjustment-range update сохраняет
observer state.

### ADRC-011 — Исправить vacuous gate tests

- Приоритет: **P1**, выполнять вместе с первым gate-related fix.
- Статус: `DONE`.
- Implementation commit(s): `5d862e6a73`.
- Затронуто: `src/test/unit/adrc_unittest.cc`.

Finding:

`GateDoesNotChatterWithInvertedThresholds` явно ставит
`liftoffIdleHoldMs=0`, а `GateReArmStillnessIsCappedIndependently` наследует
default `0`. Это полностью выключает re-arm path, который тесты должны
проверять.

Acceptance criteria:

- [x] Оба теста используют ненулевой hold и реально входят в re-arm branch.
- [x] Mutation/revert проверка показывает, что регрессия cross-clamp или
      stillness cap ломает соответствующий тест.
- [x] Отдельно сохранён test, подтверждающий, что hold `0` отключает re-arm.

### ADRC-012 — End-to-end tests и F411 cycle budget

- Приоритет: **P1**.
- Статус: `BLOCKED` — доступные host/build/mixer проверки выполнены; точный
  cycle deadline требует реального F411, остаются dynamic/closed-loop cases.
- Implementation/test commit(s): `2a8ed176cf`, `6447e513fe`, `2ddd4c4222`.

Обязательное покрытие:

- [x] Полный `pidController` ADRC path, а не только isolated `adrc.c`.
- [ ] Первый gate open и opt-in re-arm.
- [x] Crash recovery и GPS Rescue.
- [x] Real-`mixTable()` normalization/saturation feedback для legacy/linear,
      no-Airmode, yaw-spin, motor-stop и crashflip.
- [x] Real-`mixTable()` ALT_HOLD/GPS_RESCUE throttle.
- [x] TD/ESO sweep по loop rates и граничным настройкам.
- [x] High-FSR gyro и non-finite recovery.
- [x] PG rejection/reset и round-trip для нового D-term layout.
- [ ] Cycle benchmark на F411 при 8 kHz с ADRC enabled/disabled.
- [x] Зафиксированы RAM/flash delta для generic F405/F411.
- [ ] Зафиксирован измеренный запас до 125 µs deadline и stack high-water mark.

Размеры final main head `6ea7c11aac` относительно `a138a5dd19`:

| Target | Метрика | Baseline | Final | Delta | Остаток |
|---|---:|---:|---:|---:|---:|
| F405 | FLASH1 | 621420 | 624328 | +2908 B | 391480 B |
| F405 | RAM | 101756 | 101784 | +28 B | 29288 B |
| F405 | CCM | 13596 | 13600 | +4 B | 51936 B |
| F411 | FLASH1 | 469798 | 472214 | +2416 B | 19306 B |
| F411 | RAM | 102860 | 102876 | +16 B | 28196 B |

Обе generic firmware сборки проходят ARM GCC 13.3.1 с `-Werror`. Статический
assembly audit показал рост ADRC hot path (`pidController` на F405 примерно
+470 static instructions; F411 LTO `taskMainPidLoop` примерно +539), но это
не executed cycles. На F411 при 8 kHz бюджет равен 125 µs, поэтому verdict по
deadline: **NOT PROVEN, MEDIUM risk**. Нужен DWT `CYCCNT`/scheduler timing на
реальном F411 с exact target/config, ADRC и classic, worst-case logging/features.
Host x86 benchmark для такого verdict непереносим и намеренно не выдаётся за
hardware evidence.

### ADRC-013 — Rebase, CI, release b3 и точный re-flight

- Приоритет: **P1**, финальный integration gate.
- Статус: `BLOCKED` до завершения ADRC-001…012.
- Implementation/release commit(s): —.

Acceptance criteria:

- [ ] Ветка rebased на актуальный `upstream/master`.
- [ ] `src/config` обновлён до совместимой актуальной revision.
- [x] `git diff --check` чист на локальных heads.
- [x] Полный `make EXTRA_FLAGS=-Werror test-all` проходит локально.
- [ ] Официальная firmware matrix проходит.
- [ ] Release workflow либо падает при любом board failure, либо полнота
      artifacts проверяется отдельным обязательным job.
- [ ] Создан release b3 для точного reviewed SHA.
- [ ] Проверены Airmode first takeoff, zero-throttle catch, throttle punches,
      sustained saturation, crash recovery/GPS Rescue и несколько loop rates.
- [ ] Для каждого лога записаны SHA, target, diff конфигурации и verdict.
- [ ] Только после этого PR снимается с draft.

Локальный `upstream/master` на проверке: `6ecfb45f93`; remediation branch
имеет 9 upstream-only и 33 branch-only commits. `merge-tree` не показал
очевидных conflict markers, но реальный rebase и обновление `src/config` не
выполнялись: это отдельный integration stage, после которого изменятся SHA и
нужно заново прогнать firmware matrix.

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
| 2026-07-11 | ADRC-007,008 | IMPLEMENTED | `7d69539a63` | ADRC/PID, F405 | Applied mixer scale и post-override throttle; mode E2E остаётся |
| 2026-07-11 | ADRC-001,011 | IMPLEMENTED/DONE | `5d862e6a73` | ADRC/PID, mutation | Новый actuator-feedback epoch на gate open/re-open |
| 2026-07-11 | ADRC-010 | DONE | `11fd579d92` | PID handover tests | Disarmed-only transition semantics |
| 2026-07-11 | ADRC-002,003 | DONE | `73c5bc34a2` | ADRC 39/39, PID 24/24 | D=0, GPS Rescue, late-axis crash, bumpless exit |
| 2026-07-11 | ADRC-004,005,009 | DONE | `538931a201` | ADRC/PID, `-ffast-math`, F405/F411 | TD/ESO/finite/state hardening |
| 2026-07-11 | ADRC-007 | IMPLEMENTED | `6ea7c11aac` | ADRC 39/39, PID 24/24, F405/F411 | Effective yaw-spin limit; integration re-review finding fixed |
| 2026-07-11 | ADRC-007,008,012 | IMPLEMENTED/DONE | `2a8ed176cf` | mixer 8/8, ADRC 39/39, PID 24/24; normal и `-ffast-math` | Real-`mixTable()` regression harness |
| 2026-07-11 | ADRC-006 | IMPLEMENTED | `ed747bbb0c` | ADRC 43/43, PID 26/26, F405 | D-term LPF feature port, не использовать отдельно |
| 2026-07-11 | ADRC-006 | IMPLEMENTED | `a0d8dbc8d0` | PG14 reject, PG15 round-trip | Обязателен вместе с feature commit |
| 2026-07-11 | ADRC-006,007 | IMPLEMENTED | `2ca9126ac2` | `-Werror -ffast-math`, F405 | D-term head синхронизирован с yaw-spin fix |
| 2026-07-11 | ADRC-006 | IMPLEMENTED | `6447e513fe` | ADRC 46/46, PID 26/26, `-ffast-math` | Cutoff transitions, corruption recovery, debug mapping |
| 2026-07-11 | ADRC-007,008,012 | IMPLEMENTED/DONE | `2ddd4c4222` | mixer 8/8, ADRC 46/46, PID 26/26, `test-all` | Mixer E2E tests синхронизированы в D-term branch |

## Принятые решения и остаточные блокеры

1. Gate open: сохранить observer/filters, обнулить только grounded-эпоху
   `lastOutput`; новый applied output приходит от mixer на следующий loop.
2. Crash recovery: на весь recovery loop и первый clean handoff подавлять
   disturbance trim через `z3=0`/`I=0`, не останавливая rate observer.
3. ESO: ограничивать effective `wo*dT <= 0.5`; TD использовать стабильный PT1.
4. D-term LPF: для локальной экспериментальной ветки использовать PG15 reset;
   перед основной интеграцией предпочесть отдельный ADRC PG.
5. Observer feedback: delayed applied command предыдущего mixer loop; для
   uniform mixer modes передавать normalization scale. `MIXER_DYNAMIC`
   per-motor redistribution пока остаётся lumped disturbance.
6. CLASSIC↔ADRC: только disarmed transition; mid-air гарантия удалена.
7. До `DONE` по ADRC-001/006/007/012/013 нужны: `MIXER_DYNAMIC`/closed-loop
   z3 tests, F411 DWT timing, полный EEPROM/CLI boot test, upstream rebase/CI
   и полётные логи точных firmware SHA.
