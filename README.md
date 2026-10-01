![Betaflight](https://raw.githubusercontent.com/betaflight/.github/main/profile/images/bf_logo.svg#gh-light-mode-only)
![Betaflight](https://raw.githubusercontent.com/betaflight/.github/main/profile/images/bf_logo_dark.svg#gh-dark-mode-only)

[![Latest version](https://img.shields.io/github/v/release/betaflight/betaflight)](https://github.com/betaflight/betaflight/releases) [![Build](https://img.shields.io/github/actions/workflow/status/betaflight/betaflight/push.yml?branch=master)](https://github.com/betaflight/betaflight/actions/workflows/push.yml) [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) [![Join us on Discord!](https://img.shields.io/discord/868013470023548938)](https://discord.betaflight.com/invite)

Betaflight is free, open-source flight control software for every kind of drone, from racing and freestyle to cinematic filming, long range, micros and wings. It exists so that every pilot, whatever they fly and whoever made their hardware, gets precise, predictable and reliable flight, and so that the knowledge behind it stays open to everyone.

This repository holds the Betaflight firmware that runs on the flight controller.

## Release Schedule

The Betaflight release schedule and development cadence are published at [betaflight.com](https://betaflight.com/blog/2025/09/01/Calendar%20Versioning%20Change).

## Requirements for the submission of new and updated target configuration

The requirements for pull requests adding new targets or modifying existing targets are available on the [betaflight.com website](https://www.betaflight.com/docs/development/manufacturer/requirements-for-submission-of-targets).

## Features

Betaflight has the following features:

- Multi-color RGB LED strip support (each LED can be a different color using variable length WS2811 Addressable RGB strips - use for Orientation Indicators, Low Battery Warning, Flight Mode Status, Initialization Troubleshooting, etc)
- DShot (150, 300 and 600), Multishot, Oneshot (125 and 42) and Proshot1000 motor protocol support
- Blackbox flight recorder logging (to onboard flash or external microSD card where equipped)
- Support for targets that use STM32 F4, F7, G4, H5 and H7 (plus C5 and N6 in developer preview), AT32F435, APM32 and RP2350 (PICO) processors, experimental support for ESP32 and X32, and SITL for simulation (see the [hardware policy](https://betaflight.com/hardware) for current status)
- PWM, PPM, SPI, and Serial (CRSF, SBus, SumH, SumD, Spektrum 1024/2048, XBus, etc) RX connection with failsafe detection
- Multiple telemetry protocols (CRSF, FrSky, HoTT smart-port, MSP, etc)
- RSSI via ADC - Uses ADC to read PWM RSSI signals, tested with FrSky D4R-II, X8R, X4R-SB, & XSR
- OSD support & configuration without needing third-party OSD software/firmware/comm devices
- OLED Displays - Display information on: Battery voltage/current/mAh, profile, rate profile, mode, version, sensors, etc
- In-flight manual PID tuning and rate adjustment
- PID and filter tuning using sliders
- Rate profiles and in-flight selection of them
- Configurable serial ports for Serial RX, Telemetry, ESC telemetry, MSP, GPS, OSD, Sonar, etc - Use most devices on any port, softserial included
- VTX support for Unify Pro and IRC Tramp
- and MUCH, MUCH more.

## Installation & Documentation

See: https://betaflight.com/docs/wiki

## Support and Developers Channel

There's a dedicated [Discord server](https://discord.betaflight.com/invite) for help, support and general community.

## Betaflight Application

To configure Betaflight you should use the [Betaflight App](https://app.betaflight.com). It is a progressive web app, so should always be the latest version.

## Contributing

Contributions are welcome and encouraged. You can contribute in many ways:

- implement a new feature in the firmware or in the app (see [below](#developers));
- documentation updates and corrections;
- How-To guides - received help? Help others!
- bug reporting & fixes;
- new feature ideas & suggestions;
- provide a new translation for the app, or help us maintain the existing ones (see [below](#translators)).

The best place to start is the [Betaflight Discord](https://discord.betaflight.com/invite). Next place is the github issue tracker:

https://github.com/betaflight/betaflight/issues
https://github.com/betaflight/betaflight-configurator/issues

Before creating new issues please search to see if there is an existing one.

If you want to contribute to our efforts financially, please consider making a donation to us through [PayPal](https://paypal.me/betaflight).

If you want to contribute financially on an ongoing basis, you should consider becoming a patron for us on [Patreon](https://www.patreon.com/betaflight).

## Developers

Contribution of bugfixes and new features is encouraged. Please be aware that we have a thorough review process for pull requests, and be prepared to explain what you want to achieve with your pull request.
Before starting to write code, please read our [development guidelines](https://www.betaflight.com/docs/development) and [coding style definition](https://www.betaflight.com/docs/development/CodingStyle).

GitHub actions are used to run automatic builds.

### Building with Docker/Devcontainers

A preconfigured [devcontainer](.devcontainer/README.md) is included for a consistent build environment across all platforms. This is the recommended approach for Windows developers:

```bash
# With VS Code: Install "Dev Containers" extension, open folder, and select "Reopen in Container"

# Or command-line only:
docker build -t betaflight-dev -f .devcontainer/containerfile .devcontainer/
docker run --rm -v "${PWD}:/workspace" -w /workspace betaflight-dev make TARGET=SPEEDYBEEF405WING
```

See the [devcontainer documentation](.devcontainer/README.md) for detailed setup instructions including hardware flashing.

## Translators

We want to make Betaflight accessible for pilots who are not fluent in English, and for this reason we are currently maintaining translations into 21 languages for the Betaflight App: Català, Dansk, Deutsch, Español, Euskera, Français, Galego, Hrvatski, Bahasa Indonesia, Italiano, 日本語, 한국어, Latviešu, Português, Português Brasileiro, polski, Русский язык, Svenska, 简体中文, 繁體中文.
We have got a team of volunteer translators who do this work, but additional translators are always welcome to share the workload, and we are keen to add additional languages. If you would like to help us with translations, you have got the following options:

- if you help by suggesting some updates or improvements to translations in a language you are familiar with, head to [crowdin](https://crowdin.com/project/betaflight-configurator) and add your suggested translations there;
- if you would like to start working on the translation for a new language, or take on responsibility for proof-reading the translation for a language you are very familiar with, please head to the [Betaflight Discord](https://discord.betaflight.com/invite) chat and join the ['translation'](https://discord.com/channels/868013470023548938/1057773726915100702) channel - the people in there can help you to get a new language added, or set you up as a proof reader.

## Hardware Issues

We fix Betaflight. Manufacturers support their hardware. Pilots own their builds.

- The Betaflight team fixes bugs in the firmware, the Betaflight App and the cloud build, and maintains the standards and guidelines.
- Manufacturers support their hardware: design, config, documentation, faults, warranty and customer support.
- Pilots own their builds (wiring, setup, tuning, peripherals), with help from the community.

Betaflight does not make or sell hardware. We work with many manufacturers, but support for a flight controller or other component comes from the company that made it. If your hardware is faulty, or its config or documentation is wrong, please contact the manufacturer or the shop you bought it from. For help with wiring, setup and tuning, ask the community on [Discord](https://discord.betaflight.com/invite). If you have found a bug in Betaflight itself, please [open an issue](https://github.com/betaflight/betaflight/issues).

See [hardware support](https://betaflight.com/support) on betaflight.com for more on who looks after what.

## Betaflight Releases

You can find our release [here](https://github.com/betaflight/betaflight/releases) on Github and we also have more detailed [release notes](https://www.betaflight.com/docs/category/release-notes) at [betaflight.com](https://www.betaflight.com).

## Open Source / Contributors

Betaflight is software that is **open source** and is available free of charge without warranty to all users.

For a complete list of contributors (past and present) see [Github](https://github.com/betaflight/betaflight/graphs/contributors).
