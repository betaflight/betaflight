# Security Policy

Betaflight uses a single, organisation-wide security policy for all of its
repositories.

**Please report security vulnerabilities privately — never through public
GitHub issues, pull requests, or Discord.**

- **GitHub (preferred, where enabled):** on this repository's **Security** tab,
  use **Report a vulnerability** if that button is shown.
- **Email (always works):** security@betaflight.com

For the full policy — scope, what to include, our disclosure timeline, and safe
harbour — see the canonical Betaflight security policy:

https://github.com/betaflight/.github/blob/main/SECURITY.md

## Supported versions

The firmware uses calendar-based versioning, `YEAR.MONTH.PATCH` (for example
`2025.12.1`). The firmware and the
[configurator app](https://github.com/betaflight/betaflight-configurator) are
released in lockstep on the same `YEAR.MONTH` versions and are supported
together.

We provide security support for the **two most recent major (year) releases** —
a rolling window of roughly the last two years. Security fixes are made on
`master` and backported to each supported series as point releases.

| Firmware series          | Security support                             |
| ------------------------ | -------------------------------------------- |
| `2026.x`                 | ✅ Supported                                  |
| `2025.x`                 | ✅ Supported                                  |
| `4.5.x`                  | ✅ Supported — retained as an exception       |
| `4.4.x` and older `4.x`  | ❌ Not supported                              |
| `3.x`, `2.x`, `1.x`      | ❌ Not supported                              |

The `4.5.x` series is kept as a deliberate exception because of its widespread
use, and will continue to receive security backports for as long as that
remains practical. Older series are no longer maintained; users should upgrade
to a supported release.

See the [organisation-wide policy](https://github.com/betaflight/.github/blob/main/SECURITY.md#supported-versions)
for the full support rule.
