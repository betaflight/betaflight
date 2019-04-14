## Important: New requirements for the submission of new and updated targets

As announced earlier in https://github.com/betaflight/betaflight#betaflight-40, Betaflight 4.0 is introducing a radically new way to define targets, the so-called 'Unified Targets'.

This new approach makes it possible to use the same firmware binary (the so called 'Unified Target firmware') for all boards that share the same MCU type (only supported on F4 and F7). Manufacturers will be able to add support for new boards by simply publishing a new configuration (the so called 'Unified Target configuration') for their new board. Users can then simply load the already published Unified Target firmware and the new Unified Target configuration onto their new board to get it to work.

Work to give users a simple way to flash unified targets in Betaflight configurator still needs to be done, so Betaflight 4.0 will be released with targets done in the 'legacy' way. But the plan is to add support for seamless use of Unified Targets into Betaflight configurator after Betaflight 4.0 has been released, and convert all of the existing F4 and F7 targets to the new format before the release of Betaflight 4.1.

In order to be prepared for this move, the following new requirements for pull requests adding new targets or modifying existing targets are put in place from now on:

1. After the release of Betaflight 4.0.0, new F3 based targets can only be added into the `4.0.x-maintenance` branch. This ties in with the release of firmware for F3 based targets ending after 4.0: https://github.com/betaflight/betaflight#end-of-active-development-for-stm32f3-based-flight-controllers;

All subsequent rules exclude F3 based targets:

2. For any new target that is to be added, both a 'legacy' format target definition into `src/main/target/` and a new Unified Target config into `unified_targets/configs/` need to be submitted. See the for how to create a Unified Target configuration: https://github.com/betaflight/betaflight/blob/master/unified_targets/docs/CreatingAUnifiedTarget.md;

3. For changes to existing targets, the change needs to be applied to both the 'legacy' format target definition in `src/main/target/` and a new Unified Target config in `unified_targets/configs/`. If no Unified Target configuration for the target exists, a new Unified Target configuration will have to be created and submitted alongside the proposed change.


## Important considerations when opening a pull request:

1. Pull requests will only be accepted if they are opened against the `master` branch. Pull requests opened against other branches without prior consent from the maintainers will be closed;

2. Please follow the coding style guidlines: https://github.com/cleanflight/cleanflight/blob/master/docs/development/CodingStyle.md

3. Keep your pull requests as small and concise as possible. One pull request should only ever add / update one feature. If the change that you are proposing has a wider scope, consider splitting it over multiple pull requests. In particular, pull requests that combine changes to features and one or more new targets are not acceptable.

4. Ideally, a pull request should contain only one commit, with a descriptive message. If your changes use more than one commit, rebase / squash them into one commit before submitting a pull request. If you need to amend your pull request, make sure that the additional commit has a descriptive message, or - even better - use `git commit --amend` to amend your original commit.

5. All pull requests are reviewed. Be ready to receive constructive criticism, and to learn and improve your coding style. Also, be ready to clarify anything that isn't already sufficiently explained in the code and text of the pull request, and to defend your ideas.

6. We use continuos integration (CI) with [Travis](https://travis-ci.com/betaflight) to build build all targets and run the test suite for every pull request. Pull requests that fail any of the builds or fail tests will most likely not be reviewed before they are fixed to build successfully and pass the tests. In order to get a quick idea if there are things that need fixing **before** opening a pull request or pushing an update into an existing pull request, run `make pre-push` to run a representative subset of the CI build. _Note: This is not an exhaustive test(which will take hours to run on any desktop type system), so even if this passes the CI build might still fail._

7. If your pull request is a fix for one or more issues that are open in GitHub, add a comment to your pull request, and add the issue numbers of the issues that are fixed in the form `Fixes #<issue number>`. This will cause the issues to be closed when the pull request is merged;

8. Remove this Text :).
