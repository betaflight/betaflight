## Important: Embargo on new targets for Betaflight 4.0

As announced earlier in https://github.com/betaflight/betaflight#betaflight-40, Betaflight 4.0 will introduce a radically new way to define targets. At the moment, we are making the changes necessary for this, and this requires some changes to how targets are defined in the 'legacy' way. Because of this, any pull request opened against `master` that introduces a new target is likely to be outdated by the time it has been reviewed. This means extra work for the target maintainer and the reviewers, for little or no benefit.

Because of this, the following embargo is put in place:

1. From 30/12/2018 on until the release of Betaflight 4.0 (scheduled for 01/04/2019), no pull requests that introduce new targets against `master` are accepted. Because any such pull requests will be outdated by the time 4.0 is released, they will be closed;

2. During the time that 1. is in place, pull requests adding new targets (in 3.5 format) against the `3.5.x-maintenance` branch will be considered. This is an exception to the rule that all pull requests must be opened against `master`. This exception only applies to pull requests containing no other changes than a new target and a documentation page for the new target. These new targets will be included in upcoming 3.5 maintenance releases;

3. Targets added under the exception in 2. will have to be ported into 4.0 before 4.0 is released. The format that these targets will have in 4.0 is yet to be determined, and instructions for porting the targets will have to be written, but we expect the maintainers of these targets to make themselves available to assist with this task when the time comes. Please follow the news and updates that are posted to https://betaflight.com/ or the Betaflight GitHub page (https://github.com/betaflight/betaflight) to keep track of when action is required.


## Important considerations when opening a pull request:

1. Pull requests will only be accepted if they are opened against the `master` branch. Pull requests opened against other branches without prior consent from the maintainers will be closed;

2. Please follow the coding style guidlines: https://github.com/cleanflight/cleanflight/blob/master/docs/development/CodingStyle.md

3. Keep your pull requests as small and concise as possible. One pull request should only ever add / update one feature. If the change that you are proposing has a wider scope, consider splitting it over multiple pull requests. In particular, pull requests that combine changes to features and one or more new targets are not acceptable.

4. Ideally, a pull request should contain only one commit, with a descriptive message. If your changes use more than one commit, rebase / squash them into one commit before submitting a pull request. If you need to amend your pull request, make sure that the additional commit has a descriptive message, or - even better - use `git commit --amend` to amend your original commit.

5. All pull requests are reviewed. Be ready to receive constructive criticism, and to learn and improve your coding style. Also, be ready to clarify anything that isn't already sufficiently explained in the code and text of the pull request, and to defend your ideas.

6. If your pull request is a fix for one or more issues that are open in GitHub, add a comment to your pull request, and add the issue numbers of the issues that are fixed in the form `Fixes #<issue number>`. This will cause the issues to be closed when the pull request is merged;

7. Remove this Text :).
