## Important: Feature freeze / release candidate phase for Betaflight 3.5

From 22/07/2018 until the release of Betaflight 3.5.0 (scheduled for 05/08/2018), the project is in a 'feature freeze / release candidate' phase. This means:

1. Pull requests can still be submitted as normal. Comments / discussions will probably be slower than normal due to shifted priorities;

2. If your pull request is a fix for an existing bug, or an update for a single target that has a low risk of side effect for other targets, it will be reviewed, and if accepted merged into `master` for the 3.5 release;

3. All other pull requests will be scheduled for 4.0, and discussed / reviewed / merged into `master` after 3.5.0 has been released. Please keep in mind that this potentially means that you will have to rebase your changes if they are broken by bugfixes made for 3.5.


## Important considerations when opening a pull request:

1. Pull requests will only be accepted if they are opened against the `master` branch. Pull requests opened against other branches without prior consent from the maintainers will be closed;

2. Please follow the coding style guidlines: https://github.com/cleanflight/cleanflight/blob/master/docs/development/CodingStyle.md

3. Keep your pull requests as small and concise as possible. One pull request should only ever add / update one feature. If the change that you are proposing has a wider scope, consider splitting it over multiple pull requests. In particular, pull requests that combine changes to features and one or more new targets are not acceptable.

4. Ideally, a pull request should contain only one commit, with a descriptive message. If your changes use more than one commit, rebase / squash them into one commit before submitting a pull request. If you need to amend your pull request, make sure that the additional commit has a descriptive message, or - even better - use `git commit --amend` to amend your original commit.

5. All pull requests are reviewed. Be ready to receive constructive criticism, and to learn and improve your coding style. Also, be ready to clarify anything that isn't already sufficiently explained in the code and text of the pull request, and to defend your ideas.

6. If your pull request is a fix for one or more issues that are open in GitHub, add a comment to your pull request, and add the issue numbers of the issues that are fixed in the form `Fixes #<issue number>`. This will cause the issues to be closed when the pull request is merged;

7. Remove this Text :).
