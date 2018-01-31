## Important: Feature freeze / release candidate phase for Betaflight 3.3

From 01/02/2018 until the release of Betaflight 3.3.0 (scheduled for 01/03/2018), the project is in a 'feature freeze / release candidate' phase. This means:

1. Pull requests can still be submitted as normal. Comments / discussions will probably be slower than normal due to shifted priorities;

2. If your pull request is a fix for an existing bug, or an update for a single target that has a low risk of side effect for other targets, it will be reviewed, and if accepted merged into `master` for the 3.3 release;

3. All other pull requests will be scheduled for 3.4, and discussed / reviewed / merged into `master` after 3.3.0 has been released. Please keep in mind that this will potentially mean that you will have to rebase your changes if they are broken by bugfixes made to 3.3.


## Important considerations when opening a pull request:

1. Pull requests will only be accepted if they are opened against the `master` branch. Pull requests opened against other branches without prior consent from the maintainers will be closed;

2. Please follow the coding style guidlines: https://github.com/cleanflight/cleanflight/blob/master/docs/development/CodingStyle.md

3. If your pull request is a fix for one or more issues that are open in GitHub, add a comment to your pull request, and add the issue numbers of the issues that are fixed in the form `Fixes #<issue number>`. This will cause the issues to be closed when the pull request is merged;

4. Remove this Text :).
