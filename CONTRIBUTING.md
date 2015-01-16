# Contributing

Please see the Contributing section of the README.md

Please see the docs/developers folder for other notes.

Ensure you understand the github workflow: https://guides.github.com/introduction/flow/index.html

Please keep pull requests focused on one thing only, since this makes it easier to merge and test in a timely manner.

If you need help with pull requests there are guides on github here:

https://help.github.com/articles/creating-a-pull-request/

The main flow for a contributing is as follows:

1. Login to github, goto the cleanflight repository and press `fork`.
2. `git clone <url to YOUR fork>`
3. `cd cleanflight`
4. `git checkout master`
5. `git checkout -b my-new-code`
6. Make changes
7. `git add <files that have changed>`
8. `git commit`
9. `git push origin my-new-code`
10. Create pull request using github UI to merge your changes from your new branch into `cleanflight/master`
11. Repeat from step 4 for new other changes.

The primary thing to remember is that separate pull requests should be created for separate branches.  Never create a pull request from your `master` branch.

Later, you can get the changes from the cleanflight repo into your `master` branch by adding cleanflight as a git remote and merging from it as follows:

1. `git add remote cleanflight https://github.com/cleanflight/cleanflight.git`
2. `git checkout master`
3. `git fetch cleanflight`
4. `git merge cleanflight/master`
 
