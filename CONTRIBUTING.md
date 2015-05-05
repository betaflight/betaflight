# Issues and Support.

Please remember the issue tracker on github is _not_ for user support.  Please also do not email developers directly for support.  Instead please use IRC or the forums first, then if the problem is confirmed create an issue that details how to repeat the problem so it can be investigated.

Issues created without steps to repeat are likely to be closed.  E-mail requests for support will go un-answered; All support needs to be public so that other people can read the problems and solutions.

Remember that issues that are due to mis-configuration, wiring or failure to read documentation just takes time away from the developers and can often be solved without developer interaction by other users.

Please search for existing issues *before* creating new ones.

# Developers

Please see the Contributing section of the README.md

Please see the docs/developers folder for other notes.

Ensure you understand the github workflow: https://guides.github.com/introduction/flow/index.html

Please keep pull requests focused on one thing only, since this makes it easier to merge and test in a timely manner.

If you need help with pull requests there are guides on github here:

https://help.github.com/articles/creating-a-pull-request/

The main flow for a contributing is as follows:

1. Login to github, goto the cleanflight repository and press `fork`.
2. Then using the command line/terminal on your computer: `git clone <url to YOUR fork>`
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

1. `git remote add cleanflight https://github.com/cleanflight/cleanflight.git`
2. `git checkout master`
3. `git fetch cleanflight`
4. `git merge cleanflight/master`
 

You can also perform the git commands using the git client inside Eclipse.  Refer to the Eclipse git manual.
 
