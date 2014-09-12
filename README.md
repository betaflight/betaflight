# Cleanflight

Clean-code version of baseflight flight-controller - flight controllers are used to fly multi-rotor craft and fixed wing craft.

This fork differs from baseflight in that it attempts to use modern software development practices which result in:

1. greater reliability through code robustness. 
2. easier maintainance through code cleanliness.
3. easier to develop new features. 
4. easier to re-use code though code de-coupling and modularisation.

The MultiWii software, from which baseflight originated, violates many good software development best-practices. Hopefully this fork will go some way to address them. If you see any bad code in this fork please immediately raise an issue so it can be fixed, or better yet submit a pull request.

## Additional Features

Cleanflight also has additional features not found in baseflight.  Since the primary maintainer of baseflight also sells hardware there is no incentive for baseflight to support the target platforms and features that Cleanflight now provides.

For a list of features, changes and some discussion please review the thread on MultiWii forums and consult the documenation.

http://www.multiwii.com/forum/viewtopic.php?f=23&t=5149


## Documentation

There is some documentation here: https://github.com/hydra/cleanflight/tree/master/docs 

If what you need is not covered then refer to the baseflight documentation. If you still can't find what you need then visit the #cleanflight on the Freenode IRC network

## IRC Support and Developers Channel

There's a dedicated IRC channel here:

irc://irc.freenode.net/#cleanflight

If you are using windows and don't have an IRC client installed then take a look at HydraIRC - here: http://hydrairc.com/

## Videos


There is a dedicated Cleanflight youtube channel which has progress update videos, flight demonstrations, instrutions and other related videos.

https://www.youtube.com/playlist?list=PL6H1fAj_XUNVBEcp8vbMH2DrllZAGWkt8

Please subscribe and '+1' the videos if you find them useful.

## Configuration Tool

To configure Cleanflight you should use the Cleanlight-configurator GUI tool (Windows/OSX/Linux) that can be found here:

https://chrome.google.com/webstore/detail/cleanflight-configurator/enacoimjcgeinfnnnpajinjgmkahmfgb

The source for it is here:

https://github.com/hydra/cleanflight-configurator

## Contributing

Before making any contributions, take a note of the https://github.com/multiwii/baseflight/wiki/CodingStyle

For this fork it is also advised to read about clean code, here are some useful links:

* http://cleancoders.com/
* http://en.wikipedia.org/wiki/SOLID_%28object-oriented_design%29
* http://en.wikipedia.org/wiki/Code_smell
* http://en.wikipedia.org/wiki/Code_refactoring
* http://www.amazon.co.uk/Working-Effectively-Legacy-Robert-Martin/dp/0131177052
