How to build, test & debug Cleanflight in Eclipse on Linux, Windows & MacOS.

## Checklist

Use this checklist to make sure you didn't miss a step. If you need more help, use the *read more* links.

- [ ] [Download and Install](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html) the latest (currently 1.8) 64bit Oracle JDK [read more](#install-the-jdk)
- [ ] [Download and Install](https://eclipse.org/downloads/packages/eclipse-ide-cc-developers/lunasr2) Eclipse Luna (4.4) 64bit CDT edition [read more](#install-eclipse) **NOT** Mars or Neon since they are not tested by the *GNU ARM Eclipse* *(as of January 2016)* [read more](#install-the-jdk)

### Install the JDK

The [minimum JDK version](http://gnuarmeclipse.github.io/plugins/install/#java) supported by GNU Arm Eclipse is 1.7 but the current latest, 1.8, is recommended instead. While Oracle JDK is the recommended version, [they do also support](http://gnuarmeclipse.github.io/plugins/install/#java) the OpenJDK.

### Install Eclipse

The minimum Eclipse version is Kepler 4.3, the maximum is Mars 4.5 although some things are [known to be broken](http://gnuarmeclipse.github.io/plugins/install/#eclipse--cdt) in 4.5 with GNU Arm Tools.

CDT v8.6.0 as shipped in the Eclipse Luna CDT download is recommended. The minimum CDT version is 8.3.

The 64bit Eclipse is preferred but a 32bit Eclipse can be used; ensure you run it on a 32bit JDK.
