# Test coverage analysis

There are a number of possibilities to analyse test coverage and produce various reports. There are guides available from many sources, a good overview and link collection to more info can be found on Wikipedia: 

https://en.wikipedia.org/wiki/Gcov

A simple report for a single test can for example be made using this command:

```
gcov -s src/main/sensors -o obj/test/ battery_unittest.cc
```

To produce a coverage report in XML format usable by the Cobertura plugin in Jenkins requires installation of a  Python script called "gcovr" from github:

https://github.com/gcovr/gcovr/tree/dev

Example usage in Jenkins:

```
/gcovr-install-path/gcovr/scripts/gcovr obj/test --root=src/main -x > coverage.xml
```

There are many other ways to produce test coverage reports in other formats, like html etc. 
