#Development

This document is primarily for developers only.

##Unit testing

Ideally, there should be tests for any new code. However, since this is a legacy codebase which was not designed to be tested this might be a bit difficult.

If you want to make changes and want to make sure it's tested then focus on the minimal set of changes required to add a test.

Tests currently live in the `test` folder and they use the google test framework.  
The tests are compiled and run natively on your development machine and not on the target platform.
This allows you to develop tests and code and actually execute it to make sure it works without needing a development board or simulator.

This project could really do with some functional tests which test the behaviour of the application.

All pull requests to add/improve the testability of the code or testing methods are highly sought!

##General principals

1. Name everything well.
2. Strike a balance between simplicity and not-repeating code.
3. Methods that return a boolean should be named as a question, and should not change state.  e.g. 'isOkToArm()'
4. Methods that start with the word 'find' can return a null, methods that start with 'get' should not.
5. Methods should have verb or verb-phrase names, like `deletePage` or `save`.  Variables should not, they generally should be nouns.  Tell the system to 'do' something 'with' something.  e.g. deleteAllPages(pageList).
6. Keep methods short - it makes it easier to test.
7. Don't be afraid of moving code to a new file - it helps to reduce test dependencies.
8. Avoid noise-words in variable names, like 'data' or 'info'.  Think about what you're naming and name it well.  Don't be afraid to rename anything.
9. Avoid comments taht describe what the code is doing, the code should describe itself.  Comments are useful however for big-picture purposes and to document content of variables.
10. If you need to document a variable do it at the declarion, don't copy the comment to the `extern` usage since it will lead to comment rot.
11. Seek advice from other developers - know you can always learn more.
12. Be professional - attempts at humor or slating existing code in the codebase itself is not helpful when you have to change/fix it.
13. Know that there's always more than one way to do something and that code is never final - but it does have to work.

###Running the tests.

The tests and test build system is very simple and based of the googletest example files, it will be improved in due course.

```
cd test
make
```

This will build a set of executable files, one for each `*_unittest.cc` file.

You can run them on the command line to execute the tests and to see the test report.

You can also step-debug the tests in eclipse and you can use the GoogleTest test runner to make building and re-running the tests simple.

The tests are currently always compiled with debugging information enabled, there may be additional warnings, if you see any warnings please attempt to fix them and submit pull requests with the fixes.


##TODO

* Test OpenLRSNG's RSSI PWM on AUX5-8.
* Add support for UART3/4 on STM32F3.
* Cleanup validateAndFixConfig and pwm_mapping.c to use some kind of feature/timer/io pin mapping to remove #ifdef
* Split RX config into RC config and RX config.
* Enabling/disabling features should not take effect until reboot since.  Main loop executes and uses new flags as they are set in the cli but
appropriate init methods will not have been called which results in undefined behaviour and could damage connected devices - this is a legacy
problem from baseflight.
* Solve all the naze rev4/5 HSE_VALUE == 8000000/1200000 checking, the checks should only apply to the naze32 target.  See system_stm32f10x.c/SetSysClock().

##Known Issues

* Softserial RX on STM32F3 does not work. TX is fine.
* Dynamic throttle PID does not work with new pid controller.
* Autotune does not work yet with with new pid controller.

