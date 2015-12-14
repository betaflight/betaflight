# Safety

As many can attest, multirotors and RC models in general can be very dangerous, particularly on the test bench. Here are some simple golden rules to save you a trip to the local ER:
* **NEVER** arm your model with propellers fitted unless you intend to fly!
* **Always** remove your propellers if you are setting up for the first time, flashing firmware, or if in any doubt.

## Before Installing

Please consult the [Cli](Cli.md), [Controls](Controls.md), [Failsafe](Failsafe.md) and [Modes](Modes.md) 
pages for further important information.

You are highly advised to use the Receiver tab in the CleanFlight Configurator, making sure your Rx channel 
values are centered at 1500 (1520 for Futaba RC) with minimum & maximums of 1000 and 2000 (respectively) 
are reached when controls are operated.  Failure to configure these ranges properly can create
problems, such as inability to arm (because you can't reach the endpoints) or immediate activation of
[failsafe](Failsafe.md).

You may have to adjust your channel endpoints and trims/sub-trims on your RC transmitter to achieve the 
expected range of 1000 to 2000.

The referenced values for each channel have marked impact on the operation of the flight controller and the 
different flight modes.

## Props Spinning When Armed
With the default configuration, when the controller is armed, the propellers *WILL* begin spinning at low speed.
We recommend keeping this setting as it provides a good visual indication the craft is armed.

If you wish to change this behavior, see the MOTOR_STOP feature in the Configurator and relevant docuemntation pages.
Enabling this feature will stop the props from spinning when armed.
