# Safety

As many can attest, multirotors and RC models in general can be very dangerous, particularly on the test bench. Here are some simple golden rules to save you a trip to the local ER:
* **NEVER** arm your model with propellers fitted unless you intend to fly!
* **Always** remove your propellers if you are setting up for the first time, flashing firmware, or if in any doubt.

## Before installing the Flight Controller in your craft

Please consult the [Cli](Cli.md), [Controls](Controls.md), [Failsafe](Failsafe.md) and [Modes](Modes.md) 
pages for further important information.

You are highly advised to use the Receiver tab in CleanFlight Configurator, making sure your Rx channel values are centered at 1500 (1520 for Futaba RC) and Min/Max values are reached when controls are operated.
The referenced values for each channel, have marked impact on the operation of Flight Controller and its Flight Modes.

You may have to adjust your channel endpoints and trims/sub-trims, on your RC transmitter, to achieve the expected values.

## Props Spinning When Armed
With the default configuration, when the controller is armed, the propellers *WILL* begin spinning at low speed.
We recommend keeping this setting as it provides a good visual indication the craft is armed.

If you wish to change this behavior, see the MOTOR_STOP feature in the Configurator and relevant docuemntation pages.
Enabling this feature will stop the props from spinning when armed.
