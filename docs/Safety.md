# Safety

As many can attest, multirotors and RC models in general can be very dangerous, particularly on the test bench. Here are some simple golden rules to save you a trip to the local ER:
* **NEVER** arm your model with propellers fitted unless you intend to fly!
* **Always** remove your propellers if you are setting up for the first time, flashing firmware, or if in any doubt.

## Before installing the Flight Controller in your craft

Please consult the Cli.md, Controls.md, Failsafe.md and Modes.md, documentations for further important information and familiarisation with CleanFlight's terminolgies.

You are highly advised to use the Receiver tab in CleanFlight Configurator, making sure your Rx channel values are centered at 1500 (1520 for Futaba RC) and Min/Max values are reached when controls are operated.
The referenced values for each channel, have marked impact on the operation of Flight Controller and its Flight Modes.

You may have to adjust your channel endpoints and trims/sub-trims, on your RC transmitter, to achieve the expected values.

## Feature MOTOR_STOP
The default Cleanflight configuration has the MOTOR_STOP feature DISABLED by default. What this means is that as soon as the controller is armed, the propellers *WILL* begin spinning at low speed. It is recommended that this setting be retained as it provides a good visual indication that the craft is armed. You can read more about arming and setting the MOTOR_STOP feature if desired in the relevant sections of the manual. 
