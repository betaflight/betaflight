# Board - PYRODRONEF4

The PYRODRONEF4 target is new board developed by http://pirofliprc.com/

* flat bottom.(No component on the bottom)
* 6 uarts with 4 motor outputs(D-shot supported)
* uart 1 inverted for sbus(selectable) , uart 3 with inverter for frsky telemetry
* onboard 5V bec.
* direct solder pad layout for 4 IN 1 ESC or use 4 in 1 ESC Plug
* With uart RX pin layout inside the 4 IN 1 ESC plug for use ESC telemetry.
* USB -VCP

## Serial Ports

| Value | Identifier   | RX   | TX   | Notes                                                                                       |
| ----- | ------------ | -----| -----| ------------------------------------------------------------------------------------------- |
| 1     | USART1       | PA10 |  PA9 |  |
| 2     | USART2       | PA3  |  PA2 |                                                                                             |
| 3     | USART3       | PB11 |  PB10|                                                                                             |
| 4     | USART4       | PA1  |  PA0 |                                                                                             |
| 6     | USART5       | PD2  |  PC12|                                                                                             |
| 6     | USART6       | PC7  |  PC6 |                                                                                             |

## Pinout
TX1, RX1  -> UART1  sbus pad       (RX1 Use for Sbus has inverter build in)
TX2, RX2  -> UART2  Esc Telemetry  (use for esc telemetery)
TX3, RX3  -> UART3	TEL PAD	       (use for frsky temeletery has inverter build in)
TX4, RX4  -> UART4                 (Free uart)
TX5, RX5  -> UART5	TX5			   (TX5 use for smart audio, No RX5 pinout )
TX6, RX6  -> UART5  			   (design to use crossfire rx)

## board label
vtx_+       VTX power (Vbat)
vtx_-		VTX ground
video		VTX Signal
current     Current sensor input
bb+         Buzzer +
bb-			Buzzer -
m1			Motor 1
m2			Motor 2
m3          Motor 3
m4          Motor 4
Vbat        Battery input positive
Gnd			Battery input negative
Cam_C		Camera control(with build in Resistor and capacitor)
5V			5v output
led_s		Led strip signal
