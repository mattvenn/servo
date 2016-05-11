# Servo testing

![large move, no i](400.png)

![small move, no i](20.png)

![large move, with i](400 with i.png)

![small move, with i](20 with i.png)

## BOM

* 12v DC brushed motor with 80:1 gearbox
* 400 ppr encoder (1600 cpr) mounted on output of motor's gearbox
* L298 H bridge
* ATMEGA328p

## Firmware

* PID loop running at 100Hz
* PWM running at 32kHz

