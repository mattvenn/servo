# Servo testing

![large move, no i](400.png)

![small move, no i](20.png)

![large move, with i](400 with i.png)

![small move, with i](20 with i.png)


# Zeigler-Nichols method

https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method

![oscillation.png](oscillation.png)

* ku = 8
* tu = 64 ms

So for no overshoot:

* p = 4.8
* i = 0.032
* d = 0.008

![large move](ziegler 400.png)

![small move](ziegler 20.png)

## Alternative PID alg  + tuning while loaded

* ku = 7.5
* tu = 61 ms

* p = 2.47
* i = 0.03
* d = 0.02

![large move](ziegler2 400.png)

![small move](ziegler2 20.png)


## BOM

* 12v DC brushed motor with 80:1 gearbox
* 400 ppr encoder (1600 cpr) mounted on output of motor's gearbox
* L298 H bridge
* ATMEGA328p

## Firmware

* PID loop running at 100Hz
* PWM running at 32kHz

# PSU

* 12v, current limited at 0.5A.
* Typical draw while static is 0.1A
* PSU is not going switching to constant current while moving.
