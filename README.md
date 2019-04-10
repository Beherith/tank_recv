# tank_recv
RC Tank receiver and motor driver module for knockoff Tiger Tank

Requires some libraries:

http://forum.arduino.cc/index.php?topic=16612#msg121031

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h" //for the NRF library

# Power information:
The built in batteries (8x2400 mAh NiMH) provide a stable 5v supply via a DC/DC buck converter to the raspi power connector.
The USB cable of the tank can be plugged into the raspi, or into a PC for debugging/testing.
Any configuration of supplying power to the raspi and the controller and the motors should work without complications. If you smell smoke, unplug everything.

The motors can only be powered from the battery, by turning on the power switch on the bottom. No other source can power them.
The battery can only be charged by plugging in any 12v adapter to the bottom, and turning the power switch off. Charge time is 4-8 hours. Do not leave the charger plugged in for more than 24 hours. 

# DO NOT:
Run the tracks forward and backward rapidly. This will destroy the gearboxes.


# Required libraries:
My fork of the gyro driver TODO
My fork of the software spi driver TODO

# Command protocol:

Send Commands over serial to the tank. You can send multiple commands, separated by ';' characters. Commands are processed on '\n' newline characters.
e.g.: "LL:50;RR:-50;LA:1\n" means turn right slowly, and turn on the laser


## Timeout
All motion (turret and track) and lights time out and are set to 0 after the last command times out

#### LL:[-255;255]
Make the LEFT track go forward [1;255] or backward [-1; -255] or stop [0]

#### RR:[-255;255]
Make the RIGHT track go forward [1;255] or backward [-1; -255] or stop [0]

#### LF:[0;255]
Set the PWM of the left track forward driver to [0;255]

#### LB:[0;255]
Set the PWM of the left track backward driver to [0;255]

### RF:[0;255]
Set the PWM of the right track forward driver to [0;255]

#### RB:[0;255]
Set the PWM of the right track backward driver to [0;255]


#### TL:[0;255]
Turn the turret left at speed [0;255]

#### TR:[0;255]
Turn the turret right at speed [0;255]

**Note: The turret left-right has hardware endstops. No damage will be done on reaching the left and right endstops, and the turret will stop turning

#### TU:[0;255]
Turn the turret up/down at speed [0;255]
#### TD:[0;255]
Turn the turret down/up at speed [0;255]

**Note: The turret up down is circular. After going down fully, it will start to go up. It is your job to calibrate this

#### FP:[0;1]
Fire the cannon! This will fire the cannon in approx 500 ms. Sending an FP:1 command is the way to fire, as the system checks wether the cannon has returned to it's default position, and consumes the bit from the status register

#### LA:[0;1]
Turn off/on the laser

#### LI:[0;1]
Turn the lights off/on

#### GY[0;255]
Set the gyro update rate to 10 x [1;255] ms. Setting it to 0 prints the gyro status, then stops updates. Send "GY:0" commands repeatedly to 'query' the gyro. The gyro returns the following packet:
sprintf(gyrobuf, "GYRO:\tDT=%lu\tAX=%i\tAY=%i\tAZ=%i\tGX=%i\tGY=%i\tGZ=%i\tMX=%i\tMY=%i\tMZ=%i\tTDC=%i",millis(),
	mpu.ax, mpu.ay, mpu.az, mpu.gx, mpu.gy, mpu.gz, mpu.mx, mpu.my, mpu.mz, gyrotemperature);
 Units are the following: 
 	mpu.set_acc_scale(scale_4g); //+- 4g scaled to an int16
	mpu.set_gyro_scale(scale_500dps); //+- 500 degrees per second scaled to an int16

#### DB:[0;1]
Turn debug output off/on. 

#### RA:[0;1]
Turn radio off/on. Dont turn it on, its broken (TM)

#### CH[0;127]
Set radio channel to [0;127]

#### TO:[1;255]
Set command timeout to 10x[1;255] milliseconds. 
