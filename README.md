# ESP32CoffeePID
Simple ESP32 based PID controller for espresso machines. Adjustable over wifi using the ESP32 webserver functionality

[WORK IN PROGRESS] 

Intended to be installed into a Gaggia Classic to control boiler temperature for brew (initially). 

<PID_V1.h> providing the PID control for the temperature, from a MAX6675 thermocouple driver.
  - PID setpoint value stored during power-down using <SPIFFS.h>
  - M4 k-type thermocouple will replace the 'brew' thermo-switch on the side of the boiler 
  - The heater will be controlled via a 40A DA Solid State Relay.

Option to have details displayed on a TFT using <TFT_eSPI.h> and controlled via 'increase / decrease' buttons using <Button2.h>

Notes:

I am a begginer, be gentle

- Original Prototyping on a TTGO T-display (20221216 file). 
- Moved to an ESP32 DEVKIT board and SPI 2.0" TFT PANEL for final packaging reasons. (20221207 file).

TFT_eSPI will need to be configured for your boards/screen; as will the SPI pins for which ever thermocouple driver you are using.

Once it's installed, I will update with wiring suggestions. 

Problems:

Add an NTPClient to log date/time of brews (eventually), amongst other future functionality

Future dreams:

- Increase the functionality in include pump & pressure control. Inspired by gagguino, but using the native benefits of ESP32.
- Improve Web-GUI and TFT presenttation. Currently rough and ready works for me.
- Add a shot timer using optocoupler detection of for the switch press, or the like.
- Explore multiple PID options for flexibility
- I may get inspired and design a PCB / 3D printed housing etc.
