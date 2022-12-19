# ESP32CoffeePID - 20221219 latest version

Simple ESP32 based PID controller for espresso machines. Adjustable over wifi using the ESP32 webserver functionality.

*Inspired by Gagguino; but using the ESP32 platform. https://gaggiuino.github.io/#/ | Go there, build that, be awesome.*

**[WORK IN PROGRESS]** - *Entering testing in the Gaggia Classic shortly*

*Will be installed into a Gaggia Classic to control boiler temperature for brew (initially).*

  - <PID_V1.h> providing the PID control for the temperature, from a MAX6675 thermocouple driver.
  - PID setpoint value stored during power-down using <SPIFFS.h>
  - M4 k-type thermocouple will replace the 'brew' thermo-switch on the side of the boiler 
  - The heater will be controlled via a 40A DA Solid State Relay.

Option to have details displayed on a TFT using <TFT_eSPI.h> and controlled via 'increase / decrease' buttons using <Button2.h>
Can be used headless (no TFT) if desired. Just use the code, and or web interface to set the PID target. Downsides, no realtime update on boiler current temperature (Add to WIP)

**Notes:**

*I am a begginer, be gentle*

- Original Prototyping on a TTGO T-display (20221216 file);
- Moved to an ESP32 DEVKIT board and SPI 2.0" TFT PANEL for final packaging reasons. (20221207 file);
- Moved to an AP-Wifi vs. station mode on the ESP32 for simplification;
- Webserver root is configured on 192.168.1.1 and allows the user to change the PID target temp.

TFT_eSPI will need to be configured for your boards/screen; as will the SPI pins for which ever thermocouple driver you are using.

Once it's installed, I will update with wiring suggestions. 

**Future dreams:**

- Increase the functionality in include pump & pressure control. Inspired by gagguino, but using the native benefits of ESP32;
- Improve Web-GUI and TFT presentation, and include near realtime updates for the boiler temp, as well as sting target; to improve headless use cases;
- Add a shot timer using optocoupler detection for switch press, or the like;
- Explore multiple PID options/modes (fuzzy etc) for flexibility;
- I may get inspired and design a PCB / 3D printed housing; to improve packaging

**Bill of materials: (what i used for v1.0)**

- ESP32 Devkit board (you'll need to SPI-bus, so having a large number of exposed GPIO pins is a good idea;
- SPI TFT (compatible with tft_eSPI (i am using a 2.0" RGB LCD on the ST7789 driver platform);
- MAX6675 thermocouple driver | K-Type Thermocouple matching your machine. Im using an M4 model, which fits in place of the boiler thermal switch on the Gaggia;
- 40A DA (DC control, AC switching) Sold State Relay (SSR) | Read the manufactures mounting requirements, these need a heatsink or to be thermally mounted to a metallic surface (else magic smoke will happen);
- AC-DC 5V/10W power supply to power the ESP32 board and peripherals.

https://www.aliexpress.com/item/1005004237117445.html?spm=a2g0o.order_list.order_list_main.16.32bb1802EtOHys
https://www.aliexpress.com/item/32835036293.html?spm=a2g0o.order_list.order_list_main.4.15291802JzWv0Z
https://www.aliexpress.com/item/1005004268911484.html?spm=a2g0o.order_list.order_list_main.16.36901802NZTg8b
https://www.aliexpress.com/item/1005004268911484.html?spm=a2g0o.order_list.order_list_main.16.36901802NZTg8b
https://www.aliexpress.com/item/1005001626749980.html?spm=a2g0o.order_list.order_list_main.160.36901802NZTg8b

**Wiring diagram / suggestions (coming soon)**
