<snippet>
  <content>
# Encrypted LoRa Communication - Gateway

This repository includes code a Raspberry Pi Model B.
The Raspberry Pi (Gateway). The LoRa commincation parameters are adjustable. Default settings are LoRa Frequency 868,1 MHz, Spreading Factor SF7 and Coding Rate 4/5. LoRa Communication is possible encrypted and without encryption. Right now the encrypted LoRa payload is limited to 16 Bytes.

## Usage
The code should demostrate a way of using the LoRa Radio Technology for sending data encrypted. Also it is a fast introduction to the LoRa Technology. 

## Goal
The goal was to develop a Node to Gateway encrypted communiction with the LoRa Radio technolagy. (Not LoRaWan)


## Software Dependencies 
The code requires the following software libraries.

### for the Gateway
* enable SPI -> check in  ```$ raspi-config``` on RPi 
* WiringPi: to access GPIO -> install ```$ sudo apt-get install wiringpi```
* start program as root ``$ sudo ./main``

## Setup and Installation

### for the Gateway
1. Clone this repository to your Raspberry Pi ```git clone https://github.com/rokigeorg/encrypted-LoRa-Communication.git ``` 
2. Wire up the Raspberry Pi with the second RFM95 LoRa Modul (see section Hardware Wiring)
3. On the command line in our repository folder  ``$ cd HopeRF_LoRa_Receiver``
4. Compile the code via the Makefile ``$ make``
5. Run the LoRa Radio Receiver ``$ sudo ./main -f 868100000 -sf 7 -cr 5 -bw 125`` 

Please note the adjustable arguments 
```
-f 868100000
 ``` 
 specifizes the frequency
 ```
 -sf 7
 ``` 
 specifizes the spreadig factor SF7
  ```
 -cr 5
 ``` 
 specifizes the coding rate 4/5
 
   ```
  -bw 125
  ``` 
specifizes the bandwidth 125 kHz

All parameters can be adjusted. The given values are only the default values.

## Hardware 
The code functions with the following hardware components.
* 2 x HopeRF RFM95 - Low Power Long Range Transceiver Module V1.0 
* Raspberry Pi - Model B
* Arduino Uno

### Hardware Wiring - Node and Gateway

#### RFM95 LoRa Modul - Arduino Uno
* 3.3V - 3.3V
* GND - GND 
* MISO - Digital 12  
* MOSI - Digital 11  
* SCK - Digital 13  
* NSS - Digital 10  
* DIO0 - Digital 3 
* RST - Digital 9 

#### RFM95 LoRa Modul - Raspbery Pi Model B
* 3.3V - 3.3V (header pin #1) 
* GND - GND (pin #6) 
* MISO - MOSI (pin #21) 
* MOSI - MISO (pin #19) 
* SCK - CLK (pin #23) 
* NSS - GPIO6 (pin #22) 
* DIO0 - GPIO7 (pin #7) 
* RST - GPIO0 (pin #11)


## Contributing
You can help to improve the the code. 
If you test this code and find any mistakes or have suggestions to improve the code or the instrctions in the README feel free to do the following steps. Also you can send me an e-mail if you have any questions. 

1. Fork it! Click the button in the top right
2. Now Repository is forked in your github 
3. Clone your fork `git clone <your/git/ssh/PATH>`
3. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D


## In progress
The following features need to be implemeted
* forward payload to server

## History
For my bachelor thesis I try to build a Low-cost-Gateway for a LoRa Network. Currently I am supported by the Labb - Laboratory for Biosignal Processing in Leipzig Germany. 
## Credits
This repository consists partly of coped code from the following People.
* Thomas Telkamp thomas@telkamp.eu creater of the [single Channel LoRaWan Gateway](https://github.com/tftelkamp/single_chan_pkt_fwd)
* [RadioHead library](http://www.airspayce.com/mikem/arduino/RadioHead/)

## License
TODO: Write license
</content>
  <tabTrigger>readme</tabTrigger>
</snippet>