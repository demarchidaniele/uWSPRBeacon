# uWSPRBeacon
The purpose of this project is to create an Hamradio WSPR Beacon as small&inexpensive as possible, ...but with advanced features like an ethernet connection.

The hardware chosen for this project is an Arduino Nano (or a clone), an ethernet ENC28J60 shield, and AD9851 DDS.

Optionally you can add a GPS receiver and/or LCD display 16x2 characters.

The system need to sync the clock for the WSPR transmission.

If GPS is not installed (or not receive) the system will use the configured NTP Time Server.

The distinctive feature of this system is that it can be reached via Ethernet in order to monitor the status and setup the system.

IMPORTANT

Webpage need external references (JQuery, W3S Bootstrap, ...)

For this reason the PC that connect the system must be connected to the internet too.

Since the web page is coded into the arduino, Flash consumption is close to 95% of capacity, it will be hardly possible to add more features.

Anyway... it works.

--------------------------------------------------

Acknowlegements

The on-chip generation of the WSPR message algorithm is based on the work of Andy Talbot, G4JNT.

Some part of the WSPR code is derived from the Gene Marcus W3PM GM4YRE WSPR beacon project (Arduino UNO DDS-60/AD9850 WSPR/QRSS Controller)

The Gene Marcus's original sourcecode is mostly transformed to extremely reduce the FLASH needs.

--------------------------------------------------

Compilation:

Use Arduino IDE 1.6 or newer (please check it, expecially on Ubuntu)

Import these libraries before build

 MsTimer2 (http://playground.arduino.cc/Main/MsTimer2)

 Ethercard (https://github.com/jcw/ethercard)

 MicroNMEA (https://github.com/stevemarple/MicroNMEA)

--------------------------------------------------

List of materials

Mandatory:

ATmega328P Nano V3 Controller Board: around 8€

ENC28J60 Ethernet Shield Network Module V1.0: around 8€

AD9851 DDS Signal Generator Module: around 15€

Optional:

Display 16*2 characters: around 3€

Beitian BN-880 GPS receiver: around 15€

--------------------------------------------------

  Arduino Nano Digital Pin Allocation
  
  D0
  
  D1
  
  D2  
  
  D3
  
  D4  
  
  D5  Serial Tx -> GPS-Rx (Needed by the serial lib but not used)
  
  D6  DDS9851 Data
  
  D7  DDS9851 Load
  
  D8  DDS9851 Clock
  
  D9  Serial Rx <- GPS-Tx
  
  D10 
  
  D11
  
  D12 
  
  D13 TX (LED) - PTT
  
  D14 LCD D7
  
  D15 LCD D6
  
  D16 LCD D5
  
  D17 LCD D4
  
  D18 LCD enable
  
  D19 LCD RS
