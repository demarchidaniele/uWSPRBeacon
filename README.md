# uWSPRBeacon
The purpose of this project is to create a minimal and very cheap system for creatining a beacon WSPR.

The hardware chosen for this project is an Arduino Nano (or a clone), an ethernet expansion, and AD9851 DDS.
Optionally you can add a GPS receiver and an LCD display 16x2 characters.

The distinctive feature of this system is that it can be reached via Ethernet in order to monitor the status and setup the system.

Most of the WSPR code is derived from the Gene Marcus W3PM GM4YRE WSPR beacon project (Arduino UNO DDS-60/AD9850 WSPR/QRSS Controller)
The sourcecode is mostly transformed to free as mush as FLASH space.
The Flash memory is currently close to 95% of capacity, it will be hardly possible to add more features.
Anyway... it works.

List of materials
Mandatory
ATmega328P Nano V3 Controller Board: around 8€
ENC28J60 Ethernet Shield Network Module V1.0: around 8€
AD9851 DDS Signal Generator Module: around 15€

Optional
Display 16*2 characters: around 3€
Beitian BN-880 GPS receiver: around 15€

The system need to sync the clock for the WSPR transmission
If GPS is not installed (or not receive) the system try to use a Time Server.
