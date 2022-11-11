# tsip-code
Some C code for parsing Trimble GPS TSIP serial data.
Currently decoding only a few select TSIP messages.
Feel free to add decoding for more. :)


Compile:
[]$ make readTSIP
cc     readTSIP.c   -o readTSIP

Run:
[]$ ./readTSIP -b 38400 -n -p /dev/ttyUSB0
Opening port /dev/ttyUSB0 ...
Looking for next message...
Received 16 byte message: [10 54 c9 11 71 ca c8 ed 22 59 48 da 4d 60 10 03 ]
Packet ID 0x54 One Satellite Bias and Bias Rate Report
One satellite bias: -595740.62 m
Clock bias rate: -485650.78 m/s
GPS Time of position fix: 447083.00 s

Looking for next message...
Received 24 byte message: [10 56 00 00 00 00 00 00 00 00 00 00 00 00 c8 ed 22 59 48 da 4d 60 10 03 ]
Packet ID 0x56 Velocity Fix, East-North-Up (ENU)

Looking for next message...
Received 14 byte message: [10 41 48 da 4d 62 08 bb 41 90 00 00 10 03 ]
Packet ID 0x41 GPS Time
GPS time of week: 447083.06 s
Extended GPS week number: 2235 weeks
GPS UTC offset: 18.0 s

Looking for next message...
Received 6 byte message: [10 46 bb 10 10 03 ]
Packet ID 0x46 Health of Receiver
Status: 0xbb, Over Determined Mode
Battery backup: OK
Antenna feedline: Open detected

...
