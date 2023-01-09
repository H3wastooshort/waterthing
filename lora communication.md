
# General Scheme
[Magic Number][Packet ID][Packet Type][Data]

Magic Number is just 42

Packet ID increment individually on each side and rolls over to 0 when going over 255

## Gateway -> Water System
| Packet Type | Name | Anatomy | Description / Comment |
| ----------- | ---- | ------- | ----------- |
| 0 | Current Time | [1 byte Packet ID of 230 time req] [UNIX Timestamp 32bit] | yes Y2K38, i'll just add the 60-something years when it rolls over to 1970 |
| 1 | Add Water to Irrigate | [1 byte Packet ID of challange] [Liters 2 bytes] [Auth Response 32 bytes] | |
| 2 | Cancel Irrigation | [1 byte Packet ID of challange] [Auth Response 32 bytes] | |
| 3 | Set Irrigation Timer | [1 byte Packet ID of challange] [Hour 1 byte] [Minute 1 byte] [Liters 2 bytes] [Auth Response 32 bytes] | |
| 241 | Reboot Notification | No Data | Packet sent once on boot |
| 250 | Ask for Auth challange | No Data | |
| 251 | Ping | | |
| 252 | Check Auth | [1 byte Packet ID of challange] [Auth Response 32 bytes] | Debug command to see if you have the right key without doing sth |
| 255 | ACK | [Packet ID] | |

## Water System -> Gateway
| Packet Type | Name | Anatomy | Description / Comment |
| ----------- | ---- | ------- | ----------- |
| 0 | System Status | [4+4 bit status] [2 byte fixed point battery voltage]| Used to broadcast system state like STATUS_IDLE. Left 4 bits are system state, right 4 ones are more info (for ex in idle: alread watered, turned off, etc). fixed point battery voltage is a uint16_t divided by 100. max value would be 65.535V |
| 1 | Watering State | [4+4 bit status] [2 byte unsigned int: liters left] [2 byte unsigned int: liters called] | broadcasts watering state if currently watering |
| 230 | Request Time | No Data | |
| 240 | Reboot Notification | No Data | Packet sent once on boot |
| 249 | ACK | [1 byte packet ID] | |
| 250 | Auth challange | [1 byte Packet ID of 250 ask] [Auth challange 16 bytes] | |
| 251 | No Challange | [1 byte Packet ID] | Ask for a Challange Fist |
| 253 | Commands disabled | [1 byte Packet ID] | |
| 254 | Command Not Authenticated | [1 byte Packet ID] | |
| 255 | Command OK | [1 byte Packet ID] |

## Ground Sensor -> Water System
**JUST AN IDEA. PROBABLY WON'T HAPPEN**
| Packet Type | Name | Anatomy | Description / Comment |
| ----------- | ---- | ------- | ----------- |
| 100 | Ground Data | [Sensor ID 1 byte] [Wetness 1 byte] | ground wetness is 0%-100% in 0.5% steps (so 0-200 in the wetness byte) |

## broadcast
 * Broadcast packets are removed from queue after they get acknowledged by the gateway or after 5 retransmits (spaced 5 seconds apart)
 * 

## system status
0bLLLLRRRR
### Left
| Bits | State |
| ---- | ----- |
| 0000 | Idle |
| 0001 | Pumping |
| 0010 | Emptying |
| 0011 | Afterdrain |
| 0100 | NO WATER |
| 0101 | LOW BATTERY |
| 0110 | NO TIME |
| 0111 | GENERAL FAIL |
### Right
#### Idle
| Bits | Extra |
| ---- | ----- |
| 0000 | Normal |
| 0001 | Off |
| 0010 | Done Today |
| 0011 | Rain |
#### General Fail
[] = 1 bit, each can be set individually

[Tank Sensors][RTC Missing][RTC Unset][Unused]

## (probably insecure) Authentication
 0. User saves 16 byte password from Water Systems Serial Console into the Gateway and requests an action.
 1. Gateway attempts up to 5 times to request an auth challange
 2. Water System respondes with a 16 byte random value as a challange.
 3. Gateway appends puts challange pass and packet data into sha-256 ([chal][pass][packet type][data] -> sha256)
 4. Gateway transmits the command id and parameters followed by the 32 bytes of the sha-256 hash
 5. arduino performs the same sha256 on challange value + security key and checks if they are identical
 6. command is accepted or rejected depending on hashes matching or not
 
