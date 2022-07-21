
# General Scheme
[Magic Number][Packet ID][Packet Type][Data]

Magic Number is just 42
Packet ID increment individually on each side and rolls over to 0 when going over 255

## Gateway -> Water System
Packet ID | Name | Anatomy | Description
1 | Add Water to Irrigate | [Liters 2 bytes] [Auth Response 32 bytes] |
2 | Cancel Irrigation | [Auth Response 4 bytes] |
? | Request Params | [Params ID] |
250 | Ask for Auth challange | No Data |
255 | Status/Broadcast ACK | [Packet ID] |

## Water System -> Gateway
Packet ID | Name | Anatomy | Description
0 | System Status | [8 bit status] | Used to broadcast system state like STATUS_IDLE. Left 4 bits are system state, right 4 ones are more info (for ex in idle: alread watered, turned off, etc)
1 | Watering State | [2 byte liters left] | broadcasts watering state if currently watering
250 | Auth challange | [Auth challange 16 bytes] |
253 | Commands disabled | No Data |
254 | Command Not Authenticated | No Data |
255 | Command OK | No Data |

## Ground Sensor -> Water System
**JUST AN IDEA. PROBABLY WON'T HAPPEN**
Packet ID | Name | Anatomy | Description
100 | Ground Data | [Wetness 1 byte] | ground wetness is 0%-100% in 0.5% steps (so 0-200 in the wetness byte)

## broadcast
 * Broadcast packets are removed from queue after they get acknowledged by the gateway or after 5 retransmits (spaced 5 seconds apart)
 * 

## (probably insecure) Authentication
 0. User saves 16 byte password from Water Systems Serial Console into the Gateway and requests an action.
 1. Gateway attempts up to 5 times to request an auth challange
 2. Water System respondes with a 16 byte random value as a challange.
 3. Gateway appends the 16 byte password to the 16 byte random challange and puts the result into sha-256
 4. Gateway transmits the command id and parameters followed by the 32 bytes of the sha-256 hash
 5. arduino performs the same sha256 on challange value + security key and checks if they are identical
 6. command is accepted or rejected depending on hashes matching or not