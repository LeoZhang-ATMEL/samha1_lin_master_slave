# How to Using the Demo
Those demos projects present how to use LIN Master/Slave Stack for 32 bits MCU.
For using those projects, there need
[SAM HA1G16A Xplained Pro](https://www.microchip.com/DevelopmentTools/ProductDetails/PartNO/ATSAMHA1G16A-XPRO) or Custom Board

# Demo Wired Connection
* SERCOM0, LIN Master - Motor/LED
* SERCOM1, LIN Slave - VCU
* SERCOM5, LIN Slave - BCM/SBC

1. ATSAMHA1-XPRO Local Loop back

|SAMHA1-XPRO - EXT1 |SAMHA1-XPRO - EXT1 |
|-----|-----|-----|
|EXT1-PA04(SERCOM0-PAD0) TX | EXT1-PA19(SERCOM1-PAD3) RX|
|EXT1-PA05(SERCOM0-PAD1) RX | EXT1-PA18(SERCOM1-PAD2) TX|
| INTERNAL-PB30 (SERCOM5-PA0) TX | |
| INTERNAL-PB23 (SERCOM5-PA3) RX | |
| PA00(LED0) ||

|Custom Board |Custom Board |
|-----|-----|-----|
|EXT1-PA04(SERCOM0-PAD0) TX | PA02(SERCOM1-PA02) RX|
|EXT1-PA05(SERCOM0-PAD1) RX | PA01(SERCOM1-PA01) TX|
| INTERNAL-PB30 (SERCOM5-PAD0) TX | |
| INTERNAL-PB23 (SERCOM5-PAD3) RX | |
| PA18(LED0) ||

```
