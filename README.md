# NRF24LE1Flash
Linux command line tool to flash NRF24LE1 using serial lines.</br>
This program is a conversion of the GUI PC flasher http://www.diyembedded.com/bootloader/PC/nrf24lu1_bootloader_pc.zip to linux.</br>
Thanks to Brennen at diyembedded.com for writing the bootloader and GUI flasher and giving permission to use his code in this tool.</br>
Once the NRF24LE1 has the bootloader loaded, it can be programmed in situ using only the serial lines & reset</br>

## Prerequisites
To use this tool the NRF24LE1 chip has to be flashed with the bootloader.</br>
The bootloader can be downloaded from here http://www.diyembedded.com/bootloader/micro/nrf24le1/nrf24le1_chip_bootloader.zip</br>

I use https://github.com/DeanCording/nRF24LE1_Programmer to get the bootloader onto the NRF24LE1.</br>
Dean's programmer uses SPI to flash the NRF24LE1.</br>

## Compiling
```
make
```

## Usage

 Connect Power, Reset, RX and TX
 For the NRF24LE1 (32 PIN CHIP) Boards I use the connections are
 
 SERIAL | nRF24LE1(QFN32)
 --- | ---
 3.3V | VDD
 GDN | GND
 RX | P0.3
 TX | P0.4
 DTR (RTS or CTS) | RESET

```
Usage : nrf24le1flash -f hexfile -p port -b baudrate -r reset -b BL_enabled -c command
    -f [hexfile], path to hex file
    -p port, port device                    -default /dev/ttyUSB0
    -b baudrate, UART speed                 -default 38400
    -r reset, reset pin DTR, RTS or CTS     -default DTR
    -bl BL_enabled, DISABLED or DTR or RTS  -default DISBALED
    -c command, CHECK, FLASH                -default CHECK
``` 
>Reset and bootloader enable pins cannot be the same.

## Changelog

#### 1.1.1 04/19/2020
- Fix parsing hex file with linux line endings `\n`.

#### 1.1.0 09/14/2019
- Add ability to use CTS as a reset pin.
- Add makefile instead of CodeBlocks IDE stuff.