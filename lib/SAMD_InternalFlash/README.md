# SAMD_InternalFlash
A library for using the internal flash of the SAMD21/SAMD51 MCU with the FatFS file system.

The SAMD21 MCU has 256KB of Flash (512KB Flash for the SAMD51), which is quite large. Usually the firmware occupies a small portion of it. The purpose of this library is to use the remaining space as a storage using the FatFS file system. This storage can be used for saving and reading files the same way it is done with an SD card. In addition, this storage can be exposed as an USB mass storage. The SAMD21 board when plugged into the USB port of a computer appears as an external flash drive. Files can then be copied to/from the internal flash storage. See the exemple https://github.com/Mollayo/SAMD_InternalFlash/tree/main/examples/Internal_Flash_with_USB_Mass_Storage.

This library is for the Arduino IDE. It has been tested on the Seeeduino Xiaomi, the Adafruit Feather M0 Wifi and the ItsyBitsy M4 Express but it should work with all boards based on the SAMD21 or SAMD51 MCU.

Dependencies:
- FlashStorage library: https://github.com/cmaglie/FlashStorage. This library should be downloaded from GitHub and not from the Arduino IDE. The one from the Arduino IDE is not the latest version although the version number is the same. And it contains a bug for writing on SAMD51 MCU.
- Adafruit SPI Flash library: https://github.com/adafruit/Adafruit_SPIFlash


