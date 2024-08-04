You'll need PlatformIO to initialize and build the project.
It can be built manually but path configuration and toolchain installation is on you.
Libraries used are modified and are not same as the ones available somewhere else, mainly touch and LCD drive.
Pin config:

B0-B7 : LCD D0-D7 (Entire port B is dedicated to LCD data lines for faster communication)

PA8 : LCD RD | PA9 : LCD WR | PA10 : LCD RS | PA2 : LCD RST | PA3 : LCD CS

PB0 : TOUCH XP | PA1 : TOUCH XM | PA3 : TOUCH YP | PB1 : TOUCH YM

PA7 : SPI1 MOSI | PA6 : MISO | PA5: SCK | PA4 : EEPROM CS | PA1: SDCARD CS

PA0 : Analog Input

PC13 : Test Signal Generator



With current pin configuration, there is support for:

-touchscreen
-SDCARD
-EEPROM

EEPROM and SDCARD share the same SPI but have different CS pins. currently theres only support for them but saving to file is on todo list.
