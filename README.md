You'll need PlatformIO to initialize and build the project.
It can be built manually but path configuration and toolchain installation is on you.
Libraries used are modified and are not same as the ones available somewhere else, mainly touch and LCD drive.

With current pin configuration, there is support for:

-touchscreen
-SDCARD
-EEPROM

EEPROM and SDCARD share the same SPI but have different CS pins. currently theres only support for them but saving to file is on todo list.
