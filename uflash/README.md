# uflash

uflash is a userland utility to dump and flash the content of an USB3380 board
EEPROM, which contains the configuration for the USB3380 chip.

The EEPROM is accessed through registers mapped to the PCI configuration space,
so the USB3380 board needs to be plugged on a PCIe slot of the analysis machine,
and not on a USB port, like when using PCIemem or PCIleech. 

`uflash` has two commands:

- `dump` to read the EEPROM content and write it to a local `file`
- `flash` for writing the EEPROM using a local `file`
