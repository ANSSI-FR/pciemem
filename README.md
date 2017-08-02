# PCIemem
Linux kernel module for driving an USB3380 board, exposing a `/dev/pciemem`
device node on the analysis machine representing the physical memory of the
machine under test.

See also [PCIleech](https://github.com/ufrisk/pcileech) from Ulf Frisk.

## uflash

uflash is a userland tool used to read or flash the EEPROM on a PCI connected USB 3380.
