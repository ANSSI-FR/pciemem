# README

[![img](https://img.shields.io/badge/code.gouv.fr-ouvert-mediumseagreen.svg)](https://code.gouv.fr/documentation/#quels-degres-douverture-pour-les-codes-sources)

## Introduction

The main code in this repository is a kernel module intended to communicate
over USB with an USB3380 device in order to generate PCIe transactions. By
design it requires elevated privileges on the host, and is intended to generate
arbitrary PCIe transactions on the target host.

This code is intended as a **testing tool** and should **never be used on a
production systems**. This tool can cause **unexpected behavior, system
crashes, data corruption, or security vulnerabilities**. Always perform testing
in a controlled, isolated environment to ensure the safety and stability of
production systems.

## PCIemem

Linux kernel module for driving an USB3380 board, exposing a `/dev/pciemem`
device node on the analysis machine representing the physical memory of the
machine under test.

See also [PCIleech](https://github.com/ufrisk/pcileech) from Ulf Frisk.

PCIemem is licensed under the [GPL-2 license](LICENSE-GPL-2.0) since it's a Linux
kernel module.

## uflash

uflash is a userland tool used to read or flash the EEPROM on a PCI connected
USB 3380. It is licensed under the [Apache license version
2.0](LICENSE-Apache-2.0).
