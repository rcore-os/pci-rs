# PCI Driver
A platform-agnostic PCI bus management and configuration access library forked from [robigalia/pci](https://gitlab.com/robigalia/pci).

## Support status

Supported features:
* PCI capabilities listing
* Access the configuration space with I/O functions for x86_64
* Access the configuration space with CAM and ECAM for RISCV
* x86_64 on Qemu is supported
* x86_64 on PC is supported
* RISCV on Qemu is supported
* RISCV fu740 board is supported

## Example

```
cd examples/riscv
make run
```
