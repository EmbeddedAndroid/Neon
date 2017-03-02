# Neon

## Build Requirements

Make sure to have the GNU ARM Embedded Toolchain (https://launchpad.net/gcc-arm-embedded) available in your PATH.

## Building

```
$ make
```

Firmware should be available at *obj/x.bin*

## Flashing

Use stm32flash to flash Neon.

```
$ stm32flash -w x.bin -v -g 0x0 /dev/ttyAMA2
```
