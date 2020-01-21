#!/bin/sh
set -e
riscv64-linux-gnu-gcc -march=rv32i -Wall -O0 -ffreestanding -mabi=ilp32 -nostdlib -Wl,-T,firmware.lds firmware.c -o firmware.elf
riscv64-linux-gnu-objcopy -O binary firmware.elf firmware.bin
od -t x4 -A n --endian=little firmware.bin  > firmware.hex
yosys run_yosys.ys
edif2ngd example.edif
ngdbuild example -uc uart.ucf -p xc6slx9csg324-3
map -w example
par -w example.ncd example_par.ncd
bitgen -w example_par.ncd -g StartupClk:JTAGClk
