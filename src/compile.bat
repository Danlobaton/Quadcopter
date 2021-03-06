@echo off
mkdir cmm > nul 2>&1
call propeller-elf-gcc -I lib/Motor/libservo -L lib/Motor/libservo/cmm/ -I lib/Protocol/libsimplei2c -L lib/Protocol/libsimplei2c/cmm/ -I lib/Utility/libsimpletools -L lib/Utility/libsimpletools/cmm/ -I lib/TextDevices/libsimpletext -L lib/TextDevices/libsimpletext/cmm/ -o cmm/quadcopter.elf -Os -mcmm -m32bit-doubles -std=c99 quadcopter.h quadcopter.c i2chelper.c imu.c xbee.c motor.c range.c -lm -lservo -lsimplei2c -lsimpletools -lsimpletext
call propeller-elf-objdump -h cmm/quadcopter.elf > nul
call propeller-load -r cmm/quadcopter.elf
