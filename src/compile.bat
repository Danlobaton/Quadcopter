mkdir cmm
call propeller-elf-gcc.exe -I . -L . -I "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Protocol/libsimplei2c" -L "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Protocol/libsimplei2c/cmm/" -I "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Utility/libsimpletools" -L "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Utility/libsimpletools/cmm/" -I "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/TextDevices/libsimpletext" -L "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/TextDevices/libsimpletext/cmm/" -Os -mcmm -m32bit-doubles -fno-exceptions -std=c99 -c imu.c -o cmm/imu.o
call propeller-elf-gcc.exe -I . -L . -I "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Protocol/libsimplei2c" -L "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Protocol/libsimplei2c/cmm/" -I "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Utility/libsimpletools" -L "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Utility/libsimpletools/cmm/" -I "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/TextDevices/libsimpletext" -L "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/TextDevices/libsimpletext/cmm/" -Os -mcmm -m32bit-doubles -fno-exceptions -std=c99 -c i2chelper.c -o cmm/i2chelper.o
call propeller-elf-gcc.exe -I . -L . -I "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Protocol/libsimplei2c" -L "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Protocol/libsimplei2c/cmm/" -I "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Utility/libsimpletools" -L "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Utility/libsimpletools/cmm/" -I "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/TextDevices/libsimpletext" -L "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/TextDevices/libsimpletext/cmm/" -Os -mcmm -m32bit-doubles -fno-exceptions -std=c99 -c xbee.c -o cmm/xbee.o
call propeller-elf-gcc.exe -I . -L . -I "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Protocol/libsimplei2c" -L "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Protocol/libsimplei2c/cmm/" -I "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Utility/libsimpletools" -L "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Utility/libsimpletools/cmm/" -I "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/TextDevices/libsimpletext" -L "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/TextDevices/libsimpletext/cmm/" -Os -mcmm -m32bit-doubles -fno-exceptions -std=c99 -c motor.c -o cmm/motor.o
call propeller-elf-gcc.exe -I . -L . -I "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Protocol/libsimplei2c" -L "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Protocol/libsimplei2c/cmm/" -I "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Utility/libsimpletools" -L "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/Utility/libsimpletools/cmm/" -I "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/TextDevices/libsimpletext" -L "C:/Users/Shreyas/Documents/SimpleIDE/Learn/Simple Libraries/TextDevices/libsimpletext/cmm/" -o cmm/quadcopter.elf -Os -mcmm -m32bit-doubles -fno-exceptions -std=c99 cmm/imu.o cmm/i2chelper.o cmm/xbee.o cmm/motor.o quadcopter.c -lm -lsimplei2c -lm -lsimplei2c -lsimpletools -lsimpletext -lm -lsimplei2c -lm -lsimplei2c -lsimpletools -lm -lsimplei2c -lm -lsimplei2c -lm -lsimplei2c -lm -lm -lsimplei2c -lm
call propeller-load -s cmm/quadcopter.elf
call propeller-elf-objdump -h cmm/quadcopter.elf