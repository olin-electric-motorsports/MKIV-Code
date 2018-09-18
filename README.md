# MKIV-Code
MKIV-Code

## Getting Libraries
First run the `setup.sh` script to get all the defualt AVR libraries.
```bash
$ sudo bash setup.sh
```

Then you need to add an additional AVR library for the specific MCU we use (ATmega16m1)
```bash
$ bash getDir.sh
```

## Compiling and Flashing Boards
To flash your C code onto an ATmega, you need to compile that code into a certain type that the ATmega can read. Sounds compilcated! Luckily, we have a nice series of scripts that does it all for you!
```bash
$ python3 make.py
```
Then just enter what board you want to flash and select if you want to flash it or just compile it.

That's it! How exciting!
