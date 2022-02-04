# STM32F4 Firmware
A collection of projects written for the STM32F4 microcontroller.


## Organization
```
stm32f4                Root directory
├── common             Interface API, sensor drivers, math library, etc.
├── lib                Stripped-down copy of the STM32FXX_DSP_STDPERIPH_LIB_V1.2.1
├── prj                Projects
├── test               Test projects
└── tools              Debug tool configurations
```

## Prerequisites
### Compiler
```shell
sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi
```

Note: All the code in this repository was compiled and tested on `gcc-arm-none-eabi 4.9.3`.

### Programmer
For flashing the firmware, you'll need [OpenOCD](http://openocd.org/) 0.10.0, which (at the time of writing) you will need to compile from source:

```shell
autoreconf -fi
./configure
```

Make sure that the output looks something like this:
```shell
OpenOCD configuration summary
--------------------------------------------------
MPSSE mode of FTDI based devices        yes (auto)
ST-Link JTAG Programmer                 yes (auto) <----- My JTAG programmer
TI ICDI JTAG Programmer                 yes (auto)
Keil ULINK JTAG Programmer              yes (auto)
Altera USB-Blaster II Compatible        yes (auto)
Versaloon-Link JTAG Programmer          yes (auto)
Segger J-Link JTAG Programmer           yes (auto)
OSBDM (JTAG only) Programmer            yes (auto)
eStick/opendous JTAG Programmer         yes (auto)
Andes JTAG Programmer                   yes (auto)
USBProg JTAG Programmer                 no
Raisonance RLink JTAG Programmer        no
Olimex ARM-JTAG-EW Programmer           no
CMSIS-DAP Compliant Debugger            no
```

If instead it looks like this:
```shell
OpenOCD configuration summary
--------------------------------------------------
MPSSE mode of FTDI based devices        no
ST-Link JTAG Programmer                 no
TI ICDI JTAG Programmer                 no
Keil ULINK JTAG Programmer              no
Altera USB-Blaster II Compatible        no
Versaloon-Link JTAG Programmer          no
Segger J-Link JTAG Programmer           no
OSBDM (JTAG only) Programmer            no
eStick/opendous JTAG Programmer         no
Andes JTAG Programmer                   no
USBProg JTAG Programmer                 no
Raisonance RLink JTAG Programmer        no
Olimex ARM-JTAG-EW Programmer           no
CMSIS-DAP Compliant Debugger            no
```

you most likely need to install `libusb-1.0-0-dev`:
```shell
sudo apt-get install libusb-1.0-0-dev
```

If this does not fix the problem:
```
./configure --enable-stlink
```

This should print out the reason why the programmer is not enabled.

Then make and install:
```shell
make
sudo make install
```

Add yourself to the dialout group and reboot so that you don't need to be a root user for using the programmer:
```shell
sudo adduser <user-name> dialout
sudo reboot
```

#### For ST-Link JTAG Programmers
1. Create a file called 50-embedded.rules in the folder: `/etc/udev/rules.d`
2. In this file, add the following: 
```txt
#STLINK v2
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", GROUP="<group-name>", MODE="666"
```

where `<group-name>` is the group that your account belongs to. 


## Build and Flash Firware
```shell
cd prj/autopilot
make
make flash
```


## Related Work
The firmware in these projects were written for the PCBs in the [stm32f4-hardware repository](https://github.com/r-oung/stm32f4-hardware).


## Licenses
Each file includes a license notice.

All projects here use the [STM32F4 DSP and standard peripherals library](https://www.st.com/en/embedded-software/stsw-stm32065.html), which is licensed under [MCD-ST Liberty SW License Agreement V2](http://www.st.com/software_license_agreement_liberty_v2). Most of this code can be found in the `lib` folder.

The remaining code, unless otherwise indicated, is licensed under [MIT License](https://opensource.org/licenses/MIT).
