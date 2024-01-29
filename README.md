# Parallel EEPROM Programmer
Hardware programmer/extractor firmware for parallel EEPROMS, such as the Atmel AT29C020, written for the NXP LPCXpresso51U68 development board.

All the pins used are 5v-tolerant, so a 5v supply can be used to power the EEPROM, which is often necessary for this kind of EEPROM.

Firmware includes hardware validation and sector retention on partial sector writes.

This firmware was written to be used with the host software [eeprog][eeprog].

## Building
Building this requires an ARM GCC version that supports `-std=gnu++20` or `-std=gnu++2a`, which should be available in any GCC versions 8 and above. If your distribution's version is too old, you can download an up-to-date toolchain from the [Arm GNU Toolchain][armgnu] website. Alternatively, you can use the toolchain included with [NXP MCUXpresso IDE][mcuxpressoide].

To build:
* Update the `TOOLCHAINDIR` variable in the Makefile to point to the location of your toolchain. This can be blank if you're using a package from your distribution. The [MCUXpresso IDE][mcuxpressoide] toolchain is usually at `/usr/local/mcuxpressoide-<version>/ide/plugins/com.nxp.mcuxpresso.tools.bin.linux_<version>/tools/bin`. You may have to add the NXP include directories if you use this method (these are commented in the Makefile).
* Run `make`.

## Deploying
There are several ways to flash the target microcontroller. Modify the Makefile so that the `install` recipe uses the desired method, either `install-jlink`, `install-cmsis`, `install-redlink` or `install-file`, or you can invoke the `install-<method>` recipe directly. The options are explained below.

### Preparation
If you're using `install-jlink` or `install-cmsis`, you may need to prepare the LPC-Link2 probe on your development board.
* Install [LPCScrypt][lpcscrypt] and `dfu-util`.
* Fix a bug in `lpcscrypt/scripts/dfu_boot`:
  * In some versions of `dfu-util`, the device VID/PID is reported with hex prefix `0x`. The `dfu_boot` script tries to catch this based on version number, but may be inaccurate. The offending line is `[ "${dfu_ver::3}" = "0.1" ] && vp_hex="0x"`, which can be commented out. For simplicity, you can run `sudo sed -i 's/\[ \"\${dfu_ver/#[ "\${dfu_ver/' /usr/local/lpcscrypt/scripts/dfu_boot`.
* For J-Link, obtain the latest [LPC-Link2 firmware from SEGGER][link2-jlink].
* Add yourself to the `dialout` group: `sudo usermod -aG dialout $USER`. You will need to log out and log back in for this to take effect. (Or log in inside of your current login with `su $USER`.)
* Ensure the DFULink jumper (JP7) is fitted and power-cycle the board. (Note that the Link2 probe does not reset when you press the Reset button, so power cycling is required)
* Run `lpcscrypt/scripts/program_CMSIS` or `lpcscrypt/script/program_JLINK`
* Remove the DFULink jumper (JP7) and power cycle the board.

### `install-jlink`
* See above.
* Install the [SEGGER J-Link Software][jlink].
* run `make install`

### `install-cmsis`
TBA.

### `install-redlink`
Run `make bootlink` before deploying. Make sure the `MCUXPRESSOTOOLS` variable in the Makefile points to the right location.

### `install-file`
Installs the firmware by using the target microcontroller as a USB Mass Storage device.
* Plug the USB cable into the Target USB connector (J5) while holding the ISP0 switch (SW2).
* Or if the USB cable is already plugged in, press RESET (SW4) while holding the ISP0 switch (SW2).
* Run `make install`
* Press RESET (SW4) to run the firmware.

[eeprog]: https://www.github.com/c4ashley/eeprog
[armgnu]: https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain
[mcuxpressoide]: https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE
[jlink]: https://www.segger.com/downloads/jlink/
[link2-jlink]: https://www.segger.com/downloads/jlink/#LPC-Link2
[lpcscrypt]: https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/lpc4300-arm-cortex-m4-m0/lpcscrypt-v2-1-2:LPCSCRYPT
