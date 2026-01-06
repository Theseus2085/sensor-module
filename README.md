Sensor module on NUCLEO-F446RE based on STM32

## Upload (Windows)

This project is configured to upload via the NUCLEO on-board **ST-LINK**.

If upload fails with OpenOCD errors like:

- `Error: libusb_open() failed with LIBUSB_ERROR_NOT_FOUND`

check Windows Device Manager. If **ST-Link Debug** shows an error (often **Code 28: drivers not installed**), install a USB driver for that interface.

Typical fixes:

- Install STM32CubeProgrammer (includes ST-LINK drivers), then reconnect the board.
- Or use Zadig to install a **WinUSB** driver for the **ST-Link Debug** interface.

After the driver is installed, rerun:

- `pio run -t upload`
