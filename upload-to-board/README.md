# Upload to board

## Software Prerequisites
- python3
- [SEGGER JLink](https://www.segger.com/downloads/jlink/)

## Hardware Prerequisites
- Programmer (Ebay search words: "J-Link OB ARM Debugger Programmer Downloader V8 SWD")
- MAX32660 sensor board
- Jumper wires
- Optional, Serial to USB converter (3.3V) to print out debugging messages

Programmer | Serial to USB 
--- | ---
<img src="img/programmer.jpg" alt="Programmer" width="auto" height="400"> | <img src="img/serial_to_usb.jpg" alt="Serial to USB" width="auto" height="400">

Top | Side | Bottom
--- | --- | ---
<img src="img/board_top.jpg" alt="Board top" width="auto" height="400"> | <img src="img/board_side.jpg" alt="Board side" width="auto" height="400"> | <img src="img/board_bottom.jpg" alt="Board bottom" width="auto" height="400">

## Programmer Wiring
Board      | Programmer
-----------|-----------
GND        | GND
3.3V       | 3.3V
SWDCLK/AD0 | SWCLK
SWDIO/BT   | SWDIO
RST        | RST

Programmer setup | Programmer connection | Board connection
--- | --- | ---
<img src="img/programmer_setup.jpg" alt="Programmer setup" width="auto" height="400"> | <img src="img/programmer_connection.jpg" alt="Programmer connection" width="auto" height="400"> | <img src="img/programmer_to_board_connection.jpg" alt="Programmer to board connection" width="auto" height="400">

## Serial Wiring (optional)
Board | Serial to USB
------|--------------
GND   | GND
INT   | RX

Serial setup | Serial connection | Board connection
--- | --- | ---
<img src="img/serial_setup.jpg" alt="Serial setup" width="auto" height="400"> | <img src="img/serial_connection.jpg" alt="Serial connection" width="auto" height="400"> | <img src="img/serial_to_board_connection.jpg" alt="Serial to board connection" width="auto" height="400">

## Upload
- Wire up the programmer to the board
- Open command line and run command:
```
python path/to/jlink_upload.py
```

## What the script does
- Opens and SWD connection
- Executes commands from file command_file.jlink
	* Uploads the built elf file to the board
	* Resets the board
	* Starts the board
	* Closes the SWD connection