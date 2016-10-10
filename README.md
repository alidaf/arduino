##arduino

Arduino projects, libraries and utilities. Read the various header files for detailed descriptions.

###display:

Libraries providing support for various displays.

* ssd1322 provides some rudimentary functions for SSD1322 OLED displays. The intent is to use I2C from a Raspberry Pi to command what will be displayed. The Arduino uses the low level driver to facilitate the display. Early tests suggest that there is not enough memory to have an array capable of storing an entire screen worth of data (8192 bytes) so the 4bpp image used as a test for the Raspberry Pi driver doesn't work. It is therefore intended to modify the driver to accept a maximum of a full line at a time (128 Bytes) via I2C. The image can then be built up by sending the data for each line.
