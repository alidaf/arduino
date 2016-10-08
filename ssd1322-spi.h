// ============================================================================
/*
    ssd1322-spi:

    SSD1322 OLED display driver for the Arduino (SPI version).

    Copyright 2016 Darren Faulke <darren@alidaf.co.uk>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/
//  ===========================================================================

#define SSD1322_SPI_VERSION 1.01

//  Macros. -------------------------------------------------------------------

#ifndef SSD1322SPI_H
#define SSD1322SPI_H

// Default GPIO assignments.
#define SSD1322_SS     10 // Pin 10 - Chip Select.
#define SSD1322_MOSI   11 // Pin 11 - Master Out Slave In.
#define SSD1322_MISO   12 // Pin 12 - Master In Slave Out.
#define SSD1322_SCK    13 // Pin 13 - Clock.
#define SSD1322_DC      8 // Data/Command (DC#) pin..
#define SSD1322_RESET   9 // Hardware reset (RES#) pin.

// Display properties.
#define SSD1322_DISPLAYS_MAX 2 // No of displays.
#define SSD1322_COLS       256 // No of display columns (pixels).
#define SSD1322_ROWS        64 // No of display lines (pixels).
#define SSD1322_GREYSCALES  16 // No of greyscales.

/*
// SPI default properties for Arduino.
#define SPI_CHANNEL    0 // Channel.
#define SPI_BAUD 5000000 // Baud rate (5 MHz).
#define SPI_FLAGS   0x03 // Mode flags for pigpio.
*/

/*
    Setting the SPI flags:
    +-----------------------------------------------------------------+
    |21|20|19|18|17|16|15|14|13|12|11|10| 9| 8| 7| 6| 5| 4| 3| 2| 1| 0|
    |-----------------+--+--+-----------+--+--+--+--+--+--+--+--+-----|
    |      bbbbbb     | R| T|    nnnn   | W| A|u2|u1|u0|p2|p1|p0| mode|
    +-----------------------------------------------------------------+
    SSD1322 SPI interface:
        mm      = 0x03 (Clock idle high, trailing edge).
        px      = 0x00 (CS# is active low).
        ux      = 0x00 (CEx GPIO is reserved for SPI).
        A       = 0x00 (Standard SPI device).
        W       = 0x00 (4-wire interface).
        nnnn    = 0x00 (N/A).
        T       = 0x00 (MSB first).
        R       = 0x00 (N/A).
        bbbbbb  = 0x00 (N/A).
*/

// Fundamental commands.
#define SSD1322_CMD_ENABLE_GREYS    0x00 // Enable greyscale table.
#define SSD1322_CMD_SET_COLS        0x15 // Start & end cols.
#define SSD1322_CMD_SET_WRITE       0x5c // Write to RAM enable.
#define SSD1322_CMD_SET_READ        0x5d // Read from RAM enable.
#define SSD1322_CMD_SET_ROWS        0x75 // Set start & end rows.
#define SSD1322_CMD_SET_REMAP       0xa0 // RAM addressing modes.
#define SSD1322_CMD_SET_START       0xa1 // Set RAM start line.
#define SSD1322_CMD_SET_OFFSET      0xa2 // Set RAM offset (vertical scroll).
#define SSD1322_CMD_SET_PIX_OFF     0xa4 // All pixels off.
#define SSD1322_CMD_SET_PIX_ON      0xa5 // All pixels on.
#define SSD1322_CMD_SET_PIX_NORM    0xa6 // Normal display.
#define SSD1322_CMD_SET_PIX_INV     0xa7 // Inverse display.
#define SSD1322_CMD_SET_PART_ON     0xa8 // Partial display on.
#define SSD1322_CMD_SET_PART_OFF    0xa9 // Partial display off.
#define SSD1322_CMD_SET_VDD         0xab // Set VDD source.
#define SSD1322_CMD_SET_DISP_OFF    0xae // Display logic off.
#define SSD1322_CMD_SET_DISP_ON     0xaf // Display logic on.
#define SSD1322_CMD_SET_PHASE       0xb1 // Clock phase length.
#define SSD1322_CMD_SET_CLOCK       0xb3 // Clock frequency/divider.
#define SSD1322_CMD_SET_ENHANCE_A   0xb4 // Display enhancement A.
#define SSD1322_CMD_SET_GPIOS       0xb5 // GPIO modes.
#define SSD1322_CMD_SET_PERIOD      0xb6 // Pre-charge period.
#define SSD1322_CMD_SET_GREYS       0xb8 // Set greyscale table values.
#define SSD1322_CMD_SET_GREYS_DEF   0xb9 // Set greyscale table to default.
#define SSD1322_CMD_SET_PRE_VOLT    0xbb // Pre-charge voltage.
#define SSD1322_CMD_SET_COM_VOLT    0xbe // Common voltage.
#define SSD1322_CMD_SET_CONTRAST    0xc1 // Contrast.
#define SSD1322_CMD_SET_BRIGHTNESS  0xc7 // Brightness.
#define SSD1322_CMD_SET_MUX         0xca // Mux.
#define SSD1322_CMD_SET_ENHANCE_B   0xd1 // Display enhancement B.
#define SSD1322_CMD_SET_LOCK        0xfd // Command lock.

// Default values.
#define SSD1322_DEFAULT_COL1        0x00 //
#define SSD1322_DEFAULT_COL2        0x77 //
#define SSD1322_DEFAULT_ROW1        0x00 //
#define SSD1322_DEFAULT_ROW2        0x7f //
#define SSD1322_DEFAULT_REMAP1      0x00 //
#define SSD1322_DEFAULT_REMAP2      0x01 //
#define SSD1322_DEFAULT_START       0x00 //
#define SSD1322_DEFAULT_OFFSET      0x00 //
#define SSD1322_DEFAULT_VDD         0x01 //
#define SSD1322_DEFAULT_PHASE       0x74 //
#define SSD1322_DEFAULT_CLOCK       0x50 //
#define SSD1322_DEFAULT_ENHANCE_A1  0xa2 //
#define SSD1322_DEFAULT_ENHANCE_A2  0xb5 //
#define SSD1322_DEFAULT_GPIOS       0x0a //
#define SSD1322_DEFAULT_PERIOD      0x08 //
#define SSD1322_DEFAULT_PRE_VOLT    0x17 //
#define SSD1322_DEFAULT_COM_VOLT    0x04 //
#define SSD1322_DEFAULT_CONTRAST    0x7f //
#define SSD1322_DEFAULT_BRIGHTNESS  0xff //
#define SSD1322_DEFAULT_MUX         0x7f //
#define SSD1322_DEFAULT_ENHANCE_B1  0xa2 //
#define SSD1322_DEFAULT_ENHANCE_B2  0x20 //

// GPIO states.
#define SSD1322_INPUT_COMMAND 0 // Enable command mode for DC# pin.
#define SSD1322_INPUT_DATA    1 // Enable data mode for DC# pin.
#define SSD1322_RESET_ON      0 // Hardware reset.
#define SSD1322_RESET_OFF     1 // Normal operation.

// Ranges.
#define SSD1322_COLS_MIN       0x00 // Start column.
#define SSD1322_COLS_MAX       0x77 // End column.
#define SSD1322_ROWS_MIN       0x00 // Start row.
#define SSD1322_ROWS_MAX       0x7f // End row.

// Settings.
#define SSD1322_INC_COLS       0x00 // Increment cols.
#define SSD1322_INC_ROWS       0x01 // Increment rows.
#define SSD1322_SCAN_RIGHT     0x00 // Scan columns left to right.
#define SSD1322_SCAN_LEFT      0x02 // Scan columns right to left.
#define SSD1322_SCAN_DOWN      0x00 // Scan rows from top to bottom.
#define SSD1322_SCAN_UP        0x10 // Scan rows from bottom to top.
#define SSD1322_VDD_EXTERNAL   0x00 // Use external VDD regulator.
#define SSD1322_VDD_INTERNAL   0x01 // Use internal VDD regulator (reset).

// Enable/disable.
#define SSD1322_PARTIAL_ON      0x01 // Partial mode on.
#define SSD1322_PARTIAL_OFF     0x00 // Partial mode off.
#define SSD1322_SPLIT_DISABLE   0x00 // Disable odd/even split of COMs.
#define SSD1322_SPLIT_ENABLE    0x20 // Enable odd/even split of COMS.
#define SSD1322_DUAL_DISABLE    0x00 // Disable dual COM line mode.
#define SSD1322_DUAL_ENABLE     0x10 // Enable dual COM line mode.
#define SSD1322_REMAP_DISABLE   0x00 // Disable nibble re-map.
#define SSD1322_REMAP_ENABLE    0x04 // Enable nibble re-map.
#define SSD1322_COMMAND_LOCK    0x16 // Command lock.
#define SSD1322_COMMAND_UNLOCK  0x12 // Command unlock.

// Resets
#define SSD1322_CLOCK_DIV_RESET  0x01
#define SSD1322_CLOCK_FREQ_RESET 0xc0
#define SSD1322_PERIOD_RESET     0x08

// Data structures. -----------------------------------------------------------

// uint8_t ssd1322_greys[16];  // Greyscale definitions.

// Hardware functions. --------------------------------------------------------

// ----------------------------------------------------------------------------
/*
    Writes a command.
*/
// ----------------------------------------------------------------------------
void ssd1322_write_command( uint8_t command );

// ----------------------------------------------------------------------------
/*
    Writes a data byte.
*/
// ----------------------------------------------------------------------------
void ssd1322_write_data( uint8_t data );

// ----------------------------------------------------------------------------
/*
    Writes a data stream.
*/
// ----------------------------------------------------------------------------
void ssd1322_write_stream( uint8_t *buf, uint16_t count );

// ----------------------------------------------------------------------------
/*
    Triggers a hardware reset:
    Hardware is reset when RES# pin is pulled low.
    For normal operation RES# pin should be pulled high.
    Redundant if RES# pin is wired to VCC.
*/
// ----------------------------------------------------------------------------
void ssd1322_reset( void );

// ----------------------------------------------------------------------------
/*
    Enables grey scale lookup table.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_enable_greys( void );

// ----------------------------------------------------------------------------
/*
    Sets column start & end addresses within display data RAM:
    0 <= start[6:0] < end[6:0] <= 0x77 (119).
    If horizontal address increment mode is enabled, column address pointer is
    automatically incremented after a column read/write. Column address pointer
    is reset after reaching the end column address.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_cols( uint8_t start, uint8_t end );

// ----------------------------------------------------------------------------
/*
    Resets column start & end addresses within display data RAM:
    Start = 0x00.
    End   = 0x77 (119).
*/
// ----------------------------------------------------------------------------
void ssd1322_set_cols_default( void );
// ----------------------------------------------------------------------------
/*
    Enables writing data continuously into display RAM.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_write_continuous( void );

// ----------------------------------------------------------------------------
/*
    Enables reading data continuously from display RAM. Not used in SPI mode.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_read_continuous( void );

// ----------------------------------------------------------------------------
/*
    Sets column start & end addresses within display data RAM:
    0 <= Start[6:0] < End[6:0] <= 0x7f (127).
    If vertical address increment mode is enabled, column address pointer is
    automatically incremented after a column read/write. Column address pointer
    is reset after reaching the end column address.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_rows( uint8_t start, uint8_t end );

// ----------------------------------------------------------------------------
/*
    Resets row start & end addresses within display data RAM:
    Start = 0.
    End   = 0x7f (127).
*/
// ----------------------------------------------------------------------------
void ssd1322_set_rows_default( void );

// ----------------------------------------------------------------------------
/*
    Sets various addressing configurations.
    a[0] = 0: Increment display RAM by column (horizontal).
    a[0] = 1: Increment display RAM by row (vertical).
    a[1] = 0: Columns incremented left to right.
    a[1] = 1: Columns incremented right to left.
    a[2] = 0: Direct mapping (reset).
    a[2] = 1: The four nibbles of the RAM data bus are re-mapped.
    a[3] = 0.
    a[4] = 0: Rows incremented from top to bottom.
    a[4] = 1: Rows incremented from bottom to top.
    a[5] = 0: Sequential pin assignment of COM pins.
    a[5] = 1: Odd/even split of COM pins.
    a[7:6] = 0x00.
    b[3:0] = 0x01.
    b[4] = 0: Disable dual COM mode.
    b[4] = 1: Enable dual COM mode. A[5] must be disabled and MUX <= 63.
    b[5] = 0.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_remap( uint8_t a, uint8_t b );

// ----------------------------------------------------------------------------
/*
    Sets default addressing configurations.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_remap_default( void );

// ----------------------------------------------------------------------------
/*
    Sets display RAM start line.
    0 <= start <= 127 (0x7f).
*/
// ----------------------------------------------------------------------------
void ssd1322_set_start( uint8_t start );

// ----------------------------------------------------------------------------
/*
    Sets default display RAM start line.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_start_default( void );

// ----------------------------------------------------------------------------
/*
    Sets display RAM offset.
    0 <= start <= 127 (0x7f).
*/
// ----------------------------------------------------------------------------
void ssd1322_set_offset( uint8_t offset );

// ----------------------------------------------------------------------------
/*
    Sets default display RAM offset.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_offset_default( void );

// ----------------------------------------------------------------------------
/*
    Sets display mode to normal (display shows contents of display RAM).
*/
// ----------------------------------------------------------------------------
void ssd1322_set_display_normal( void );

// ----------------------------------------------------------------------------
/*
    Sets all pixels on regardless of display RAM content.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_display_all_on( void );

// ----------------------------------------------------------------------------
/*
    Sets all pixels off regardless of display RAM content.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_display_all_off( void );

// ----------------------------------------------------------------------------
/*
    Displays inverse of display RAM content.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_display_inverse( void );

// ----------------------------------------------------------------------------
/*
    Enables partial display mode (subset of available display).
    0 <= start <= end <= 0x7f.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_part_display_on( uint8_t start, uint8_t end );

// ----------------------------------------------------------------------------
/*
    Enables partial display mode (subset of available display).
    0 <= start <= end <= 0x7f.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_part_display_off( void );

// ----------------------------------------------------------------------------
/*
    Enables internal VDD regulator.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_vdd_internal( void );

// ----------------------------------------------------------------------------
/*
    Disables internal VDD regulator (requires external regulator).
*/
// ----------------------------------------------------------------------------
void ssd1322_set_vdd_external( void );

// ----------------------------------------------------------------------------
/*
    Turns display circuit on.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_display_on( void );

// ----------------------------------------------------------------------------
/*
    Turns display circuit off.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_display_off( void );

// ----------------------------------------------------------------------------
/*
    Sets length of phase 1 & 2 of segment waveform driver.
    Phase 1: phase[3:0] Set from 5 DCLKS - 31 DCLKS in steps of 2. Higher OLED
             capacitance may require longer period to discharge.
    Phase 2: phase[7:4]Set from 3 DCLKS - 15 DCLKS in steps of 1. Higher OLED
             capacitance may require longer period to charge.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_phase( uint8_t phase );

// ----------------------------------------------------------------------------
/*
    Sets default length of phase 1 & 2 of segment waveform driver.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_phase_default( void );

// ----------------------------------------------------------------------------
/*
    Sets clock divisor/frequency.
    Front clock divider  A[3:0] = 0x0-0xa, reset = 0x1.
    Oscillator frequency A[7:4] = 0x0-0xf, reset = 0xc.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_clock( uint8_t clock );

// ----------------------------------------------------------------------------
/*
    Sets default clock divisor/frequency.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_clock_default( void );

// ----------------------------------------------------------------------------
/*
    Sets display enhancement A.
    a[1:0] 0x00 = external VSL, 0x02 = internal VSL.
    a[7:2] = 0xa0.
    b[2:0] = 0x05.
    b[7:3] 0xf8 = enhanced, 0xb0 = normal.
    Typical values: a = 0xa0, b = 0xfd.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_enhance_a( uint8_t a, uint8_t b );

// ----------------------------------------------------------------------------
/*
    Sets default display enhancement A.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_enhance_a_default( void );

// ----------------------------------------------------------------------------
/*
    Sets state of GPIO pins (GPIO0 & GPIO1).
    gpio[1:0] GPIO0
    gpio[3:2] GPIO1
*/
// ----------------------------------------------------------------------------
void ssd1322_set_gpios( uint8_t gpio );

// ----------------------------------------------------------------------------
/*
    Sets default state of GPIO pins (GPIO0 & GPIO1).
*/
// ----------------------------------------------------------------------------
void ssd1322_set_gpios_default( void );

// ----------------------------------------------------------------------------
/*
    Sets second pre-charge period.
    period[3:0] 0 to 15DCLK.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_period( uint8_t period );

// ----------------------------------------------------------------------------
/*
    Sets default second pre-charge period.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_period_default( void );

// ----------------------------------------------------------------------------
/*
    Sets user-defined greyscale table GS1 - GS15 (Not GS0).
    gs[7:0] 0 <= GS1 <= GS2 .. <= GS15.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_greys( uint8_t gs[SSD1322_GREYSCALES] );

// ----------------------------------------------------------------------------
/*
    Sets greyscale lookup table to default linear values.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_greys_default( void );

// ----------------------------------------------------------------------------
/*
    Sets pre-charge voltage level for segment pins with reference to VCC.
    voltage[4:0] 0.2xVCC (0x00) to 0.6xVCC (0x3e).
*/
// ----------------------------------------------------------------------------
void ssd1322_set_pre_volt( uint8_t volts );

// ----------------------------------------------------------------------------
/*
    Sets default pre-charge voltage level for segment pins.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_pre_volt_default( void );

// ----------------------------------------------------------------------------
/*
    Sets the high voltage level of common pins, VCOMH with reference to VCC.
    voltage[3:0] 0.72xVCC (0x00) to 0.86xVCC (0x07).
*/
// ----------------------------------------------------------------------------
void ssd1322_set_com_volt( uint8_t volts );

// ----------------------------------------------------------------------------
/*
    Sets the default high voltage level of common pins, VCOMH.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_com_volt_default( void );

// ----------------------------------------------------------------------------
/*
    Sets the display contrast (segment output current ISEG).
    contrast[7:0]. Contrast varies linearly from 0x00 to 0xff.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_contrast( uint8_t contrast );

// ----------------------------------------------------------------------------
/*
    Sets the default display contrast (segment output current ISEG).
*/
// ----------------------------------------------------------------------------
void ssd1322_set_contrast_default( void );

// ----------------------------------------------------------------------------
/*
    Sets the display brightness (segment output current scaling factor).
    factor[3:0] 1/16 (0x00) to 16/16 (0x0f).
    The smaller the value, the dimmer the display.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_brightness( uint8_t factor );

// ----------------------------------------------------------------------------
/*
    Sets the default display brightness.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_brightness_default( void );

// ----------------------------------------------------------------------------
/*
    Sets multiplex ratio (number of common pins enabled)
    ratio[6:0] 16 (0x00) to 128 (0x7f).
*/
// ----------------------------------------------------------------------------
void ssd1322_set_mux( uint8_t ratio );

// ----------------------------------------------------------------------------
/*
    Sets default multiplex ratio (number of common pins enabled)
*/
// ----------------------------------------------------------------------------
void ssd1322_set_mux_default( void );

// ----------------------------------------------------------------------------
/*
    Sets display enhancement B.
    a[3:0] = 0x02.
    a[5:4] 0x00 = reserved, 0x02 = normal (reset).
    a[7:6] = 0x80.
    b[7:0] = 0x20.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_enhance_b( uint8_t a, uint8_t b );

// ----------------------------------------------------------------------------
/*
    Sets default display enhancement B.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_enhance_b_default( void );

// ----------------------------------------------------------------------------
/*
    Sets command lock - prevents all command except unlock.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_command_lock( void );

// ----------------------------------------------------------------------------
/*
    Unsets command lock.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_command_unlock( void );

// ----------------------------------------------------------------------------
/*
    Clears display RAM.
*/
// ----------------------------------------------------------------------------
void ssd1322_clear_display( void );

// ----------------------------------------------------------------------------
/*
    Sets default display operation settings.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_defaults( void );

// ----------------------------------------------------------------------------
/*
    Sets typical display operation settings.
*/
// ----------------------------------------------------------------------------
void ssd1322_set_typical( void );

// ----------------------------------------------------------------------------
/*
    Initialises display.
*/
// ----------------------------------------------------------------------------
void ssd1322_init( void );

#endif
