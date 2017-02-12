//  ===========================================================================
//  Arduino Stereo Audio Spectrum Analyser.
//  ===========================================================================
/*

    Copyright 2017 Darren Faulke <darren@alidaf.co.uk>

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
/*
    This project requires the ArduinoFHT4 library, which needs to be installed
    manually, and the Adafruit NeoPixel library, which can be installed with
    the Arduino library manager.

    see http://wiki.openmusiclabs.com/wiki/ArduinoFHT
    and https://learn.adafruit.com/adafruit-neopixel-uberguide/overview
*/
//  ===========================================================================
//  Hardware used.
//  ===========================================================================
/*
    List:

    Arduino Pro Mini 5V/16MHz.
    Breadboard for prototyping/Stripboard for production.

    Circuit:
    
    The L channel is read in through ADC0.
    The R channel is read in through ADC1.
    Output to the WS2812 LED strip is sent from digital pin PD7

     +5V   GND
      |     |
      |-----(-------------------------,-------,---,
      |     |                         |       |   |
      |     |-------------------------(---,   |   |
      |     |                         |   |   |   |
      |     |          Pro Mini       |   |   |   |
      |     |         [TX0   RAW]-----'   |   |   |
      |     |         [RXI   GND]---------|   |   |
      |     |         [RST   RST]             |   |    
      |     |         [GND   VCC]           [R3] [R4]    
      |     |         [2      A3]             |   |    
      |     |         [3      A2]             |   |   C2          Phono
      |     |         [4      A1]-ADC1--------(---+---||---[R2]-----o R
      |     |         [5      A0]-ADC0--------+---(---||---[R1]-----o L       
      |     |         [6      13]       .     |   |   C1    ,-------o GND     
      |     |   ,-PD7-[7      12]             |   |         |
      |     |   |     [8      11]           [R5] [R6]       |
      |     |   |     [9      10]             |   |         |
      |     |   |                             |   |         |
      |     |---(-----------------------------'---'         |
      |     |   |                                           |
      |     |---(-------------------------------------------'
      |     |   |
      |     |   |                                                 WS8212
      |-----(---(---------------------,-----------------------------o +5V
      |     |   |                     |
      |     |   '---------------------(--------------------[R7]-----o Data
      |     |                         |
      |     |-----------------,-------(-----------------------------o GND
      |     |                 |   +   |
      |     |                 '--||---'
     +5V   GND                   C3                                       

      ,-------------------------,
      |   n   |   R    |   C    |
      |-------+--------+--------|
      |   1   |  1kOhm | 0.47pF |
      |   2   |  1kOhm | 0.47pF |
      |   3   | 10kOhm | 1000uF |
      |   4   | 10kOhm |        |
      |   5   | 10kOhm |        |
      |   6   | 10kOhm |        |
      |   7   | 470Ohm |        |
      '-------------------------'
*/
//  ===========================================================================
//  Functional description of the ADC registers.
//  ===========================================================================
/*
  The ADC behaviour is defined by a number of internal registers:

    ,---------------------------------------,
    | Register | Description                |
    |----------+----------------------------|
    | ADCSRA   | ADC Control and Status A.  |
    | ADCSRB   | ADC Control and Status B.  |
    | ADMUX    | ADC Multiplexer Selection. |
    | DIDR0    | Digital Input Disable.     |
    | ADCH     | ADC Data (High).           |
    | ADCL     | ADC Data (Low).            |
    '---------------------------------------' 

    ADCSRA register:
    ,---------------------------------------------------------------,
    |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
    |-------+-------+-------+-------+-------+-------+-------+-------|
    | ADEN  | ADSC  | ADATE | ADIF  | ADIE  | ADPS2 | ADPS1 | ADPS0 |
    '---------------------------------------------------------------'
    ADEN  (ADC Enable): 1 = enable, 0 = disable.
    ADSC  (ADC Start Conversion): 1 = start, auto set to zero on completion.
    ADATE (ADC Auto Trigger Enable): 1 = enable, 0 = disable.
    ADIF  (ADC Interrupt Flag): Set by hardware.
    ADIE  (ADC Interrupt Enable): 1 = enable, 0 = disable.
    ADPS  (ADC Prescaler Select Bits): 0-2 used to set clock frequency. 

    ,-----------------,
    |   ADPSn   | Div |
    | 2 | 1 | 0 |     |
    |---+---+---+-----|
    | 0 | 0 | 0 |   2 |
    | 0 | 0 | 1 |   2 |
    | 0 | 1 | 0 |   4 |
    | 0 | 1 | 1 |   8 |
    | 1 | 0 | 0 |  16 |
    | 1 | 0 | 1 |  32 |
    | 1 | 1 | 0 |  64 |
    | 1 | 1 | 1 | 128 |
    '-----------------'

    ADCSRB register:
    ,---------------------------------------------------------------,
    |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
    |-------+-------+-------+-------+-------+-------+-------+-------|
    |   -   | ACME  |   -   |   -   |   -   | ADTS2 | ADTS1 | ADTS0 |
    '---------------------------------------------------------------'

    ACME (Analog Comparator Mux Enable): 1 = enable, 0 = disable.
    ADTSn (ADC Auto Trigger Source):
    ,---------------------------------------------,
    |   ADTSn   | Trigger Select.                 |
    | 2 | 1 | 0 |                                 |
    |---+---+---+---------------------------------|
    | 0 | 0 | 0 | Free Running.                   |
    | 0 | 0 | 1 | Analog Comparator.              |
    | 0 | 1 | 0 | External Interrupt Request 0.   |
    | 0 | 1 | 1 | Timer/Counter0 Compare Match A. |
    | 1 | 0 | 0 | Timer/Counter0 Overflow.        |
    | 1 | 0 | 1 | Timer/Counter1 Compare Match B. |
    | 1 | 1 | 0 | Timer/Counter1 Compare Match A. |
    | 1 | 1 | 1 | Timer/Counter1 Capture Event.   |
    '---------------------------------------------'

    ADMUX register:
    ,---------------------------------------------------------------,
    |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
    |-------+-------+-------+-------+-------+-------+-------+-------|
    | REFS1 | REFS0 | ADLAR |   -   | MUX3  | MUX2  | MUX1  | MUX0  |
    '---------------------------------------------------------------'

    REFSn (Set reference voltage):
    ,-----------------------------------------------------------,
    | REFSn |                                                   |
    | 1 | 0 | Vref selection.                                   |
    |---+---+---------------------------------------------------|
    | 0 | 0 | AREF, Internal Vref turned off.                   |
    | 0 | 1 | AVcc with external cap on AREF pin.               |
    | 1 | 0 | Reserved.                                         |
    | 1 | 1 | Internal 1.1V (ATmega168/328) or 2.56V (ATmega8). |
    '-----------------------------------------------------------'

    ADLAR (ADC Left Adjust Result):
    ,------------------------------------------------------------------------,
    | ADLAR |           ADCH                ||           ADCL                |
    |       | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 || 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
    |-------+---+---+---+---+---+---+---+---++---+---+---+---+---+---+---+---|
    |   0   | - | - | - | - | - | - | * | * || * | * | * | * | * | * | * | * |
    |   1   | * | * | * | * | * | * | * | * || * | * | - | - | - | - | - | - |
    '------------------------------------------------------------------------'

    MUXn (Analog Channel Selection Bits): Select analog ports ADC0-ADC5.
    ,-----------------------,
    |      MUXn     | Input |
    | 3 | 2 | 1 | 0 |       |
    |---+---+---+---+-------|
    | 0 | 0 | 0 | 0 | ADC0  |
    | 0 | 0 | 0 | 1 | ADC1  |
    | 0 | 0 | 1 | 0 | ADC2  |
    | 0 | 0 | 1 | 1 | ADC3  |
    | 0 | 1 | 0 | 0 | ADC4  |
    | 0 | 1 | 0 | 1 | ADC5  |
    '-----------------------'
    
    DIDR0 register:
    ,---------------------------------------------------------------,
    |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
    |-------+-------+-------+-------+-------+-------+-------+-------|
    |   -   |   -   | ADC5D | ADC4D | ADC3D | ADC2D | ADC1D | ADC0D |
    '---------------------------------------------------------------'
    When set, the corresponding digital input buffer is disabled.

    DIDR1 register:
    ,---------------------------------------------------------------,
    |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
    |-------+-------+-------+-------+-------+-------+-------+-------|
    |   -   |   -   |   -   |   -   |   -   |   -   | AIN1D | AIN0D |
    '---------------------------------------------------------------'
    When set, the corresponding digital input buffer is disabled.
*/
//  ===========================================================================
//  Register macros.
//  ===========================================================================
/*
    These are already defined in a core library but have been included for
    a better understanding and in case the definitions are removed in future.

    Note: The macros as defined in the Arduino core libraries appear to be the
    bit positions and not effective address. The Arduino way of doing things is
    therefore to use a bit shift to set the register bits, e.g.

                  ADCSRA |= ( 1 << ADSC );  // Start ADC.
                  
*/
//  ADSRA register definitions ------------------------------------------------
/*
#define ADEN  0x80  // ADC Enable.
#define ADSC  0x40  // ADC Start Conversion.
#define ADFR  0x20  // ADC Free Running.
#define ADIF  0x10  // ADC Interrupt Flag.
#define ADIE  0x08  // ADC Interrupt Enable.
#define ADPS2 0x04  // ADC Prescaler Select 2.
#define ADPS1 0x02  // ADC Prescaler Select 1.
#define ADPS0 0x01  // ADC Prescaler Select 0.
*/
//  ADSRB register definitions ------------------------------------------------
/*
#define ACME  0x40  // ADC Analog Comparator Multiplexer Enable.
#define ADTS2 0x04  // ADC Trigger Select 2.
#define ADTS1 0x02  // ADC Trigger Select 1.
#define ADTS0 0x01  // ADC Trigger Select 0.
*/
//  ADMUX register definitions ------------------------------------------------
/*
#define REFS1 0x80  // Reference Selection Bit 1.
#define REFS0 0x40  // Reference Selection Bit 0.
#define ADLAR 0x20  // ADC Left Adjust Result.
#define MUX3  0x08  // Analogue Channel Selection Bit 3.
#define MUX2  0x04  // Analogue Channel Selection Bit 2.
#define MUX1  0x02  // Analogue Channel Selection Bit 1.
#define MUX0  0x01  // Analogue Channel Selection Bit 0.
*/
//  DIDR0 register definitions ------------------------------------------------
/*
#define ADC5D 0x20  // Disable Digital Input ADC5.
#define ADC4D 0x10  // Disable Digital Input ADC4.
#define ADC3D 0x08  // Disable Digital Input ADC3.
#define ADC2D 0x04  // Disable Digital Input ADC2.
#define ADC1D 0x02  // Disable Digital Input ADC1.
#define ADC0D 0x01  // Disable Digital Input ADC0.
*/

//  Bit tweaking macros --------------------------------------------------------
#define bitset( reg, bit ) ( reg |= ( 1 << bit ))   // Set register bit
#define bitclr( reg, bit ) ( reg &= ~( 1 << bit ))  // Clear register bit.
#define bittst( reg, bit ) ( reg & ( 1 << bit ))    // Test register bit.

//  ===========================================================================
//  Settings.
//  ===========================================================================

//  Divisor -------------------------------------------------------------------
/*

    The Arduino ADC runs at a clock rate determined by the divisor, which is
    set to 64 by default, i.e. the ADC runs at 1/64 of the full clock rate.
    Changing te divisor is as simmple as setting the ADPSn registers:

    Typical audio range = 20Hz - 20kHz (Target is 20kHz).
    Arduino clock = 16MHz.
    ADC capture takes 13 cycles.
    Nyquist Effective Bandwidth = Sample Frequency / 2.
    Therefore approx bandwidth = 16000000/(div*13*2) 

            ,-----------------------------,
            |   ADPSn   | Div |  Nyquist  |
            | 2 | 1 | 0 |     |   (Hz)    |
            |---+---+---+-----+-----------|
            | 0 | 0 | 0 |   2 |  307692   |
            | 0 | 0 | 1 |   2 |  307692   |
            | 0 | 1 | 0 |   4 |  153846   |
            | 0 | 1 | 1 |   8 |   76923   |
            | 1 | 0 | 0 |  16 |   38461   |
            | 1 | 0 | 1 |  32 |   19230   |<--Closest - use this!
            | 1 | 1 | 0 |  64 |    9615   |
            | 1 | 1 | 1 | 128 |    4807   |
            '-----------------------------'
*/
//  FHT -----------------------------------------------------------------------
/*
  FHT output settings. These are mutually exclusive and affect which
  variables and outputs are available.
*/
#define LIN_OUT      0  // Toggle linear output (word).
#define LIN_OUT8     0  // Toggle linear output (byte).
#define LOG_OUT      1  // Toggle logarithmic output (byte).
#define OCTAVE       0  // Toggle octave output (byte).
/*
  FHT_OUT is normally FHT_N/2, except for OCTAVE output where
  FHT_OUT = 8 for FHT_N = 256,
  FHT_OUT = 7 for FHT_N = 128.
*/
#define FHT_N      256  // Number of FHT input bins per channel.
#define FHT_OUT    128  // Number of FHT output bins per channel.

//  Display -------------------------------------------------------------------
/*
    The settings determine the output that is displayed.
*/
#define CHANNELS        2 // Number of audio channels.
#define FREQ_BANDS      8 // Number of frequency bands per channel.
#define FREQ_LEDS      15 // Number of LEDS representing each band.
#define LED_DATA_PIN    7 // Data pin for WS2812 LED strip.
#define INIT_BRIGHT    50 // Initial LED brightness.
#define INIT_REFRESH 50000 // Initial refresh time for LEDs.

//  Debugging -----------------------------------------------------------------
/*
    This setting toggles some debugging output via the serial port. Since
    there is ony 1 serial channel, these are mutually exclusive. Output is
    sent as bytes and needs a suitable means of displaying the output to be
    purposeful.

    PD-extended has been used here with the patch from the 
    openmusiclabs site.

            see the link to FHT_128_channel_analyser.zip at
            http://wiki.openmusiclabs.com/wiki/ArduinoFHT.
*/

#define DEBUG     0 // Print debug info to serial monitor.
#define PLOT_ADC  0 // Send ADC data to serial plotter.
#define PLOT_FHT  1 // Send FHT data to serial plotter.
#define PRINT_ADC 0 // Print ADC data to serial monitor.
#define PRINT_FHT 0 // Print FHT data to serial monitor.

//  Includes ------------------------------------------------------------------

#include <Arduino.h>
#include <FHT.h>
#include <TimerOne.h>
#include <Adafruit_NeoPixel.h>

//  Defines -------------------------------------------------------------------

// Bit manipulation.
#define bitset( reg, bit ) ( reg |= ( 1 << bit ))   // Set register bit
#define bitclr( reg, bit ) ( reg &= ~( 1 << bit ))  // Clear register bit.
#define bittst( reg, bit ) ( reg & ( 1 << bit ))    // Test register bit.

//  ===========================================================================
//  Prototypes.
//  ===========================================================================

void startADC( void );
void stopADC( void );
void getFHTOutput( void );
void getBands( void );


