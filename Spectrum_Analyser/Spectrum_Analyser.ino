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
    | ADEN  | ADSC  | ADFR  | ADIF  | ADIE  | ADPS2 | ADPS1 | ADPS0 |
    '---------------------------------------------------------------'
    ADEN (ADC Enable): 1 = enable, 0 = disable.
    ADSC (ADC Start Conversion): 1 = start, auto set to zero on completion.
    ADFR (ADC Free Running Select): 1 = free running, 0 = single conversion.
    ADIF (ADC Interrupt Flag): Set by hardware.
    ADIE (ADC Interrupt Enable): 1 = enable, 0 = disable.
    ADPS (ADC Prescaler Select Bits): 0-2 used to set clock frequency. 

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
    ADTSn (ADC Trigger Select bits):
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
                  
    The macros defined here are the effective addresses of the bit, so setting
    the bit is simpler and quicker, e.g.

                  ADCSRA |= ADSC;           // Start ADC.

*/
//  ADSRA register definitions ------------------------------------------------

#define ADEN  0x80  // ADC Enable.
#define ADSC  0x40  // ADC Start Conversion.
#define ADFR  0x20  // ADC Free Running.
#define ADIF  0x10  // ADC Interrupt Flag.
#define ADIE  0x08  // ADC Interrupt Enable.
#define ADPS2 0x04  // ADC Prescaler Select 2.
#define ADPS1 0x02  // ADC Prescaler Select 1.
#define ADPS0 0x01  // ADC Prescaler Select 0.

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
            |   ADPSn   | Div | Bandwidth |
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
#define FHT_N      128  // Number of FHT input bins per channel.
#define FHT_OUT     64  // Number of FHT output bins per channel.

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
    These settings toggle some debugging output via the serial port. Since
    there is ony 1 serial channel, these are mutually exclusive. Output is
    sent as bytes and needs a suitable means of displaying the output to be
    purposeful.

    PD-extended has been used here (on OSX), with the patch from the 
    openmusiclabs site.

            see the link to FHT_128_channel_analyser.zip at
            http://wiki.openmusiclabs.com/wiki/ArduinoFHT.
*/

//  Includes ------------------------------------------------------------------

#include <FHT.h>
#include <TimerOne.h>
#include <Adafruit_NeoPixel.h>

//  ===========================================================================
//  Global variables.
//  ===========================================================================

byte bandvals[CHANNELS][FREQ_BANDS];  // Frequency bands.
byte peakvals[CHANNELS][FREQ_BANDS];  // Peak hold values.
byte fht_lin_fix[FHT_OUT];            // Debugging output

byte channel;                         // Current channel.

const byte admux[CHANNELS] = { ADC0D, ADC1D };  // ADC assignments.
const byte bins = FHT_OUT / FREQ_BANDS;         // Number of bins in each band.

/*
    There are typically more output bins than can be displayed. For the
    display, each frequency band can cover a number of bins so there are a
    few options available:

    1) Average or RMS the bins over each frequency band. This is likely to
       have a significant overhead of cycles to complete and may slow down
       the overall processing too much.
    2) Display the maximum of all bins within the frequency band. This is
       likely to have relatively little overhead but the frequency bands
       would all be fixed in size and position.
    3) Specify the frequencies to display by calculating the specific bin
       positions of those frequencies. Probably the best option in terms of
       performance but also allows a lot more customisation. This option
       is implemented here as function getbands(). The bands can be weighted
       or equispaced:

    The frequency of a particular bin, i, is found from:
    
              f = i * Sample Rate / FHT_N

    Therefore, the bin number for a particular frequency can be found from:

              i = f * FHT_N / ( 2 * Sample Rate )

    A starting point for picking suitable frequencies over a fixed range
    can be calculated with the following formula (based on linear FHT output):

              fn = n * ( fe/fs )^((n-1)/(N-1))
    where

      fn = the frequency at ordinate n (from 1 to N).
      N  = the number of frequencies to display.
      fs = the starting frequency.
      fe = the end frequency.

    For a nominal frequency range of 60Hz (fs) - 18kHz (fe) over 8 ordinates,
    FHT_N = 128 & Sample Rate = 19230Hz (based on a divider of 32):

                      ,-----------------,
                      | x | f(Hz) | bin |
                      |---+-------+-----|
                      | 1 |    60 |   0 |
                      | 2 |   136 |   1 |
                      | 3 |   306 |   2 |
                      | 4 |   691 |   3 |
                      | 5 |  1562 |   5 |
                      | 6 |  3528 |  12 |
                      | 7 |  7969 |  27 |
                      | 8 | 18000 |  60 |
                      '-----------------'
*/

const byte sampbins[FREQ_BANDS] = { 0, 1, 2, 3, 5, 12, 27, 60 }; // Log.
/*
    Equispaced bins.
*/
//const byte sampbins[FREQ_BANDS] = {  4, 12, 20, 28, 36, 44, 52, 60 }; // Median.
//const byte sampbins[FREQ_BANDS] = {  0,  8, 16, 24, 32, 40, 48, 56 }; // Start.
//const byte sampbins[FREQ_BANDS] = {  7, 15, 23, 31, 39, 47, 55, 63 }; // End.
/*
    Determined by eye.
*/
//const byte sampbins[FREQ_BANDS] = { 0, 1, 2, 3, 5, 10, 22, 43 }; // Determined

/*
    The output of FHT is typically a word or byte value, which exceeds the
    number of LEDs. This is a factor that scales the output to account for
    the nmber of LEDs available to display each bin range.
*/
float scaleLED;

Adafruit_NeoPixel ledstrip = Adafruit_NeoPixel( FREQ_BANDS * FREQ_LEDS,
                                                LED_DATA_PIN,
                                                NEO_GRBW + NEO_KHZ800);

byte brightness = INIT_BRIGHT;        // Set initial LED brightness.
long refresh = INIT_REFRESH;          // Set initial time (uS) between updates.
volatile byte filling = 0;

//  ===========================================================================
//  Code.
//  ===========================================================================

//  ---------------------------------------------------------------------------
//  Initialises the ADC.
//  ---------------------------------------------------------------------------
void setup( void )
{
/*    
  To set individual bits, use REG |= _BV( BIT ).
  To clear individual bits, use REG &= ~(_BV( BIT ))
*/
  ADCSRA = B11100101; // Initialise ADCSRA:
/*  
  ADEN  1   // Enable ADC.
  ADSC  1   // Start ADC.
  ADFR  1   // Free Running mode.
  ADIF  0   // Interrupt flag.
  ADIE  0   // Interrupt enable.
  ADPS2 1   // }
  ADPS1 0   // } Set Prescaler to 32.
  ADPS0 1   // }

  Note: Setting free running mode is a performance enhancement over using
        single conversion mode. In free running mode the ADC will perform
        successive conversions every 13 cycles, whereas a single conversion
        will take 25 cycles and then wait to be restarted.
*/
  ADCSRB = B00000000; // Initialise ADCSRB.
/*
  -     0   // Reserved.
  ACME  0   // ADC Analog Comparator Multiplexer Disabled.
  -     0   // Reserved.
  -     0   // Reserved.
  -     0   // Reserved.
  ADTS2 0   // Trigger Source. }
  ADTS1 0   // Trigger Source. } Free running.
  ADTS0 0   // Trigger Source. }
*/

  ADMUX = B01000000;  // Initialise ADMUX.
/*
  REFS1 0 // Reference Selection Bit 1. }
  REFS0 1 // Reference Selection Bit 0. } Use the external Vcc.
  ADLAR 0 // ADC Left Adjust Result.
  -     0 // Reserved.
  MUX3  0 // Analogue Channel Selection Bit 3. }
  MUX2  0 // Analogue Channel Selection Bit 2. } Setting channel later.
  MUX1  0 // Analogue Channel Selection Bit 1. } Default = 0.
  MUX0  0 // Analogue Channel Selection Bit 0. }
*/

//  Turning off some functions can reduce power consumption and reduce noise.
/*
    Timer0 is an 8-bit timer used for Arduino functions such as delay(),
    millis() and micros().
*/
  TIMSK0 = 0x00;        // Turn off timer0 for lower jitter
  DIDR0  = B00111111;   // Turn off digital inputs.
  DIDR1  = B00000011;   // TUrn off digital inputs.

  Serial.begin( 115200 );
/*
    Calculate scaling factor according to FHT output type.
*/
//  if ( LIN_OUT ) scaleLED = ( FREQ_LEDS / 0xff );
//  else scaleLED = ( FREQ_LEDS / 0xff );
//  scaleLED = ( FREQ_LEDS / 0xff );
//  delay( 5000 );

//  ---------------------------------------------------------------------------
//  Initialises the LED strip.
//  ---------------------------------------------------------------------------

//  ledstrip.setBrightness( brightness );
//  ledstrip.begin();
//  ledstrip.show();

//  ---------------------------------------------------------------------------
//  Initialises the timer.
//  ---------------------------------------------------------------------------

//  Timer1.initialize( refresh );
//  Timer1.attachInterrupt( updateLED );
  delay( 1000 );
}

//  ---------------------------------------------------------------------------
/*
    This updates the LED string after receiving an interrupt from Timer1.
*/
//  ---------------------------------------------------------------------------
void updateLED( void )
{
//  while (filling); // Wait until output array is full.
    {
//        filling = 1;
        Serial.write( fht_lin_fix, FHT_OUT );
//      Serial.write( bandvals[channel], FREQ_BANDS );
//        filling = 0;
    }
}

//  ---------------------------------------------------------------------------
/*
    This fills the FHT output array with the sorted frequency magnitudes
    depending on the type of output requested, i.e.
        Linear
        Linear (byte)
        Logarithmic
        Octave
*/
//  ---------------------------------------------------------------------------
void getFHTsamples( void )
{
  byte adch;  // Content of ADCH regiser.
  byte adcl;  // Content of ADCL register.
  int  adcx;  // Content of combined ADCH and ADCL registers.
  int  bin;   // FHT bin counter.
  
  cli();      // Disable interrupts because UDRE interrupt slows this down.

  for ( bin = 0; bin < FHT_N; bin++)
  {
    while( !( ADCSRA & ADIF )); // Wait for ADC to complete.
/*
    The ADC returns a 10-bit value stored in ADCL & ADCH according to the
    setting of the ADLAR bit in the ADMUX register.
*/
    adcl = ADCL;                // Fetch ADCL register data.
    adch = ADCH;                // Fetch ADCH register data.
    adcx = adch << 8 | adcl;  // Combine register bytes into an int.
    adcx -= 0x0200;             // Form into a signed int.
    adcx <<= 6;                 // Form into a 16b signed int.
    fht_input[bin] = adcx;      // Put real data into bins.
  }

  fht_window();     // Window the data for better frequency response.
  fht_reorder();    // Re-order the data before the FHT.
  fht_run();        // Process the FHT.

//  fht_mag_lin();    // Produce linear output distribution.
//  fht_mag_lin8();   // Produce linear 8-bit output distribution.
  fht_mag_log();    // Produce log FHT output.
//  fht_mag_octave(); // Produce octave output distribution.

  sei(); // Re-enable interrupts.
}

//  ---------------------------------------------------------------------------
/*
    Fills the bandvals array with the values of the specific bins in sampbins
    array.
*/
//  ---------------------------------------------------------------------------
void getBands( void )
{

  byte bandcount; // Frequency band counter.
  byte bin;       // Bin number;

  for ( bandcount = 0; bandcount < FREQ_BANDS; bandcount++ )
  {
    bin = sampbins[bandcount];

//  bandvals[channel][bandcount] = fht_lin_out[bin];
//  bandvals[channel][bandcount] = fht_lin_out8[bin];
//  bandvals[channel][bandcount] = fht_log_out[bin];
//  bandvals[channel][bandcount] = fht_oct_out[bin];
  }
}

//  ---------------------------------------------------------------------------
/*
    Main loop.
*/
//  ---------------------------------------------------------------------------
void loop()
{
  byte reg;   // Temp storage for ADMUX register.
  int  i;     // Loop control.


  Serial.write( 255 );
  for ( channel = 0; channel < CHANNELS; channel++ )
  {
    reg = ADMUX;              // Store ADMUX register.
    ADMUX |= admux[channel];  // Mask ADC bits for channel.
    getFHTsamples();          // Carry out FHT.
    ADMUX=reg;                // Restore ADMUX register..

//    filling = 1;
    for ( i = 0; i < FHT_OUT; i++ )
    {
      fht_lin_fix[i] = fht_log_out[i];
    }
//    filling = 0;

//    getBands();   // Get frequency band output.
    updateLED();  // Update LEDs. This is called by interrupt.
  }
  delay( 5000 );
}

