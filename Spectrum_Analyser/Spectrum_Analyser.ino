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

    See http://wiki.openmusiclabs.com/wiki/ArduinoFHT
    and https://learn.adafruit.com/adafruit-neopixel-uberguide/overview
*/

//  ===========================================================================
//  Defines.
//  ===========================================================================

//  Debug output --------------------------------------------------------------
/*
    This setting toggles some debugging output via the serial port. Since
    there is ony 1 serial channel, some of these are mutually exclusive.
    Processing 3.2.4 has been used to visualise the full spectrum FHT output
    with a patch from the openmusiclabs site. There is a copy in the githib
    folder - Spectrum_Analyser_n_Bands.pde.
*/

#define DEBUG       0 // Print debug info to serial console.
#define FHT_PLOT    1 // Send FHT output to Processing.
#define ADC_PRINT   0 // Print ADC data to serial console.
#define FHT_PRINT   0 // Print FHT data to serial console.

//  LED output ----------------------------------------------------------------

#define ENABLE_LED  0 // Enable FHT output to LED strip/matrix.

//  FHT -----------------------------------------------------------------------
/*
  FHT settings. These are mutually exclusive and affect which variables and
  outputs are available. Turn off whichever ones aren't being used to preserve
  resources.
*/
#define LIN_OUT      0  // Toggle linear output (word).
#define LIN_OUT8     0  // Toggle linear output (byte).
#define LOG_OUT      1  // Toggle logarithmic output (byte).
#define OCTAVE       0  // Toggle octave output (byte).
#define SCALE      256  // Scaling factor for LIN_OUT8.

#define FHT_N      128  // Number of FHT input bins per channel.
#define FHT_OUT     64  // Number of FHT output bins per channel.
/*
  Note:
  FHT_N must be either of 256, 128, 64, 32, 16.
  FHT_OUT = FHT_N/2, except for OCTAVE output where
  FHT_OUT = log(FHT_N)/log(2), i.e.

  ,-------------------,
  |  FHT_N  | FHT_OUT |
  |---------+---------|
  |    256  |    8    |
  |    128  |    7    |
  |     64  |    6    |
  |     32  |    5    |
  |     16  |    4    |
  '-------------------'
*/
//  Display -------------------------------------------------------------------
/*
    These settings define the LED layout and settings.
*/
#define CHANNELS         2  // Number of audio channels.
#define LED_BANDS        8  // Number of frequency bands per channel.
#define LED_COUNT       15  // Number of LEDS representing each band.
#define LED_DATA_PIN     7  // Data pin for WS2812 LED strip.

#define INIT_BRIGHT      50  // Initial LED brightness.
#define INIT_REFRESH 100000  // Initial refresh time for LEDs (us).

//  Bit tweaking macros --------------------------------------------------------

#define bitset( reg, bit ) ( reg |= ( 1 << bit ))   // Set register bit
#define bitclr( reg, bit ) ( reg &= ~( 1 << bit ))  // Clear register bit.
#define bittst( reg, bit ) ( reg & ( 1 << bit ))    // Test register bit.

//  ===========================================================================
//  Includes.
//  ===========================================================================

#include <Arduino.h>
#include <FHT.h>
#include <TimerOne.h>
#include <Adafruit_NeoPixel.h>

//  ===========================================================================
//  Global variables.
//  ===========================================================================

volatile int      ADC_buffer[FHT_N];  // Ring buffer for ADC data.
volatile uint8_t  ADC_channel;        // Keeps track of ADC channel.
volatile uint16_t ADC_sample;         // Keeps track of ADC sample.
volatile uint8_t  FHT_channel;        // Keeps track of FHT channel.

volatile boolean  ADC_buffer_full;    // Flag to start FHT process.
volatile boolean  ADC_ready;          // Flag to control ADC reads.
volatile boolean  ADC_skip;           // Flag to skip a sample.
volatile boolean  display_ready;      // Flag to update display.

byte FHT_output[CHANNELS][FHT_OUT];   // FHT output.
byte LED_bands[CHANNELS][LED_BANDS];  // Frequency band values.
byte LED_peaks[CHANNELS][LED_BANDS];  // Peak hold values.

const byte admux[CHANNELS] = { ADC0D, ADC1D };  // ADC assignments.

const byte sampbins[LED_BANDS] = { 0, 1, 2, 3, 5, 12, 27, 60 }; // Log.

// Alternative equispaced bins.
//const byte sampbins[LED_BANDS] = {  4, 12, 20, 28, 36, 44, 52, 60 }; // Median.
//const byte sampbins[LED_BANDS] = {  0,  8, 16, 24, 32, 40, 48, 56 }; // Start.
//const byte sampbins[LED_BANDS] = {  7, 15, 23, 31, 39, 47, 55, 63 }; // End.
//const byte sampbins[LED_BANDS] = {  0,  1,  2,  3,  5, 10, 22, 43 }; // Determined

float scaleLED;

Adafruit_NeoPixel ledstrip = Adafruit_NeoPixel( LED_BANDS * LED_COUNT,
                                                LED_DATA_PIN,
                                                NEO_GRBW + NEO_KHZ800);

byte brightness = INIT_BRIGHT;  // Set initial LED brightness.
long refresh = INIT_REFRESH;    // Set initial time (uS) between updates.


//  ===========================================================================
//  Code.
//  ===========================================================================

//  ---------------------------------------------------------------------------
//  Initialises the ADC registers and LED.
//  ---------------------------------------------------------------------------
void setup( void )
{

  if ( DEBUG | 
       ADC_PRINT | 
       FHT_PRINT | 
       FHT_PLOT | 
       ENABLE_LED ) Serial.begin( 115200 );

  if ( DEBUG ) Serial.println( "Starting Initialisation" );

  // ADCSRA.
  ADCSRA = B10100101;

  // ADEN  - 1  // Enable ADC.
  // ADSC  - 0  // Disable conversions.
  // ADATE - 1  // Enable ADC auto trigger.
  // ADIF  - 0  // Set by hardware
  // ADIE  - 0  // Enable interrupt.
  // ADPS2 - 1  // }
  // ADPS1 - 0  // }- ADC prescaler (See table below).
  // ADPS0 - 1  // }
/*
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
*/

  // ADCSRB.
  ADCSRB = B00000000;

  // N/A        //
  // ACME  - 0  // Disable Analogue Comparator Mux.
  // N/A        //
  // N/A        //
  // N/A        //
  // ADTS2 - 0  // }
  // ADTS1 - 0  // }- Free Running Mode.
  // ADTS0 - 0  // }

  // ADMUX.
  ADMUX = B01000000;
  
  // REFS1 - 0  // }- Use Vcc as reference.
  // REFS0 - 1  // } 
  // ADLAR - 0  // Right adjust ADC results (10-bit mode).
  // N/A        //
  // MUX3  - 0  // } 
  // MUX2  - 0  // }- ADC input 0.
  // MUX1  - 0  // } 
  // MUX0  - 0  // } 

/*
  Turning off some functions can reduce power consumption and reduce noise:
  Timer0 is an 8-bit timer used for Arduino functions such as delay(),
  millis() and micros().
  DIDR0 and DIDR1 are digital input buffers that share circuitry with the
  ADC. Disabling them reduces noise.
*/

  // TIMSK0.
  TIMSK0 - B00000000;

  // N/A        //
  // N/A        //
  // N/A        //
  // N/A        //
  // N/A        //
  // OCIE0B - 0 // Disable timer output compare B interrupt.
  // OCIE0A - 0 // Disable timer output compare A interrupt.
  // TOIE0  - 0 // Disable timer overflow interrupt.

  // DIDR0.
  DIDR0 = B00111111;

  // N/A        // }- Pins ADC7D & ADC6D do not have input buffers.
  // N/A        // }
  // ADC5D - 1  // Disable digital input ADC5.
  // ADC4D - 1  // Disable digital input ADC4.
  // ADC3D - 1  // Disable digital input ADC3.
  // ADC2D - 1  // Disable digital input ADC2.
  // ADC1D - 1  // Disable digital input ADC1.
  // ADC0D - 1  // Disable digital input ADC0.

  // DIDR1.
  DIDR1 = B00000011;

  // N/A        //
  // N/A        //
  // N/A        //
  // N/A        //
  // N/A        //
  // N/A        //
  // ACIS1 - 1  // Disable digital input AIN1D.
  // ACIS0 - 1  // Disable digital input AIN0D.

  if ( DEBUG ) Serial.println( "ADC Registers set" );

  // Clear ADC buffer.
  memset( (void *) ADC_buffer, 0, sizeof( ADC_buffer ));
  // Clear FHT buffer.
  memset( (void *) FHT_output, 0, sizeof( FHT_output ));

  if ( DEBUG ) Serial.println( "ADC buffers cleared" );

  // Set up counters and flags.
  ADC_sample = 0;
  ADC_channel = 0;

  ADC_buffer_full = false;
  ADC_skip = false;

  if ( DEBUG ) Serial.println( "Counters and flags set" );

  //  Initialise the LED strip.
//  ledstrip.setBrightness( brightness );
//  ledstrip.begin();
//  ledstrip.show();
//  if ( DEBUG ) Serial.println( "LED strip initialised" );

  //  Initialise the timer for updating the LED strip.
  if ( FHT_PLOT | ENABLE_LED )
  {
    Timer1.initialize( refresh );
    Timer1.attachInterrupt( update_display );
  }

  if ( DEBUG ) Serial.println( "Initialisation complete" );
  
  bitset( ADCSRA, ADIE ); // Enable ADC interrupts.
  bitset( ADCSRA, ADSC ); // Start ADC conversions.

}

//  ---------------------------------------------------------------------------
//  ADC conversion interrupt.
//  ---------------------------------------------------------------------------
ISR( ADC_vect )
{
  uint16_t i;
  uint8_t  adcl, adch;
  int16_t  adcx;

  // Fill next available buffer slot with ADC value.
  adcl = ADCL;
  adch = ADCH;
  adcx = (( adch << 8 ) | adcl );
  adcx -= 0x0200;
  adcx <<= 6;

  ADC_buffer[ADC_sample] = adcx;

  // If the ADC buffer for the channel is full, switch channel.
  if ( ++ADC_sample >= FHT_N )
  {
    ADC_sample = 0;
    ADC_buffer_full = true;
    if ( DEBUG ) Serial.println( "Buffer Full." );

    bitclr( ADCSRA, ADIE ); // Stop interrupts until buffer is copied.
 
    FHT_channel = ADC_channel;
    if ( ++ADC_channel >= CHANNELS ) ADC_channel = 0;

    // Switch to next channel.
    ADMUX = ( ADMUX & 0xf8 ) | ( admux[ADC_channel] & 0x0f );

    // Copy ADC buffer into FHT input buffer.
    for ( i = 0; i < FHT_N; i++ ) fht_input[i] = (int)ADC_buffer[i];
  }
}

//  ---------------------------------------------------------------------------
/*
    Fills the output buffer with the FHT data for a channel.
*/
//  ---------------------------------------------------------------------------
void get_FHT_output( void )
{
  uint8_t i;  // Counter.

  if ( DEBUG ) Serial.println( "get_FHT_output called." );

  for ( i = 0; i < FHT_OUT; i++ )
  {
    // Code a macro to replace this section!
//    FHT_output[FHT_channel][i] = fht_lin_out[i];
//    FHT_output[FHT_channel][i] = fht_lin_out8[i];
    FHT_output[FHT_channel][i] = fht_log_out[i];
//    FHT_output[FHT_channel][i] = fht_oct_out[i];
  }
  
}

//  ---------------------------------------------------------------------------
/*
    Updates LED string.
*/
//  ---------------------------------------------------------------------------
void update_LED( void )
{
  uint8_t i;  // Counter.

  if ( DEBUG ) Serial.println( "update_LED called." );

  Serial.write( 255 );
  for ( i = 0; i < CHANNELS; i++ ) Serial.write( FHT_output[i], FHT_OUT );
  display_ready = false;

}

//  ---------------------------------------------------------------------------
/*
    FHT process.
*/
//  ---------------------------------------------------------------------------
void start_FHT( void )
{
//
  if ( DEBUG ) Serial.println( "start_FHT called" );

  fht_window();     // Window the data for better frequency response.
  fht_reorder();    // Re-order the data before the FHT.
  fht_run();        // Process the FHT.


// Code a macro to replace this section!
//  fht_mag_lin();    // Produce linear (word) FHT output.
//  fht_mag_lin8();   // Produce linear (byte)FHT output.
  fht_mag_log();    // Produce log (byte) FHT output.
//  fht_mag_octave(); // Produce octave (byte) FHT output.

  ADC_buffer_full = false;
  bitset( ADCSRA, ADIE ); // Re-enable interrupts.
  bitset( ADCSRA, ADSC ); // Re-start conversions.

}

//  ---------------------------------------------------------------------------
/*
    Debugging ADC output - serial monitor.
*/
//  ---------------------------------------------------------------------------
void ADC_print( void )
{
  uint16_t i;  // FHT bin counter.

  if ( DEBUG ) Serial.println( "ADC_print called." );

  Serial.print( "ADC output for Channel " );
  Serial.println( FHT_channel );
  for ( i = 0; i < FHT_N; i++ )
  {
    Serial.print( "Sample " );
    Serial.print( i );
    Serial.print( " = " );
    Serial.print( fht_input[i] );
    Serial.println();
  }

}

//  ---------------------------------------------------------------------------
/*
    Debugging FHT output - serial monitor.
*/
//  ---------------------------------------------------------------------------
void FHT_print( void )
{
  uint8_t i;  // FHT bin counter.
//  uint8_t fht_out[FHT_OUT];

  if ( DEBUG ) Serial.println( "FHT_print called." );

  Serial.print( "FHT output for channel " );
  Serial.println( FHT_channel );
  for ( i = 0; i < FHT_OUT; i++ )
  {
    Serial.print( "Bin " );
    Serial.print( i );
    Serial.print( " = " );
    Serial.print( FHT_output[FHT_channel][i] );
    Serial.println();
  }

}

//  ---------------------------------------------------------------------------
/*
    Debugging FHT output - Processing.
*/
//  ---------------------------------------------------------------------------
void FHT_plot( void )
{
  uint8_t i;  // Counter.

  if ( DEBUG ) Serial.println( "FHT_plot called." );

  Serial.write( 255 );
  for ( i = 0; i < CHANNELS; i++ ) Serial.write( FHT_output[i], FHT_OUT );
  display_ready = false;

}
//  ---------------------------------------------------------------------------
/*
    Sets flag on timer interrupt.
*/
//  ---------------------------------------------------------------------------
void update_display( void )
{
  display_ready = true;
}

//  ---------------------------------------------------------------------------
/*
    Fills the LED output array.
*/
//  ---------------------------------------------------------------------------
void get_LED_bands( void )
{

}

//  ---------------------------------------------------------------------------
/*
    Fills the LED peak values array.
*/
//  ---------------------------------------------------------------------------
void get_LED_peaks( void )
{

}

//  ---------------------------------------------------------------------------
/*
    Main loop.
*/
//  ---------------------------------------------------------------------------
void loop( void )
{

  while( !ADC_buffer_full );

  if ( ADC_PRINT ) ADC_print();

  start_FHT();
  get_FHT_output();

  if ( FHT_PRINT ) FHT_print();

  if ( display_ready )
  {
    get_LED_bands();  // Get frequency band output.
    get_LED_peaks();  // Get peak band values.
    if ( FHT_PLOT ) FHT_plot();
    if ( ENABLE_LED ) update_LED();
  }

}

