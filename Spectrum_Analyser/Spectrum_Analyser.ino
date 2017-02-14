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
//  Defines.
//  ===========================================================================

//  Bit tweaking macros --------------------------------------------------------
#define bitset( reg, bit ) ( reg |= ( 1 << bit ))   // Set register bit
#define bitclr( reg, bit ) ( reg &= ~( 1 << bit ))  // Clear register bit.
#define bittst( reg, bit ) ( reg & ( 1 << bit ))    // Test register bit.

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
    This setting toggles some debugging output via the serial port. Since
    there is ony 1 serial channel, these are mutually exclusive. Output is
    sent as bytes and needs a suitable means of displaying the output to be
    purposeful.

    PD-extended has been used here with the patch from the 
    openmusiclabs site.

            see the link to FHT_128_channel_analyser.zip at
            http://wiki.openmusiclabs.com/wiki/ArduinoFHT.
*/

#define DEBUG     0 // Print debug info to serial console.
#define PLOT_FHT  1 // Send FHT byte data to serial console.
#define PRINT_ADC 0 // Print ADC data to serial console.
#define PRINT_FHT 0 // Print FHT data to serial console.

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

volatile int8_t   ADC_buffer[FHT_N];  // Ring buffer for ADC data.
volatile uint8_t  ADC_channel;        // Keeps track of ADC channel.
volatile uint16_t ADC_sample;         // Keeps track of ADC sample.
volatile uint8_t  FHT_channel;        // Keeps track of FHT channel.

volatile boolean  ADC_buffer_full;    // Flag to start FHT process.
volatile boolean  ADC_ready;          // Flag to control ADC reads.
volatile boolean  FHT_busy;           // Flag to stop ADC if FHT is busy.


byte bandvals[CHANNELS][FREQ_BANDS];    // Frequency bands.
byte peakvals[CHANNELS][FREQ_BANDS];    // Peak hold values.

const byte admux[CHANNELS] = { ADC0D, ADC1D };  // ADC assignments.
const byte bins = FHT_OUT / FREQ_BANDS;         // Number of bins in each band.

const byte sampbins[FREQ_BANDS] = { 0, 1, 2, 3, 5, 12, 27, 60 }; // Log.

// Equispaced bins.
//const byte sampbins[FREQ_BANDS] = {  4, 12, 20, 28, 36, 44, 52, 60 }; // Median.
//const byte sampbins[FREQ_BANDS] = {  0,  8, 16, 24, 32, 40, 48, 56 }; // Start.
//const byte sampbins[FREQ_BANDS] = {  7, 15, 23, 31, 39, 47, 55, 63 }; // End.
//const byte sampbins[FREQ_BANDS] = {  0,  1,  2,  3,  5, 10, 22, 43 }; // Determined

float scaleLED;

Adafruit_NeoPixel ledstrip = Adafruit_NeoPixel( FREQ_BANDS * FREQ_LEDS,
                                                LED_DATA_PIN,
                                                NEO_GRBW + NEO_KHZ800);

byte brightness = INIT_BRIGHT;        // Set initial LED brightness.
long refresh = INIT_REFRESH;          // Set initial time (uS) between updates.
byte filling = 0;

//  ===========================================================================
//  Code.
//  ===========================================================================

//  ---------------------------------------------------------------------------
//  ADC conversion interrupt.
//  ---------------------------------------------------------------------------
ISR( ADC_vect )
{
  uint16_t i;

  // Fill next available buffer slot with ADC value.
  ADC_buffer[ADC_sample] = ADCH - 0x80; // Read & store ADC value.

  // If the ADC buffer for the channel is full, switch channel.
  if ( ++ADC_sample >= FHT_N )
  {
    ADC_sample = 0;
    ADC_buffer_full = true;
    if ( DEBUG ) Serial.println( "Buffer Full." );

    bitclr( ADCSRA, ADIE ); // Stop interrupts.
 
    FHT_channel = ADC_channel;
    if ( ++ADC_channel >= CHANNELS ) ADC_channel = 0;

    // Switch to next channel.
    ADMUX = ( ADMUX & 0xf8 ) | ( admux[ADC_channel] & 0x0f );

    // Copy ADC buffer into FHT input buffer.
    for ( i = 0; i < FHT_N; i++ ) fht_input[i] = (int)ADC_buffer[i];
  }
}

//  ---------------------------------------------------------------------------
//  Initialises the ADC registers and LED.
//  ---------------------------------------------------------------------------
void setup( void )
{

  if ( PLOT_FHT ) Serial.begin( 115200 );
  if ( DEBUG | PRINT_ADC | PRINT_FHT ) Serial.begin( 115200 );

  if ( DEBUG ) Serial.println( "Starting Initialisation" );

  // ADCSRA.
  ADCSRA = B10100101;

  // ADEN  - 1  // Enable ADC.
  // ADSC  - 0  // Disable conversions.
  // ADATE - 1  // Enable ADC auto trigger.
  // ADIF  - 0  // Set by hardware
  // ADIE  - 0  // Enable interrupt.
  // ADPS2 - 0  // }
  // ADPS1 - 1  // }- ADC prescaler = 8.
  // ADPS0 - 1  // }

  // ADCSRB.
  ADCSRB = B00000000;

  // N/A        //
  // ACME  - 0  // Disable Analogue Comparator Mux.
  // N/A        //
  // N/A        //
  // N/A        //
  // ADTS2 - 0  // }
  // ADTS1 - 0  // }- Free Running Mode - not active (ADATE = 0).
  // ADTS0 - 0  // }

  // ADMUX.
  ADMUX = B01100000;
  
  // REFS1 - 0  // }- Use Vcc as reference.
  // REFS0 - 1  // } 
  // ADLAR - 1  // Left adjust ADC results (8-bit mode).
  // N/A        //
  // MUX3  - 0  // } 
  // MUX2  - 0  // }- ADC input 0.
  // MUX1  - 0  // } 
  // MUX0  - 0  // } 

  //  Turning off some functions can reduce power consumption and reduce noise.
  //
  //  Timer0 is an 8-bit timer used for Arduino functions such as delay(),
  //  millis() and micros().
  //  DIDR0 and DIDR1 are digital input buffers that share circuitry with the
  //  ADC. Disabling them reduces noise.

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

  // Clear buffers.
  memset( (void *) ADC_buffer, 0, sizeof( ADC_buffer ));

  if ( DEBUG ) Serial.println( "ADC buffers cleared" );

  // Set up counters and flags.
  ADC_sample = 0;
  ADC_channel = 0;

  ADC_buffer_full = false;
  FHT_busy = false;

  if ( DEBUG ) Serial.println( "Counters and flags set" );

//  ---------------------------------------------------------------------------
//  Initialises the LED strip.
//  ---------------------------------------------------------------------------

//  ledstrip.setBrightness( brightness );
//  ledstrip.begin();
//  ledstrip.show();
  if ( DEBUG ) Serial.println( "LED strip initialised" );


//  ---------------------------------------------------------------------------
//  Initialises the timer.
//  ---------------------------------------------------------------------------

//  Timer1.initialize( refresh );
//  Timer1.attachInterrupt( updateLED );

  if ( DEBUG ) Serial.println( "Initialisation complete" );
  
  if ( DEBUG ) Serial.println( "Starting ADC" );

  bitset( ADCSRA, ADIE ); // Enable ADC interrupts.
  bitset( ADCSRA, ADSC ); // Start ADC conversions.

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
//        Serial.write( fht_lin_fix, FHT_OUT );
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
void start_FHT( void )
{
  uint16_t sample;  // FHT bin counter.

  if ( DEBUG ) Serial.println( "Starting FHT" );

//  while ( bittst( ADCSRA, ADIE ));

  fht_window();     // Window the data for better frequency response.
  fht_reorder();    // Re-order the data before the FHT.
  fht_run();        // Process the FHT.

//  fht_mag_lin();    // Produce linear output distribution.
//  fht_mag_lin8();   // Produce linear 8-bit output distribution.
  fht_mag_log();    // Produce log FHT output.
//  fht_mag_octave(); // Produce octave output distribution.

//  FHT_busy = false;

  if ( DEBUG ) Serial.println( "FHT Processing Complete." );

}

//  ---------------------------------------------------------------------------
/*
    This provides some debugging output for the ADC.
*/
//  ---------------------------------------------------------------------------
void ADC_monitor( void )
{
  uint16_t i;  // FHT bin counter.

  if ( PRINT_ADC )
  {
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

}

//  ---------------------------------------------------------------------------
/*
    This provides some debugging output for the FHT.
*/
//  ---------------------------------------------------------------------------
void FHT_monitor( void )
{
  uint16_t i;  // FHT bin counter.
  uint8_t fht_out[FHT_OUT];

  if ( PRINT_FHT )
  {
    Serial.print( "FHT output for channel " );
    Serial.println( FHT_channel );
    for ( i = 0; i < FHT_OUT; i++ )
    {
      Serial.print( "Bin " );
      Serial.print( i );
      Serial.print( " = " );
      Serial.print( fht_log_out[i] );
      Serial.println();
    }
  }

  if ( PLOT_FHT ) // Should be used with an external serial grapher.
  {
    for ( i = 0; i < FHT_OUT; i++ ) fht_out[i] = fht_log_out[i];
    if ( FHT_channel == 0 ) Serial.write( 255 );
    Serial.write( fht_out, FHT_OUT );
  }

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
void loop( void )
{

  while( !ADC_buffer_full );
  {

    if ( DEBUG ) Serial.println( "Calling FHT routine" );
    if ( PRINT_ADC ) ADC_monitor();

    start_FHT();

    if ( PLOT_FHT | PRINT_FHT ) FHT_monitor();

    ADC_buffer_full = false;
    bitset( ADCSRA, ADIE ); // Re-enable interrupts.
    bitset( ADCSRA, ADSC ); // Re-start conversions.

  }

//  getBands();   // Get frequency band output.
//  updateLED();  // Update LEDs. This is called by interrupt.
}

