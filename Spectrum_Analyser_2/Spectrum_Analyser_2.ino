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

#define DEBUG     0 // Print debug info to serial monitor.
#define PLOT_ADC  0 // Send ADC data to serial plotter.
#define PLOT_FHT  0 // Send FHT data to serial plotter.
#define PRINT_ADC 1 // Print ADC data to serial monitor.
#define PRINT_FHT 1 // Print FHT data to serial monitor.

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

volatile int8_t   ADC_buffer[CHANNELS][FHT_N];  // Ring buffer for ADC data.
volatile uint8_t  ADC_channel;                  // Keeps track of ADC channel.
volatile uint16_t ADC_sample;                   // Keeps track of ADC sample.
volatile boolean  ADC_buffer_full;                    // Flag to start FHT process.

volatile int8_t   ADC_copy[CHANNELS][FHT_N];     // Working copy of ADC input buffers.
volatile uint8_t  FHT_output[CHANNELS][FHT_OUT]; // FHT output buffers.

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

  // Fill next available buffer slot with ADC value.
  ADC_buffer[ADC_channel][ADC_sample] = ADCH - 0x80; // Read ADC value.

  // If the ADC buffer for the channel is full, switch channel.
  if ( ++ADC_sample >= FHT_N )
  {
    ADC_sample = 0;

    if ( ++ADC_channel >= CHANNELS )
    {
      ADC_channel = 0;
      ADC_buffer_full = true; // Set flag to enable start of FHT.
      // Make a working copy of the ADC input buffer.
      memcpy( ADC_copy, ADC_buffer, sizeof( ADC_buffer ));
    }

    // Switch to next channel.
    ADMUX = ( ADMUX & 0xf8 ) | ( admux[ADC_channel] & 0x0f );
  }

  startADC(); // Not needed in free running mode.
}

//  ---------------------------------------------------------------------------
//  Starts the ADC.
//  ---------------------------------------------------------------------------
void startADC( void )
{
  bitset( ADCSRA, ADSC );
}

//  ---------------------------------------------------------------------------
//  Stops the ADC.
//  ---------------------------------------------------------------------------
void stopADC( void )
{
  bitclr( ADCSRA, ADSC );
}

//  ---------------------------------------------------------------------------
//  Initialises the ADC registers and LED.
//  ---------------------------------------------------------------------------
void setup( void )
{
  if ( PLOT_ADC | PLOT_FHT ) Serial.begin( 250000 );
  if ( DEBUG | PRINT_ADC | PRINT_FHT ) Serial.begin( 250000 );

  if ( DEBUG ) Serial.println( "Starting Initialisation" );

  // ADCSRA.
  ADCSRA = B10001011;

  // ADEN  - 1  // Enable ADC.
  // ADSC  - 0  // Disable conversions.
  // ADATE - 0  // Disable ADC auto trigger.
  // ADIF  - 0  // Set by hardware
  // ADIE  - 1  // Enable interrupt.
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

  startADC();

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
  uint8_t  channel; // Channel counter.


  if ( DEBUG ) Serial.println( "Starting FHT" );

  for ( channel = 0; channel < CHANNELS; channel++ )
  {
    // Copy the working channel buffer into FHT input buffer.
//    memcpy( fht_input, ADC_copy[channel], sizeof( ADC_copy[channel] ));
    for ( sample = 0; sample < FHT_N; sample++ )
    {
      fht_input[sample] = ADC_copy[channel][sample];
    }

    fht_window();     // Window the data for better frequency response.
    fht_reorder();    // Re-order the data before the FHT.
    fht_run();        // Process the FHT.

//    fht_mag_lin();    // Produce linear output distribution.
//    fht_mag_lin8();   // Produce linear 8-bit output distribution.
    fht_mag_log();    // Produce log FHT output.
//    fht_mag_octave(); // Produce octave output distribution.

    if ( DEBUG ) Serial.println( "FHT Processing Complete." );

//    if ( PLOT_FHT ) Serial.write( (uint8_t*)fht_mag_log, FHT_OUT );

    // Copy FHT output for channel into output buffer.
    for ( sample = 0; sample < FHT_OUT; sample++ )
    {
      FHT_output[channel][sample] = fht_mag_log[sample];
    }
  }

}

//  ---------------------------------------------------------------------------
/*
    This provides some debugging output for the ADC.
*/
//  ---------------------------------------------------------------------------
void ADC_monitor( void )
{
  uint16_t sample;  // FHT bin counter.
  uint8_t  channel; // Channel counter.

  if ( PRINT_ADC )
  {
    for ( channel = 0; channel < CHANNELS; channel++ )
    {
      Serial.print( "ADC output content for Channel " );
      Serial.println( channel );
      for ( sample = 0; sample < FHT_N; sample++ )
      {
        Serial.print( "Bin " );
        Serial.print( sample );
        Serial.print( " = " );
        Serial.print( ADC_copy[channel][sample] );
        Serial.println();
      }
    }
  }

  if ( PLOT_ADC ) // Should be used with Arduino Plotter.
  {
    for ( sample = 0; sample < FHT_N; sample++ )
    {
      for ( channel = 0; channel < CHANNELS; channel++ )
      {
        Serial.print( ADC_copy[channel][sample] );
        Serial.print( "\t" );
      }
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
  uint16_t sample;  // FHT bin counter.
  uint8_t  channel; // Channel counter.

  if ( PRINT_FHT )
  {
    for ( channel = 0; channel < CHANNELS; channel++ )
    {
      Serial.print( "FHT output content for channel " );
      Serial.println( channel );
      for ( sample = 0; sample < FHT_OUT; sample++ )
      {
        Serial.print( "Bin " );
        Serial.print( sample );
        Serial.print( " = " );
        Serial.print( FHT_output[channel][sample] );
        Serial.println();
      }
    }
  }

//  if ( PLOT_FHT ) // Should be used with an external serial grapher.
//  {
//    Serial.write( 255 );
//    for ( channel = 0; channel < CHANNELS; channel++ )
//    {
//      Serial.write( (uint8_t*)FHT_output[channel], FHT_OUT );
//    }
//  }

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
  while ( 1 )
  {
  
    while ( !ADC_buffer_full );
    if ( DEBUG ) Serial.println( "Calling FHT routine" );
    if ( PLOT_ADC | PRINT_ADC ) ADC_monitor();
    start_FHT();
    if ( PLOT_FHT | PRINT_FHT ) FHT_monitor();
    ADC_buffer_full = false;

//    getBands();   // Get frequency band output.
//    updateLED();  // Update LEDs. This is called by interrupt.
  }
}

