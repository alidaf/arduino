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
//  Includes.
//  ===========================================================================

#include "Spectrum_Analyser_2.h"

//  ===========================================================================
//  Global variables.
//  ===========================================================================

volatile int8_t   ADCBuffer[CHANNELS][FHT_N];
volatile uint8_t  ChannelIndex;
volatile uint16_t ADCCounter;
volatile boolean  FHTStart; // Flag to allow FHT data to be processed.

byte bandvals[CHANNELS][FREQ_BANDS];    // Frequency bands.
byte peakvals[CHANNELS][FREQ_BANDS];    // Peak hold values.

//volatile uint8_t channel;             // Current channel.

const byte admux[CHANNELS] = { ADC0D, ADC1D };  // ADC assignments.
const byte bins = FHT_OUT / FREQ_BANDS;         // Number of bins in each band.

const byte sampbins[FREQ_BANDS] = { 0, 1, 2, 3, 5, 12, 27, 60 }; // Log.

// Equispaced bins.
//const byte sampbins[FREQ_BANDS] = {  4, 12, 20, 28, 36, 44, 52, 60 }; // Median.
//const byte sampbins[FREQ_BANDS] = {  0,  8, 16, 24, 32, 40, 48, 56 }; // Start.
//const byte sampbins[FREQ_BANDS] = {  7, 15, 23, 31, 39, 47, 55, 63 }; // End.
//const byte sampbins[FREQ_BANDS] = { 0, 1, 2, 3, 5, 10, 22, 43 };      // Determined

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
//  ADC conversion interrupt.
//  ---------------------------------------------------------------------------
ISR( ADC_vect )
{
  uint8_t temp;
  byte adch;  // Content of ADCH regiser.
  byte adcl;  // Content of ADCL register.
  int  adcx;  // Content of combined ADCH and ADCL registers.
  
  // Fill next available buffer slot with ADC value.
  ADCBuffer[ChannelIndex][ADCCounter] = ADCH - 0x80; // Read ADC value.

  // If the ADC buffer for the channel is full, switch channel.
  if ( ++ADCCounter >= FHT_N )
  {
    ADCCounter = 0;

    if ( ++ChannelIndex >= CHANNELS )
    {
      ChannelIndex = 0;
      FHTStart = true; // Set flag to enable start of FHT.
    }

    // Switch to next channel.
    ADMUX &= 0xE0;                // Clear MUX bits.
    ADMUX |= admux[ChannelIndex]; // Set MUX bits for new channel.

    // Channel is not set until previous conversion is complete.
    while ( !bittst( ADCSRA, ADIF )); // Loop until ADIF bit is set.

  }
}

//  ---------------------------------------------------------------------------
//  Starts the ADC.
//  ---------------------------------------------------------------------------
void startADC( void )
{
  bitset( ADCSRA, ADEN );
  bitset( ADCSRA, ADSC );

//  bitset( ADCSRA, ADATE ); // doesn't work
}

//  ---------------------------------------------------------------------------
//  Stops the ADC.
//  ---------------------------------------------------------------------------
void stopADC( void )
{
//  bitclr( ADCSRA, ADATE ); // doesn't work.

  bitclr( ADCSRA, ADSC );
}

//  ---------------------------------------------------------------------------
//  Initialises the ADC registers and LED.
//  ---------------------------------------------------------------------------
void setup( void )
{

  // ADCSRA.
  ADCSRA = B00101011;

  // ADEN  - 0  // Disable ADC.
  // ADSC  - 0  // Disable conversions.
  // ADATE - 1  // Enable ADC auto trigger.
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
  // ADTS1 - 0  // }- Free Running Mode.
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

  // Clear buffers.
  memset( (void *) ADCBuffer, 0, sizeof( ADCBuffer ));

  // Set up counters and flags.
  ADCCounter = 0;
  ChannelIndex = 0;
  FHTStart = false;

  if ( PLOT_ADC | PLOT_FHT ) Serial.begin( 250000 );
  if ( DEBUG | PRINT_ADC | PRINT_FHT ) Serial.begin( 250000 );

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

  if ( DEBUG ) Serial.println( "Initialisation complete" );
  
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
void getFHTOutput( void )
{
  uint16_t bin;     // FHT bin counter.
  uint8_t  channel; // Channel counter.
  uint8_t  fht_out_debug[CHANNELS][FHT_OUT];  // Debugging output.
  uint8_t  fht_in_debug[CHANNELS][FHT_N];     // Debugging input.

  for ( channel = 0; channel < CHANNELS; channel++ )
  {
    if ( DEBUG )
    {
      Serial.print( "Channel " );
      Serial.print( channel );
      Serial.println();  
    }

    memcpy( fht_input, ADCBuffer[channel], FHT_N );
    if ( DEBUG ) Serial.println( "Array copied." );

//    fht_window();     // Window the data for better frequency response.
    if ( DEBUG ) Serial.println( "FHT Windowed." );
//    fht_reorder();    // Re-order the data before the FHT.
    if ( DEBUG ) Serial.println( "FHT Reordered." );
//    fht_run();        // Process the FHT.
    if ( DEBUG ) Serial.println( "FHT Run." );

//    fht_mag_lin();    // Produce linear output distribution.
//    fht_mag_lin8();   // Produce linear 8-bit output distribution.
//    fht_mag_log();    // Produce log FHT output.
//    fht_mag_octave(); // Produce octave output distribution.

    if ( DEBUG ) Serial.println( "FHT Processing Complete." );

    if ( PLOT_FHT | PRINT_FHT )
    {
      memcpy( fht_out_debug[channel], fht_mag_log, FHT_OUT );
    }

    if ( PRINT_ADC )
    {
      Serial.print( "ADC output content for Channel " );
      Serial.println( channel );
      for ( bin = 0; bin < FHT_N; bin++ )
      {
        Serial.print( "Bin " );
        Serial.print( bin );
        Serial.print( " = " );
        Serial.print( ADCBuffer[channel][bin] );
        Serial.println();
      }
    }

    if ( PRINT_FHT )
    {
      Serial.println( "FHT output content for channel " );
      Serial.println( channel );
      for ( bin = 0; bin < FHT_OUT; bin++ )
      {
        Serial.print( "Bin " );
        Serial.print( bin );
        Serial.print( " = " );
        Serial.print( fht_out_debug[channel][bin] );
        Serial.println();
      }
    }

  if ( PLOT_FHT ) Serial.write( fht_out_debug[channel], FHT_OUT );

  }


  if ( PLOT_ADC )
  {
    for ( bin = 0; bin < FHT_N; bin++ )
    {
      for ( channel = 0; channel < CHANNELS; channel++ )
      {
        Serial.print( ADCBuffer[channel][bin] );
        Serial.print( "\t" );
      }
      Serial.println();
    }
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
  if ( FHTStart )
  {
//    stopADC();
    if ( DEBUG )
    {
      Serial.println( "Calling FHT routine" );
    }
    getFHTOutput();
    FHTStart = false;
//    startADC();
  }

//    getBands();   // Get frequency band output.
//    updateLED();  // Update LEDs. This is called by interrupt.

}

