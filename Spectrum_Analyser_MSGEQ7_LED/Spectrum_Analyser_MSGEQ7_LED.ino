//  ===========================================================================
//  Arduino Stereo Audio Spectrum Analyser using MSGEQ7 and LED strip.
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
    This project requires an MSGEQ7 IC for each audio channel.
    See Circuit.txt for an example circuit. 
*/
//  ===========================================================================

//  ===========================================================================
//  Defines.
//  ===========================================================================

//  Debug output --------------------------------------------------------------
/*
    This setting toggles some debugging output via the serial port.
*/
#define DEBUG        0 // Print debug info to serial console.
#define MSGEQ7_PRINT 0 // Print MSGEQ7 data to serial console.

//  MSGEQ7 --------------------------------------------------------------------

// Note pins 0-7 are on PORTD while pins 8-13 are on PORTB.
#define MSGEQ7_STROBE   6 // Arduino digital pin for MSGEQ7 strobe.
#define MSGEQ7_RESET    7 // Arduino digital pin for MSGEQ7 reset.

#define MSGEQ7_BINS     7 // Number of frequency bands.
#define MSGEQ7_CHANNELS 2 // Number of audio channels (1 MSGEQ7 for each).
#define MSGEQ7_MAX   1023 // Max value for MSGEQ7 output (5V).

//  LED strip -----------------------------------------------------------------

// Note pins 0-7 are on PORTD while pins 8-13 are on PORTB.
#define LED_DATA        10 // Data pin for WS2812 LED strip.
#define LED_COLOUR     GRB // Colour order for LED strip.

#define LED_BRIGHT       5 // Initial LED brightness.
#define LED_REFRESH   5000 // Initial refresh time for LEDs (us).
#define LED_SCALE        1 // Scale factor for MSGEQ7 output.
#define LED_GAP          2 // Number of LEDS skipped between channels.
#define LED_COUNT       17 // Number of LEDS representing each band.
#define LED_HOLD        30 // Number of cycles to sustain peak values.
#define LED_DECAY        2 // Number of cycles before dropping peak level.
#define LED_DEVICE WS2812B // FastLED device name.
#define LED_NUM MSGEQ7_CHANNELS * MSGEQ7_BINS * LED_COUNT + LED_GAP

//  Colours -------------------------------------------------------------------
/*
#define RED   0xff0000
#define GREEN 0x00ff00
#define BLUE  0x0000ff
#define AMBER 0xfeff00
#define GREY1 0xf0f0f0
#define GREY2 0x0f0f0f
#define BLACK 0x000000
*/
#define COLOUR_BASE CRGB::Green // Base level colour.
#define COLOUR_PEAK CRGB::Red   // Peak level colour.
#define COLOUR_HOLD CRGB::Blue  // Peak hold colour.
#define COLOUR_VOID CRGB::Black // Void colour.

//  ADC -----------------------------------------------------------------------

#define ADC_CHANNELS 6 // Maximum number of ADC channels.

//  Bit tweaking macros --------------------------------------------------------

#define bitset( reg, bit ) ( reg |= ( 1 << bit ))   // Set register bit
#define bitclr( reg, bit ) ( reg &= ~( 1 << bit ))  // Clear register bit.
#define bittst( reg, bit ) ( reg & ( 1 << bit ))    // Test register bit.

//  ===========================================================================
//  Includes.
//  ===========================================================================

#include <Arduino.h>
#include <FastLED.h>

//  ===========================================================================
//  Global variables.
//  ===========================================================================

volatile uint16_t MSGEQ7_data[MSGEQ7_CHANNELS][MSGEQ7_BINS];

volatile uint8_t LED_mean_level[MSGEQ7_CHANNELS][MSGEQ7_BINS];
volatile uint8_t LED_peak_level[MSGEQ7_CHANNELS][MSGEQ7_BINS];
volatile uint8_t LED_peak_count[MSGEQ7_CHANNELS][MSGEQ7_BINS];
volatile uint8_t LED_hold_count[MSGEQ7_CHANNELS][MSGEQ7_BINS];

// ADC assignments.
const byte admux[ADC_CHANNELS] = { ADC0D, ADC1D, ADC2D, ADC3D, ADC4D, ADC5D };

#if MSGEQ7_CHANNELS > ADC_CHANNELS
  #define MSGEQ7_CHANNELS ADC_CHANNELS
#endif

CRGB leds[LED_NUM];

byte brightness = LED_BRIGHT;  // Set initial LED brightness.
long refresh    = LED_REFRESH; // Set initial time (uS) between updates.

uint16_t led_level = (uint16_t)( MSGEQ7_MAX / ( LED_COUNT * LED_SCALE ));

// Colour reference table for vertical meter scale.
uint32_t LED_colours[LED_COUNT];

//  ===========================================================================
//  Code.
//  ===========================================================================

//  ---------------------------------------------------------------------------
//  Initialises the ADC registers and LED.
//  ---------------------------------------------------------------------------
void setup( void )
{

  if ( DEBUG | MSGEQ7_PRINT ) Serial.begin( 115200 );

  if ( DEBUG ) Serial.println( "Starting Initialisation" );

  // ADCSRA.
  ADCSRA = B10000101;

  // ADEN  - 1  // Enable ADC.
  // ADSC  - 0  // Disable conversions.
  // ADATE - 0  // Disable ADC auto trigger.
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

  if ( DEBUG ) Serial.println( "ADC Registers set." );

  // Clear MSGEQ7 buffers.
  memset( (void *) MSGEQ7_data,    0, sizeof( MSGEQ7_data ));
  memset( (void *) LED_mean_level, 0, sizeof( LED_mean_level ));
  memset( (void *) LED_peak_level, 0, sizeof( LED_peak_level ));
  memset( (void *) LED_peak_count, 0, sizeof( LED_peak_count ));
  memset( (void *) LED_hold_count, 0, sizeof( LED_hold_count ));

  if ( DEBUG ) Serial.println( "ADC buffers cleared." );

  // Initialise MSGEQ7.
  bitset( DDRD, MSGEQ7_RESET );  // Set pin as output.
  bitset( DDRD, MSGEQ7_STROBE ); // Set pin as output.

  if ( DEBUG ) Serial.println( "MSGEQ7 initialised." );

  // Initialise LED strip.
//  delay( 3000 ); // Safety delay for power-up.
  FastLED.addLeds<LED_DEVICE, LED_DATA, LED_COLOUR>( leds, LED_NUM );
  FastLED.setBrightness( LED_BRIGHT );

  if ( DEBUG ) Serial.println( "LED strip initialised." );

  // Set up colour table for vertical meter scale.
  uint16_t i;
  uint16_t  r1, r2, g1, g2, b1, b2; // Need to be
  uint32_t  r, g, b;

  r1 = ( COLOUR_BASE >> 16 ) & 0x00ff;
  r2 = ( COLOUR_PEAK >> 16 ) & 0x00ff;
  g1 = ( COLOUR_BASE >>  8 ) & 0x00ff;
  g2 = ( COLOUR_PEAK >>  8 ) & 0x00ff;
  b1 = ( COLOUR_BASE >>  0 ) & 0x00ff;
  b2 = ( COLOUR_PEAK >>  0 ) & 0x00ff;

  // Need to add weight factors to bias RGB colours.
  
  for ( i = 0; i < LED_COUNT; i++ )
  {
    r = r1 + i * ( r2 - r1 ) / ( LED_COUNT - 1 );
    g = g1 + i * ( g2 - g1 ) / ( LED_COUNT - 1 );
    b = b1 + i * ( b2 - b1 ) / ( LED_COUNT - 1 );
    LED_colours[i] = (( r << 16 ) & 0xff0000 ) + 
                     (( g <<  8 ) & 0x00ff00 ) + 
                     (( b <<  0 ) & 0x0000ff );
  }

  if ( DEBUG )
  {
    Serial.println( "LED colour scale defined:" );
    for ( i = 0; i < LED_COUNT; i++ )
    {
      Serial.print( i );
      Serial.print( " " );
      Serial.println( LED_colours[i], HEX );
    }
  }

  // Basic LED test - light up first 3 LEDs.
  leds[0] = CRGB::Green;
  leds[1] = CRGB::Yellow;
  leds[2] = CRGB::Red;
  FastLED.show();
  delay( 1000 );

  fill_gradient_RGB( leds, 0, CRGB::COLOUR_BASE, LED_NUM - 1, CRGB::COLOUR_PEAK );
  FastLED.show();
  delay( 1000 );

  fill_solid( leds, LED_NUM, COLOUR_BASE );
  FastLED.show();
  delay( 1000 );

  fill_solid( leds, LED_NUM, COLOUR_PEAK );
  FastLED.show();
  delay( 1000 );

  fill_solid( leds, LED_NUM, COLOUR_HOLD );
  FastLED.show();
  delay( 1000 );

  // Wipe LED strip.
  fill_solid( leds, LED_NUM, COLOUR_VOID );
  FastLED.show();
  delay( 50 );

  if ( DEBUG )
  {
    Serial.println( "Levels have been set to:" );
    for ( uint8_t i = 0; i < LED_COUNT; i++ )
    {
      Serial.print( "Level " );
      Serial.print( i );
      Serial.print( " = " );
      Serial.print( i * led_level );
      Serial.print( " - " );
      Serial.println(( i + 1 ) * led_level );
    }
  }

  if ( DEBUG ) Serial.println( "Initialisation complete." );
  
}

//  ---------------------------------------------------------------------------
//  Reads data from MSGEQ7s via ADCs.
//  ---------------------------------------------------------------------------
void MSGEQ7_get_data( void )
{
  uint8_t  channel;
  uint8_t  bin;

  uint8_t  adcl, adch;
  uint16_t adcx;

  // Reset MSGEQ7.
  bitset( PORTD, MSGEQ7_RESET );
  bitclr( PORTD, MSGEQ7_RESET );
  delayMicroseconds( 75 );

  for ( bin = 0; bin < MSGEQ7_BINS; bin++ )
  {
    bitclr( PORTD, MSGEQ7_STROBE );
//    digitalWrite( MSGEQ7_STROBE, LOW );
    delayMicroseconds( 40 );

    for ( channel = 0; channel < MSGEQ7_CHANNELS; channel++ )
    {

      ADMUX = ( ADMUX & 0xf8 ) | ( admux[channel] & 0x0f );
      bitset( ADCSRA, ADSC ); // Start ADC conversion.
      while bittst( ADCSRA, ADSC );
      adcl = ADCL;
      adch = ADCH;
      adcx = (( adch << 8 ) | adcl );
      MSGEQ7_data[channel][bin] = adcx;
      
    }
    bitset( PORTD, MSGEQ7_STROBE );
    delayMicroseconds( 10 );

  }
}
//  ---------------------------------------------------------------------------
//  Updates LED levels.
//  ---------------------------------------------------------------------------
void update_levels( void )
{
  uint8_t channel;
  uint8_t bin;

  uint8_t mean;
  uint8_t peak;

  if ( DEBUG ) Serial.println( "update_levels called." );

  for ( channel = 0; channel < MSGEQ7_CHANNELS; channel++ )
  {
    for ( bin = 0; bin < MSGEQ7_BINS; bin++ )
    {
      for ( mean = LED_COUNT - 1; mean > 0; mean-- )
      {
        if ( MSGEQ7_data[channel][bin] >= mean * led_level ) break;
      }
      LED_mean_level[channel][bin] = mean;

      // Compare mean level against peak and hold if greater.
      if ( mean > LED_peak_level[channel][bin] )
      {
        LED_peak_level[channel][bin] = mean;
        LED_hold_count[channel][bin] = LED_HOLD;
      }
      else
      {
        // If hold count = 0 then peak level is decaying.
        if ( LED_hold_count[channel][bin] == 0 )
        {
          // if decay count = 0 then move level down and reset count
          if ( LED_peak_count[channel][bin] == 0 )
          {
            if ( LED_peak_level[channel][bin] > 0 )
            {
              LED_peak_level[channel][bin]--;
            }
            LED_peak_count[channel][bin] = LED_DECAY;
          }
          else
          {
            LED_peak_count[channel][bin]--;
          }
        }
        else
        {
          LED_hold_count[channel][bin]--;
        }
      }
    }
  }

  if ( DEBUG )
  {
    Serial.println( "LED levels:" );
    for ( channel = 0; channel < MSGEQ7_CHANNELS; channel++ )
    {
      for ( bin = 0; bin < MSGEQ7_BINS; bin++ )
      {
        Serial.print( "Channel " );
        Serial.print( channel );
        Serial.print( ", Bin " );
        Serial.print( bin );
        Serial.print( ", Mean " );
        Serial.print( MSGEQ7_data[channel][bin] );
        Serial.print( " (" );
        Serial.print( LED_mean_level[channel][bin] );
        Serial.print( ") (" );
        Serial.print( LED_peak_level[channel][bin] );
        Serial.print( ")" );
        Serial.println();
      }
    }
  }
}
//  ---------------------------------------------------------------------------
//  Updates LED string.
//  ---------------------------------------------------------------------------
void LED_update( void )
{
  uint8_t channel;
  uint8_t bin;

  uint8_t mean;
  uint8_t peak;
  uint8_t led_write;

  uint16_t pos;
  uint16_t base;
  uint16_t index;

  if ( DEBUG ) Serial.println( "update_LED called." );

  for ( channel = 0; channel < MSGEQ7_CHANNELS; channel++ )
  {
    base = channel * ( MSGEQ7_BINS * LED_COUNT + LED_GAP );
    for ( bin = 0; bin < MSGEQ7_BINS; bin++ )
    {
      index = base + bin * LED_COUNT;

      mean = LED_mean_level[channel][bin];
      peak = LED_peak_level[channel][bin];

      fill_gradient_RGB( leds, index, COLOUR_BASE, 
                               index + LED_COUNT - 1, COLOUR_PEAK );

      fill_solid( &( leds[index+mean] ), LED_COUNT - mean, COLOUR_VOID );

      // Add peak level
      leds[index + peak] = COLOUR_HOLD;
    }
  }
  FastLED.show();
  delay( 20 );
}

//  ---------------------------------------------------------------------------
//  Debugging MSGEQ7 output - serial monitor.
//  ---------------------------------------------------------------------------
void MSGEQ7_print( void )
{
  uint8_t channel; // MSGEQ7 channel counter.
  uint8_t bin;     // MSGEQ7 bin counter.

  if ( DEBUG ) Serial.println( "MSGEQ7_print called." );

  for ( channel = 0; channel < MSGEQ7_CHANNELS; channel++ )
  {
    Serial.print( "MSGEQ7 output for Channel " );
    Serial.print( channel );
    Serial.println( ":" );
    for ( bin = 0; bin < MSGEQ7_BINS; bin++ )
    {
      Serial.print( MSGEQ7_data[channel][bin] );
      Serial.print( ", " );
    }
    Serial.println();
  }
}

//  ---------------------------------------------------------------------------
//  Main loop.
//  ---------------------------------------------------------------------------
void loop( void )
{

  MSGEQ7_get_data();

  if ( MSGEQ7_PRINT ) MSGEQ7_print();
  update_levels();
  LED_update();

}

// Set up palette for meter levels.
DEFINE_GRADIENT_PALETTE( base_to_peak )
{
  0,               0, 255,  0,
  LED_COUNT - 1, 255,   0,  0,
  255,             0,   0,  0
};


