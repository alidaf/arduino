//  ===========================================================================
//  Arduino Stereo Audio Spectrum Analyser using MSGEQ7 and LCD.
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

#define MSGEQ7_STROBE   7 // Arduino digital pin for MSGEQ7 strobe.
#define MSGEQ7_RESET    8 // Arduino digital pin for MSGEQ7 reset.
#define MSGEQ7_BINS     7 // Number of frequency bands.
#define MSGEQ7_CHANNELS 2 // Number of audio channels (1 MSGEQ7 for each).
#define MSGEQ7_CYCLES   3 // Number of cycles to keep peak values.
#define MSGEQ7_MAX   1023 // Max value for MSGEQ7 output (5V).

//  LCD display ---------------------------------------------------------------

#define LCD_RS    12 // LCD RS pin.
#define LCD_EN    11 // LCD Enable pin.
#define LCD_D4     5 // LCD data pin D4.
#define LCD_D5     4 // LCD data pin D5.
#define LCD_D6     3 // LCD data pin D6.
#define LCD_D7     2 // LCD data pin D7.
#define LCD_SCALE  1 // Scale factor for MSGEQ7 output.
#define LCD_GAP    2 // Display gap between channels.
#define LCD_COUNT  8 // Number of possible display levels.

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
#include <Wire.h>
#include <LiquidCrystal.h>
#include <TimerOne.h>
#include <Adafruit_NeoPixel.h>

//  ===========================================================================
//  Global variables.
//  ===========================================================================

volatile uint16_t MSGEQ7_data[MSGEQ7_CHANNELS][MSGEQ7_BINS];
volatile uint16_t MSGEQ7_peak[MSGEQ7_CHANNELS][MSGEQ7_BINS];
volatile uint8_t  MSGEQ7_count[MSGEQ7_CHANNELS][MSGEQ7_BINS];

// ADC assignments.
const byte admux[ADC_CHANNELS] = { ADC0D, ADC1D, ADC2D, ADC3D, ADC4D, ADC5D };

#if MSGEQ7_CHANNELS > ADC_CHANNELS
  #define MSGEQ7_CHANNELS ADC_CHANNELS
#endif

LiquidCrystal lcd( LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7 );

uint8_t lcd_char0[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f};
uint8_t lcd_char1[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f};
uint8_t lcd_char2[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x1f};
uint8_t lcd_char3[8] = { 0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x1f, 0x1f};
uint8_t lcd_char4[8] = { 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f};
uint8_t lcd_char5[8] = { 0x00, 0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f};
uint8_t lcd_char6[8] = { 0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f};
uint8_t lcd_char7[8] = { 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f};

uint16_t lcd_level = ( MSGEQ7_MAX / ( LCD_COUNT * LCD_SCALE ));

//  ===========================================================================
//  Code.
//  ===========================================================================

//  ---------------------------------------------------------------------------
//  Initialises the ADC registers and LED.
//  ---------------------------------------------------------------------------
void setup( void )
{

  if ( DEBUG |  MSGEQ7_PRINT ) Serial.begin( 115200 );

  if ( DEBUG ) Serial.println( "Starting Initialisation" );

  // ADCSRA.
//  ADCSRA = B10000101;

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
//  ADCSRB = B00000000;

  // N/A        //
  // ACME  - 0  // Disable Analogue Comparator Mux.
  // N/A        //
  // N/A        //
  // N/A        //
  // ADTS2 - 0  // }
  // ADTS1 - 0  // }- Free Running Mode.
  // ADTS0 - 0  // }

  // ADMUX.
//  ADMUX = B01000000;
  
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
//  TIMSK0 - B00000000;

  // N/A        //
  // N/A        //
  // N/A        //
  // N/A        //
  // N/A        //
  // OCIE0B - 0 // Disable timer output compare B interrupt.
  // OCIE0A - 0 // Disable timer output compare A interrupt.
  // TOIE0  - 0 // Disable timer overflow interrupt.

  // DIDR0.
//  DIDR0 = B00111111;

  // N/A        // }- Pins ADC7D & ADC6D do not have input buffers.
  // N/A        // }
  // ADC5D - 1  // Disable digital input ADC5.
  // ADC4D - 1  // Disable digital input ADC4.
  // ADC3D - 1  // Disable digital input ADC3.
  // ADC2D - 1  // Disable digital input ADC2.
  // ADC1D - 1  // Disable digital input ADC1.
  // ADC0D - 1  // Disable digital input ADC0.

  // DIDR1.
//  DIDR1 = B00000011;

  // N/A        //
  // N/A        //
  // N/A        //
  // N/A        //
  // N/A        //
  // N/A        //
  // ACIS1 - 1  // Disable digital input AIN1D.
  // ACIS0 - 1  // Disable digital input AIN0D.

  if ( DEBUG ) Serial.println( "ADC Registers set" );

  // Clear MSGEQ7 buffers.
  memset( (void *) MSGEQ7_data, 0, sizeof( MSGEQ7_data ));
  memset( (void *) MSGEQ7_peak, 0, sizeof( MSGEQ7_peak ));

  if ( DEBUG ) Serial.println( "ADC buffers cleared" );

  // Initialise MSGEQ7.
  pinMode( MSGEQ7_RESET, OUTPUT );
  pinMode( MSGEQ7_STROBE, OUTPUT );

  // Initialise LCD.
  lcd.begin( 16, 2 );
  lcd.clear();
  lcd.createChar( 0, lcd_char0 );
  lcd.createChar( 1, lcd_char1 );
  lcd.createChar( 2, lcd_char2 );
  lcd.createChar( 3, lcd_char3 );
  lcd.createChar( 4, lcd_char4 );
  lcd.createChar( 5, lcd_char5 );
  lcd.createChar( 6, lcd_char6 );
  lcd.createChar( 7, lcd_char7 );
  lcd.setCursor( 0, 0 );
            
  lcd.print( "  L  MSGEQ7  R  " );

  if ( DEBUG ) Serial.println( "LCD display initialised" );

  if ( DEBUG )
  {
    Serial.println( "Levels have been set to:" );
    for ( uint8_t i = 0; i < LCD_COUNT; i++ )
    {
      Serial.print( "Level " );
      Serial.print( i );
      Serial.print( " = " );
      Serial.print( i * lcd_level );
      Serial.print( " - " );
      Serial.println(( i + 1 ) * lcd_level );
    }
  }

  if ( DEBUG ) Serial.println( "Initialisation complete" );
  
//  bitset( ADCSRA, ADIE ); // Enable ADC interrupts.
//  bitset( ADCSRA, ADSC ); // Start ADC conversions.
}

//  ---------------------------------------------------------------------------
//  ADC conversion interrupt.
//  ---------------------------------------------------------------------------
void MSGEQ7_get_data( void )
{
  uint8_t  channel;
  uint8_t  bin;

  uint8_t  adcl, adch;
  uint16_t adcx;
  uint16_t peak;

  // Reset MSGEQ7.
  digitalWrite( MSGEQ7_RESET, HIGH );
  digitalWrite( MSGEQ7_RESET, LOW );
  delayMicroseconds( 75 );


//    ADMUX = ( ADMUX & 0xf8 ) | ( admux[channel] & 0x0f );

  for ( bin = 0; bin < MSGEQ7_BINS; bin++ )
  {
//      bitset( ADCSRA, ADSC ); // Start ADC conversions.
    digitalWrite( MSGEQ7_STROBE, LOW );
    delayMicroseconds( 40 );

//      adcl = ADCL;
//      adch = ADCH;
//      adcx = (( adch << 8 ) | adcl );
//      MSGEQ7_data[channel][bin] = adcx;

    for ( channel = 0; channel < MSGEQ7_CHANNELS; channel++ )
    {

      MSGEQ7_data[channel][bin] = analogRead( channel );

      // Refresh peak hold value.
      if ( MSGEQ7_data[channel][bin] > MSGEQ7_peak[channel][bin] )
      {
        MSGEQ7_peak[channel][bin] = MSGEQ7_data[channel][bin];
        MSGEQ7_count[channel][bin] = MSGEQ7_CYCLES;
      }
      else
      {
        if ( --MSGEQ7_count[channel][bin] == 0 )
        {
          MSGEQ7_count[channel][bin] = MSGEQ7_CYCLES;
          peak = MSGEQ7_peak[channel][bin] - lcd_level;
          MSGEQ7_peak[channel][bin] = peak;
        }
      }
    }
//      bitclr( ADCSRA, ADSC ); // Stop ADC conversions.
    digitalWrite( MSGEQ7_STROBE, HIGH );
    delayMicroseconds( 40 );

  }
}

//  ---------------------------------------------------------------------------
/*
    Updates LCD display.
*/
//  ---------------------------------------------------------------------------
void LCD_update( void )
{
  uint8_t channel;
  uint8_t bin;
  uint8_t level;
  bool    lcd_write;

  if ( DEBUG ) Serial.println( "LCD_update called." );

  for ( channel = 0; channel < MSGEQ7_CHANNELS; channel++ )
  {
    lcd.setCursor(( channel * ( MSGEQ7_BINS + LCD_GAP )), 1);

    for ( bin = 0; bin < MSGEQ7_BINS; bin++ )
    {
      lcd_write = false;
      level = LCD_COUNT;
      while ( !lcd_write )
      {
        if ( level >= 0 ) level--;
        else lcd_write = true;
        if ( MSGEQ7_data[channel][bin] >= level * lcd_level ) lcd_write = true;
      }
      lcd.write((byte) level);
      if ( DEBUG )
      {
        Serial.print( "Value = " );
        Serial.print( MSGEQ7_data[channel][bin] );
        Serial.print( ", LCD band = " );
        Serial.println( level );
      }
    }
  }
//  display_ready = true;
}

//  ---------------------------------------------------------------------------
/*
    Debugging MSGEQ7 output - serial monitor.
*/
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
    for ( bin = 0; bin < MSGEQ7_BINS; bin++ )
    {
      Serial.print( MSGEQ7_peak[channel][bin] );
      Serial.print( ", " );
    }
    Serial.println();
  }
}

//  ---------------------------------------------------------------------------
/*
    Main loop.
*/
//  ---------------------------------------------------------------------------
void loop( void )
{

  MSGEQ7_get_data();

  if ( MSGEQ7_PRINT ) MSGEQ7_print();
  LCD_update();
  delayMicroseconds( 1000 );

}

