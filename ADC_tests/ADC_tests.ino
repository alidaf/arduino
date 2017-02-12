//  ===========================================================================
//  Arduino ADC tests.
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
    This is essentially a test program to determine the best (fastest) way
    to take ADC readings from multiple channels.
*/
//  ===========================================================================

//  ===========================================================================
//  Defines.
//  ===========================================================================

#define SAMPLES     256 // Number of FHT input bins per channel.
#define CHANNELS      2 // Number of audio channels.
#define CYCLES       10 // Number of ADC buffer cycles to complete.

#define DEBUG         0 // Send debug info to console.
#define OUTPUT_PRINT  1 // Send printed output to console.
#define OUTPUT_PLOT   0 // Send byte output to plotter.

//  Bit tweaking macros --------------------------------------------------------

#define bitset( reg, bit ) ( reg |= ( 1 << bit ))   // Set register bit
#define bitclr( reg, bit ) ( reg &= ~( 1 << bit ))  // Clear register bit.
#define bittst( reg, bit ) ( reg & ( 1 << bit ))    // Test register bit.

//  ===========================================================================
//  Global variables.
//  ===========================================================================

volatile int8_t   ADC_buffer[CHANNELS][SAMPLES];  // Ring buffers for ADC data.
volatile uint8_t  ADC_channel;                    // Keeps track of channel.
volatile uint16_t ADC_sample;                     // Keeps track of samples.
volatile boolean  ADC_buffer_full;                // Flags buffer is full.

const uint8_t admux[CHANNELS] = { ADC0D, ADC1D }; // ADC MUX channels.

uint8_t cycle;  // Current ADC buffer fill cycle.

//  ===========================================================================
//  Code.
//  ===========================================================================

//  ---------------------------------------------------------------------------
//  ADC conversion interrupt.
//  ---------------------------------------------------------------------------
ISR( ADC_vect )
{

//  uint8_t admux_temp;
  
  // Fill next available buffer slot with ADC value.
  ADC_buffer[ADC_channel][ADC_sample] = ADCH - 0x80; // Read ADC value.

  // If the ADC buffer for the channel is full, switch channel.
  if ( ++ADC_sample >= SAMPLES )
  {
    ADC_sample = 0;

    if ( ++ADC_channel >= CHANNELS )
    {
      ADC_channel = 0;
      ADC_buffer_full = true; // ADC sample buffer is full.
    }

     // Switch to next channel.
    ADMUX = ( ADMUX & 0xf8 ) | ( admux[ADC_channel] & 0x0f );

  }

  startADC(); // Needed if not in free running mode.
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

  // ADCSRA.
  ADCSRA = B10001011;

  // ADEN  - 1  // Enable ADC.
  // ADSC  - 0  // Disable conversions.
  // ADATE - 0  // Disable ADC auto trigger.
  // ADIF  - 0  // N/A - set by hardware.
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
  // ADTS1 - 0  // }- Free Running Mode - not active due to ADATE = 0.
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

  // Clear buffer.
  memset( (void *) ADC_buffer, 0, sizeof( ADC_buffer ));

  // Set up counters and flags.
  ADC_sample = 0;
  ADC_channel = 0;
  ADC_buffer_full = false;
  cycle = 0;

  Serial.begin( 115200 );

  startADC(); // Start conversions.
}

//  ---------------------------------------------------------------------------
/*
    This outputs the ADC buffers to the serial console or plotter.
*/
//  ---------------------------------------------------------------------------
void ADC_print_buffers( void )
{
  uint16_t sample;  // Sample Counter.
  uint8_t  channel; // Channel Counter.
  int8_t   ADC_copy[CHANNELS][SAMPLES]; // Copy of ADC buffers.

  memcpy( ADC_copy, ADC_buffer, sizeof( ADC_buffer ));

  if ( OUTPUT_PRINT )
  {
    Serial.print( "Output for cycle " );
    Serial.print( cycle );
    Serial.println();

    for ( channel = 0; channel < CHANNELS; channel++ )
    {
      for ( sample = 0; sample < SAMPLES; sample++ )
      {
        Serial.print( "Channel " );
        Serial.print( channel );
        Serial.print( "=" );
        Serial.print( ADC_copy[channel][sample] );
        Serial.println();
      }
    }
  }

  if ( OUTPUT_PLOT )
  {
    for ( sample = 0; sample < SAMPLES; sample++ )
    { 
      for ( channel = 0; channel < CHANNELS; channel++ )
      {
        Serial.print( ADC_copy[channel][sample] );
        if ( channel < ( CHANNELS - 1 ))
        {
          Serial.print( "\t" ); // Channel delimiter for Arduino plotter.
          Serial.println();
        }
      }
    }
  }
}

//  ---------------------------------------------------------------------------
/*
    Main loop.
*/
//  ---------------------------------------------------------------------------
void loop( void )
{

  long time_start, time_end, time_taken;

  time_start = millis();

  while( cycle < CYCLES )
  {
    while( !ADC_buffer_full ); // Wait until buffer is full.
    ADC_print_buffers();
    ADC_buffer_full = false;
    cycle++;
  }

  time_end = millis;
  time_taken = time_end - time_start;

  Serial.println();
  Serial.print( "Time taken for " );
  Serial.print( CYCLES * CHANNELS * SAMPLES );
  Serial.print( " samples = " );
  Serial.print( time_taken );
  Serial.print( " ms." );
  Serial.println();
  Serial.end();
  cli(); // Turn off interrupts.
}

