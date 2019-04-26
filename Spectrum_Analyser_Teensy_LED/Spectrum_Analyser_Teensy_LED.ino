//  ===========================================================================
//  Teensy Stereo Audio Spectrum Analyser using LED strip.
//  ===========================================================================
/*
    Copyright 2019 Darren Faulke <darren@alidaf.co.uk>

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
    This project uses the Teensy Audio shield and the OctoWS2811 module.
*/
//  ===========================================================================

//  ===========================================================================
//  Defines.
//  ===========================================================================

//  Debug output --------------------------------------------------------------
/*
    This setting toggles some debugging output via the serial port.
*/
#define DEBUG     0 // Print debug info to serial console.
#define FFT_PRINT 0 // Print MSGEQ7 data to serial console.

//  LED Display --------------------------------------------------------------- 

#define LED_BRIGHT       5 // Initial LED brightness.
#define LED_REFRESH   5000 // Initial refresh time for LEDs (us).
#define LED_GAP          2 // Number of LEDS skipped between channels.
#define LED_HOLD        30 // Number of cycles to sustain peak values.
#define LED_DECAY        2 // Number of cycles before dropping peak level.

#define NUM_CHANNELS     2 // Number of audio channels being analysed.
#define NUM_FREQ_BANDS   7 // Number of frequency bands being displayed.
#define NUM_LEDS_BAND   17 // Number of LEDS in each band.
#define NUM_LEDS_STRIP 240 // Number of LEDS in each strip.

#define FFT_BINS       128 // Number of bins in 1024 FFT.

//  Colours -------------------------------------------------------------------
#define RED    0x160000
#define GREEN  0x001600
#define BLUE   0x000016
#define YELLOW 0xFFFF00
#define BLACK  0x000000

#define COLOUR_BASE GREEN // Base level colour.
#define COLOUR_PEAK RED   // Peak level colour.
#define COLOUR_HOLD BLUE  // Peak hold colour.
#define COLOUR_VOID BLACK // Void colour.

//  ===========================================================================
//  Includes.
//  ===========================================================================

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <OctoWS2811.h>

//  ===========================================================================
//  Set up audio.
//  ===========================================================================
AudioInputI2S            i2s1;
AudioAnalyzePeak         peak1;
AudioAnalyzePeak         peak2;
AudioAnalyzeFFT256       fft_1;
AudioAnalyzeFFT256       fft_2;
AudioConnection          patchCord1(i2s1, 0, fft_1, 0);
AudioConnection          patchCord2(i2s1, 0, peak1, 0);
AudioConnection          patchCord3(i2s1, 1, fft_2, 0);
AudioConnection          patchCord4(i2s1, 1, peak2, 0);
AudioControlSGTL5000     audioShield;

const int audioInput = AUDIO_INPUT_LINEIN;

//  ===========================================================================
//  Set up LED strips.
//  ===========================================================================
volatile float   FFT_data[NUM_CHANNELS][NUM_FREQ_BANDS];
volatile uint8_t LED_mean_level[NUM_CHANNELS][NUM_FREQ_BANDS];
volatile uint8_t LED_peak_level[NUM_CHANNELS][NUM_FREQ_BANDS];
volatile uint8_t LED_peak_count[NUM_CHANNELS][NUM_FREQ_BANDS];
volatile uint8_t LED_hold_count[NUM_CHANNELS][NUM_FREQ_BANDS];

uint16_t binSizes[2][NUM_FREQ_BANDS];

//byte brightness = LED_BRIGHT;  // Set initial LED brightness.
//long refresh    = LED_REFRESH; // Set initial time (uS) between updates.

DMAMEM int displayMemory[NUM_LEDS_STRIP * 6];
int drawingMemory[NUM_LEDS_STRIP * 6];

const int octoConfig = WS2811_GRB | WS2811_800kHz;

OctoWS2811 leds(NUM_LEDS_STRIP, displayMemory, drawingMemory, octoConfig);

// Colour reference table for vertical meter scale.
uint32_t LED_colours[NUM_LEDS_BAND];

// LED offsets for each channel.
uint8_t channelOffsets[NUM_CHANNELS];

//  ===========================================================================
//  Code.
//  ===========================================================================

//  ---------------------------------------------------------------------------
//  Initialises the audio and LED.
//  ---------------------------------------------------------------------------
void setup()
{

  //while (!Serial);  // Wait for Serial interface.
  if (DEBUG) Serial.println("Starting Initialisation");

  // Initialise LED strip.
  if (DEBUG) Serial.println("Initialising LED strips.");
  leds.begin();
  leds.show();

  if (DEBUG) Serial.println("LED strips initialised.");

  AudioMemory(12);
  audioShield.enable();
  audioShield.inputSelect(audioInput);
  if (DEBUG) Serial.println("Audio shield initialised.");

  // Clear FFT buffers.
  memset((void *) FFT_data,       0, sizeof(FFT_data));
  memset((void *) LED_mean_level, 0, sizeof(LED_mean_level));
  memset((void *) LED_peak_level, 0, sizeof(LED_peak_level));
  memset((void *) LED_peak_count, 0, sizeof(LED_peak_count));
  memset((void *) LED_hold_count, 0, sizeof(LED_peak_count));
  if (DEBUG) Serial.println("LED buffers cleared.");

  // Calculate logarithmic scales for number of bands.
  float e, n;
  int count=0, d;

  e = FindE(NUM_FREQ_BANDS, FFT_BINS);
  if (e)
  {
    if (DEBUG) Serial.printf("E = %4.4f\n", e);
    for (int b = 0; b < NUM_FREQ_BANDS; ++b)
    {
      n = pow(e, b);
      d = int(n + 0.5);

      binSizes[0][b] = count;
      count += d - 1;
      binSizes[1][b] = count;
      ++count;
    }
  }
  else
    Serial.println("Error\n");

  if (DEBUG)
  {
    Serial.println("FFT bin sizes:");
    for (int b = 0; b < NUM_FREQ_BANDS; ++b)
    {
      Serial.printf("%4d ", binSizes[0][b]);
      Serial.printf("%4d\n", binSizes[1][b]);
    }
  }

  if (DEBUG) Serial.println("FFT bins optimised for logarithmic scale.");

  // Set up colour table for vertical meter scale.
  uint16_t i;
  uint16_t  r1, r2, g1, g2, b1, b2; // Need to be
  uint32_t  r, g, b;

  r1 = (COLOUR_PEAK >> 16) & 0x00ff;
  r2 = (COLOUR_BASE >> 16) & 0x00ff;
  g1 = (COLOUR_PEAK >>  8) & 0x00ff;
  g2 = (COLOUR_BASE >>  8) & 0x00ff;
  b1 = (COLOUR_PEAK >>  0) & 0x00ff;
  b2 = (COLOUR_BASE >>  0) & 0x00ff;

  // Need to add weight factors to bias RGB colours.
  for (i = 0; i < NUM_LEDS_BAND; i++)
  {
    r = r1 + i * (r2 - r1) / (NUM_LEDS_BAND - 1);
    g = g1 + i * (g2 - g1) / (NUM_LEDS_BAND - 1);
    b = b1 + i * (b2 - b1) / (NUM_LEDS_BAND - 1);
    LED_colours[NUM_LEDS_BAND - i - 1] = ((r << 16) & 0xff0000) + 
                     ((g <<  8) & 0x00ff00) + 
                     ((b <<  0) & 0x0000ff);
  }

  if (DEBUG)
  {
    Serial.println("LED colour scale defined:");
    for (i = 0; i < NUM_LEDS_BAND; i++)
    {
      Serial.print(i);
      Serial.print(" ");
      Serial.println(LED_colours[i], HEX);
    }
  }
  if (DEBUG) Serial.println("LED colours have been set.");

  // Basic LED test - light up first 3 LEDs.
  for (int i = 0; i < NUM_LEDS_BAND; ++i)
  {
    leds.setPixel(i, LED_colours[i]);
    leds.show();
    delayMicroseconds(100000);
  }
  delayMicroseconds(1000000);

  // Now light up full strip with various colours.
  //setColourStrip(COLOUR_BASE, 1000000);
  //setColourStrip(COLOUR_PEAK, 1000000);
  //setColourStrip(COLOUR_HOLD, 1000000);
  //setColourStrip(COLOUR_VOID, 1000000);

  // Calculate LED offsets for channels.
  if (DEBUG)
    Serial.println("Channel offsets:");
  for (uint8_t channel = 0; channel < NUM_CHANNELS; ++channel)
  {
    channelOffsets[channel] = channel * (NUM_FREQ_BANDS * NUM_LEDS_BAND + LED_GAP - 1);
    if (DEBUG)
    {
      Serial.print("Channel ");
      Serial.print(channel);
      Serial.print(": ");
      Serial.print(channelOffsets[channel]);
      Serial.println(".");
    }
  }

  if (DEBUG) Serial.println("Initialisation complete.");
}

//  ---------------------------------------------------------------------------
//  Main loop.
//  ---------------------------------------------------------------------------
void loop(void)
{
  
  if (fft_1.available() & fft_2.available())
  {
    for (uint8_t band = 0; band < NUM_FREQ_BANDS; ++band)
    {
      FFT_data[0][band] = fft_1.read(binSizes[0][band], binSizes[1][band]);
      FFT_data[1][band] = fft_2.read(binSizes[0][band], binSizes[1][band]);
    }
    updateLevels();
    updateDisplay();
  }
}

//  ===========================================================================
//  Functions
//  ===========================================================================

//  ---------------------------------------------------------------------------
//  Finds the optimised bins for logarithmic scale.
//  ---------------------------------------------------------------------------
float FindE(int bands, int bins)
{
  float increment=0.1, eTest, n;
  int count, d;

  for (eTest = 1; eTest < bins; eTest += increment)
  {
    count = 0;
    for (int b = 0; b < bands; b++)
    {
      n = pow(eTest, b);
      d = int(n + 0.5);
      count += d;
    }
    if (count > bins)
    {
      eTest -= increment;
      increment /= 10.0;
    }
    else
      if (count == bins)
        return eTest;
    if (increment < 0.0000001)
      return (eTest - increment);
  }
  return 0;
}

//  ---------------------------------------------------------------------------
//  Updates LED levels.
//  ---------------------------------------------------------------------------

void updateLevels(void)
{
  for (uint8_t channel = 0; channel < NUM_CHANNELS; ++channel )
  {
    for (uint8_t band = 0; band < NUM_FREQ_BANDS; ++band)
    {
      LED_mean_level[channel][band] = round(FFT_data[channel][band] * NUM_LEDS_BAND);

      // Update peak level.
      if (LED_mean_level[channel][band] >= LED_peak_level[channel][band])
      {
        LED_peak_level[channel][band] = LED_mean_level[channel][band];
        LED_hold_count[channel][band] = LED_HOLD;
      }
      else
      {
        // If hold count = 0 then peak level is decaying.
        if ( LED_hold_count[channel][band] == 0 )
        {
          // if decay count = 0 then move level down and reset count
          if ( LED_peak_count[channel][band] == 0 )
          {
            if ( LED_peak_level[channel][band] > 0 )
            {
              LED_peak_level[channel][band]--;
            }
            LED_peak_count[channel][band] = LED_DECAY;
          }
          else
          {
            LED_peak_count[channel][band]--;
          }
        }
        else
        {
          LED_hold_count[channel][band]--;
        }
      }
    }
  }

  if (FFT_PRINT)
  {
    Serial.print("Levels        ");
    for (uint8_t channel = 0; channel < NUM_CHANNELS; ++channel)
    {
      Serial.print("Channel ");
      Serial.print(channel);
      Serial.print(": ");
      for (uint8_t band = 0; band < NUM_FREQ_BANDS; ++band)
      {
        Serial.print(LED_mean_level[channel][band]);
        Serial.print(" ");
      }
    }
    Serial.println();
    Serial.print("Peaks         ");
    for (uint8_t channel = 0; channel < NUM_CHANNELS; ++channel)
    {
      Serial.print("Channel ");
      Serial.print(channel);
      Serial.print(": ");
      for (uint8_t band = 0; band < NUM_FREQ_BANDS; ++band)
      {
        Serial.print(LED_peak_level[channel][band]);
        Serial.print(" ");
      }
    }
    Serial.println();
    Serial.print("Peak counts   ");
    for (uint8_t channel = 0; channel < NUM_CHANNELS; ++channel)
    {
      Serial.print("Channel ");
      Serial.print(channel);
      Serial.print(": ");
      for (uint8_t band = 0; band < NUM_FREQ_BANDS; ++band)
      {
        Serial.print(LED_peak_count[channel][band]);
        Serial.print(" ");
      }
    }
    Serial.println();
    Serial.print("Hold counts   ");
    for (uint8_t channel = 0; channel < NUM_CHANNELS; ++channel)
    {
      Serial.print("Channel ");
      Serial.print(channel);
      Serial.print(": ");
      for (uint8_t band = 0; band < NUM_FREQ_BANDS; ++band)
      {
        Serial.print(LED_hold_count[channel][band]);
        Serial.print(" ");
      }
    }
    Serial.println();
  }
}

//  ---------------------------------------------------------------------------
//  Updates LED string.
//  ---------------------------------------------------------------------------
void updateDisplay(void)
{
  if (DEBUG) Serial.println("update_LED called.");

  for (uint8_t channel = 0; channel < NUM_CHANNELS; ++channel)
  {
    if (DEBUG)
    {
      Serial.print("Channel ");
      Serial.print(channel);
      Serial.println(":");
    }

    for (uint8_t band = 0; band < NUM_FREQ_BANDS; ++band)
    {
      if (DEBUG)
      {
        Serial.print("Band ");
        Serial.print(band);
        Serial.println(":");
      }
      uint8_t mean = LED_mean_level[channel][band];
      uint8_t peak = LED_peak_level[channel][band];

      for (uint8_t pos = 0; pos < mean; pos++ )
      {
        uint8_t ledNum = channelOffsets[channel] + (NUM_LEDS_BAND * band) + pos;
        leds.setPixel(ledNum, LED_colours[pos]);
        if (DEBUG)
        {
          Serial.print("LED ");
          Serial.print(ledNum);
          Serial.println(" = SIGNAL.");
        }
      }
      for (uint8_t pos = mean; pos < NUM_LEDS_BAND; ++pos)
      {
        // Turn off blank LEDs.
        uint8_t ledNum = channelOffsets[channel] + (NUM_LEDS_BAND * band) + pos;
        leds.setPixel(ledNum, COLOUR_VOID);
        if (DEBUG)
        {
          Serial.print("LED ");
          Serial.print(ledNum);
          Serial.println(" = VOID.");
        }
      }
      // Add peak level.
      uint8_t ledPeak = channelOffsets[channel] + (NUM_LEDS_BAND * band) + peak;
      leds.setPixel(ledPeak, COLOUR_HOLD);
      if (DEBUG)
      {
        Serial.print("LED ");
        Serial.print(ledPeak);
        Serial.println(" = PEAK.");
      }
    }
  }
  leds.show();
  delayMicroseconds(100);
}

//  ---------------------------------------------------------------------------
//  Wipes entire LED strip with specified colour.
//  ---------------------------------------------------------------------------
void setColourStrip(int colour, int wait)
{
  for (int i = 0; i < NUM_LEDS_STRIP; ++i)
  {
    leds.setPixel(i, colour);
  }
  leds.show();
  delayMicroseconds(wait);
}
