//  ===========================================================================
/*
    test-ssd1322:
    
    Tests SSD1322 OLED display driver for the Arduino.
    
    Copyright 2016 Darren Faulke <darren@alidaf.co.uk>

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

// Info -----------------------------------------------------------------------
/*

                                                       GND +5V 
                                                        |   |
                                                        |   |
            ,-------------------------------------------|   |
            |     SSD1322                               |   |
            |                                           |   |
            |       1 2                                 |   |
            |---GND[o o]+5V-----------------------------)---|
            |      [o o]SCLK----------------,           |   |
        ,---(--SDIN[o o]                    |           |   |
        |   |  ,---[o o]---,                |           |   |
        |   |--+---[o o]---+----------------)-----------|   |
        |   |  '---[o o]---'                |           |   |
        |   '---WR#[o o]DC#---,             |           |   |
    ,---)----RESET#[o o]CS#---)-------------)-------,   |   |
    |   |          15 16      |             |       |   |   |
    |   |                     |             |       |   |   |
    |   '---------------------)-------------)---,   |   |   |   
    |                         |             |   |   |   |   |
    |   ,---------------------'             |   |   |   |   |
    |   |                                   |   |   |   |   |
    |   |  Arduino Pro Mini (5V 16MHz)      |   |   |   |   |
    |   |                                   |   |   |   |   |
    |   |       [ o o o o o o ]             |   |   |   |   |
    |   |    TXO[o           o]RAW          |   |   |   |   |
    |   |    RXI[o           o]GND          |   |   |   |   |
    |   |    RST[o           o]RST          |   |   |   |   |
    |   |    GND[o           o]VCC (+5V)----)---)---)---)---|
    |   |      2[o           o]A3           |   |   |   |   |
    |   |      3[o           o]A2           |   |   |   |   |
    |   |      4[o           o]A1           |   |   |   |   |
    |   |      5[o           o]A0           |   |   |   |   |
    |   |      6[o           o]13 SCK-------'   |   |   |   |
    |   |      7[o           o]12 MISO          |   |   |   |
    |   '--DC# 8[o   reset   o]11 MOSI----------'   |   |   | 
    '---RESET# 9[o     0     o]10 SS----------------'   |   |
                                                        |   |
                                                        |   |
                                                        |   |
                                                        |   |
                                                       GND +5V

*/

#include <ssd1322-spi.h>
#include <Wire.h>
#include "graphics.h"

//  ===========================================================================

// SSD1322 supports 480x128 but display is 256x64.
#define COLS_VIS_MIN 0x00 // Visible cols - start.
#define COLS_VIS_MAX 0x3f // Visible cols - end.
#define ROWS_VIS_MIN 0x00 // Visible rows - start.
#define ROWS_VIS_MAX 0x3f // Visible rows - end.

// RG LED pins
#define LED_PIN_RED 2
#define LED_PIN_GRN 3

//  ===========================================================================

void setup()
{  
  pinMode( LED_PIN_RED, OUTPUT );
  pinMode( LED_PIN_GRN, OUTPUT );

  digitalWrite( LED_PIN_RED, HIGH );
  digitalWrite( LED_PIN_GRN, LOW );

  // Temporary power supply for OLED.
//  pinMode( 6, OUTPUT );
//  digitalWrite( 6, HIGH );

  // Serial monitor for debugging.
  Serial.begin(9600);

  ssd1322_init();
//  ssd1322_set_enable_greys();
}

//  ===========================================================================

// ----------------------------------------------------------------------------
/*
    Sets the RAM address for position x,y.
*/
// ----------------------------------------------------------------------------
void ssd1322_gotoXY( uint8_t x, uint8_t y )
{
    uint8_t col, row;
    col = x / 4 + 0x1c;
    row = y;
    ssd1322_set_cols( col, col );
    ssd1322_set_rows( row, row );
}


// ----------------------------------------------------------------------------
/*
    Draws a pixel with grey value.
*/
// ----------------------------------------------------------------------------
void ssd1322_draw_pixel( uint8_t x, uint8_t y, uint8_t grey )
{
    uint8_t pixel[2];

    switch ( x % 4 )
    {
      case 0: pixel[0] = ( grey << 4 ) | ( pixel[0] & 0x0f );
              break;
      case 1: pixel[0] = ( grey ) | ( pixel[0] & 0xf0 );
              break;
      case 2: pixel[1] = ( grey << 4 ) | ( pixel[1] & 0x0f );
              break;
      case 3: pixel[1] = ( grey ) | ( pixel[1] & 0xf0 );
    }
    ssd1322_gotoXY( x, y );
    ssd1322_write_data( pixel[0] );
    ssd1322_write_data( pixel[1] );
}

// ----------------------------------------------------------------------------
/*
    Draws a filled block with grey value.
*/
// ----------------------------------------------------------------------------
void ssd1322_draw_block( uint8_t row, uint8_t col,
                         uint8_t height, uint8_t width,
                         uint8_t grey )
{
    uint8_t i;

    ssd1322_set_cols( col + 0x1c, col + width + 0x1c );
    ssd1322_set_rows( row, row + height );
    ssd1322_set_write_continuous();

    for ( i = 0; i < height * width; i++ )
    {
            ssd1322_write_data( grey );
//            ssd1322_write_data( grey );
    }
}

// ----------------------------------------------------------------------------
/*
    Displays 4-bit image.
*/
// ----------------------------------------------------------------------------
void draw_image( uint8_t row, uint8_t col, uint8_t rows, uint8_t cols,
                 uint8_t image[] )
{
  uint8_t stream[8192];
  uint16_t i;

  for ( i = 0; i < rows * cols / 2; i++ );
  {
      stream[i] = ( image[i*2]<<4 | image[i*2+1] );
      Serial.print( i );
      Serial.print( "[" );
      Serial.print( image[i*2] );
      Serial.print( "," );
      Serial.print( image[i*2+1] );
      Serial.print( "] -> " );
      Serial.println( stream[i] );
  }
  ssd1322_set_cols( col + 0x1c, col + cols + 0x1c );
  ssd1322_set_rows( row, row + rows );
  ssd1322_set_write_continuous();

  ssd1322_write_stream( stream, rows * cols / 2 );
//    Serial.print( "i = " );
//    Serial.print( i );
//    Serial.print( ". grey = " );
//    Serial.println( grey, HEX );
}

// ----------------------------------------------------------------------------
/*
    Main
*/
// ----------------------------------------------------------------------------
void loop()
{
    uint8_t col, row;
  
    ssd1322_clear_display();
    draw_image( 0, 0, 64, 64, falloutOK01 );
    delay( 1000 );
}
