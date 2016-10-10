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
// #include <beach.h>

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
  pinMode( 6, OUTPUT );
  digitalWrite( 6, HIGH );

  // Serial monitor for debugging.
  Serial.begin(9600);

  // Hardware reset.
  pinMode( SSD1322_RESET, OUTPUT );
  digitalWrite( SSD1322_RESET, HIGH );
  delay( 100 );
  digitalWrite( SSD1322_RESET, LOW );
    
  ssd1322_init();
 
}

//  ===========================================================================

// ----------------------------------------------------------------------------
/*
    Display test - checkerboard pattern.
*/
// ----------------------------------------------------------------------------
void test_checkerboard( void )
{
    uint8_t row, col;

    ssd1322_clear_display();
    ssd1322_set_cols( COLS_VIS_MIN + 0x1c, COLS_VIS_MAX + 0x1c );
    ssd1322_set_rows( ROWS_VIS_MIN, ROWS_VIS_MAX );
    ssd1322_set_write_continuous();
    for ( row = 0; row <= ROWS_VIS_MAX; row++ )
    {
        for ( col = 0; col <= COLS_VIS_MAX; col++ )
        {
            ssd1322_write_data( 0xf0 );
            ssd1322_write_data( 0x0f );
        }
        for ( col = 0; col < COLS_VIS_MAX; col++ )
        {
            ssd1322_write_data( 0x0f );
            ssd1322_write_data( 0xf0 );
        }
    }
}

// ----------------------------------------------------------------------------
/*
    Fills a block with a greyscale.
*/
// ----------------------------------------------------------------------------
void fill_block( uint8_t col1, uint8_t col2,
                 uint8_t row1, uint8_t row2,
                 uint8_t grey )
{
    uint8_t row, col;
    ssd1322_set_cols( col1 + 0x1c, col2 + 0x1c );
    ssd1322_set_rows( row1, row2 );
    ssd1322_set_write_continuous();

    for ( row = 0; row < ( row2 - row1 + 1 ); row++ )
    {
        for ( col = 0; col < ( col2 - col1 + 1 ); col++ )
        {
            ssd1322_write_data( grey );
            ssd1322_write_data( grey );
        }
    }
}

// ----------------------------------------------------------------------------
/*
    Display test - display greyscales.
*/
// ----------------------------------------------------------------------------
void test_greyscales( void )
{
    ssd1322_clear_display();
    // Upper 32 rows.
    fill_block( 0x00, 0x03, 0x00, 0x1f, 0xff ); // Col   0- 15.
    fill_block( 0x04, 0x07, 0x00, 0x1f, 0xee ); // Col  16- 31.
    fill_block( 0x08, 0x0b, 0x00, 0x1f, 0xdd ); // Col  32- 47.
    fill_block( 0x0c, 0x0f, 0x00, 0x1f, 0xcc ); // Col  48- 63.
    fill_block( 0x10, 0x13, 0x00, 0x1f, 0xbb ); // Col  64- 79.
    fill_block( 0x14, 0x17, 0x00, 0x1f, 0xaa ); // Col  80- 95.
    fill_block( 0x18, 0x1b, 0x00, 0x1f, 0x99 ); // Col  96-111.
    fill_block( 0x1c, 0x1f, 0x00, 0x1f, 0x88 ); // Col 112-127.
    fill_block( 0x20, 0x23, 0x00, 0x1f, 0x77 ); // Col 128-143.
    fill_block( 0x24, 0x27, 0x00, 0x1f, 0x66 ); // Col 144-159.
    fill_block( 0x28, 0x2b, 0x00, 0x1f, 0x55 ); // Col 160-175.
    fill_block( 0x2c, 0x2f, 0x00, 0x1f, 0x44 ); // Col 176-191.
    fill_block( 0x30, 0x33, 0x00, 0x1f, 0x33 ); // Col 192-207.
    fill_block( 0x34, 0x37, 0x00, 0x1f, 0x22 ); // Col 208-223.
    fill_block( 0x38, 0x3b, 0x00, 0x1f, 0x11 ); // Col 224-239.
    fill_block( 0x3c, 0x3f, 0x00, 0x1f, 0x00 ); // Col 240-255.
    // Lower 32 rows.
    fill_block( 0x00, 0x03, 0x20, 0x3f, 0x00 ); // Col   0- 15.
    fill_block( 0x04, 0x07, 0x20, 0x3f, 0x11 ); // Col  16- 31.
    fill_block( 0x08, 0x0b, 0x20, 0x3f, 0x22 ); // Col  32- 47.
    fill_block( 0x0c, 0x0f, 0x20, 0x3f, 0x33 ); // Col  48- 63.
    fill_block( 0x10, 0x13, 0x20, 0x3f, 0x44 ); // Col  64- 79.
    fill_block( 0x14, 0x17, 0x20, 0x3f, 0x55 ); // Col  80- 95.
    fill_block( 0x18, 0x1b, 0x20, 0x3f, 0x66 ); // Col  96-111.
    fill_block( 0x1c, 0x1f, 0x20, 0x3f, 0x77 ); // Col 112-127.
    fill_block( 0x20, 0x23, 0x20, 0x3f, 0x88 ); // Col 128-143.
    fill_block( 0x24, 0x27, 0x20, 0x3f, 0x99 ); // Col 144-159.
    fill_block( 0x28, 0x2b, 0x20, 0x3f, 0xaa ); // Col 160-175.
    fill_block( 0x2c, 0x2f, 0x20, 0x3f, 0xbb ); // Col 176-191.
    fill_block( 0x30, 0x33, 0x20, 0x3f, 0xcc ); // Col 192-207.
    fill_block( 0x34, 0x37, 0x20, 0x3f, 0xdd ); // Col 208-223.
    fill_block( 0x38, 0x3b, 0x20, 0x3f, 0xee ); // Col 224-239.
    fill_block( 0x3c, 0x3f, 0x20, 0x3f, 0xff ); // Col 240-255.
}

// ----------------------------------------------------------------------------
/*
    Display test - display greyscales.
*/
// ----------------------------------------------------------------------------
void test_blocks( void )
{
    uint8_t i, j;
    uint8_t grey;

    ssd1322_clear_display();
    for ( i = 0; i < 16; i++ )
    {
      grey = 0xff - i * 17;
      for ( j = 0; j < 16; j++ )
      {
          fill_block( 0x00 + j * 4, 0x03 + j * 4, 0x00, 0x1f, grey );
          delay( 10 );
      }
      for ( j = 0; j < 16; j++ )
      {
          fill_block( 0x00 + j * 4, 0x03 + j * 4, 0x20, 0x3f, grey );
          delay( 10 );
      }
   }

}

// ----------------------------------------------------------------------------
/*
    Display test - 256x64 4bpp image.
*/
// ----------------------------------------------------------------------------

void test_load_image( void )
{
    uint16_t i,j;
//    uint8_t  image[8192];
    uint8_t data;

    ssd1322_clear_display();
    ssd1322_set_cols( COLS_VIS_MIN + 0x1c, COLS_VIS_MAX + 0x1c );
    ssd1322_set_rows( ROWS_VIS_MIN, ROWS_VIS_MAX );
    ssd1322_set_write_continuous();

//    Serial.println( "Loading image into array." );
    for ( i = 0; i < 8192; i++ )
    {
//      Serial.print( beach[i*2] );
//      Serial.print( beach[i*2+1] );
//      data = ( beach[i*2]<<4 | beach[i*2+1] );
//      ssd1322_write_data( data );
    }    
//    test_draw_image( image );
}

// ----------------------------------------------------------------------------

/*
    Main
*/
// ----------------------------------------------------------------------------
void loop()
{
    test_checkerboard();
    delay( 5000 );

    test_greyscales();
    delay( 1000 );
    
    test_blocks();
    delay( 500 );
}
