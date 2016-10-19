// Libraries
#include <U8glib.h>
#include "graphics.h"
#include "fonts.h"

// Default GPIO assignments.
#define SSD1322_SS     10 // Pin 10 - Chip Select.
#define SSD1322_MOSI   11 // Pin 11 - Master Out Slave In.
#define SSD1322_MISO   12 // Pin 12 - Master In Slave Out.
#define SSD1322_SCK    13 // Pin 13 - Clock.
#define SSD1322_DC      8 // Data/Command (DC#) pin..
#define SSD1322_RESET   9 // Hardware reset (RES#) pin.

U8GLIB_NHD31OLED_GR u8g(SSD1322_SS, SSD1322_DC, SSD1322_RESET);

// Graphics.
void setup( void )
{
   // assign default color value
  if ( u8g.getMode() == U8G_MODE_R3G3B2 )
  {
    u8g.setColorIndex( 255 );
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT )
  {
    u8g.setColorIndex( 3 );
  }
  else if ( u8g.getMode() == U8G_MODE_BW )
  {
    u8g.setColorIndex( 1 );
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR )
  {
    u8g.setHiColorByRGB( 255,255,255 );
  }
}

void loop( void ) 
{
  u8g.firstPage();
  do
  {
    draw();
  } while( u8g.nextPage() );

  delay( 1000 );
}

void draw( void )
{
  u8g.setFont( FalloutPipBoy16 );
  u8g.drawStr( 0, 16, "Fallout VAULT-TEC" );
  u8g.setFont( FalloutPipBoy24 );
  u8g.drawStr( 0, 40, "Fallout VAULT-TEC" );
  u8g.drawStr( 0, 64, "0123456789" );
  u8g.setFont( FalloutPipBoy32 );
  u8g.drawStr( 170, 32, "Fallout" );
  u8g.drawStr( 140, 64, "0123456789" );
//  u8g.drawBitmap( 0, 0, 8, 64, FalloutOK01 );
}

