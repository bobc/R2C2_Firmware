/* From Arduino LiquidCrystal.cpp library, modifications for R2C2
   Copyright (c) 2012 Bob Cousins bobcousins42@googlemail.com       
*/

/* Copyright (c) 2009 David A. Mellis */
/*********************************************************************
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
  Boston, MA 02110-1301 USA 
*********************************************************************/

#include "LiquidCrystal.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ios.h"

//#include "WProgram.h"

static  uint8_t _rs_pin;          // LOW: command.  HIGH: character.
static  uint8_t _rw_pin;          // LOW: write to LCD.  HIGH: read from LCD.
static  uint8_t _enable_pin;      // activated by a HIGH pulse.
static  uint8_t _data_pins[8];

static  uint8_t _displayfunction; // function set 0x20
static  uint8_t _displaycontrol;  // Display Control 0x08
static  uint8_t _displaymode;     // Entry Mode Set 0x04

static  uint8_t _initialized;

static  uint8_t _numlines;
static  uint8_t _currline;



//private:
static void lcd_init(uint8_t fourbitmode, uint8_t rs, uint8_t rw, uint8_t enable,
	    uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
	    uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);

static  void lcd_send(uint8_t value, uint8_t mode);
static  void lcd_write4bits(uint8_t value);
static  void lcd_write8bits(uint8_t value);
static  void lcd_pulseEnable();



// LPC1343 @ 72 MHz = 5
// LPC1758 @ 100 MHz = 5
void delayMicroseconds(uint32_t delayUs)
{
  delayUs = delayUs * 5;
  while (delayUs)
  {
      delayUs--;
  }
}

void delayMs (uint32_t delay)
{
  while (delay--)
    delayMicroseconds (1000);
}

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set: 
//    DL = 1; 8-bit interface data 
//    N = 0; 1-line display 
//    F = 0; 5x8 dot character font 
// 3. Display on/off control: 
//    D = 0; Display off 
//    C = 0; Cursor off 
//    B = 0; Blinking off 
// 4. Entry mode set: 
//    I/D = 1; Increment by 1 
//    S = 0; No shift 
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

void LiquidCrystal_8bit_rw(uint8_t rs, uint8_t rw, uint8_t enable,
			     uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
			     uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
  lcd_init(0, rs, rw, enable, d0, d1, d2, d3, d4, d5, d6, d7);
}

void LiquidCrystal_8bit (uint8_t rs, uint8_t enable,
			     uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
			     uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
  lcd_init(0, rs, 255, enable, d0, d1, d2, d3, d4, d5, d6, d7);
}

void LiquidCrystal_4bit_rw(uint8_t rs, uint8_t rw, uint8_t enable,
			     uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
  lcd_init(1, rs, rw, enable, d0, d1, d2, d3, 0, 0, 0, 0);
}

void LiquidCrystal_4bit(uint8_t rs,  uint8_t enable,
			     uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
  lcd_init(1, rs, 255, enable, d0, d1, d2, d3, 0, 0, 0, 0);
}

static void lcd_init(uint8_t fourbitmode, uint8_t rs, uint8_t rw, uint8_t enable,
			 uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
			 uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
  _rs_pin = rs;
  _rw_pin = rw;
  _enable_pin = enable;
  
  _data_pins[0] = d0;
  _data_pins[1] = d1;
  _data_pins[2] = d2;
  _data_pins[3] = d3; 
  _data_pins[4] = d4;
  _data_pins[5] = d5;
  _data_pins[6] = d6;
  _data_pins[7] = d7; 

  pinMode(_rs_pin, OUTPUT);
  // we can save 1 pin by not using RW. Indicate by passing 255 instead of pin#
  if (_rw_pin != 255) { 
    pinMode(_rw_pin, OUTPUT);
  }
  pinMode(_enable_pin, OUTPUT);
  
  if (fourbitmode)
    _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  else 
    _displayfunction = LCD_8BITMODE | LCD_1LINE | LCD_5x8DOTS;
  
  //begin(16, 1);  
}

void lcd_begin(uint8_t cols, uint8_t rows) 
{
  uint8_t dotsize = LCD_5x8DOTS;

  if (rows > 1) {
    _displayfunction |= LCD_2LINE;
  }
  _numlines = rows;
  _currline = 0;

  // for some 1 line displays you can select a 10 pixel high font
  if ((dotsize != 0) && (rows == 1)) {
    _displayfunction |= LCD_5x10DOTS;
  }

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way before 4.5V so we'll wait 50
  delayMicroseconds(50000); 
  
  // Now we pull both RS and R/W low to begin commands
  digitalWrite(_enable_pin, LOW);
  digitalWrite(_rs_pin, LOW);
  if (_rw_pin != 255) { 
    digitalWrite(_rw_pin, LOW);
  }

  //put the LCD into 4 bit or 8 bit mode
  if (! (_displayfunction & LCD_8BITMODE)) {
    // this is according to the hitachi HD44780 datasheet
    // page 46, figure 24

    // we may not start in 8bit mode, try to set 8 bit mode
    lcd_write4bits(0x03);
    delayMicroseconds(4500); // wait min 4.1ms

    // second try
    lcd_write4bits(0x03);
    delayMicroseconds(4500); // wait min 4.1ms
    
    // third go!
    lcd_write4bits(0x03); 
    delayMicroseconds(150);

    // finally, set to 4-bit interface
    lcd_write4bits(0x02); 

  } else {
    // this is according to the hitachi HD44780 datasheet
    // page 45 figure 23

    // Send function set command sequence
    lcd_command(LCD_FUNCTIONSET | _displayfunction);
    delayMicroseconds(4500);  // wait more than 4.1ms

    // second try
    //lcd_command(LCD_FUNCTIONSET | _displayfunction);
    delayMicroseconds(150);

    // third go
    lcd_command(LCD_FUNCTIONSET | _displayfunction);
  }

//return;

  // finally, set # lines, font size, etc.
  lcd_command(LCD_FUNCTIONSET | _displayfunction);  

  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;  
  lcd_display();

  // Initialize to default text direction (for romance languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  lcd_command(LCD_ENTRYMODESET | _displaymode);

  // clear it off
  lcd_clear();

}

/********** high level commands, for the user! */


void lcd_initialise (int dev_num)
{

}

void lcd_writechar (int dev_num, char c)
{
  lcd_write (c);
}



void lcd_clear()
{
  lcd_command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  delayMicroseconds(2000);  // this lcd_command takes a long time!
}

void lcd_home()
{
  lcd_command(LCD_RETURNHOME);  // set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!
}

void lcd_setCursor(uint8_t col, uint8_t row)
{
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if ( row > _numlines ) {
    row = _numlines-1;    // we count rows starting w/0
  }
  
  lcd_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void lcd_noDisplay() {
  _displaycontrol &= ~LCD_DISPLAYON;
  lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_display() {
  _displaycontrol |= LCD_DISPLAYON;
  lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void lcd_noCursor() {
  _displaycontrol &= ~LCD_CURSORON;
  lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_cursor() {
  _displaycontrol |= LCD_CURSORON;
  lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void lcd_noBlink() {
  _displaycontrol &= ~LCD_BLINKON;
  lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_blink() {
  _displaycontrol |= LCD_BLINKON;
  lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void lcd_scrollDisplayLeft(void) {
  lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void lcd_scrollDisplayRight(void) {
  lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void lcd_leftToRight(void) {
  _displaymode |= LCD_ENTRYLEFT;
  lcd_command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void lcd_rightToLeft(void) {
  _displaymode &= ~LCD_ENTRYLEFT;
  lcd_command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void lcd_autoscroll(void) {
  _displaymode |= LCD_ENTRYSHIFTINCREMENT;
  lcd_command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void lcd_noAutoscroll(void) {
  _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  lcd_command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void lcd_createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  lcd_command(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    lcd_write(charmap[i]);
  }
}

void lcd_print(const char *str)
{
  while (*str)
    lcd_write(*str++);
}

/*********** mid level commands, for sending data/cmds */

inline void lcd_command(uint8_t value) {
  lcd_send(value, LOW);
}

inline void lcd_write(uint8_t value) {
  lcd_send(value, HIGH);
}

/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
void lcd_send(uint8_t value, uint8_t mode) 
{
  digitalWrite(_rs_pin, mode);

  // if there is a RW pin indicated, set it low to Write
  if (_rw_pin != 255) { 
    digitalWrite(_rw_pin, LOW);
  }
  
  if (_displayfunction & LCD_8BITMODE) {
    lcd_write8bits(value); 
  } else {
    lcd_write4bits(value>>4);
    lcd_write4bits(value & 0x0F);
  }
}

void lcd_pulseEnable(void) 
{
  digitalWrite(_enable_pin, LOW);
  delayMicroseconds(1);    

  digitalWrite(_enable_pin, HIGH);
  delayMicroseconds(1);    // enable pulse must be >450ns

  digitalWrite(_enable_pin, LOW);
  delayMicroseconds(100);   // commands need > 37us to settle
}

void lcd_write4bits(uint8_t value) 
{
  for (int i = 0; i < 4; i++) 
  {
    pinMode(_data_pins[i], OUTPUT);
    digitalWrite(_data_pins[i], (value >> i) & 0x01);
  }

  lcd_pulseEnable();
}

void lcd_write8bits(uint8_t value) 
{
  for (int i = 0; i < 8; i++) {
    pinMode(_data_pins[i], OUTPUT);
    digitalWrite(_data_pins[i], (value >> i) & 0x01);
  }
  
  lcd_pulseEnable();
}
