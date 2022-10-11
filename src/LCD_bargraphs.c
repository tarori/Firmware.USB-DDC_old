/*
 * LCD_bargraphs.c
 *
 *! \brief Load the LCD with Bargraph symbols
 * and print bargraphs
 *
 * Created on: 2010-06-20
 *      Author: Loftur Jonasson, TF3LJ
 */

//**  Note: The lcdProgressBar() function is mostly swiped from the AVRLIB lcd.c/h.
//**
//**  Copy/Paste of copyright notice from AVRLIB lcd.h:
//*****************************************************************************
//
// File Name	: 'lcd.h'
// Title		: Character LCD driver for HD44780/SED1278 displays
//					(usable in mem-mapped, or I/O mode)
// Author		: Pascal Stang
// Created		: 11/22/2000
// Revised		: 4/30/2002
// Version		: 1.1
// Target MCU	: Atmel AVR series
// Editor Tabs	: 4
//
///	\ingroup driver_hw
/// \defgroup lcd Character LCD Driver for HD44780/SED1278-based displays (lcd.c)
/// \code #include "lcd.h" \endcode
/// \par Overview
///		This display driver provides an interface to the most common type of
///	character LCD, those based on the HD44780 or SED1278 controller chip
/// (about 90% of character LCDs use one of these chips).  The display driver
/// can interface to the display through the CPU memory bus, or directly via
/// I/O port pins.  When using the direct I/O port mode, no additional
/// interface hardware is needed except for a contrast potentiometer.
/// Supported functions include initialization, clearing, scrolling, cursor
/// positioning, text writing, and loading of custom characters or icons
/// (up to 8).  Although these displays are simple, clever use of the custom
/// characters can allow you to create animations or simple graphics.  The
/// "progress bar" function that is included in this driver is an example of
/// graphics using limited custom-chars.
///
/// \Note The driver now supports both 8-bit and 4-bit interface modes.
///
/// \Note For full text output functionality, you may wish to use the rprintf
/// functions along with this driver
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

#include "board.h"

