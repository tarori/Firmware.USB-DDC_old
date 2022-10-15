/* -*- mode: c++; tab-width: 4; c-basic-offset: 4 -*- */
/*
 * taskPushButtonMenu.c
 *
 * \brief This task provides LCD Menu functions, triggered by the use of the
 * push button associated with the Rotary Encoder.  There are two levels
 * of functionality.
 *
 * The initial level is just to cycle through 9 frequency
 * memories, when doing short pushes of the push button.
 *
 * One long push triggers the Menu functions, which utilize the Rotary
 * Encoder for menu choice selection, and use the push button for confirmation
 * of choice.  The Menu functions may be several levels deep, depending on
 * choice.
 *
 *  Created on: 2010-06-26
 *      Author: Loftur Jonasson, TF3LJ
 */

#include "board.h"
