/* -*- mode: c; tab-width: 4; c-basic-offset: 4 -*- */
/*
 * taskPowerDisplay.c
 *
 * \brief This task does a best effort update of the lower two LCD lines.
 * It uses the A/D input values made available by the taskMoboCtrl, and
 * the audio input values made available by the audio task.  During Transmit
 * it prints Power/SWR, during Receive it prints samples of the audio inputs
 * and/or dB bargraphs showing the total received input power into the full
 * bandwidth of the audio channels.
 *
 *  Created on: 2010-06-13
 *      Author: Loftur Jonasson, TF3LJ
 */

#include "board.h"

