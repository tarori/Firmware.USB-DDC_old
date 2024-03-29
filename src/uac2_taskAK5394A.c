/* -*- mode: c++; tab-width: 4; c-basic-offset: 4 -*- */
/*
 * uac2_taskAK5394A.c
 *
 *  Created on: Feb 14, 2010
 *  Refactored on: Feb 26, 2011
 *      Author: Alex
 *
 * Copyright (C) Alex Lee
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

// #define LINUX_QUIRK
// replaced by FEATURE_LINUX_QUIRK_ON() BSB 20120413/20

//_____  I N C L U D E S ___________________________________________________

//#include <stdio.h>
#include "usart.h"  // Shall be included before FreeRTOS header files, since 'inline' is defined to ''; leading to
                    // link errors
#include "conf_usb.h"

#include <avr32/io.h>
#if __GNUC__
#include "intc.h"
#endif
#include "board.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "task.h"
#include "Mobo_config.h"
#include "device_audio_task.h"
#include "features.h"
#include "gpio.h"
#include "pdca.h"
#include "pm.h"
#include "ssc_i2s.h"
#include "taskAK5394A.h"
#include "uac2_device_audio_task.h"
#include "uac2_taskAK5394A.h"
#include "uac2_usb_descriptors.h"
#include "usb_drv.h"
#include "usb_specific_request.h"
#include "usb_standard_request.h"

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________
//_____ D E C L A R A T I O N S ____________________________________________

void uac2_AK5394A_task(void*);

//!
//! @brief This function initializes the hardware/software resources
//! required for device CDC task.
//!
void uac2_AK5394A_task_init(void)
{
    current_freq.frequency = FREQ_44;
    AK5394A_task_init(FALSE);

    // clear samplerate indication FIX: move to different _init() routine
    gpio_clr_gpio_pin(SAMPLEFREQ_VAL1);
    gpio_set_gpio_pin(SAMPLEFREQ_VAL0);

    xTaskCreate(uac2_AK5394A_task,
        configTSK_AK5394A_NAME,
        configTSK_AK5394A_STACK_SIZE,
        NULL,
        UAC2_configTSK_AK5394A_PRIORITY,
        NULL);
}

//!
//! @brief Entry point of the AK5394A task management
//!
void uac2_AK5394A_task(void* pvParameters)
{
    (void)pvParameters;
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    volatile S32 usb_buffer_toggle;

    while (TRUE) {
        // All the hard work is done by the pdca and the interrupt handler.
        // This does some periodic checking such as whether USB data out is stalled

        vTaskDelayUntil(&xLastWakeTime, UAC2_configTSK_AK5394A_PERIOD);

        // silence speaker if USB data out is stalled, as indicated by heart-beat counter
        if (old_spk_usb_heart_beat == spk_usb_heart_beat) {
            if ((input_select == MOBO_SRC_UAC2) || (input_select == MOBO_SRC_NONE)) {

                // This is quite busy while idle
                mobo_clear_dac_channel();
            }

            // BSB 20131209 attempting improved playerstarted detection
            // Next iteration of uacX_device_audio_task will set playerStarted to FALSE
            if (usb_buffer_toggle < USB_BUFFER_TOGGLE_LIM)
                usb_buffer_toggle = USB_BUFFER_TOGGLE_LIM;
        }
        old_spk_usb_heart_beat = spk_usb_heart_beat;
    }  // end while (TRUE)
}
