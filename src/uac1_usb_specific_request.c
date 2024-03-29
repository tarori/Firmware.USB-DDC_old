/* -*- mode: c++; tab-width: 4; c-basic-offset: 4 -*- */
/* This source file is part of the ATMEL AVR32-SoftwareFramework-AT32UC3-1.5.0 Release */

/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file ******************************************************************
 *
 * \brief Processing of USB device specific enumeration requests.
 *
 * This file contains the specific request decoding for enumeration process.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with a USB module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 ***************************************************************************/

/* Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an Atmel
 * AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * Additions and Modifications to ATMEL AVR32-SoftwareFramework-AT32UC3 are:
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
 * Modified by Alex Lee and sdr-widget team since Feb 2010.  Copyright General Purpose Licence v2.
 * Please refer to http://code.google.com/p/sdr-widget/
 */

//_____ I N C L U D E S ____________________________________________________

#include "conf_usb.h"


#include "Mobo_config.h"
#include "device_audio_task.h"
#include "pdca.h"
#include "pm.h"
#include "taskAK5394A.h"
#include "uac1_device_audio_task.h"
#include "uac1_usb_descriptors.h"
#include "usart.h"
#include "usb_audio.h"
#include "usb_descriptors.h"
#include "usb_drv.h"
#include "usb_specific_request.h"
#include "usb_standard_request.h"

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

/* volume definition moved to usb_specific_request.h (for lack of a better place)
#define VOL_MIN      (S16)0x8000 // only allowed for CUR
#define VOL_MAX      (S16)0x7Fff
#define VOL_RES      0x000A
*/

//_____ P R I V A T E   D E C L A R A T I O N S ____________________________

// U8 usb_feature_report[3];
// U8 usb_report[3];

// U8 g_u8_report_rate=0;

// S_line_coding   line_coding;

// S_freq current_freq;
// Bool freq_changed = FALSE;

static U8 usb_type;
static U8 wValue_msb;
static U8 wValue_lsb;
static U16 wIndex;
static U16 wLength;

static U8 speed = 1;  // speed == 0, sample rate = 44.1khz
                      // speed == 1, sample rate = 48khz

extern const void* pbuffer;
extern U16 data_to_transfer;

//_____ D E C L A R A T I O N S ____________________________________________

//! @brief This function configures the endpoints of the device application.
//! This function is called when the set configuration request has been received.
//!
void uac1_user_endpoint_init(U8 conf_nb)
{
    (void)conf_nb;
    if (Is_usb_full_speed_mode()) {
        (void)Usb_configure_endpoint(UAC1_EP_AUDIO_OUT, EP_ATTRIBUTES_3, DIRECTION_OUT, EP_SIZE_3_FS, DOUBLE_BANK, 0);
        // BSB 20130604 disabling UAC1 IN		(void)Usb_configure_endpoint(UAC1_EP_AUDIO_IN, EP_ATTRIBUTES_4, DIRECTION_IN, EP_SIZE_4_FS, DOUBLE_BANK, 0);
        (void)Usb_configure_endpoint(UAC1_EP_AUDIO_OUT_FB, EP_ATTRIBUTES_5, DIRECTION_IN, EP_SIZE_5_FS, DOUBLE_BANK, 0);
    } else {
        (void)Usb_configure_endpoint(UAC1_EP_AUDIO_OUT, EP_ATTRIBUTES_3, DIRECTION_OUT, EP_SIZE_3_HS, DOUBLE_BANK, 0);
        // BSB 20130604 disabling UAC1 IN		(void)Usb_configure_endpoint(UAC1_EP_AUDIO_IN, EP_ATTRIBUTES_4, DIRECTION_IN, EP_SIZE_4_HS, DOUBLE_BANK, 0);
        (void)Usb_configure_endpoint(UAC1_EP_AUDIO_OUT_FB, EP_ATTRIBUTES_5, DIRECTION_IN, EP_SIZE_5_HS, DOUBLE_BANK, 0);
    }
}

//! @brief This function handles usb_set_interface side effects.
//! This function is called from usb_set_interface in usb_standard_request
//! in case there are side effects of the interface change to be handled.
void uac1_user_set_interface(U8 wIndex, U8 wValue)
{
    //* Check whether it is the audio streaming interface and Alternate Setting that is being set
    usb_interface_nb = wIndex;
    if (usb_interface_nb == STD_AS_INTERFACE_OUT) {
        usb_alternate_setting_out = wValue;
        usb_alternate_setting_out_changed = TRUE;
    }

    // BSB 20130604 disabling UAC1 IN
    /*
        else if (usb_interface_nb == STD_AS_INTERFACE_IN) {
                usb_alternate_setting = wValue;
                usb_alternate_setting_changed = TRUE;
        } */
}

static Bool uac1_user_get_interface_descriptor()
{
    return TRUE;
}

//! @brief This function manages hid set idle request.
//!
//! @param Duration     When the upper byte of wValue is 0 (zero), the duration is indefinite else from 0.004 to 1.020 seconds
//! @param Report ID    0 the idle rate applies to all input reports, else only applies to the Report ID
//!
void uac1_usb_hid_set_idle(U8 u8_report_id, U8 u8_duration)
{  // BSB 20120710 prefix "uac1_" added
    (void)u8_report_id, (void)u8_duration;
}

//! @brief This function manages hid get idle request.
//!
//! @param Report ID    0 the idle rate applies to all input reports, else only applies to the Report ID
//!
void uac1_usb_hid_get_idle(U8 u8_report_id)
{  // BSB 20120710 prefix "uac1_" added
    (void)u8_report_id;
}

#ifdef FEATURE_VOLUME_CTRL
void audio_get_min(void)
{
    U16 i_unit;              // in wIndex
    U16 length;              // in wLength
    i_unit = (wIndex >> 8);  // wIndex high byte is interface number
    length = wLength;

    Usb_ack_setup_received_free();
    Usb_reset_endpoint_fifo_access(EP_CONTROL);

    if (i_unit == SPK_FEATURE_UNIT_ID) {

        switch (wValue_msb) {
        case CS_MUTE:
            if (length == 1) {
                Usb_write_endpoint_data(EP_CONTROL, 8, usb_spk_mute);
            }
            break;
        case CS_VOLUME:
            if (length == 2) {
                Usb_write_endpoint_data(EP_CONTROL, 16, Usb_format_mcu_to_usb_data(16, VOL_MIN));
            }
            break;
        }
    }

    // BSB 20130604 disabling UAC1 IN
    /*
   else if( i_unit==MIC_FEATURE_UNIT_ID )
   {
      switch (wValue_msb)
      {
      case CS_MUTE:
         if( length==1 )
         {
            Usb_write_endpoint_data(EP_CONTROL, 8, mute);
         }
         break;
      case CS_VOLUME:
         if( length==2 )
         {
            Usb_write_endpoint_data(EP_CONTROL, 16, Usb_format_mcu_to_usb_data(16, VOL_MIN));
         }
         break;
      }
    }
    */

    /*
                // 44.1khz min sampling freq
        Usb_write_endpoint_data(EP_CONTROL, 8, 0x44);
        Usb_write_endpoint_data(EP_CONTROL, 8, 0xac);
        Usb_write_endpoint_data(EP_CONTROL, 8, 0x00);
*/

    Usb_ack_control_in_ready_send();
    while (!Is_usb_control_out_received())
        ;
    Usb_ack_control_out_received_free();
}
#endif

#ifdef FEATURE_VOLUME_CTRL
void audio_get_max(void)
{
    U16 i_unit;
    U16 length;
    i_unit = (wIndex >> 8);  // wIndex high byte is interface number
    length = wLength;

    Usb_ack_setup_received_free();
    Usb_reset_endpoint_fifo_access(EP_CONTROL);

    if (i_unit == SPK_FEATURE_UNIT_ID) {
        switch (wValue_msb) {
        case CS_MUTE:
            if (length == 1) {
                Usb_write_endpoint_data(EP_CONTROL, 8, usb_spk_mute);
            }
            break;
        case CS_VOLUME:
            if (length == 2) {
                Usb_write_endpoint_data(EP_CONTROL, 16, Usb_format_mcu_to_usb_data(16, VOL_MAX));
            }
            break;
        }
    }

    // BSB 20130604 disabling UAC1 IN
    /*
   else if( i_unit==MIC_FEATURE_UNIT_ID )
   {
      switch (wValue_msb)
      {
      case CS_MUTE:
         if( length==1 )
         {
            Usb_write_endpoint_data(EP_CONTROL, 8, mute);
         }
         break;
      case CS_VOLUME:
         if( length==2 )
         {
            Usb_write_endpoint_data(EP_CONTROL, 16, Usb_format_mcu_to_usb_data(16, VOL_MAX));
         }
         break;
      }
   }
   */

    /*
                // 48khz max sampling freq
                Usb_write_endpoint_data(EP_CONTROL, 8, 0x80);
                Usb_write_endpoint_data(EP_CONTROL, 8, 0xbb);
                Usb_write_endpoint_data(EP_CONTROL, 8, 0x00);
*/

    Usb_ack_control_in_ready_send();
    while (!Is_usb_control_out_received())
        ;
    Usb_ack_control_out_received_free();
}
#endif

#ifdef FEATURE_VOLUME_CTRL
void audio_get_res(void)
{
    U16 i_unit;
    U16 length;
    i_unit = (wIndex >> 8);  // wIndex high byte is interface number
    length = wLength;

    Usb_ack_setup_received_free();
    Usb_reset_endpoint_fifo_access(EP_CONTROL);

    if (i_unit == SPK_FEATURE_UNIT_ID) {  // FIX: Something is seriously wrong with the value of i_unit
        switch (wValue_msb) {
        case CS_MUTE:
            if (length == 1) {
                Usb_write_endpoint_data(EP_CONTROL, 8, usb_spk_mute);
            }
            break;
        case CS_VOLUME:
            if (length == 2) {
                Usb_write_endpoint_data(EP_CONTROL, 16, Usb_format_mcu_to_usb_data(16, VOL_RES));
            }
            break;
        }
    }

    // BSB 20130604 disabling UAC1 IN
    /*
   else if( i_unit==MIC_FEATURE_UNIT_ID )
   {
      switch (wValue_msb)
      {
      case CS_MUTE:
         if( length==1 )
         {
            Usb_write_endpoint_data(EP_CONTROL, 8, mute);
         }
         break;
      case CS_VOLUME:
         if( length==2 )
         {
            Usb_write_endpoint_data(EP_CONTROL, 16, Usb_format_mcu_to_usb_data(16, VOL_RES));
         }
         break;
      }
   }
   */

    /*
        // 48000 - 44100 = 3900
        Usb_write_endpoint_data(EP_CONTROL, 8, 0x3c);
        Usb_write_endpoint_data(EP_CONTROL, 8, 0x0f);
        Usb_write_endpoint_data(EP_CONTROL, 8, 0x00);
*/

    Usb_ack_control_in_ready_send();
    while (!Is_usb_control_out_received())
        ;
    Usb_ack_control_out_received_free();
}
#endif

void audio_get_cur(void)
{
    U16 i_unit;
    U16 length;
    i_unit = (wIndex >> 8);  // wIndex high byte is interface number
    length = wLength;

    Usb_ack_setup_received_free();
    Usb_reset_endpoint_fifo_access(EP_CONTROL);

    if ((usb_type == USB_SETUP_GET_CLASS_ENDPOINT) && (wValue_msb == UAC_EP_CS_ATTR_SAMPLE_RATE)) {
        if (speed == 0) {
            Usb_write_endpoint_data(EP_CONTROL, 8, 0x44);
            Usb_write_endpoint_data(EP_CONTROL, 8, 0xac);
            Usb_write_endpoint_data(EP_CONTROL, 8, 0x00);
        } else {
            Usb_write_endpoint_data(EP_CONTROL, 8, 0x80);
            Usb_write_endpoint_data(EP_CONTROL, 8, 0xbb);
            Usb_write_endpoint_data(EP_CONTROL, 8, 0x00);
        }
    }

    // BSB 20130604 disabling UAC1 IN
    /*
   else if( i_unit==MIC_FEATURE_UNIT_ID )
   {
      switch (wValue_msb)
      {
      case CS_MUTE:
         if( length==1 )
         {
            Usb_write_endpoint_data(EP_CONTROL, 8, mute);
         }
         break;
      case CS_VOLUME:
         if( length==2 )
         {
            Usb_write_endpoint_data(EP_CONTROL, 16, Usb_format_mcu_to_usb_data(16, volume));
         }
         break;
      }

   }
   */

#ifdef FEATURE_VOLUME_CTRL
    else if (i_unit == SPK_FEATURE_UNIT_ID) {
        switch (wValue_msb) {
        case CS_MUTE:
            Usb_write_endpoint_data(EP_CONTROL, 8, usb_spk_mute);
            break;
        case CS_VOLUME:
            if (length == 2) {
                if (wValue_lsb == CH_LEFT) {
                    // Be on the safe side here, even though fetch is done in uac1_device_audio_task.c init
                    if (spk_vol_usb_L == VOL_INVALID) {
                        // Without working volume flash:
                        spk_vol_usb_L = VOL_DEFAULT;
                        // With working volume flash:
                        // spk_vol_usb_L = usb_volume_flash(CH_LEFT, 0, VOL_READ);
                        spk_vol_mult_L = usb_volume_format(spk_vol_usb_L);
                    }
                    Usb_write_endpoint_data(EP_CONTROL, 16, Usb_format_mcu_to_usb_data(16, spk_vol_usb_L));


                } else if (wValue_lsb == CH_RIGHT) {
                    // Be on the safe side here, even though fetch is done in uac1_device_audio_task.c init
                    if (spk_vol_usb_R == VOL_INVALID) {
                        // Without working volume flash:
                        spk_vol_usb_R = VOL_DEFAULT;
                        // With working volume flash:
                        // spk_vol_usb_R = usb_volume_flash(CH_RIGHT, 0, VOL_READ);
                        spk_vol_mult_R = usb_volume_format(spk_vol_usb_R);
                    }
                    Usb_write_endpoint_data(EP_CONTROL, 16, Usb_format_mcu_to_usb_data(16, spk_vol_usb_R));
                }
            }
            break;
        }
    }
#endif

    Usb_ack_control_in_ready_send();
    while (!Is_usb_control_out_received())
        ;
    Usb_ack_control_out_received_free();
}

void audio_set_cur(void)
{
    U16 i_unit;
    U16 length;
    i_unit = (wIndex >> 8);  // wIndex high byte is interface number
    length = wLength;

    Usb_ack_setup_received_free();
    while (!Is_usb_control_out_received())
        ;
    Usb_reset_endpoint_fifo_access(EP_CONTROL);

    if ((usb_type == USB_SETUP_SET_CLASS_ENDPOINT) && (wValue_msb == UAC_EP_CS_ATTR_SAMPLE_RATE)) {
        if (Usb_read_endpoint_data(EP_CONTROL, 8) == 0x44)
            speed = 0;
        else
            speed = 1;

        freq_changed = TRUE;
        if (speed == 0) {  // 44.1khz


            current_freq.frequency = FREQ_44;
            // BSB 20130602: code section moved here from uac1_device_audio_task.c
            FB_rate = (44 << 14) + (1 << 14) / 10;
            FB_rate_initial = FB_rate;                      // BSB 20131031 Record FB_rate as it was set by control system
            FB_rate_nominal = FB_rate + FB_NOMINAL_OFFSET;  // BSB 20131115 Record FB_rate as it was set by control system;
        } else {                                            // 48khz


            current_freq.frequency = FREQ_48;
            // BSB 20130602: code section moved here from uac1_device_audio_task.c
            FB_rate = 48 << 14;
            FB_rate_initial = FB_rate;                      // BSB 20131031 Record FB_rate as it was set by control system
            FB_rate_nominal = FB_rate + FB_NOMINAL_OFFSET;  // BSB 20131115 Record FB_rate as it was set by control system;
        }

        spk_mute = TRUE;  // mute speaker while changing frequency and oscillator
        mobo_clear_dac_channel();

        mobo_xo_select(current_freq.frequency, MOBO_SRC_UAC1);  // GPIO XO control and frequency indication
        mobo_clock_division(current_freq.frequency);            // This is redundant in UAC1, but we attempt restart for good measure!

    }

    // BSB 20160318 experimenting with mute and playback volume control
#ifdef FEATURE_VOLUME_CTRL
    else if (i_unit == SPK_FEATURE_UNIT_ID) {
        uint8_t temp1 = 0;
        uint8_t temp2 = 0;

        if (wValue_msb == CS_MUTE) {
            temp1 = Usb_read_endpoint_data(EP_CONTROL, 8);
            usb_spk_mute = temp1;
        } else if (wValue_msb == CS_VOLUME) {
            if (length == 2) {
                temp1 = Usb_read_endpoint_data(EP_CONTROL, 8);
                temp2 = Usb_read_endpoint_data(EP_CONTROL, 8);
                if (wValue_lsb == CH_LEFT) {
                    LSB(spk_vol_usb_L) = temp1;
                    MSB(spk_vol_usb_L) = temp2;
                    spk_vol_mult_L = usb_volume_format(spk_vol_usb_L);


                } else if (wValue_lsb == CH_RIGHT) {
                    LSB(spk_vol_usb_R) = temp1;
                    MSB(spk_vol_usb_R) = temp2;
                    spk_vol_mult_R = usb_volume_format(spk_vol_usb_R);
                }
            }
        }
    }
#endif

    // BSB 20130604 disabling UAC1 IN
    /*
   else if( i_unit==MIC_FEATURE_UNIT_ID )
   {
      switch (wValue_msb)
      {
      case CS_MUTE:
         if( length==1 )
         {
            mute=Usb_read_endpoint_data(EP_CONTROL, 8);
         }
         break;
      case CS_VOLUME:
         if( length==2 )
         {
            LSB(volume)= Usb_read_endpoint_data(EP_CONTROL, 8);
            MSB(volume)= Usb_read_endpoint_data(EP_CONTROL, 8);
         }
         break;
      }
   }
   */

    Usb_ack_control_out_received_free();
    Usb_ack_control_in_ready_send();
    while (!Is_usb_control_in_ready())
        ;
}

//! This function is called by the standard USB read request function when
//! the USB request is not supported. This function returns TRUE when the
//! request is processed. This function returns FALSE if the request is not
//! supported. In this case, a STALL handshake will be automatically
//! sent by the standard USB read request function.
//!
Bool uac1_user_read_request(U8 type, U8 request)
{

    usb_type = type;

    // this should vector to specified interface handler
    if (type == IN_INTERFACE && request == GET_DESCRIPTOR)
        return uac1_user_get_interface_descriptor();
    // Read wValue
    // why are these file statics?
    wValue_lsb = Usb_read_endpoint_data(EP_CONTROL, 8);
    wValue_msb = Usb_read_endpoint_data(EP_CONTROL, 8);
    wIndex = usb_format_usb_to_mcu_data(16, Usb_read_endpoint_data(EP_CONTROL, 16));
    wLength = usb_format_usb_to_mcu_data(16, Usb_read_endpoint_data(EP_CONTROL, 16));

    //** Specific request from Class HID
    // this should vector to specified interface handler

    //  assume all other requests are for AUDIO interface

    switch (request) {
    case BR_REQUEST_SET_CUR:
        audio_set_cur();
        return TRUE;
        // No need to break here !

    case BR_REQUEST_SET_MIN:  //! Set MIN,MAX and RES not supported
    case BR_REQUEST_SET_MAX:
    case BR_REQUEST_SET_RES:
        return FALSE;
        // No need to break here !

    case BR_REQUEST_GET_CUR:
        audio_get_cur();
        return TRUE;
        // No need to break here !

#ifdef FEATURE_VOLUME_CTRL
    case BR_REQUEST_GET_MIN:
        audio_get_min();
        return TRUE;
        // No need to break here !

    case BR_REQUEST_GET_MAX:
        audio_get_max();
        return TRUE;
        // No need to break here !

    case BR_REQUEST_GET_RES:
        audio_get_res();
        return TRUE;
        // No need to break here !
#endif

    default:
        return FALSE;
        // No need to break here !
    }

    return FALSE;  // No supported request
}
