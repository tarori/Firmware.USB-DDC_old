/* -*- mode: c++; tab-width: 4; c-basic-offset: 4 -*- */
/*
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
 */

//_____ I N C L U D E S ____________________________________________________

#include "conf_usb.h"
//#include "features.h"


#include "uac2_usb_descriptors.h"
#include "usb_audio.h"
#include "usb_descriptors.h"
#include "usb_drv.h"
#include "usb_specific_request.h"
#include "usb_standard_request.h"

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

// usb_user_device_descriptor
const S_usb_device_descriptor uac2_dg8saq_usb_dev_desc = {
    sizeof(S_usb_device_descriptor),
    DEVICE_DESCRIPTOR,
    Usb_format_mcu_to_usb_data(16, USB_SPECIFICATION),
    DEVICE_CLASS_UAC2,
    DEVICE_SUB_CLASS_UAC2,
    DEVICE_PROTOCOL_UAC2,
    EP_CONTROL_LENGTH,
    Usb_format_mcu_to_usb_data(16, DG8SAQ_VENDOR_ID),
    Usb_format_mcu_to_usb_data(16, DG8SAQ_PRODUCT_ID),
    Usb_format_mcu_to_usb_data(16, RELEASE_NUMBER),
    MAN_INDEX,
    PROD_INDEX,
    SN_INDEX,
    NB_CONFIGURATION};

const S_usb_device_descriptor uac2_audio_usb_dev_desc = {
    sizeof(S_usb_device_descriptor),
    DEVICE_DESCRIPTOR,
    Usb_format_mcu_to_usb_data(16, USB_SPECIFICATION),
    DEVICE_CLASS_UAC2,
    DEVICE_SUB_CLASS_UAC2,
    DEVICE_PROTOCOL_UAC2,
    EP_CONTROL_LENGTH,
    Usb_format_mcu_to_usb_data(16, AUDIO_VENDOR_ID),

    // BSB 20120928 new VID/PID system
    Usb_format_mcu_to_usb_data(16, AUDIO_PRODUCT_ID_10),

    Usb_format_mcu_to_usb_data(16, RELEASE_NUMBER),
    MAN_INDEX,
    PROD_INDEX,
    SN_INDEX,
    NB_CONFIGURATION};

// usb_user_configuration_descriptor FS
const S_usb_user_configuration_descriptor uac2_usb_conf_desc_fs
    = {
        {sizeof(S_usb_configuration_descriptor),
            CONFIGURATION_DESCRIPTOR,
            Usb_format_mcu_to_usb_data(16, sizeof(S_usb_user_configuration_descriptor)),
            NB_INTERFACE,
            CONF_NB,
            CONF_INDEX,
            CONF_ATTRIBUTES,
            MAX_POWER}

        // Config interface at endpoint 0
        // Interface used by Widget-Control. No endpoints. Comes up as "Other device" in Windows

        ,
        {
            sizeof(S_usb_interface_association_descriptor), DESCRIPTOR_IAD, FIRST_INTERFACE1  // bFirstInterface
            ,
            INTERFACE_COUNT1  // bInterfaceCount
            ,
            FUNCTION_CLASS  // INTERFACE_CLASS1
            ,
            FUNCTION_SUB_CLASS  // INTERFACE_SUB_CLASS1
            ,
            FUNCTION_PROTOCOL  // INTERFACE_PROTOCOL1
            ,
            INTERFACE_INDEX1  // FUNCTION_INDEX
        },

        {sizeof(S_usb_interface_descriptor),
            INTERFACE_DESCRIPTOR,
            INTERFACE_NB1,
            ALTERNATE_NB1,
            NB_ENDPOINT1,
            INTERFACE_CLASS1,
            INTERFACE_SUB_CLASS1,
            INTERFACE_PROTOCOL1,
            INTERFACE_INDEX1},

        {sizeof(S_usb_ac_interface_descriptor_2), CS_INTERFACE, HEADER_SUB_TYPE, Usb_format_mcu_to_usb_data(16, AUDIO_CLASS_REVISION_2), HEADSET_CATEGORY, Usb_format_mcu_to_usb_data(16, sizeof(S_usb_ac_interface_descriptor_2) + /*2* */ sizeof(S_usb_clock_source_descriptor) + /*2* */ sizeof(S_usb_in_ter_descriptor_2)
#ifdef FEATURE_VOLUME_CTRL  // Only if volume control is compiled in do we expose it in the feature unit
                                                                                                                                                                                              + /*2* */ sizeof(S_usb_feature_unit_descriptor_2)
#endif
                                                                                                                                                                                              + /*2* */ sizeof(S_usb_out_ter_descriptor_2)),
            MIC_LATENCY_CONTROL}

        ,
        {
            sizeof(S_usb_clock_source_descriptor), CS_INTERFACE, DESCRIPTOR_SUBTYPE_AUDIO_AC_CLOCK_SOURCE, CSD_ID_2, CSD_ID_2_TYPE, CSD_ID_2_CONTROL, 0x00  // no association or SPK_INPUT_TERMINAL_ID	// Was: INPUT_TERMINAL_ID
            ,
            CLOCK_SOURCE_2_INDEX  //   Was: 0x00 BSB UAC2 debug WHY?
        },
        {sizeof(S_usb_in_ter_descriptor_2), CS_INTERFACE, INPUT_TERMINAL_SUB_TYPE, SPK_INPUT_TERMINAL_ID, Usb_format_mcu_to_usb_data(16, SPK_INPUT_TERMINAL_TYPE), SPK_INPUT_TERMINAL_ASSOCIATION,
            CSD_ID_2  // Straight clock
            ,
            SPK_INPUT_TERMINAL_NB_CHANNELS, Usb_format_mcu_to_usb_data(32, SPK_INPUT_TERMINAL_CHANNEL_CONF)  // 0 in Pro-Ject
            ,
            SPK_INPUT_TERMINAL_CH_NAME_ID, Usb_format_mcu_to_usb_data(16, INPUT_TERMINAL_CONTROLS), SPK_INPUT_TERMINAL_STRING_DESC},
#ifdef FEATURE_VOLUME_CTRL  // Only if volume control is compiled in do we expose it in the feature unit
        {sizeof(S_usb_feature_unit_descriptor_2), CS_INTERFACE, FEATURE_UNIT_SUB_TYPE, SPK_FEATURE_UNIT_ID, SPK_FEATURE_UNIT_SOURCE_ID, Usb_format_mcu_to_usb_data(32, SPK_BMA_CONTROLS), Usb_format_mcu_to_usb_data(32, SPK_BMA_CONTROLS_CH_1), Usb_format_mcu_to_usb_data(32, SPK_BMA_CONTROLS_CH_2), 0x00},
#endif

        {sizeof(S_usb_out_ter_descriptor_2), CS_INTERFACE, OUTPUT_TERMINAL_SUB_TYPE, SPK_OUTPUT_TERMINAL_ID, Usb_format_mcu_to_usb_data(16, SPK_OUTPUT_TERMINAL_TYPE), SPK_OUTPUT_TERMINAL_ASSOCIATION, SPK_OUTPUT_TERMINAL_SOURCE_ID,
            CSD_ID_2  // Straight clock
            ,
            Usb_format_mcu_to_usb_data(16, SPK_OUTPUT_TERMINAL_CONTROLS), 0x00}

        ,
        // ALT0 has no endpoints
        {sizeof(S_usb_as_interface_descriptor), INTERFACE_DESCRIPTOR, STD_AS_INTERFACE_OUT, ALT0_AS_INTERFACE_INDEX, ALT0_AS_NB_ENDPOINT, ALT0_AS_INTERFACE_CLASS, ALT0_AS_INTERFACE_SUB_CLASS, ALT0_AS_INTERFACE_PROTOCOL, 0x00},
        // ALT1 is for 24-bit audio streaming
        {sizeof(S_usb_as_interface_descriptor), INTERFACE_DESCRIPTOR, STD_AS_INTERFACE_OUT, ALT1_AS_INTERFACE_INDEX, ALT1_AS_NB_ENDPOINT_OUT, ALT1_AS_INTERFACE_CLASS, ALT1_AS_INTERFACE_SUB_CLASS, ALT1_AS_INTERFACE_PROTOCOL, 0x00},

        {
            sizeof(S_usb_as_g_interface_descriptor_2), CS_INTERFACE, GENERAL_SUB_TYPE, SPK_INPUT_TERMINAL_ID, AS_CONTROLS, AS_FORMAT_TYPE, Usb_format_mcu_to_usb_data(32, AS_FORMATS), AS_NB_CHANNELS, Usb_format_mcu_to_usb_data(32, AS_CHAN_CONFIG), SPK_INPUT_TERMINAL_CH_NAME_ID  // 0x00
        },
        {sizeof(S_usb_format_type_2), CS_INTERFACE, FORMAT_SUB_TYPE, FORMAT_TYPE_1, FORMAT_SUBSLOT_SIZE_1, FORMAT_BIT_RESOLUTION_1}

        ,
        {sizeof(S_usb_endpoint_audio_descriptor_2), ENDPOINT_DESCRIPTOR, ENDPOINT_NB_2, EP_ATTRIBUTES_2, Usb_format_mcu_to_usb_data(16, EP_SIZE_2_FS), EP_INTERVAL_2_FS},
        {sizeof(S_usb_endpoint_audio_specific_2), CS_ENDPOINT, GENERAL_SUB_TYPE, AUDIO_EP_ATRIBUTES, AUDIO_EP_CONTROLS, AUDIO_EP_DELAY_UNIT, Usb_format_mcu_to_usb_data(16, AUDIO_EP_LOCK_DELAY)},
        {sizeof(S_usb_endpoint_audio_descriptor_2), ENDPOINT_DESCRIPTOR, ENDPOINT_NB_3, EP_ATTRIBUTES_3, Usb_format_mcu_to_usb_data(16, EP_SIZE_3_FS), EP_INTERVAL_3_FS},
        // ALT2 is for 16-bit audio streaming, otherwise identical to ALT1

        {sizeof(S_usb_as_interface_descriptor), INTERFACE_DESCRIPTOR, STD_AS_INTERFACE_OUT, ALT2_AS_INTERFACE_INDEX, ALT2_AS_NB_ENDPOINT_OUT, ALT2_AS_INTERFACE_CLASS, ALT2_AS_INTERFACE_SUB_CLASS, ALT2_AS_INTERFACE_PROTOCOL, 0x00},

        {
            sizeof(S_usb_as_g_interface_descriptor_2), CS_INTERFACE, GENERAL_SUB_TYPE, SPK_INPUT_TERMINAL_ID, AS_CONTROLS, AS_FORMAT_TYPE, Usb_format_mcu_to_usb_data(32, AS_FORMATS), AS_NB_CHANNELS, Usb_format_mcu_to_usb_data(32, AS_CHAN_CONFIG), SPK_INPUT_TERMINAL_CH_NAME_ID  // 0x00
        }

        ,

        {
            sizeof(S_usb_format_type_2), CS_INTERFACE, FORMAT_SUB_TYPE, FORMAT_TYPE_2, FORMAT_SUBSLOT_SIZE_2  // bBitResolution
            ,
            FORMAT_BIT_RESOLUTION_2  // bBitResolution
        }

        ,
        {sizeof(S_usb_endpoint_audio_descriptor_2), ENDPOINT_DESCRIPTOR, ENDPOINT_NB_2, EP_ATTRIBUTES_2, Usb_format_mcu_to_usb_data(16, EP_SIZE_2_FS), EP_INTERVAL_2_FS},
        {sizeof(S_usb_endpoint_audio_specific_2), CS_ENDPOINT, GENERAL_SUB_TYPE, AUDIO_EP_ATRIBUTES, AUDIO_EP_CONTROLS, AUDIO_EP_DELAY_UNIT, Usb_format_mcu_to_usb_data(16, AUDIO_EP_LOCK_DELAY)},
        {sizeof(S_usb_endpoint_audio_descriptor_2), ENDPOINT_DESCRIPTOR, ENDPOINT_NB_3, EP_ATTRIBUTES_3, Usb_format_mcu_to_usb_data(16, EP_SIZE_3_FS), EP_INTERVAL_3_FS}

        // End of audio streaming interface and its ALTs

        // BSB 20120720 Insert EP 4 and 5, HID TX and RX begin
        // BSB 20120720 Insert EP 4 and 5, HID TX and RX end
};

#if (USB_HIGH_SPEED_SUPPORT == ENABLED)

// usb_user_configuration_descriptor HS
const S_usb_user_configuration_descriptor uac2_usb_conf_desc_hs
    = {
        {sizeof(S_usb_configuration_descriptor),
            CONFIGURATION_DESCRIPTOR,
            Usb_format_mcu_to_usb_data(16, sizeof(S_usb_user_configuration_descriptor)),
            NB_INTERFACE,  // BSB 20120720 enabled
            CONF_NB,
            CONF_INDEX,
            CONF_ATTRIBUTES,
            MAX_POWER}

        // Interface used by Widget-Control. No endpoints. Comes up as "Other device" in Windows
        // Config interface at endpoint 0

        ,

        //! Here is where Audio Class 2 specific stuff is

        {
            sizeof(S_usb_interface_association_descriptor)  // 4.6
            ,
            DESCRIPTOR_IAD, FIRST_INTERFACE1  // bFirstInterface
            ,
            INTERFACE_COUNT1  // bInterfaceCount
            ,
            FUNCTION_CLASS, FUNCTION_SUB_CLASS, FUNCTION_PROTOCOL, FUNCTION_INDEX},

        {sizeof(S_usb_interface_descriptor),
            INTERFACE_DESCRIPTOR,
            INTERFACE_NB1,
            ALTERNATE_NB1,
            NB_ENDPOINT1,
            INTERFACE_CLASS1,
            INTERFACE_SUB_CLASS1,
            INTERFACE_PROTOCOL1,
            INTERFACE_INDEX1},

        {sizeof(S_usb_ac_interface_descriptor_2), CS_INTERFACE, HEADER_SUB_TYPE, Usb_format_mcu_to_usb_data(16, AUDIO_CLASS_REVISION_2), HEADSET_CATEGORY, Usb_format_mcu_to_usb_data(16, sizeof(S_usb_ac_interface_descriptor_2) + /*2* */ sizeof(S_usb_clock_source_descriptor) + /*2* */ sizeof(S_usb_in_ter_descriptor_2)
#ifdef FEATURE_VOLUME_CTRL  // Only if volume control is compiled in do we expose it in the feature unit
                                                                                                                                                                                              + /*2* */ sizeof(S_usb_feature_unit_descriptor_2)
#endif
                                                                                                                                                                                              + /*2* */ sizeof(S_usb_out_ter_descriptor_2)),
            MIC_LATENCY_CONTROL},
        {sizeof(S_usb_clock_source_descriptor), CS_INTERFACE, DESCRIPTOR_SUBTYPE_AUDIO_AC_CLOCK_SOURCE, CSD_ID_2, CSD_ID_2_TYPE, CSD_ID_2_CONTROL, 0x00  // no association SPK_INPUT_TERMINAL_ID	// Was: OUTPUT_TERMINAL_ID
            ,
            CLOCK_SOURCE_2_INDEX},


        {sizeof(S_usb_in_ter_descriptor_2), CS_INTERFACE, INPUT_TERMINAL_SUB_TYPE, SPK_INPUT_TERMINAL_ID, Usb_format_mcu_to_usb_data(16, SPK_INPUT_TERMINAL_TYPE), SPK_INPUT_TERMINAL_ASSOCIATION,
            CSD_ID_2  // Straight clock
            ,
            SPK_INPUT_TERMINAL_NB_CHANNELS, Usb_format_mcu_to_usb_data(32, SPK_INPUT_TERMINAL_CHANNEL_CONF)  // 0 in Pro-Ject
            ,
            SPK_INPUT_TERMINAL_CH_NAME_ID, Usb_format_mcu_to_usb_data(16, INPUT_TERMINAL_CONTROLS), SPK_INPUT_TERMINAL_STRING_DESC},
#ifdef FEATURE_VOLUME_CTRL  // Only if volume control is compiled in do we expose it in the feature unit
        {sizeof(S_usb_feature_unit_descriptor_2), CS_INTERFACE, FEATURE_UNIT_SUB_TYPE, SPK_FEATURE_UNIT_ID, SPK_FEATURE_UNIT_SOURCE_ID, Usb_format_mcu_to_usb_data(32, SPK_BMA_CONTROLS), Usb_format_mcu_to_usb_data(32, SPK_BMA_CONTROLS_CH_1), Usb_format_mcu_to_usb_data(32, SPK_BMA_CONTROLS_CH_2), 0x00},
#endif
        {sizeof(S_usb_out_ter_descriptor_2), CS_INTERFACE, OUTPUT_TERMINAL_SUB_TYPE, SPK_OUTPUT_TERMINAL_ID, Usb_format_mcu_to_usb_data(16, SPK_OUTPUT_TERMINAL_TYPE), SPK_OUTPUT_TERMINAL_ASSOCIATION, SPK_OUTPUT_TERMINAL_SOURCE_ID,
            CSD_ID_2  // Straight clock
            ,
            Usb_format_mcu_to_usb_data(16, SPK_OUTPUT_TERMINAL_CONTROLS), 0x00},

        // ALT0 has no endpoints
        {sizeof(S_usb_as_interface_descriptor), INTERFACE_DESCRIPTOR, STD_AS_INTERFACE_OUT, ALT0_AS_INTERFACE_INDEX, ALT0_AS_NB_ENDPOINT, ALT0_AS_INTERFACE_CLASS, ALT0_AS_INTERFACE_SUB_CLASS, ALT0_AS_INTERFACE_PROTOCOL, 0x00},
        // ALT1 is for 24-bit audio streaming
        {sizeof(S_usb_as_interface_descriptor), INTERFACE_DESCRIPTOR, STD_AS_INTERFACE_OUT, ALT1_AS_INTERFACE_INDEX, ALT1_AS_NB_ENDPOINT_OUT, ALT1_AS_INTERFACE_CLASS, ALT1_AS_INTERFACE_SUB_CLASS, ALT1_AS_INTERFACE_PROTOCOL, 0x00},
        {
            sizeof(S_usb_as_g_interface_descriptor_2), CS_INTERFACE, GENERAL_SUB_TYPE, SPK_INPUT_TERMINAL_ID, AS_CONTROLS, AS_FORMAT_TYPE, Usb_format_mcu_to_usb_data(32, AS_FORMATS), AS_NB_CHANNELS, Usb_format_mcu_to_usb_data(32, AS_CHAN_CONFIG), SPK_INPUT_TERMINAL_CH_NAME_ID  // 0x00
        },

        {sizeof(S_usb_format_type_2), CS_INTERFACE, FORMAT_SUB_TYPE, FORMAT_TYPE_1, FORMAT_SUBSLOT_SIZE_1, FORMAT_BIT_RESOLUTION_1}

        ,
        {sizeof(S_usb_endpoint_audio_descriptor_2), ENDPOINT_DESCRIPTOR, ENDPOINT_NB_2, EP_ATTRIBUTES_2, Usb_format_mcu_to_usb_data(16, EP_SIZE_2_HS), EP_INTERVAL_2_HS},
        {sizeof(S_usb_endpoint_audio_specific_2), CS_ENDPOINT, GENERAL_SUB_TYPE, AUDIO_EP_ATRIBUTES, AUDIO_EP_CONTROLS, AUDIO_EP_DELAY_UNIT, Usb_format_mcu_to_usb_data(16, AUDIO_EP_LOCK_DELAY)},
        {sizeof(S_usb_endpoint_audio_descriptor_2), ENDPOINT_DESCRIPTOR, ENDPOINT_NB_3, EP_ATTRIBUTES_3, Usb_format_mcu_to_usb_data(16, EP_SIZE_3_HS), EP_INTERVAL_3_HS},

        // ALT2 is for 16-bit audio streaming, otherwise identical to ALT1
        {sizeof(S_usb_as_interface_descriptor), INTERFACE_DESCRIPTOR, STD_AS_INTERFACE_OUT, ALT2_AS_INTERFACE_INDEX, ALT2_AS_NB_ENDPOINT_OUT, ALT2_AS_INTERFACE_CLASS, ALT2_AS_INTERFACE_SUB_CLASS, ALT2_AS_INTERFACE_PROTOCOL, 0x00},
        {
            sizeof(S_usb_as_g_interface_descriptor_2), CS_INTERFACE, GENERAL_SUB_TYPE, SPK_INPUT_TERMINAL_ID, AS_CONTROLS, AS_FORMAT_TYPE, Usb_format_mcu_to_usb_data(32, AS_FORMATS), AS_NB_CHANNELS, Usb_format_mcu_to_usb_data(32, AS_CHAN_CONFIG), SPK_INPUT_TERMINAL_CH_NAME_ID  // 0x00
        },

        {
            sizeof(S_usb_format_type_2), CS_INTERFACE, FORMAT_SUB_TYPE, FORMAT_TYPE_2, FORMAT_SUBSLOT_SIZE_2  // bBitResolution
            ,
            FORMAT_BIT_RESOLUTION_2  // bBitResolution
        }

        ,
        {sizeof(S_usb_endpoint_audio_descriptor_2), ENDPOINT_DESCRIPTOR, ENDPOINT_NB_2, EP_ATTRIBUTES_2, Usb_format_mcu_to_usb_data(16, EP_SIZE_2_HS), EP_INTERVAL_2_HS},
        {sizeof(S_usb_endpoint_audio_specific_2), CS_ENDPOINT, GENERAL_SUB_TYPE, AUDIO_EP_ATRIBUTES, AUDIO_EP_CONTROLS, AUDIO_EP_DELAY_UNIT, Usb_format_mcu_to_usb_data(16, AUDIO_EP_LOCK_DELAY)},
        {sizeof(S_usb_endpoint_audio_descriptor_2), ENDPOINT_DESCRIPTOR, ENDPOINT_NB_3, EP_ATTRIBUTES_3, Usb_format_mcu_to_usb_data(16, EP_SIZE_3_HS), EP_INTERVAL_3_HS}

        // End of audio streaming interface and its ALTs

        // BSB 20120720 Insert EP 4 and 5, HID TX and RX begin
        // BSB 20120720 Insert EP 4 and 5, HID TX and RX end
};

// usb_qualifier_desc FS
const S_usb_device_qualifier_descriptor uac2_usb_qualifier_desc = {
    sizeof(S_usb_device_qualifier_descriptor),
    DEVICE_QUALIFIER_DESCRIPTOR,
    Usb_format_mcu_to_usb_data(16, USB_SPECIFICATION),
    DEVICE_CLASS_UAC2,
    DEVICE_SUB_CLASS_UAC2,
    DEVICE_PROTOCOL_UAC2,
    EP_CONTROL_LENGTH,
    NB_CONFIGURATION,
    0};
#endif
