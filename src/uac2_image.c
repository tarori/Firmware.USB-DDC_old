/* -*- mode: c++; tab-width: 4; c-basic-offset: 4 -*- */
/*
** uac2_star_image.c
**
**  Created on: 2011-02-25
**      Author: Roger E Critchlow Jr, AD5DZ
*/

#include <stdio.h>

#include "board.h"
#include "compiler.h"
#include "gpio.h"
#include "intc.h"
#include "pm.h"
#include "print_funcs.h"
#include "FreeRTOS.h"
#include "task.h"
#include "conf_usb.h"
#include "usb_task.h"
#include "device_mouse_hid_task.h"
#include "composite_widget.h"
#include "taskAK5394A.h"
#include "uac2_taskAK5394A.h"
//#include "I2C.h"

/*
 * Task specific headers.
 */

#include "features.h"
#include "image.h"
#include "queue.h"
#include "taskEXERCISE.h"
#include "taskMoboCtrl.h"
#include "taskPowerDisplay.h"
#include "taskPushButtonMenu.h"
#include "wdt.h"


/*
** Image specific headers
*/
#include "uac2_device_audio_task.h"
#include "uac2_usb_descriptors.h"
#include "uac2_usb_specific_request.h"
#include "usb_descriptors.h"
#include "usb_specific_request.h"

// image launch
static void x_image_boot(void)
{
    configTSK_USB_DEV_PERIOD = UAC2_configTSK_USB_DEV_PERIOD;
}

static void x_image_init(void)
{
}

static void x_image_task_init(void)
{
    // Initialize USB task


    usb_task_init();

    mutexEP_IN = xSemaphoreCreateMutex();  // for co-ordinating multiple tasks using EP IN

    vStartTaskMoboCtrl();
    // vStartTaskEXERCISE( tskIDLE_PRIORITY );
    uac2_AK5394A_task_init();
    device_mouse_hid_task_init(UAC2_EP_HID_TX);  // Added BSB 20120719
    uac2_device_audio_task_init(UAC2_EP_AUDIO_IN, UAC2_EP_AUDIO_OUT, UAC2_EP_AUDIO_OUT_FB);
}

// descriptor accessors
static uint8_t* x_audio_get_dev_desc_pointer(void)
{
    return (uint8_t*)&uac2_audio_usb_dev_desc;
}
static uint16_t x_audio_get_dev_desc_length(void)
{
    return (uint16_t)sizeof(uac2_audio_usb_dev_desc);
}
static uint8_t* x_dg8saq_get_dev_desc_pointer(void)
{
    return (uint8_t*)&uac2_dg8saq_usb_dev_desc;
}
static uint16_t x_dg8saq_get_dev_desc_length(void)
{
    return (uint16_t)sizeof(uac2_dg8saq_usb_dev_desc);
}
static uint8_t* x_image_get_conf_desc_pointer(void)
{
    return (uint8_t*)&uac2_usb_conf_desc_fs;
}
static uint16_t x_image_get_conf_desc_length(void)
{
    return sizeof(uac2_usb_conf_desc_fs);
}
static uint8_t* x_image_get_conf_desc_fs_pointer(void)
{
    return (uint8_t*)&uac2_usb_conf_desc_fs;
}
static uint16_t x_image_get_conf_desc_fs_length(void)
{
    return sizeof(uac2_usb_conf_desc_fs);
}
#if USB_HIGH_SPEED_SUPPORT == ENABLED
static uint8_t* x_image_get_conf_desc_hs_pointer(void)
{
    return (uint8_t*)&uac2_usb_conf_desc_hs;
}
static uint16_t x_image_get_conf_desc_hs_length(void)
{
    return sizeof(uac2_usb_conf_desc_hs);
}
static uint8_t* x_image_get_qualifier_desc_pointer(void)
{
    return (uint8_t*)&uac2_usb_qualifier_desc;
}
static uint16_t x_image_get_qualifier_desc_length(void)
{
    return sizeof(uac2_usb_qualifier_desc);
}
#endif
// specific request handlers
static void x_image_user_endpoint_init(uint8_t conf_nb)
{
    uac2_user_endpoint_init(conf_nb);
}
static Bool x_image_user_read_request(uint8_t type, uint8_t request)
{
    return uac2_user_read_request(type, request);
}
static void x_image_user_set_interface(U8 wIndex, U8 wValue)
{
    uac2_user_set_interface(wIndex, wValue);
}

// image vector table
const image_t uac2_audio_image = {
    x_image_boot,
    x_image_init,
    x_image_task_init,
    x_audio_get_dev_desc_pointer,
    x_audio_get_dev_desc_length,
    x_image_get_conf_desc_pointer,
    x_image_get_conf_desc_length,
    x_image_get_conf_desc_fs_pointer,
    x_image_get_conf_desc_fs_length,
#if USB_HIGH_SPEED_SUPPORT == ENABLED
    x_image_get_conf_desc_hs_pointer,
    x_image_get_conf_desc_hs_length,
    x_image_get_qualifier_desc_pointer,
    x_image_get_qualifier_desc_length,
#else
    x_image_get_conf_desc_fs_pointer,
    x_image_get_conf_desc_fs_length,
    NULL,
    NULL,
#endif
    x_image_user_endpoint_init,
    x_image_user_read_request,
    x_image_user_set_interface};

const image_t uac2_dg8saq_image = {
    x_image_boot,
    x_image_init,
    x_image_task_init,
    x_dg8saq_get_dev_desc_pointer,
    x_dg8saq_get_dev_desc_length,
    x_image_get_conf_desc_pointer,
    x_image_get_conf_desc_length,
    x_image_get_conf_desc_fs_pointer,
    x_image_get_conf_desc_fs_length,
#if USB_HIGH_SPEED_SUPPORT == ENABLED
    x_image_get_conf_desc_hs_pointer,
    x_image_get_conf_desc_hs_length,
    x_image_get_qualifier_desc_pointer,
    x_image_get_qualifier_desc_length,
#else
    x_image_get_conf_desc_fs_pointer,
    x_image_get_conf_desc_fs_length,
    NULL,
    NULL,
#endif
    x_image_user_endpoint_init,
    x_image_user_read_request,
    x_image_user_set_interface};
