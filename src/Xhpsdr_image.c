/* -*- mode: c++; tab-width: 4; c-basic-offset: 4 -*- */
/*
** hpsdr_image.c
**
**  Created on: 2011-02-25
**      Author: Roger E Critchlow Jr, AD5DZ
*/

#include "image.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "board.h"
#include "compiler.h"
#include "conf_usb.h"
#include "gpio.h"
#include "intc.h"
#include "pm.h"
#include "print_funcs.h"
#include "task.h"
#include "usb_task.h"
#include "device_mouse_hid_task.h"
#include "composite_widget.h"
#include "hpsdr_taskAK5394A.h"
#include "taskAK5394A.h"
//#include "I2C.h"

/*
 * Task specific headers.
 */

#include "device_audio_task.h"
#include "hpsdr_device_audio_task.h"
#include "queue.h"
#include "taskEXERCISE.h"
#include "taskMoboCtrl.h"
#include "taskPowerDisplay.h"
#include "taskPushButtonMenu.h"
#include "wdt.h"


/*
** Image specific headers
*/
#include "hpsdr_device_audio_task.h"
#include "hpsdr_usb_descriptors.h"
#include "hpsdr_usb_specific_request.h"
#include "usb_descriptors.h"
#include "usb_specific_request.h"

// image launch
static void x_image_boot(void)
{
    configTSK_USB_DEV_PERIOD = HPSDR_configTSK_USB_DEV_PERIOD;
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
    hpsdr_AK5394A_task_init();
    hpsdr_device_audio_task_init(HPSDR_EP_IQ_IN, HPSDR_EP_IQ_OUT, 0);
}

// descriptor accessors
static uint8_t* x_image_get_dev_desc_pointer(void)
{
    return (uint8_t*)&hpsdr_usb_dev_desc;
}
static uint16_t x_image_get_dev_desc_length(void)
{
    return (uint16_t)sizeof(hpsdr_usb_dev_desc);
}
static uint8_t* x_image_get_conf_desc_pointer(void)
{
    return (uint8_t*)&hpsdr_usb_conf_desc_fs;
}
static uint16_t x_image_get_conf_desc_length(void)
{
    return sizeof(hpsdr_usb_conf_desc_fs);
}
static uint8_t* x_image_get_conf_desc_fs_pointer(void)
{
    return (uint8_t*)&hpsdr_usb_conf_desc_fs;
}
static uint16_t x_image_get_conf_desc_fs_length(void)
{
    return sizeof(hpsdr_usb_conf_desc_fs);
}
#if USB_HIGH_SPEED_SUPPORT == ENABLED
static uint8_t* x_image_get_conf_desc_hs_pointer(void)
{
    return (uint8_t*)&hpsdr_usb_conf_desc_hs;
}
static uint16_t x_image_get_conf_desc_hs_length(void)
{
    return sizeof(hpsdr_usb_conf_desc_hs);
}
static uint8_t* x_image_get_qualifier_desc_pointer(void)
{
    return (uint8_t*)&hpsdr_usb_qualifier_desc;
}
static uint16_t x_image_get_qualifier_desc_length(void)
{
    return sizeof(hpsdr_usb_qualifier_desc);
}
#endif
// specific request handlers
static void x_image_user_endpoint_init(uint8_t conf_nb)
{
    hpsdr_user_endpoint_init(conf_nb);
}
static Bool x_image_user_read_request(uint8_t type, uint8_t request)
{
    return hpsdr_user_read_request(type, request);
}
static void x_image_user_set_interface(U8 wIndex, U8 wValue)
{
    hpsdr_user_set_interface(wIndex, wValue);
}

const image_t hpsdr_image = {
    x_image_boot,
    x_image_init,
    x_image_task_init,
    x_image_get_dev_desc_pointer,
    x_image_get_dev_desc_length,
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
