/* -*- mode: c; tab-width: 4; c-basic-offset: 4 -*- */
/*
 * taskMoboCtrl.c
 *
 * This task takes care of the Mobo specific functions, such as Si570
 * frequency control, A/D inputs, Bias management and control (D/A output),
 * Transmit/Receive switchover and so on.
 * It accepts parameter updates from the USB task through the DG8SAQ_cmd.c/h
 * And it also makes A/D inputs available to the taskPowerDisplay.c/h, which
 * in turn does a best effort LCD update of the lower two LCD lines.
 *
 *  Created on: 2010-06-13
 *      Author: Loftur Jonasson, TF3LJ
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "board.h"
#include "features.h"
#include "flashc.h"
#include "gpio.h"
#include "queue.h"
#include "rtc.h"
#include "usb_drv.h"
#include "wdt.h"
#include "widget.h"

#include "AD7991.h"
#include "DG8SAQ_cmd.h"
#include "Mobo_config.h"
#include "PCF8574.h"
#include "TMP100.h"
#include "composite_widget.h"
#include "device_audio_task.h"
#include "freq_and_filters.h"
#include "rotary_encoder.h"
#include "taskAK5394A.h"
#include "taskMoboCtrl.h"
#include "usb_specific_request.h"


//#define GPIO_PIN_EXAMPLE_3    GPIO_PUSH_BUTTON_SW2

// Set up NVRAM (EEPROM) storage
#if defined(__GNUC__)
__attribute__((__section__(".userpage")))
#endif
mobo_data_t nvram_cdata;

char lcd_pass1[20];  // Pass data to LCD
char lcd_pass2[20];  // Pass data to LCD
char lcd_pass3[20];  // Pass data to LCD

i2c_avail i2c;  // Availability of probed i2c devices

// Various flags, may be moved around
volatile bool MENU_mode = FALSE;  // LCD Menu mode, used in conjunction with taskPushButtonMenu.c

bool TX_state = FALSE;    // Keep tabs on current TX status
bool TX_flag = FALSE;     // Request for TX to be set
bool SWR_alarm = FALSE;   // SWR alarm condition
bool TMP_alarm = FALSE;   // Temperature alarm condition
bool PA_cal_lo = FALSE;   // Used by PA Bias auto adjust routine
bool PA_cal_hi = FALSE;   // Used by PA Bias auto adjust routine
bool PA_cal = FALSE;      // Indicates PA Bias auto adjust in progress
bool COOLING_fan = TRUE;  // Power Amplifier Cooling Fan (blower)

uint8_t biasInit = 0;  // Power Amplifier Bias initiate flag
                       // (0 = uninitiated => forces init, 1 = class AB, 2 = class A)

uint16_t measured_SWR;  // SWR value x 100, in unsigned int format

/*! \brief Probe and report presence of individual I2C devices
 *
 * \retval none
 */

/*! \brief Print stuff in the second line of the LCD
 *
 * \retval nothing returned.
 */
void lcd_display_V_C_T_in_2nd_line(void)
{
}

/*! \brief Convert AD reading into "Measured Power in centiWatts"
 *
 * \retval Measured Power in centiWatts
 */
// A simplified integer arithmetic version, still with decent accuracy
// (the maximum return value overflows above 655.35W max)
uint16_t measured_Power(uint16_t voltage)
{
    // All standard stuff
    // normalise the measured value from the VSWR bridge
    // Reference voltage is 5V,
    // diode offset ~ .10V
    // R.PWR_Calibrate = Power meter calibration value
    uint32_t measured_P;

    if (voltage > 0)
        voltage = voltage / 0x10 + 82;  // If no input voltage, then we do not add offset voltage
                                        // as this would otherwise result in a bogus minimum power
                                        // reading
                                        // voltage is a MSB adjusted 12 bit value, therefore
                                        // dividing by 10 does not lose any information
                                        // 4096 = 5V,
                                        // 82 = 100mV, compensating for schottky diode loss
    // Formula roughly adjusted for the ratio in the SWR bridge
    measured_P = (uint32_t)voltage * cdata.PWR_Calibrate / 84;
    measured_P = (measured_P * measured_P) / 500000;
    return measured_P;  // Return power in cW
}

/*! \brief Do SWR calcultions and control the PTT2 output
 *
 * \retval nothing returned.
 */
// Read the ADC inputs.
// The global variable ad7991_adc[AD7991_POWER_OUT] contains a measurement of the
// Power transmitted,
// and the global variable ad7991_adc[AD7991_POWER_REF] contains a measurement of the
// Power reflected,

/*! \brief RD16HHF1 PA Bias management
 *
 * \retval Nothing returned
 */
void PA_bias(void)
{
    static uint8_t calibrate = 0;  // Current calibrate value

    switch (cdata.Bias_Select) {
    //-------------------------------------------------------------
    // Set RD16HHF1 Bias to LO setting, using stored calibrated value
    //-------------------------------------------------------------
    case 1:                                               // Set RD16HHF1 PA bias for Class AB
        if (biasInit != 1)                                // Has this been done already?
            ad5301(cdata.AD5301_I2C_addr, cdata.cal_LO);  // No, set bias
        biasInit = 1;
        break;
    //-------------------------------------------------------------
    // Set RD16HHF1 Bias to HI setting,  using stored calibrated value
    //-------------------------------------------------------------
    case 2:             // Set RD16HHF1 PA bias for Class A
        if (SWR_alarm)  // Whoops, we have a SWR situation
        {
            ad5301(cdata.AD5301_I2C_addr, cdata.cal_LO);  // Set lower bias setting
            biasInit = 0;
        } else if (biasInit != 2)  // Has this been done already?
        {
            ad5301(cdata.AD5301_I2C_addr, cdata.cal_HI);  // No, set bias
            biasInit = 2;
        }
        break;
    //-------------------------------------------------------------
    // Calibrate RD16HHF1 Bias
    //-------------------------------------------------------------
    default:                              // Calibrate RD16HHF1 PA bias
        if ((!TMP_alarm) && (!TX_state))  // Proceed if there are no inhibits
        {
            TX_flag = TRUE;                                  // Ask for transmitter to be keyed on
            PA_cal = TRUE;                                   // Indicate PA Calibrate in progress
            ad5301(cdata.AD5301_I2C_addr, 0);                // Set bias to 0 in preparation for step up
        } else if ((!TMP_alarm) && (TX_flag) && (TX_state))  // We have been granted switch over to TX
        {                                                    // Start calibrating
            // Is current larger or equal to setpoint for class AB?
            if ((ad7991_adc[AD7991_PA_CURRENT] / 256 >= cdata.Bias_LO) && (!PA_cal_lo)) {
                PA_cal_lo = TRUE;          // Set flag, were done with class AB
                cdata.cal_LO = calibrate;  // We have bias, store
                flashc_memset8((void*)&nvram_cdata.cal_LO, cdata.cal_LO, sizeof(cdata.cal_LO), TRUE);
            }

            // Is current larger or equal to setpoint for class A?
            if ((ad7991_adc[AD7991_PA_CURRENT] / 256 >= cdata.Bias_HI) && (!PA_cal_hi)) {
                PA_cal_hi = TRUE;          // Set flag, we're done with class A
                cdata.cal_HI = calibrate;  // We have bias, store
                flashc_memset8((void*)&nvram_cdata.cal_HI, cdata.cal_HI, sizeof(cdata.cal_HI), TRUE);
            }

            // Have we reached the end of our rope?
            if (calibrate == 0xff) {
                PA_cal_hi = TRUE;  // Set flag as if done with class AB
                cdata.cal_HI = 0;
                cdata.cal_LO = 0;  // We have no valid bias setting
                // store 0 for both Class A and Class AB
                flashc_memset8((void*)&nvram_cdata.cal_LO, cdata.cal_LO, sizeof(cdata.cal_LO), TRUE);
                flashc_memset8((void*)&nvram_cdata.cal_HI, cdata.cal_HI, sizeof(cdata.cal_HI), TRUE);
            }

            // Are we finished?
            if (PA_cal_hi)  // We're done, Clear all flags
            {
                PA_cal_hi = FALSE;
                PA_cal_lo = FALSE;
                PA_cal = FALSE;
                TX_flag = FALSE;  // Ask for transmitter to be keyed down

                calibrate = 0;          // Clear calibrate value (for next round)
                cdata.Bias_Select = 2;  // Set bias select for class A and store
                flashc_memset8((void*)&nvram_cdata.Bias_Select, cdata.Bias_Select, sizeof(cdata.Bias_Select), TRUE);
                // implicit, no need:
                // ad5301(cdata.AD5301_I2C_addr, cdata.cal_HI);// Set bias	at class A value
            }

            // Crank up the bias
            else {
                calibrate++;                               // Crank up the bias by one notch
                ad5301(cdata.AD5301_I2C_addr, calibrate);  // for the next round of measurements
            }
        }
    }
}

static void mobo_ctrl_factory_reset_handler(void)
{
    // Force an EEPROM update in the mobo config
    flashc_memset8((void*)&nvram_cdata.EEPROM_init_check, 0xFF, sizeof(uint8_t), TRUE);
}
#define LOGGING 1
/*! \brief Initialize and run Mobo functions, including Si570 frequency control, filters and so on
 *
 * \retval none
 */
static void vtaskMoboCtrl(void* pcParameters)
{
    (void)pcParameters;
    uint32_t time = 0;  // Time management


    widget_initialization_start();
    widget_factory_reset_handler_register(mobo_ctrl_factory_reset_handler);

    //----------------------------------------------------
    // Initialize all Mobo Functions *********************
    //----------------------------------------------------

    // Enforce "Factory default settings" when a mismatch is detected between the
    // COLDSTART_REF defined serial number and the matching number in the NVRAM storage.
    // This can be the result of either a fresh firmware upload, or cmd 0x41 with data 0xff
    if (nvram_cdata.EEPROM_init_check != cdata.EEPROM_init_check) {
        widget_startup_log_line("reset mobo nvram");
        flashc_memcpy((void*)&nvram_cdata, &cdata, sizeof(cdata), TRUE);
    } else {
        memcpy(&cdata, &nvram_cdata, sizeof(nvram_cdata));
    }

    // Enable Pin Pullups for Input Pins
    gpio_enable_pin_pull_up(GPIO_CW_KEY_1);
    gpio_enable_pin_pull_up(GPIO_CW_KEY_2);

    // Initialize Real Time Counter
    // rtc_init(&AVR32_RTC, RTC_OSC_RC, 0);	// RC clock at 115kHz
    // rtc_disable_interrupt(&AVR32_RTC);
    // rtc_set_top_value(&AVR32_RTC, RTC_COUNTER_MAX);	// Counter reset once per 10 seconds
    // rtc_enable(&AVR32_RTC);


    //Todo! may want a better name for function, function has changed
    features_display_all();

#if !LOGGING
#endif

    // Create I2C comms semaphore
    mutexI2C = xSemaphoreCreateMutex();

    // Initialize I2C communications

#if !LOGGING
#endif


    // Fetch last frequency stored
    cdata.Freq[0] = cdata.Freq[cdata.SwitchFreq];
    // Initialize Startup frequency
    freq_from_usb = cdata.Freq[0];
    // Indicate new frequency for Si570
    FRQ_fromusb = TRUE;

    // Initialise Rotary Encoder Function
    encoder_init();

    // Force an initial reading of AD7991 etc

    widget_initialization_finish();

    //----------------------------------------------------
    // Mobo Functions Loop *******************************
    //----------------------------------------------------

    // Prog button poll stuff BSB 20110903
    gpio_enable_pin_glitch_filter(PRG_BUTTON);

    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, configTSK_MoboCtrl_PERIOD);  // This is the delay method used in other tasks.

        // Prog button poll stuff BSB 20110903

        //-------------------------------------------
        // Routines accessed every 0.5s on 115kHz timer
        //-------------------------------------------
        static uint8_t btn_poll_temp = 0;
        static uint32_t btn_poll_lastIteration = 0, btn_poll_Timerval;  // Counters to keep track of time

        /*	// Only needed with working flash writes, see below
                static S16 spk_vol_usb_L_local = VOL_INVALID;
                static S16 spk_vol_usb_R_local = VOL_INVALID;
*/

        // Poll Real Time Clock, used for 0.5s, 100ms and 10s timing below
        time = rtc_get_value(&AVR32_RTC);
        btn_poll_Timerval = time / 57000;  // RTC on 115kHz, divide by 57000 for about 0.5s poll time

        if (btn_poll_Timerval != btn_poll_lastIteration) {  // Once every 0.5 second, do stuff
            btn_poll_lastIteration = btn_poll_Timerval;     // Make ready for next iteration

            /*		// FIX: Flash writes creates ticks. Much faster (or interruptable code) is needed!
                // Has volume setting changed recently? If so store it to flash
                if (spk_vol_usb_L_local == VOL_INVALID) {
                        spk_vol_usb_L_local = spk_vol_usb_L;		// 1st time, establish history
                }
                if (spk_vol_usb_R_local == VOL_INVALID) {
                        spk_vol_usb_R_local = spk_vol_usb_R;		// 1st time, establish history
                }
                else if (spk_vol_usb_L_local != spk_vol_usb_L) {
                        spk_vol_usb_L_local = spk_vol_usb_L;
                usb_volume_flash(CH_LEFT, spk_vol_usb_L, VOL_WRITE);
                }
                else if (spk_vol_usb_R_local != spk_vol_usb_R) {	// Not both in a row! These suckers seem to take TIME away from scheduler!
                        spk_vol_usb_R_local = spk_vol_usb_R;
                usb_volume_flash(CH_RIGHT, spk_vol_usb_R, VOL_WRITE);
                }
*/

            if (gpio_get_pin_value(PRG_BUTTON) == 0) {
            }

            if ((gpio_get_pin_value(PRG_BUTTON) == 0) && (btn_poll_temp != 100)) {         // If Prog button pressed and not yet handled..
                                                                                           // At first detection of Prog pin change AB-1.x / USB DAC 128 mkI/II front LEDs for contrast:
                                                                                           // RED->GREEN / GREEN->RED depending on LED_AB_FRONT
                                                                                           // Historical note: Here used to be a pink definition and a bunch of defines. Removed 20150403
                if (feature_get_nvram(feature_image_index) == feature_image_uac1_audio) {  // With UAC1:
                    mobo_led(FLED_RED);
                } else {  // With UAC != 1
                    mobo_led(FLED_GREEN);
                }


                if (btn_poll_temp > 2)  // If button pressed during at least 2 consecutive 2Hz polls...
                {
                    // Perform feature swap between UAC1 audio and UAC2 audio
                    if (feature_get_nvram(feature_image_index) == feature_image_uac1_audio) {
                        feature_set_nvram(feature_image_index, feature_image_uac2_audio);
                        if (feature_get_nvram(feature_image_index) == feature_image_uac2_audio)
                            btn_poll_temp = 100;  // Ready reset after change and Prog release
                    } else if (feature_get_nvram(feature_image_index) == feature_image_uac2_audio) {
                        feature_set_nvram(feature_image_index, feature_image_uac1_audio);
                        if (feature_get_nvram(feature_image_index) == feature_image_uac1_audio)
                            btn_poll_temp = 100;  // Ready reset after change and Prog release
                    }

                    if (btn_poll_temp == 100) {
                        mobo_led(FLED_DARK);
                    }
                } else
                    btn_poll_temp++;
            }                                                                         // if ( (gpio_get_pin_value(PRG_BUTTON) == 0) && (btn_poll_temp != 100) ) 	// If Prog button pressed and not yet handled..
            else if ((gpio_get_pin_value(PRG_BUTTON) != 0) && (btn_poll_temp > 0)) {  // If Prog button released..
                                                                                      //    			if (btn_poll_temp == 100)		// Only reset after Prog button is released and successfull nvram change.
                                                                                      //					widget_reset();		 		// If Prog were still pressed, device would go to bootloader
                                                                                      // Doesn't seem to reset Audio Widget.....

                if (btn_poll_temp != 100)  // Prog released without nvram change -> default front LED color
                {                          // Keep front LEDs dark after nvram change

                    if (feature_get_nvram(feature_image_index) == feature_image_uac1_audio) {  // With UAC1:
                        mobo_led(FLED_GREEN);
                    } else {  // With UAC != 1
                        mobo_led(FLED_RED);
                    }
                }
                btn_poll_temp = 0;
            }

            // Is the task switcher running???

        }  // if (btn_poll_Timerval != btn_poll_lastIteration)	// Once every 1second, do stuff

        // End Prog button poll stuff BSB 20110903

        //-----------------------------
        // Routines accessed every 10ms Now probably every 12ms due to vTaskDelay(120) below...
        //-----------------------------

        // The below is only applicable if I2C bus is available

        LED_Toggle(LED2);  // FIX: Needed???


        //        vTaskDelay(120);						// Changed from 100 to 120 to match device_mouse_hid_task and wm8805_poll()
    }
}

/*! \brief RTOS initialisation of the Mobo task
 *
 * \retval none
 */
void vStartTaskMoboCtrl(void)
{
    xTaskCreate(vtaskMoboCtrl,
        configTSK_MoboCtrl_NAME,
        configTSK_MoboCtrl_STACK_SIZE,
        NULL,
        configTSK_MoboCtrl_PRIORITY,
        NULL);
}
