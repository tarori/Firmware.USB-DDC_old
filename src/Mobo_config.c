/* -*- mode: c++; tab-width: 4; c-basic-offset: 4 -*- */
/*
 * Mobo_config.c
 *
 *  Created on: 2010-06-13
 *      Author: Loftur Jonasson, TF3LJ
 */

#include "Mobo_config.h"
#include "features.h"

// To compile sample rate detector we need low-level hardware access
#include "compiler.h"
#include "gpio.h"
#include <avr32/io.h>

// Power module and clock control
#include "pm.h"

// Real-time counter management
#include "rtc.h"

// To access global input source variable
#include "device_audio_task.h"

// To access DAC_BUFFER_SIZE and clear audio buffer
#include "taskAK5394A.h"
#include "usb_specific_request.h"

/*
#include "AD5301.h"
#include "AD7991.h"
#include "PCF8574.h"
#include "Si570.h"
#include "TMP100.h"
#include "rotary_encoder.h"
*/

// Low-power sleep for a number of milliseconds by means of RTC
// Use only during init, before any MCU hardware (including application use of RTC) is enabled.

void mobo_rtc_waken(volatile avr32_rtc_t* rtc, uint8_t enable)
{
    while (rtc_is_busy(rtc))
        ;
    if (enable)
        rtc->ctrl |= AVR32_RTC_WAKE_EN_MASK;  // Set waken
    else
        rtc->ctrl &= ~AVR32_RTC_WAKE_EN_MASK;  // Clear waken
}

void mobo_sleep_rtc_ms(uint16_t time_ms)
{
    mobo_rtc_waken(&AVR32_RTC, 0);                                         // Clear waken before sleeping
    rtc_init(&AVR32_RTC, RTC_OSC_RC, 0);                                   // RC clock at 115kHz, clear RTC, prescaler n=0 encoding freq/=2^(n+1)
    rtc_disable_interrupt(&AVR32_RTC);                                     // For good measure
    rtc_set_top_value(&AVR32_RTC, RTC_COUNTER_FREQ / 1000 / 2 * time_ms);  // Counter reset after time_ms ms, accounting for prescaler n=0 encoding freq/=2^(n+1)
    mobo_rtc_waken(&AVR32_RTC, 1);                                         // Set waken before sleeping
    rtc_enable(&AVR32_RTC);
    SLEEP(AVR32_PM_SMODE_DEEP_STOP);  // Disable all but RC clock
}

void mobo_led(uint8_t fled)
{
    gpio_enable_pin_pull_up(AVR32_PIN_PA04);  // Floating: Active high. GND: Active low

    if (gpio_get_pin_value(AVR32_PIN_PA04) == 1) {  // Active high
        if (fled == FLED_DARK) {
            gpio_clr_gpio_pin(AVR32_PIN_PX33);  // Clear RED light on external AB-1.1 LED
            gpio_clr_gpio_pin(AVR32_PIN_PX29);  // Clear GREEN light on external AB-1.1 LED
        } else if (fled == FLED_RED) {
            gpio_set_gpio_pin(AVR32_PIN_PX33);  // Set RED light on external AB-1.1 LED
            gpio_clr_gpio_pin(AVR32_PIN_PX29);  // Clear GREEN light on external AB-1.1 LED
        } else if (fled == FLED_GREEN) {
            gpio_clr_gpio_pin(AVR32_PIN_PX33);  // Clear RED light on external AB-1.1 LED
            gpio_set_gpio_pin(AVR32_PIN_PX29);  // Set GREEN light on external AB-1.1 LED
        } else if (fled == FLED_YELLOW) {
            gpio_set_gpio_pin(AVR32_PIN_PX33);  // Set RED light on external AB-1.1 LED
            gpio_set_gpio_pin(AVR32_PIN_PX29);  // Set GREEN light on external AB-1.1 LED
        }
    } else {  // Active low
        if (fled == FLED_DARK) {
            gpio_set_gpio_pin(AVR32_PIN_PX33);  // Clear RED light on external AB-1.1 LED
            gpio_set_gpio_pin(AVR32_PIN_PX29);  // Clear GREEN light on external AB-1.1 LED
        } else if (fled == FLED_RED) {
            gpio_clr_gpio_pin(AVR32_PIN_PX33);  // Set RED light on external AB-1.1 LED
            gpio_set_gpio_pin(AVR32_PIN_PX29);  // Clear GREEN light on external AB-1.1 LED
        } else if (fled == FLED_GREEN) {
            gpio_set_gpio_pin(AVR32_PIN_PX33);  // Clear RED light on external AB-1.1 LED
            gpio_clr_gpio_pin(AVR32_PIN_PX29);  // Set GREEN light on external AB-1.1 LED
        } else if (fled == FLED_YELLOW) {
            gpio_clr_gpio_pin(AVR32_PIN_PX33);  // Set RED light on external AB-1.1 LED
            gpio_clr_gpio_pin(AVR32_PIN_PX29);  // Set GREEN light on external AB-1.1 LED
        }
    }
    gpio_disable_pin_pull_up(AVR32_PIN_PA04);  // Floating: Active high. GND: Active low
}


/*! \brief Audio Widget select oscillator
 *
 * \retval none
 */
void mobo_xo_select(U32 frequency, uint8_t source)
{
    (void)source;
    // XO control and SPI muxing on ab1x hardware generation
    static U32 prev_frequency = FREQ_INVALID;

    if ((frequency != prev_frequency) || (prev_frequency == FREQ_INVALID)) {  // Only run at startup or when things change
        switch (frequency) {
        case FREQ_44:
            if (FEATURE_BOARD_USBI2S)
                gpio_clr_gpio_pin(AVR32_PIN_PX16);  // BSB 20110301 MUX in 22.5792MHz/2 for AB-1
            else if (FEATURE_BOARD_USBDAC)
                gpio_clr_gpio_pin(AVR32_PIN_PX51);
            gpio_clr_gpio_pin(SAMPLEFREQ_VAL0);
            gpio_clr_gpio_pin(SAMPLEFREQ_VAL1);
            break;
        case FREQ_48:
            if (FEATURE_BOARD_USBI2S)
                gpio_set_gpio_pin(AVR32_PIN_PX16);  // BSB 20110301 MUX in 24.576MHz/2 for AB-1
            else if (FEATURE_BOARD_USBDAC)
                gpio_set_gpio_pin(AVR32_PIN_PX51);
            gpio_clr_gpio_pin(SAMPLEFREQ_VAL0);
            gpio_clr_gpio_pin(SAMPLEFREQ_VAL1);
            break;
        case FREQ_88:
            if (FEATURE_BOARD_USBI2S)
                gpio_clr_gpio_pin(AVR32_PIN_PX16);  // BSB 20110301 MUX in 22.5792MHz/2 for AB-1
            else if (FEATURE_BOARD_USBDAC)
                gpio_clr_gpio_pin(AVR32_PIN_PX51);
            gpio_clr_gpio_pin(SAMPLEFREQ_VAL1);
            gpio_set_gpio_pin(SAMPLEFREQ_VAL0);
            break;
        case FREQ_96:
            if (FEATURE_BOARD_USBI2S)
                gpio_set_gpio_pin(AVR32_PIN_PX16);  // BSB 20110301 MUX in 24.576MHz/2 for AB-1
            else if (FEATURE_BOARD_USBDAC)
                gpio_set_gpio_pin(AVR32_PIN_PX51);
            gpio_clr_gpio_pin(SAMPLEFREQ_VAL1);
            gpio_set_gpio_pin(SAMPLEFREQ_VAL0);
            break;
        case FREQ_176:
            if (FEATURE_BOARD_USBI2S)
                gpio_clr_gpio_pin(AVR32_PIN_PX16);  // BSB 20110301 MUX in 22.5792MHz/2 for AB-1
            else if (FEATURE_BOARD_USBDAC)
                gpio_clr_gpio_pin(AVR32_PIN_PX51);
            gpio_clr_gpio_pin(SAMPLEFREQ_VAL0);
            gpio_set_gpio_pin(SAMPLEFREQ_VAL1);
            break;
        case FREQ_192:
            if (FEATURE_BOARD_USBI2S)
                gpio_set_gpio_pin(AVR32_PIN_PX16);  // BSB 20110301 MUX in 24.576MHz/2 for AB-1
            else if (FEATURE_BOARD_USBDAC)
                gpio_set_gpio_pin(AVR32_PIN_PX51);
            gpio_clr_gpio_pin(SAMPLEFREQ_VAL0);
            gpio_set_gpio_pin(SAMPLEFREQ_VAL1);
            break;
        default:  // same as 44.1
            if (FEATURE_BOARD_USBI2S)
                gpio_clr_gpio_pin(AVR32_PIN_PX16);  // BSB 20110301 MUX in 22.5792MHz/2 for AB-1
            else if (FEATURE_BOARD_USBDAC)
                gpio_clr_gpio_pin(AVR32_PIN_PX51);
            gpio_clr_gpio_pin(SAMPLEFREQ_VAL0);
            gpio_clr_gpio_pin(SAMPLEFREQ_VAL1);
            break;
        }  // switch

        // XO control and I2S muxing on Digital Input 1.0 / 2.0 generation
        // NB: updated to support SPDIF buffering in MCU. That is highly experimental code!

        prev_frequency = frequency;
    }  // if (frequency != prev_frequency)
}

// Master clock to DAC's I2S port frequency setup
void mobo_clock_division(U32 frequency)
{

    static U32 prev_frequency = FREQ_INVALID;

    if ((frequency != prev_frequency) || (prev_frequency == FREQ_INVALID)) {  // Only run at startup or when things change
        gpio_enable_pin_pull_up(AVR32_PIN_PA03);                              // Floating: stock AW with external /2. GND: modded AW with no ext. /2

        pm_gc_disable(&AVR32_PM, AVR32_PM_GCLK_GCLK1);

        // External /2 variety, unmodded hardware with floating, pulled-up PA03 interpreted as 1
        if (gpio_get_pin_value(AVR32_PIN_PA03) == 1) {
            switch (frequency) {
            case FREQ_192:
                pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK1,  // gc
                    0,                                       // osc_or_pll: use Osc (if 0) or PLL (if 1)
                    1,                                       // pll_osc: select Osc0/PLL0 or Osc1/PLL1
                    0,                                       // diven - disabled
                    0);                                      // not divided
                break;
            case FREQ_176:
                pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK1,  // gc
                    0,                                       // osc_or_pll: use Osc (if 0) or PLL (if 1)
                    1,                                       // pll_osc: select Osc0/PLL0 or Osc1/PLL1
                    0,                                       // diven - disabled
                    0);                                      // not divided
                break;
            case FREQ_96:
                pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK1,  // gc
                    0,                                       // osc_or_pll: use Osc (if 0) or PLL (if 1)
                    1,                                       // pll_osc: select Osc0/PLL0 or Osc1/PLL1
                    1,                                       // diven - enabled
                    0);                                      // divided by 2
                break;
            case FREQ_88:
                pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK1,  // gc
                    0,                                       // osc_or_pll: use Osc (if 0) or PLL (if 1)
                    1,                                       // pll_osc: select Osc0/PLL0 or Osc1/PLL1
                    1,                                       // diven - enabled
                    0);                                      // divided by 2
                break;
            case FREQ_48:
                pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK1,  // gc
                    0,                                       // osc_or_pll: use Osc (if 0) or PLL (if 1)
                    1,                                       // pll_osc: select Osc0/PLL0 or Osc1/PLL1
                    1,                                       // diven - enabled
                    1);                                      // divided by 4
                break;
            case FREQ_44:
                pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK1,  // gc
                    0,                                       // osc_or_pll: use Osc (if 0) or PLL (if 1)
                    1,                                       // pll_osc: select Osc0/PLL0 or Osc1/PLL1
                    1,                                       // diven - enabled
                    1);                                      // divided by 4
            default:                                         // Treated as 44.1
                pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK1,  // gc
                    0,                                       // osc_or_pll: use Osc (if 0) or PLL (if 1)
                    1,                                       // pll_osc: select Osc0/PLL0 or Osc1/PLL1
                    1,                                       // diven - enabled
                    1);                                      // divided by 4
                break;
            }
        }

        // No external /2 variety, modded hardware with resistor tying PA03 to 0
        else {
            switch (frequency) {
            case FREQ_192:
                pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK1,  // gc
                    0,                                       // osc_or_pll: use Osc (if 0) or PLL (if 1)
                    1,                                       // pll_osc: select Osc0/PLL0 or Osc1/PLL1
                    1,                                       // diven - enabled
                    0);                                      // divided by 2
                break;
            case FREQ_176:
                pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK1,  // gc
                    0,                                       // osc_or_pll: use Osc (if 0) or PLL (if 1)
                    1,                                       // pll_osc: select Osc0/PLL0 or Osc1/PLL1
                    1,                                       // diven - enabled
                    0);                                      // divided by 2
                break;
            case FREQ_96:
                pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK1,  // gc
                    0,                                       // osc_or_pll: use Osc (if 0) or PLL (if 1)
                    1,                                       // pll_osc: select Osc0/PLL0 or Osc1/PLL1
                    1,                                       // diven - enabled
                    1);                                      // divided by 4
                break;
            case FREQ_88:
                pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK1,  // gc
                    0,                                       // osc_or_pll: use Osc (if 0) or PLL (if 1)
                    1,                                       // pll_osc: select Osc0/PLL0 or Osc1/PLL1
                    1,                                       // diven - enabled
                    1);                                      // divided by 4
                break;
            case FREQ_48:
                pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK1,  // gc
                    0,                                       // osc_or_pll: use Osc (if 0) or PLL (if 1)
                    1,                                       // pll_osc: select Osc0/PLL0 or Osc1/PLL1
                    1,                                       // diven - enabled
                    3);                                      // divided by 8
                break;
            case FREQ_44:
                pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK1,  // gc
                    0,                                       // osc_or_pll: use Osc (if 0) or PLL (if 1)
                    1,                                       // pll_osc: select Osc0/PLL0 or Osc1/PLL1
                    1,                                       // diven - enabled
                    3);                                      // divided by 8
            default:                                         // Treated as 44.1
                pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK1,  // gc
                    0,                                       // osc_or_pll: use Osc (if 0) or PLL (if 1)
                    1,                                       // pll_osc: select Osc0/PLL0 or Osc1/PLL1
                    1,                                       // diven - enabled
                    3);                                      // divided by 8
                break;
            }
        }

        gpio_disable_pin_pull_up(AVR32_PIN_PA03);  // Floating: stock AW with external /2. GND: modded AW with no ext. /2

        pm_gc_enable(&AVR32_PM, AVR32_PM_GCLK_GCLK1);

        AK5394A_pdca_tx_enable(frequency);  // LRCK inversion will occur with FREQ_INVALID

        prev_frequency = frequency;
    }
}

// Empty the contents of the incoming pdca buffers
void mobo_clear_adc_channel(void)
{
    int i;

    for (i = 0; i < ADC_BUFFER_SIZE; i++) {
        audio_buffer_0[i] = 0;
        audio_buffer_1[i] = 0;
    }
}

// Empty the contents of the outgoing pdca buffers
void mobo_clear_dac_channel(void)
{
    int i;

    for (i = 0; i < DAC_BUFFER_SIZE; i++) {
        spk_buffer_0[i] = 0;
        spk_buffer_1[i] = 0;
    }
}

//
//-----------------------------------------------------------------------------
// The below structure contains a number of implementation dependent definitions (user tweak stuff)
//-----------------------------------------------------------------------------
//
mobo_data_t cdata  // Variables in ram/flash rom (default)
    = {
        COLDSTART_REF  // Update into eeprom if value mismatch
        ,
        FALSE  // FALSE if UAC1 Audio, TRUE if UAC2 Audio.
        ,
        SI570_I2C_ADDRESS  // Si570 I2C address or Si570_I2C_addr
        ,
        TMP100_I2C_ADDRESS  // I2C address for the TMP100 chip
        ,
        AD5301_I2C_ADDRESS  // I2C address for the AD5301 DAC chip
        ,
        AD7991_I2C_ADDRESS  // I2C address for the AD7991 4 x ADC chip
        ,
        PCF_MOBO_I2C_ADDR  // I2C address for the onboard PCF8574
        ,
        PCF_LPF1_I2C_ADDR  // I2C address for the first MegaFilterMobo PCF8574
        ,
        PCF_LPF2_I2C_ADDR  // I2C address for the second MegaFilterMobo PCF8574
        ,
        PCF_EXT_I2C_ADDR  // I2C address for an external PCF8574 used for FAN, attenuators etc
        ,
        HI_TMP_TRIGGER  // If PA temperature goes above this point, then
                        // disable transmission
        ,
        P_MIN_TRIGGER  // Min P out measurement for SWR trigger
        ,
        SWR_PROTECT_TIMER  // Timer loop value (in 10ms increments)
        ,
        SWR_TRIGGER  // Max SWR threshold (10 x SWR)
        ,
        PWR_CALIBRATE  // Power meter calibration value
        ,
        BIAS_SELECT  // Which bias, 0 = Cal, 1 = LO, 2 = HI
        ,
        BIAS_LO  // PA Bias in 10 * mA, typically  20mA or Class B
        ,
        BIAS_HI  // PA Bias in 10 * mA, typically 350mA or Class A
        ,
        CAL_LO  // PA Bias setting, Class LO
        ,
        CAL_HI  // PA Bias setting, Class HI
        ,
        DEVICE_XTAL  // FreqXtal
        ,
        3500  // SmoothTunePPM
        ,
        {(7.050000 * _2(23))  // Freq at startup, Default
            ,
            (1.820000 * _2(23))  // Freq at startup, Memory 1
            ,
            (3.520000 * _2(23))  // Freq at startup, Memory 2
            ,
            (7.020000 * _2(23))  // Freq at startup, Memory 3
            ,
            (10.120000 * _2(23))  // Freq at startup, Memory 4
            ,
            (14.020000 * _2(23))  // Freq at startup, Memory 5
            ,
            (18.090000 * _2(23))  // Freq at startup, Memory 6
            ,
            (21.020000 * _2(23))  // Freq at startup, Memory 7
            ,
            (24.910000 * _2(23))  // Freq at startup, Memory 8
            ,
            (28.020000 * _2(23))}  // Freq at startup, Memory 9
        ,
        3  // Which memory was last in use
        ,
        {(2.0 * 4.0 * _2(5))  // Default filter cross over
            ,
            (4.0 * 4.0 * _2(5))  // frequencies for Mobo V4.3
            ,
            (8.0 * 4.0 * _2(5))  // BPF. eight value array.
            ,
            (11.0 * 4.0 * _2(5)), (14.5 * 4.0 * _2(5)), (22.0 * 4.0 * _2(5)), (25.0 * 4.0 * _2(5)), (TRUE)}
        //, ( {  2.0 * 4.0 * _2(5) )	// Default filter crossover
        //,	(  4.0 * 4.0 * _2(5) )	// frequencies for the K5OOR
        //,	(  7.5 * 4.0 * _2(5) )	// HF Superpacker Pro LPF bank
        //,	( 14.5 * 4.0 * _2(5) )	// Six values in an eight value array.
        //,	( 21.5 * 4.0 * _2(5) )
        //,	( 30.0 * 4.0 * _2(5) )	// The highest two values parked above 30 MHz
        //,	( 30.0 * 4.0 * _2(5) )
        //,	( True ) }
        ,
        {(2.0 * 4.0 * _2(5))  // Default filter crossover
            ,
            (4.0 * 4.0 * _2(5))  // frequencies as per Alex email
            ,
            (8.0 * 4.0 * _2(5))  // 2009-08-15
            ,
            (11.0 * 4.0 * _2(5)), (14.5 * 4.0 * _2(5)), (18.2 * 4.0 * _2(5)), (21.0 * 4.0 * _2(5)), (30.0 * 4.0 * _2(5)), (31.0 * 4.0 * _2(5)), (32.0 * 4.0 * _2(5)), (33.0 * 4.0 * _2(5)), (34.0 * 4.0 * _2(5)), (35.0 * 4.0 * _2(5)), (36.0 * 4.0 * _2(5)), (37.0 * 4.0 * _2(5)), (TRUE)},
        PWR_FULL_SCALE  // Full Scale setting for Power Output Bargraph, in W
        ,
        SWR_FULL_SCALE  // Full Scale setting for SWR Bargraph,
                        // (Max SWR = Value + 1, or 4 = SWR of 5.0)
        ,
        PEP_PERIOD  // Number of samples in PEP measurement
        ,
        ENC_PULSES  // Number of Resolvable States per Revolution
        ,
        1  // VFO Resolution 1/2/5/10/50/100kHz per revolution
        ,
        0  // PSDR-IQ Freq offset value is in +/- kHz
        ,
        45  // Fan On trigger temp in degrees C
        ,
        40  // Fan Off trigger temp in degrees C
        ,
        PCF_EXT_FAN_BIT  // Which bit is used to control the Cooling Fan
};
