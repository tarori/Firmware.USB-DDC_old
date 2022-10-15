/* -*- mode: c; tab-width: 4; c-basic-offset: 4 -*- */
/*
 * freq_and_filters.c
 *
 *  Created on: 2010-06-13
 *      Author: Loftur Jonasson, TF3LJ
 */

#include <stdint.h>
#include <stdio.h>

#include "DG8SAQ_cmd.h"
#include "Mobo_config.h"
#include "PCF8574.h"
#include "Si570.h"
#include "freq_and_filters.h"
#include "rotary_encoder.h"
#include "widget.h"


char frq_lcd[13];  // Pass frequency information to LCD
char flt_lcd[5];   // LCD Print formatting for filters

/*! \brief Display the running frequency on an LCD
 *
 * \retval None
 */
void display_frequency(void)
{
}

/*! \brief Set Band Pass and Low Pass filters
 *
 * \retval None or RX frequency band, depending on #define CALC_BAND_MUL_ADD
 */
void SetFilter(uint32_t freq)
{
    uint8_t selectedFilters[2] = {0, 0};  // Contains info on which filters are selected, for LCD print


    uint8_t i;

    uint8_t data;

    sint32_t Freq;

    Freq.dw = freq;  // Freq.w1 is 11.5bits

    //-------------------------------------------
    // Set RX Band Pass filters
    //-------------------------------------------
    //
    // Set RX BPF using the Mobo PCF8574 (max 8 filters)
    if (i2c.pcfmobo) {
        for (i = 0; i < 7; i++) {
            if (Freq.w0 < cdata.FilterCrossOver[i])
                break;
        }
        // i = i & 0x07;								// We only want 3 bits
        pcf8574_mobo_data_out &= 0xf8;                 // clear and leave upper 5 bits untouched
        pcf8574_mobo_set(cdata.PCF_I2C_Mobo_addr, i);  // Combine the two and write out
        selectedFilters[0] = i;                        // Used for LCD Print indication

    }
    //
    // Set RX BPF, using the Widget PTT_2 and PTT_3 outputs (max 4 filters)
    else {
        for (i = 0; i < 3; i++) {
            if (Freq.w0 < cdata.FilterCrossOver[i])
                break;
        }
        data = i;
        selectedFilters[0] = i;  // Used for LCD Print indication
        // Manipulate 2 bit binary output to set the 4 BPF
        if (data & 0b00000001)
            gpio_set_gpio_pin(PTT_2);
        else
            gpio_clr_gpio_pin(PTT_2);
        if (data & 0b00000010)
            gpio_set_gpio_pin(PTT_3);
        else
            gpio_clr_gpio_pin(PTT_3);
    }
}

/*! \brief Si570 Set frequency (as a 32bit value) and filters, frequency [MHz] * 2^21
 *
 * \retval TWI status
 */
uint8_t new_freq_and_filters(uint32_t freq)
{
    uint8_t status = 0;    // Is the Si570 On Line?
    double set_frequency;  // Frequency in double precision floating point

    // Translate frequency to a double precision float
    set_frequency = (double)freq / _2(21);

    // PSDR-IQ writes 0.000 MHz in certain instances
    // Enforce a sane lower frequency boundary, at 3.45 MHz, verified as lowest frequency
    // at which the Si570 will give output.
    if (set_frequency < 3.45)
        return 0;

    cdata.Freq[0] = freq;  // Some Command calls to this func do not update si570.Freq[0]

    if (TX_flag)  // Oops, we are transmitting... return without changing frequency
        return TWI_INVALID_ARGUMENT;


    status = SetFrequency(set_frequency);


    return status;
}

/*! \brief Set Si570 Frequency + Manage BPF and LPF
 *
 * \retval none.
 */
void freq_and_filter_control(void)
{
    uint32_t frq_from_encoder;

    if (i2c.si570) {
        // Check for a frequency change request from USB or Encoder
        // USB always takes precedence
        if (FRQ_fromusbreg == TRUE) {
            freq_from_usb = Freq_From_Register((double)cdata.FreqXtal / _2(24)) * _2(21);
            new_freq_and_filters(freq_from_usb);  // Write usb frequency to Si570
            FRQ_fromusbreg = FALSE;               // Clear input flags
            FRQ_fromusb = FALSE;                  // Clear input flags
            FRQ_fromenc = FALSE;                  // USB takes precedence
            FRQ_lcdupdate = TRUE;                 // Update LCD
        } else if (FRQ_fromusb == TRUE) {
            new_freq_and_filters(freq_from_usb);  // Write usb frequency to Si570
            FRQ_fromusbreg = FALSE;               // Clear input flags
            FRQ_fromusb = FALSE;                  // Clear input flags
            FRQ_fromenc = FALSE;                  // USB takes precedence
            FRQ_lcdupdate = TRUE;                 // Update LCD
        }
        // This is ignored while in Menu Mode, then Menu uses the Encoder.
        else if (FRQ_fromenc == TRUE) {
            if (!MENU_mode) {
                // Add the accumulated but yet unenacted frequency delta from the
                // encoder to the current Si570 frequency
                frq_from_encoder = cdata.Freq[0] + freq_delta_from_enc;
                freq_delta_from_enc = 0;                 // Zero the accumulator
                new_freq_and_filters(frq_from_encoder);  // Write new freq to Si570
                FRQ_fromenc = FALSE;                     // Clear input flag
                FRQ_lcdupdate = TRUE;                    // Update LCD
            } else
                freq_delta_from_enc = 0;  // Zero any changes while Menu Control
        }

        // Check if a simple LCD update of Frequency and filters display is required
        if ((FRQ_lcdupdate == TRUE) && (!MENU_mode)) {
            display_frequency();
            SetFilter(cdata.Freq[0]);  // Select Band Pass Filter, according to the frequency selected
            FRQ_lcdupdate = FALSE;
        }
    }
}
