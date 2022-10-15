/*
 * PCF8574.h
 *
 *  Created on: 2010-02-27
 *      Author: Loftur Jonasson, TF3LJ
 */

#ifndef _PCF8574_h_
#define _PCF8574_h_

#include <stdint.h>

#include "board.h"

// Defs for onboard PCF8574 chip
//  I2C Addresses for this chip can be:
//  If NXP 8574P,  then: 0x20, 0x21. 0x22, 0x23, 0x24, 0x25, 0x26, 0x27
//  If NXP 8574Ax, then: 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f
//  Normal address is 0x3f
#define PCF_MOBO_I2C_ADDR 0x3f    // I2C address for the onboard PCF8574
#define Mobo_PCF_FILTER (3 << 0)  // First bit of three consecutive for BPF
//#define MoboPCFUndefined1	(1 << 3)		// 8bit BCD or M0RZF style for LPF switching
//#define MoboPCFUndefined2	(1 << 4)		// 8bit BCD or M0RZF style for LPF switching
//#define MoboPCFUndefined3	(1 << 5)		// 8bit BCD or M0RZF style for LPF switching
#define Mobo_PCF_TX2 (1 << 6)  // SWR Protect secondary PTT
#define Mobo_PCF_TX (1 << 7)   // TX PTT, active low


// Defs for two offboard PCF8574A chips in the MegaFilter Mobo
//  I2C Addresses for these chips can be:
//  If NXP 8574P,  then: 0x20, 0x21. 0x22, 0x23, 0x24, 0x25, 0x26, 0x27
//  If NXP 8574Ax, then: 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f
//  Normal addresses are 0x39 and 0x3a

//#define PCF_LPF1_I2C_ADDR	0x20			// I2C address for first Megafilter Mobo PCF8574
#define PCF_LPF1_I2C_ADDR 0x39  // I2C address for first Megafilter Mobo PCF8574
#define PCF_LPF2_I2C_ADDR 0x3a  // I2C address for second Megafilter Mobo PCF8574


// Defs used for a PCF8574 used for control of a cooling FAN, attenuators, etc...
// Normally this will be an external PCF8574, but alternately the on-board PCF (PCF_MOBO_I2C_ADDR)
// can be used to provide the cooling fan control, depending on features selection.
#define PCF_EXT_I2C_ADDR 0x38  // I2C address for an external PCF8574 used for FAN, attenuators etc
#define PCF_EXT_FAN_BIT 0x01   // Bit 0 (of 0-7) used for FAN control (high for ON)
// IF the onboard PCF8574 is used, then use the below bit for FAN control
#define PCF_MOBO_FAN_BIT 0x20  // Bit 5 (of 0-7) used for FAN control (high for ON)

#define PCF_EXT2_I2C_ADDR 0x39  // I2C address for an external PCF8574 used for FAN, attenuators etc
#define PCF_EXT3_I2C_ADDR 0x3a  // I2C address for an external PCF8574 used for FAN, attenuators etc

// pcf_data_out contains the current output data on the builtin PCF8574 on the Mobo
extern volatile uint8_t pcf8574_mobo_data_out;

/*! \brief Set output bits in the PCF8574 built into to the Mobo 4.3
 *
 * \retval I2C status.
 */
extern uint8_t pcf8574_mobo_set(uint8_t i2c_address, uint8_t byte);

/*! \brief Clear output bits in the PCF8574 built into to the Mobo 4.3
 *
 * \retval I2C status.
 */
extern uint8_t pcf8574_mobo_clear(uint8_t i2c_address, uint8_t byte);

/*! \brief Write all 8 bits to a PCF8574
 *
 * \retval I2C status.
 */
extern uint8_t pcf8574_out_byte(uint8_t i2c_address, uint8_t data);

/*! \brief Read all 8 input bits from a PCF8574
 *
 * \retval PCF8574 input register (uint8_t)
 */
extern uint8_t pcf8574_in_byte(uint8_t i2c_address, uint8_t* data_to_return);

#endif
