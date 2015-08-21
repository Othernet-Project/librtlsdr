/*
 * Maxim 2120 tuner driver
 *
 * Copyright (C) 2013 Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2015 Kyle Keen <kyle@outernet.is>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef M2120_H
#define M2120_H

#define M2120_I2C_ADDR		0xC0
#define M2120_XTAL_FREQ		4000000

/* WAIT is mS */
#define M2120_PLL_TRIES		10
#define M2120_PLL_WAIT		5

//#define M2120_DIV4_HZ		1125000000
#define M2120_DIV4_HZ		3000000000

#define M2120_CHECK_ADDR	0x03
#define M2120_CHECK_VAL		0xF6
/* Register 0x03 and 0x04 are unused.
   Assume they are constant between MAX2120 chips.
   0x04:0x84 is the other pair.
*/

#define M2120_IF_FREQ		3570000

#define M2120_NUM_REGS		14
#define NUM_IMR			5
#define IMR_TRIAL		9

#define VER_NUM			49

enum m2120_reg {
	M2120_N_DIV_MSB		= 0x00,
	M2120_N_DIV_LSB		= 0x01,
	M2120_CHARGE_PUMP	= 0x02,
	/* gap */
	M2120_XR_DIV		= 0x05,
	M2120_PLL		= 0x06,
	M2120_VCO		= 0x07,
	M2120_LPF		= 0x08,
	M2120_CONTROL		= 0x09,
	M2120_SDOWN		= 0x0A,
	M2120_TEST		= 0x0B,
	M2120_STATUS1		= 0x0C,
	M2120_STATUS2		= 0x0D,
};

// todo, remove typedef
typedef enum {
	ADC_OUT_OF_LOCK,
	ADC_LOCKED,
	ADC_VAS_LOCKED,
} m2120_lock_status;

/*
    identifying the different chips is going to be hard
    for example, the 2112 has no unused registers at all
*/

enum m21xx_chip {
	CHIP_M2106,
	CHIP_M2112,
	CHIP_M2114,
	CHIP_M2120,
	CHIP_M2121,
};

int m2120_init(void *dev);
int m2120_exit(void *dev);
int m2120_set_freq(void *dev, uint32_t freq);
int m2120_set_if_gain(void *dev, int stage, int gain);

#endif
