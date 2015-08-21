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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
// for usleep

#include "rtlsdr_i2c.h"
#include "tuner_max2120.h"

#define MHZ(x)		((x)*1000*1000)
#define KHZ(x)		((x)*1000)

/*
 * Static constants
 */

static const m2120_lock_status m2120_lock_table[] = {
	ADC_OUT_OF_LOCK, ADC_LOCKED, ADC_VAS_LOCKED,
	ADC_VAS_LOCKED, ADC_LOCKED, ADC_OUT_OF_LOCK};

static const int m2120_init_regs[][2] = {
	/* {register, value} */
	{M2120_CHARGE_PUMP, 0x00},
	{M2120_XR_DIV, 0x1F},
	{M2120_PLL, 0xC0},
	/* M2120_VCO defaults okay? */
	{M2120_LPF, 0x00},
	{M2120_CONTROL, 0x00},
	{M2120_SDOWN, 0x00},
	{M2120_TEST, 0x08}, 
	{-1, -1},  /* sentinel */
};

static int m2120_reg_write(void *dev, uint8_t reg, uint8_t val)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = val;

	if (rtlsdr_i2c_write_fn(dev, M2120_I2C_ADDR, data, 2) < 0)
		{return -1;}
	return 0;
}

/* returns positive 8bit register contents on success, negative on error */
static int m2120_reg_read(void *dev, uint8_t reg)
{
        uint8_t data = reg;

        if (rtlsdr_i2c_write_fn(dev, M2120_I2C_ADDR, &data, 1) < 1)
                {return -1;}

        if (rtlsdr_i2c_read_fn(dev, M2120_I2C_ADDR, &data, 1) < 1)
                {return -1;}

        return data;
}

int m2120_if_low_pass_bw(uint8_t lpf_reg)
{
	return 4000000 + (((int)lpf_reg) - 12) * 290000;
}

int _m2120_init(void *dev)
{
	int i, r, reg, val;
	for (i=0; m2120_init_regs[i][0] >= 0; i++) {
		reg = m2120_init_regs[i][0];
		val = m2120_init_regs[i][1];
		r = m2120_reg_write(dev, reg, val);
		if (r >= 0)
			{continue;}
		/* error condition */
		fprintf(stderr, "[MAX2120] Failed to write register 0x%02X\n", reg);
		fprintf(stderr, "%s: failed=%d\n", __FUNCTION__, r);
		return r;
	}
	return 0;
}

int m2120_exit(void *dev)
{
	return m2120_reg_write(dev, M2120_CONTROL, 0x80);
}

/* stage is ignored, gain is 10x value */
int m2120_set_if_gain(void *dev, int stage, int gain)
{
	gain = gain / 10;
	if (gain < 0 || gain > 15)
		{return -1;}
	return m2120_reg_write(dev, M2120_CONTROL, gain & 0x0F);
}

int m2120_set_ndiv(void *dev, int n)
{
	/* 16 <= n <= 2175   but ignore for extended tuning */
	uint8_t msb, lsb;
	int r;
	if (n < 0)
		{return -1;}
	if (n > 0x7FFF)
		{return -1;}
	msb = (uint8_t)((n >> 8) & 0x7F);
	lsb = (uint8_t)(n & 0xFF);
	r = m2120_reg_write(dev, M2120_N_DIV_MSB, msb);
	r = m2120_reg_write(dev, M2120_N_DIV_LSB, lsb) | r;
	return r;
}

int m2120_pll_wait(void *dev)
{
	int i, r;
	int status = -1;
	for (i=M2120_PLL_TRIES; i; i--)
	{
		usleep(M2120_PLL_WAIT * 1000);
		r = m2120_reg_read(dev, M2120_STATUS1);
		if (r < 0)
			{status = -1; continue;}
		r &= 0x50;
		if (r != 0x50)
			{status = -2; continue;}
		// check STATUS2 for VAS?
		return 0;
	}
	return status;
}

int m2120_manual_vco(void *dev, int vco)
{
	int r;
	uint8_t reg;
	vco &= 0x1F;
	if (vco > 23)
		{vco = 23;}
	reg = (vco << 3) | 0x03;  /* latch ADC, enable ADC read */
	r = m2120_reg_write(dev, M2120_VCO, (uint8_t)vco);
	// check vco status?
	return r;
}

/*
tuner notes
xtal is 4-8MHz
tuning is 925M - 2175M
ref divider is unused
freq synth -> div2/div4 -> mixer
rf divider N     16 ... 2175
detector freq    1MHz ... 2MHz
xtal divider     1 ... 8  (assume datasheet has typo)
div2 for freq >= 1125MHz
div4 for freq <  1125MHz
24 VCOs ?  might be able to do finer tuning
    nope, this doesn't seem to be feasible
types of failure: I2C fail, out of range, VCO fail

25mS: 816M - 1053M
50mS: 704M - 1192M 

xtal_div 4: 176M - 298M

ultimately this will have to switch to the '2112
unless I am very wrong about the tuning granularity of the '2120
*/

int m2120_set_freq(void *dev, uint32_t tunefreq)
{

	uint32_t xtalfreq = M2120_XTAL_FREQ;
	double real_freq = 0;
	double target_ratio = 0;
	int n = 0;
	uint8_t regval = 0;
	int xtal_div = 4;
	int div24 = 2;
	int r;

	/* set PLL register: div2, VAS enabled, 1200uA */
	regval = 0x60;
	if (M2120_DIV4_HZ <= tunefreq)  /* div4 */
	{
		div24 = 4;
		regval |= 0x80;
	}
	// seems to better with div4 disabled?
	r = m2120_reg_write(dev, M2120_PLL, regval);
	if (r<0)
	    {fprintf(stderr, "MAX2120 write fail: pll");}

	target_ratio = (double)tunefreq / (double)xtalfreq;
	target_ratio *= div24;
	target_ratio *= xtal_div;
	n = (int)round(target_ratio);
	fprintf(stderr, "MAX2120 n:%i  div24:%i  xtal_div:%i\n", n, div24, xtal_div);

	// calculate the exact output frequency
	real_freq = (double)n * (double)xtalfreq / (double)(div24 * xtal_div);
	fprintf(stderr, "MAX2120 requested %uHz, real %fHz\n", tunefreq, real_freq);

	// check limits?
	r = m2120_set_ndiv(dev, n);
	if (r<0)
	    {fprintf(stderr, "MAX2120 write fail: ndiv");}

	// set xtal divider and reference divider
	regval = ((xtal_div-1) % 8) << 5;  /* xtal */
	regval |= 0x01;  /* reference = 1 */
	r = m2120_reg_write(dev, M2120_XR_DIV, regval);
	if (r<0)
	    {fprintf(stderr, "MAX2120 write fail: xr_div");}

	r = m2120_pll_wait(dev);
	if (r < 0)
	{
		fprintf(stderr, "MAX2120 failure for %uHz!\n", tunefreq);
	}

	return (int)real_freq;
}

