/*
 * si5351.cpp - Si5351 library for STM32F411 (HAL)
 *
 * Ported from the Arduino library by Jason Milldrum <milldrum@gmail.com>
 *                                    Dana H. Myers <k6jq@comcast.net>
 *
 * Original copyright (C) 2015 - 2019 Jason Milldrum / Dana H. Myers
 * Some tuning algorithms derived from clk-si5351.c in the Linux kernel.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Porting changes vs. the Arduino original:
 *  1. Wire / Arduino headers replaced with stm32f4xx_hal.h.
 *  2. I2C_HandleTypeDef* stored and used for all bus access.
 *  3. HAL_I2C_IsDeviceReady() used for presence check instead of
 *     Wire.endTransmission().
 *  4. HAL_I2C_Master_Transmit() / _Receive() replace Wire calls.
 *  5. HAL requires the 8-bit address (7-bit << 1); the constructor does
 *     this shift once so callers still pass the plain 7-bit value 0x60.
 *  6. `new uint8_t[20]` / `delete` replaced with local stack arrays to
 *     avoid heap fragmentation on a microcontroller.
 *  7. Return types of write functions changed to HAL_StatusTypeDef.
 */

#include <stdint.h>
#include "si5351.h"

/* ================================================================== */
/* Public functions                                                     */
/* ================================================================== */

/*
 * Constructor
 *
 * hi2c     - Pointer to the initialised HAL I2C handle.
 * i2c_addr - 7-bit device address (default 0x60). Stored shifted left
 *            by one bit as required by the HAL API.
 */
Si5351::Si5351(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr)
    : i2c_bus_addr((uint8_t)(i2c_addr << 1)),
	  hi2c(hi2c)
{
    xtal_freq[0] = SI5351_XTAL_FREQ;

    /* Default: crystal oscillator as reference for both PLLs */
    plla_ref_osc = SI5351_PLL_INPUT_XO;
    pllb_ref_osc = SI5351_PLL_INPUT_XO;
    clkin_div    = SI5351_CLKIN_DIV_1;
}

/*
 * init(uint8_t xtal_load_c, uint32_t xo_freq, int32_t corr)
 *
 * Initialise the Si5351.
 *
 * xtal_load_c - Crystal load capacitance; use SI5351_CRYSTAL_LOAD_*PF.
 * xo_freq     - Crystal/reference frequency in Hz (0 → use 25 MHz default).
 * corr        - Frequency correction in parts-per-billion.
 *
 * Returns true when the device is found and initialised, false otherwise.
 *
 * NOTE: The HAL I2C peripheral must already be initialised (MX_I2Cx_Init
 *       called) before calling this function.
 */
bool Si5351::init(uint8_t xtal_load_c, uint32_t xo_freq, int32_t corr)
{
    /* Check that the device is present on the bus */
    if (HAL_I2C_IsDeviceReady(hi2c, i2c_bus_addr, 3, SI5351_I2C_TIMEOUT) != HAL_OK)
    {
        return false;
    }

    /* Wait for the SYS_INIT flag to clear (device ready) */
    uint8_t status_reg;
    do
    {
        status_reg = si5351_read(SI5351_DEVICE_STATUS);
    } while ((status_reg >> 7) == 1);

    /* Set crystal load capacitance */
    si5351_write(SI5351_CRYSTAL_LOAD,
                 (xtal_load_c & SI5351_CRYSTAL_LOAD_MASK) | 0x12);

    /* Set reference oscillator frequencies */
    if (xo_freq != 0)
    {
        set_ref_freq(xo_freq, SI5351_PLL_INPUT_XO);
        set_ref_freq(xo_freq, SI5351_PLL_INPUT_CLKIN);
    }
    else
    {
        set_ref_freq(SI5351_XTAL_FREQ, SI5351_PLL_INPUT_XO);
        set_ref_freq(SI5351_XTAL_FREQ, SI5351_PLL_INPUT_CLKIN);
    }

    /* Apply frequency calibration */
    set_correction(corr, SI5351_PLL_INPUT_XO);
    set_correction(corr, SI5351_PLL_INPUT_CLKIN);

    reset();

    return true;
}

/*
 * reset(void)
 *
 * Reset the Si5351 to the state set up by this library.
 */
void Si5351::reset(void)
{
    /* Power down all outputs first */
    si5351_write(16, 0x80);
    si5351_write(17, 0x80);
    si5351_write(18, 0x80);
    si5351_write(19, 0x80);
    si5351_write(20, 0x80);
    si5351_write(21, 0x80);
    si5351_write(22, 0x80);
    si5351_write(23, 0x80);

    /* Turn outputs back on */
    si5351_write(16, 0x0c);
    si5351_write(17, 0x0c);
    si5351_write(18, 0x0c);
    si5351_write(19, 0x0c);
    si5351_write(20, 0x0c);
    si5351_write(21, 0x0c);
    si5351_write(22, 0x0c);
    si5351_write(23, 0x0c);

    /* Set PLLA and PLLB to 800 MHz for automatic tuning */
    set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    set_pll(SI5351_PLL_FIXED, SI5351_PLLB);

    /* Default PLL → clock assignments */
    pll_assignment[0] = SI5351_PLLA;
    pll_assignment[1] = SI5351_PLLA;
    pll_assignment[2] = SI5351_PLLA;
    pll_assignment[3] = SI5351_PLLA;
    pll_assignment[4] = SI5351_PLLA;
    pll_assignment[5] = SI5351_PLLA;
    pll_assignment[6] = SI5351_PLLB;
    pll_assignment[7] = SI5351_PLLB;

    set_ms_source(SI5351_CLK0, SI5351_PLLA);
    set_ms_source(SI5351_CLK1, SI5351_PLLA);
    set_ms_source(SI5351_CLK2, SI5351_PLLA);
    set_ms_source(SI5351_CLK3, SI5351_PLLA);
    set_ms_source(SI5351_CLK4, SI5351_PLLA);
    set_ms_source(SI5351_CLK5, SI5351_PLLA);
    set_ms_source(SI5351_CLK6, SI5351_PLLB);
    set_ms_source(SI5351_CLK7, SI5351_PLLB);

    /* Reset VCXO parameters */
    si5351_write(SI5351_VXCO_PARAMETERS_LOW,  0);
    si5351_write(SI5351_VXCO_PARAMETERS_MID,  0);
    si5351_write(SI5351_VXCO_PARAMETERS_HIGH, 0);

    /* Reset both PLLs */
    pll_reset(SI5351_PLLA);
    pll_reset(SI5351_PLLB);

    /* Clear clock frequencies and disable all outputs */
    for (uint8_t i = 0; i < 8; i++)
    {
        clk_freq[i]      = 0;
        clk_first_set[i] = false;
        output_enable((enum si5351_clock)i, 0);
    }
}

/*
 * set_freq(uint64_t freq, enum si5351_clock clk)
 *
 * Set the output frequency of a CLK output (8 kHz – 150 MHz).
 * freq is in Hz (no multiplier — multiply by SI5351_FREQ_MULT yourself
 * if you need sub-Hz resolution in the same way the original library does).
 *
 * Returns 0 on success, 1 if the frequency cannot be set.
 */
uint8_t Si5351::set_freq(uint64_t freq, enum si5351_clock clk)
{
    struct Si5351RegSet ms_reg;
    uint64_t pll_freq;
    uint8_t  int_mode = 0;
    uint8_t  div_by_4 = 0;
    uint8_t  r_div    = 0;

    if ((uint8_t)clk <= (uint8_t)SI5351_CLK5)
    {
        /* MS0 – MS5 ------------------------------------------------ */

        /* Clamp to valid range */
        if (freq > 0 && freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT)
            freq = SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT;

        if (freq > SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT)
            freq = SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT;

        if (freq > (SI5351_MULTISYNTH_SHARE_MAX * SI5351_FREQ_MULT))
        {
            /* Frequencies above 100 MHz: check no other clock on the
             * same PLL is already above 100 MHz. */
            for (uint8_t i = 0; i < 6; i++)
            {
                if (clk_freq[i] > (SI5351_MULTISYNTH_SHARE_MAX * SI5351_FREQ_MULT))
                {
                    if (i != (uint8_t)clk && pll_assignment[i] == pll_assignment[clk])
                        return 1;
                }
            }

            if (clk_first_set[(uint8_t)clk] == false)
            {
                output_enable(clk, 1);
                clk_first_set[(uint8_t)clk] = true;
            }

            clk_freq[(uint8_t)clk] = freq;

            pll_freq = multisynth_calc(freq, 0, &ms_reg);
            set_pll(pll_freq, pll_assignment[clk]);

            /* Recalculate all other synths on the same PLL */
            for (uint8_t i = 0; i < 6; i++)
            {
                if (clk_freq[i] != 0 && pll_assignment[i] == pll_assignment[clk])
                {
                    struct Si5351RegSet temp_reg;
                    uint64_t temp_freq = clk_freq[i];

                    r_div = select_r_div(&temp_freq);
                    multisynth_calc(temp_freq, pll_freq, &temp_reg);

                    if (temp_freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT)
                    {
                        div_by_4 = 1;
                        int_mode = 1;
                    }
                    else
                    {
                        div_by_4 = 0;
                        int_mode = 0;
                    }

                    set_ms((enum si5351_clock)i, temp_reg, int_mode, r_div, div_by_4);
                }
            }

            pll_reset(pll_assignment[clk]);
        }
        else
        {
            clk_freq[(uint8_t)clk] = freq;

            if (clk_first_set[(uint8_t)clk] == false)
            {
                output_enable(clk, 1);
                clk_first_set[(uint8_t)clk] = true;
            }

            r_div = select_r_div(&freq);

            if (pll_assignment[clk] == SI5351_PLLA)
                multisynth_calc(freq, plla_freq, &ms_reg);
            else
                multisynth_calc(freq, pllb_freq, &ms_reg);

            set_ms(clk, ms_reg, int_mode, r_div, div_by_4);
        }

        return 0;
    }
    else
    {
        /* MS6 / MS7 ------------------------------------------------ */

        if (freq > 0 && freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT)
            freq = SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT;

        if (freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT)
            freq = SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT - 1;

        if (clk == SI5351_CLK6)
        {
            if (clk_freq[7] != 0)
            {
                if (pllb_freq % freq == 0)
                {
                    if ((pllb_freq / freq) % 2 != 0)
                        return 1;

                    clk_freq[(uint8_t)clk] = freq;
                    r_div = select_r_div_ms67(&freq);
                    multisynth67_calc(freq, pllb_freq, &ms_reg);
                }
                else
                {
                    return 1;
                }
            }
            else
            {
                clk_freq[(uint8_t)clk] = freq;
                r_div = select_r_div_ms67(&freq);
                pll_freq = multisynth67_calc(freq, 0, &ms_reg);
                set_pll(pll_freq, SI5351_PLLB);
            }
        }
        else /* SI5351_CLK7 */
        {
            if (clk_freq[6] != 0)
            {
                if (pllb_freq % freq == 0)
                {
                    if ((pllb_freq / freq) % 2 != 0)
                        return 1;

                    clk_freq[(uint8_t)clk] = freq;
                    r_div = select_r_div_ms67(&freq);
                    multisynth67_calc(freq, pllb_freq, &ms_reg);
                }
                else
                {
                    return 1;
                }
            }
            else
            {
                clk_freq[(uint8_t)clk] = freq;
                r_div = select_r_div_ms67(&freq);
                pll_freq = multisynth67_calc(freq, 0, &ms_reg);
                set_pll(pll_freq, pll_assignment[clk]);
            }
        }

        div_by_4 = 0;
        int_mode = 0;
        set_ms(clk, ms_reg, int_mode, r_div, div_by_4);
        return 0;
    }
}

/*
 * set_freq_manual(uint64_t freq, uint64_t pll_freq, enum si5351_clock clk)
 *
 * Set the output frequency using a manually specified PLL frequency.
 * The caller is responsible for ensuring the PLL is correctly configured
 * and the MS is assigned to it.
 */
uint8_t Si5351::set_freq_manual(uint64_t freq, uint64_t pll_freq,
                                 enum si5351_clock clk)
{
    struct Si5351RegSet ms_reg;
    uint8_t int_mode = 0;
    uint8_t div_by_4 = 0;
    uint8_t r_div;

    if (freq > 0 && freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT)
        freq = SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT;

    if (freq > SI5351_CLKOUT_MAX_FREQ * SI5351_FREQ_MULT)
        freq = SI5351_CLKOUT_MAX_FREQ * SI5351_FREQ_MULT;

    clk_freq[(uint8_t)clk] = freq;

    set_pll(pll_freq, pll_assignment[clk]);
    output_enable(clk, 1);

    r_div = select_r_div(&freq);
    multisynth_calc(freq, pll_freq, &ms_reg);

    if (freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT)
    {
        div_by_4 = 1;
        int_mode = 1;
    }

    set_ms(clk, ms_reg, int_mode, r_div, div_by_4);
    return 0;
}

/*
 * set_pll(uint64_t pll_freq, enum si5351_pll target_pll)
 *
 * Set one of the two PLLs to a specific VCO frequency.
 * pll_freq is in Hz * SI5351_FREQ_MULT (i.e. Hz * 100).
 */
void Si5351::set_pll(uint64_t pll_freq, enum si5351_pll target_pll)
{
    struct Si5351RegSet pll_reg;

    if (target_pll == SI5351_PLLA)
        pll_calc(SI5351_PLLA, pll_freq, &pll_reg, ref_correction[plla_ref_osc], 0);
    else
        pll_calc(SI5351_PLLB, pll_freq, &pll_reg, ref_correction[pllb_ref_osc], 0);

    /* Build the 8-byte parameter block on the stack */
    uint8_t params[8];
    uint8_t i = 0;

    params[i++] = (uint8_t)((pll_reg.p3 >> 8) & 0xFF);
    params[i++] = (uint8_t)( pll_reg.p3        & 0xFF);
    params[i++] = (uint8_t)((pll_reg.p1 >> 16) & 0x03);
    params[i++] = (uint8_t)((pll_reg.p1 >>  8) & 0xFF);
    params[i++] = (uint8_t)( pll_reg.p1        & 0xFF);
    params[i++] = (uint8_t)((pll_reg.p3 >> 12) & 0xF0) |
                  (uint8_t)((pll_reg.p2 >> 16) & 0x0F);
    params[i++] = (uint8_t)((pll_reg.p2 >>  8) & 0xFF);
    params[i++] = (uint8_t)( pll_reg.p2        & 0xFF);

    if (target_pll == SI5351_PLLA)
    {
        si5351_write_bulk(SI5351_PLLA_PARAMETERS, i, params);
        plla_freq = pll_freq;
    }
    else
    {
        si5351_write_bulk(SI5351_PLLB_PARAMETERS, i, params);
        pllb_freq = pll_freq;
    }
}

/*
 * set_ms(enum si5351_clock clk, struct Si5351RegSet ms_reg,
 *        uint8_t int_mode, uint8_t r_div, uint8_t div_by_4)
 *
 * Write Multisynth parameters directly (advanced use).
 */
void Si5351::set_ms(enum si5351_clock clk, struct Si5351RegSet ms_reg,
                    uint8_t int_mode, uint8_t r_div, uint8_t div_by_4)
{
    uint8_t params[8];
    uint8_t i = 0;
    uint8_t temp;
    uint8_t reg_val;

    if ((uint8_t)clk <= (uint8_t)SI5351_CLK5)
    {
        params[i++] = (uint8_t)((ms_reg.p3 >> 8) & 0xFF);
        params[i++] = (uint8_t)( ms_reg.p3        & 0xFF);

        reg_val  = si5351_read((SI5351_CLK0_PARAMETERS + 2) + (clk * 8));
        reg_val &= ~(0x03);
        params[i++] = reg_val | (uint8_t)((ms_reg.p1 >> 16) & 0x03);

        params[i++] = (uint8_t)((ms_reg.p1 >>  8) & 0xFF);
        params[i++] = (uint8_t)( ms_reg.p1        & 0xFF);
        params[i++] = (uint8_t)((ms_reg.p3 >> 12) & 0xF0) |
                      (uint8_t)((ms_reg.p2 >> 16) & 0x0F);
        params[i++] = (uint8_t)((ms_reg.p2 >>  8) & 0xFF);
        params[i++] = (uint8_t)( ms_reg.p2        & 0xFF);
    }
    else
    {
        /* MS6 / MS7: single-register, integer-only divider */
        temp = (uint8_t)ms_reg.p1;
    }

    switch (clk)
    {
    case SI5351_CLK0:
        si5351_write_bulk(SI5351_CLK0_PARAMETERS, i, params);
        set_int(clk, int_mode);
        ms_div(clk, r_div, div_by_4);
        break;
    case SI5351_CLK1:
        si5351_write_bulk(SI5351_CLK1_PARAMETERS, i, params);
        set_int(clk, int_mode);
        ms_div(clk, r_div, div_by_4);
        break;
    case SI5351_CLK2:
        si5351_write_bulk(SI5351_CLK2_PARAMETERS, i, params);
        set_int(clk, int_mode);
        ms_div(clk, r_div, div_by_4);
        break;
    case SI5351_CLK3:
        si5351_write_bulk(SI5351_CLK3_PARAMETERS, i, params);
        set_int(clk, int_mode);
        ms_div(clk, r_div, div_by_4);
        break;
    case SI5351_CLK4:
        si5351_write_bulk(SI5351_CLK4_PARAMETERS, i, params);
        set_int(clk, int_mode);
        ms_div(clk, r_div, div_by_4);
        break;
    case SI5351_CLK5:
        si5351_write_bulk(SI5351_CLK5_PARAMETERS, i, params);
        set_int(clk, int_mode);
        ms_div(clk, r_div, div_by_4);
        break;
    case SI5351_CLK6:
        si5351_write(SI5351_CLK6_PARAMETERS, temp);
        ms_div(clk, r_div, div_by_4);
        break;
    case SI5351_CLK7:
        si5351_write(SI5351_CLK7_PARAMETERS, temp);
        ms_div(clk, r_div, div_by_4);
        break;
    }
}

/*
 * output_enable(enum si5351_clock clk, uint8_t enable)
 *
 * Enable (1) or disable (0) a clock output.
 */
void Si5351::output_enable(enum si5351_clock clk, uint8_t enable)
{
    uint8_t reg_val = si5351_read(SI5351_OUTPUT_ENABLE_CTRL);

    if (enable == 1)
        reg_val &= ~(1 << (uint8_t)clk);
    else
        reg_val |=  (1 << (uint8_t)clk);

    si5351_write(SI5351_OUTPUT_ENABLE_CTRL, reg_val);
}

/*
 * drive_strength(enum si5351_clock clk, enum si5351_drive drive)
 *
 * Set the output drive strength of a clock output.
 */
void Si5351::drive_strength(enum si5351_clock clk, enum si5351_drive drive)
{
    uint8_t reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);
    reg_val &= ~(0x03);

    switch (drive)
    {
    case SI5351_DRIVE_2MA: reg_val |= 0x00; break;
    case SI5351_DRIVE_4MA: reg_val |= 0x01; break;
    case SI5351_DRIVE_6MA: reg_val |= 0x02; break;
    case SI5351_DRIVE_8MA: reg_val |= 0x03; break;
    default: break;
    }

    si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * update_status(void)
 *
 * Refresh dev_status and dev_int_status from the device registers.
 */
void Si5351::update_status(void)
{
    update_sys_status(&dev_status);
    update_int_status(&dev_int_status);
}

/*
 * set_correction(int32_t corr, enum si5351_pll_input ref_osc)
 *
 * Store a parts-per-billion correction for the chosen reference
 * oscillator and recalculate both PLLs.
 */
void Si5351::set_correction(int32_t corr, enum si5351_pll_input ref_osc)
{
    ref_correction[(uint8_t)ref_osc] = corr;
    set_pll(plla_freq, SI5351_PLLA);
    set_pll(pllb_freq, SI5351_PLLB);
}

/*
 * set_phase(enum si5351_clock clk, uint8_t phase)
 *
 * Write the 7-bit phase offset register for the given clock.
 * Units are VCO/4 period.
 */
void Si5351::set_phase(enum si5351_clock clk, uint8_t phase)
{
    si5351_write(SI5351_CLK0_PHASE_OFFSET + (uint8_t)clk,
                 phase & 0x7F);
}

/*
 * get_correction(enum si5351_pll_input ref_osc)
 *
 * Return the stored ppb correction for the given reference oscillator.
 */
int32_t Si5351::get_correction(enum si5351_pll_input ref_osc)
{
    return ref_correction[(uint8_t)ref_osc];
}

/*
 * pll_reset(enum si5351_pll target_pll)
 *
 * Issue a soft reset to the specified PLL.
 */
void Si5351::pll_reset(enum si5351_pll target_pll)
{
    if (target_pll == SI5351_PLLA)
        si5351_write(SI5351_PLL_RESET, SI5351_PLL_RESET_A);
    else
        si5351_write(SI5351_PLL_RESET, SI5351_PLL_RESET_B);
}

/*
 * set_ms_source(enum si5351_clock clk, enum si5351_pll pll)
 *
 * Select which PLL drives the given Multisynth.
 */
void Si5351::set_ms_source(enum si5351_clock clk, enum si5351_pll pll)
{
    uint8_t reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);

    if (pll == SI5351_PLLA)
        reg_val &= ~SI5351_CLK_PLL_SELECT;
    else
        reg_val |=  SI5351_CLK_PLL_SELECT;

    si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
    pll_assignment[(uint8_t)clk] = pll;
}

/*
 * set_int(enum si5351_clock clk, uint8_t enable)
 *
 * Enable (1) or disable (0) integer mode for a Multisynth.
 */
void Si5351::set_int(enum si5351_clock clk, uint8_t enable)
{
    uint8_t reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);

    if (enable == 1)
        reg_val |=  SI5351_CLK_INTEGER_MODE;
    else
        reg_val &= ~SI5351_CLK_INTEGER_MODE;

    si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * set_clock_pwr(enum si5351_clock clk, uint8_t pwr)
 *
 * Enable (1) or disable (0) power to a clock output.
 */
void Si5351::set_clock_pwr(enum si5351_clock clk, uint8_t pwr)
{
    uint8_t reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);

    if (pwr == 1)
        reg_val &= 0x7F;
    else
        reg_val |= 0x80;

    si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * set_clock_invert(enum si5351_clock clk, uint8_t inv)
 *
 * Invert (1) or normal (0) the clock output waveform.
 */
void Si5351::set_clock_invert(enum si5351_clock clk, uint8_t inv)
{
    uint8_t reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);

    if (inv == 1)
        reg_val |=  SI5351_CLK_INVERT;
    else
        reg_val &= ~SI5351_CLK_INVERT;

    si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * set_clock_source(enum si5351_clock clk, enum si5351_clock_source src)
 *
 * Set the clock source for a Multisynth (XTAL, CLKIN, MS0, or own MS).
 */
void Si5351::set_clock_source(enum si5351_clock clk,
                               enum si5351_clock_source src)
{
    uint8_t reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);
    reg_val &= ~SI5351_CLK_INPUT_MASK;

    switch (src)
    {
    case SI5351_CLK_SRC_XTAL:
        reg_val |= SI5351_CLK_INPUT_XTAL;
        break;
    case SI5351_CLK_SRC_CLKIN:
        reg_val |= SI5351_CLK_INPUT_CLKIN;
        break;
    case SI5351_CLK_SRC_MS0:
        if (clk == SI5351_CLK0) return;
        reg_val |= SI5351_CLK_INPUT_MULTISYNTH_0_4;
        break;
    case SI5351_CLK_SRC_MS:
        reg_val |= SI5351_CLK_INPUT_MULTISYNTH_N;
        break;
    default:
        return;
    }

    si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * set_clock_disable(enum si5351_clock clk, enum si5351_clock_disable dis_state)
 *
 * Set the output state when the clock is disabled (low, high, Hi-Z, never).
 */
void Si5351::set_clock_disable(enum si5351_clock clk,
                                enum si5351_clock_disable dis_state)
{
    uint8_t reg_val, reg;

    if      (clk >= SI5351_CLK0 && clk <= SI5351_CLK3) reg = SI5351_CLK3_0_DISABLE_STATE;
    else if (clk >= SI5351_CLK4 && clk <= SI5351_CLK7) reg = SI5351_CLK7_4_DISABLE_STATE;
    else return;

    reg_val = si5351_read(reg);

    if (clk >= SI5351_CLK0 && clk <= SI5351_CLK3)
    {
        reg_val &= ~(0x03 << (clk * 2));
        reg_val |= (dis_state << (clk * 2));
    }
    else
    {
        reg_val &= ~(0x03 << ((clk - 4) * 2));
        reg_val |= (dis_state << ((clk - 4) * 2));
    }

    si5351_write(reg, reg_val);
}

/*
 * set_clock_fanout(enum si5351_clock_fanout fanout, uint8_t enable)
 *
 * Enable (1) or disable (0) a fanout option (CLKIN, XO, Multisynth).
 */
void Si5351::set_clock_fanout(enum si5351_clock_fanout fanout, uint8_t enable)
{
    uint8_t reg_val = si5351_read(SI5351_FANOUT_ENABLE);

    switch (fanout)
    {
    case SI5351_FANOUT_CLKIN:
        if (enable) reg_val |=  SI5351_CLKIN_ENABLE;
        else        reg_val &= ~SI5351_CLKIN_ENABLE;
        break;
    case SI5351_FANOUT_XO:
        if (enable) reg_val |=  SI5351_XTAL_ENABLE;
        else        reg_val &= ~SI5351_XTAL_ENABLE;
        break;
    case SI5351_FANOUT_MS:
        if (enable) reg_val |=  SI5351_MULTISYNTH_ENABLE;
        else        reg_val &= ~SI5351_MULTISYNTH_ENABLE;
        break;
    }

    si5351_write(SI5351_FANOUT_ENABLE, reg_val);
}

/*
 * set_pll_input(enum si5351_pll pll, enum si5351_pll_input input)
 *
 * Select XO or CLKIN as the reference for the given PLL.
 */
void Si5351::set_pll_input(enum si5351_pll pll, enum si5351_pll_input input)
{
    uint8_t reg_val = si5351_read(SI5351_PLL_INPUT_SOURCE);

    switch (pll)
    {
    case SI5351_PLLA:
        if (input == SI5351_PLL_INPUT_CLKIN)
        {
            reg_val |= SI5351_PLLA_SOURCE;
            reg_val |= clkin_div;
            plla_ref_osc = SI5351_PLL_INPUT_CLKIN;
        }
        else
        {
            reg_val &= ~SI5351_PLLA_SOURCE;
            plla_ref_osc = SI5351_PLL_INPUT_XO;
        }
        break;
    case SI5351_PLLB:
        if (input == SI5351_PLL_INPUT_CLKIN)
        {
            reg_val |= SI5351_PLLB_SOURCE;
            reg_val |= clkin_div;
            pllb_ref_osc = SI5351_PLL_INPUT_CLKIN;
        }
        else
        {
            reg_val &= ~SI5351_PLLB_SOURCE;
            pllb_ref_osc = SI5351_PLL_INPUT_XO;
        }
        break;
    default:
        return;
    }

    si5351_write(SI5351_PLL_INPUT_SOURCE, reg_val);
    set_pll(plla_freq, SI5351_PLLA);
    set_pll(pllb_freq, SI5351_PLLB);
}

/*
 * set_vcxo(uint64_t pll_freq, uint8_t ppm)
 *
 * Configure the VCXO parameters (Si5351B only).
 * pll_freq - Desired PLLB VCO frequency in Hz * 100.
 * ppm      - VCXO pull range in ppm (30 – 240).
 */
void Si5351::set_vcxo(uint64_t pll_freq, uint8_t ppm)
{
    struct Si5351RegSet pll_reg;
    uint64_t vcxo_param;
    uint8_t  temp;

    if (ppm < SI5351_VCXO_PULL_MIN) ppm = SI5351_VCXO_PULL_MIN;
    if (ppm > SI5351_VCXO_PULL_MAX) ppm = SI5351_VCXO_PULL_MAX;

    vcxo_param = pll_calc(SI5351_PLLB, pll_freq, &pll_reg,
                          ref_correction[pllb_ref_osc], 1);

    uint8_t params[8];
    uint8_t i = 0;

    params[i++] = (uint8_t)((pll_reg.p3 >> 8) & 0xFF);
    params[i++] = (uint8_t)( pll_reg.p3        & 0xFF);
    params[i++] = (uint8_t)((pll_reg.p1 >> 16) & 0x03);
    params[i++] = (uint8_t)((pll_reg.p1 >>  8) & 0xFF);
    params[i++] = (uint8_t)( pll_reg.p1        & 0xFF);
    params[i++] = (uint8_t)((pll_reg.p3 >> 12) & 0xF0) |
                  (uint8_t)((pll_reg.p2 >> 16) & 0x0F);
    params[i++] = (uint8_t)((pll_reg.p2 >>  8) & 0xFF);
    params[i++] = (uint8_t)( pll_reg.p2        & 0xFF);

    si5351_write_bulk(SI5351_PLLB_PARAMETERS, i, params);

    vcxo_param = ((vcxo_param * ppm * SI5351_VCXO_MARGIN) / 100ULL) / 1000000ULL;

    temp = (uint8_t)(vcxo_param & 0xFF);
    si5351_write(SI5351_VXCO_PARAMETERS_LOW, temp);

    temp = (uint8_t)((vcxo_param >> 8) & 0xFF);
    si5351_write(SI5351_VXCO_PARAMETERS_MID, temp);

    temp = (uint8_t)((vcxo_param >> 16) & 0x3F);
    si5351_write(SI5351_VXCO_PARAMETERS_HIGH, temp);
}

/*
 * set_ref_freq(uint32_t ref_freq, enum si5351_pll_input ref_osc)
 *
 * Set the reference oscillator frequency (and CLKIN pre-divider) for
 * the chosen reference input.
 */
void Si5351::set_ref_freq(uint32_t ref_freq, enum si5351_pll_input ref_osc)
{
    if (ref_freq <= 30000000UL)
    {
        xtal_freq[(uint8_t)ref_osc] = ref_freq;
        if (ref_osc == SI5351_PLL_INPUT_CLKIN)
            clkin_div = SI5351_CLKIN_DIV_1;
    }
    else if (ref_freq > 30000000UL && ref_freq <= 60000000UL)
    {
        xtal_freq[(uint8_t)ref_osc] = ref_freq / 2;
        if (ref_osc == SI5351_PLL_INPUT_CLKIN)
            clkin_div = SI5351_CLKIN_DIV_2;
    }
    else if (ref_freq > 60000000UL && ref_freq <= 100000000UL)
    {
        xtal_freq[(uint8_t)ref_osc] = ref_freq / 4;
        if (ref_osc == SI5351_PLL_INPUT_CLKIN)
            clkin_div = SI5351_CLKIN_DIV_4;
    }
    /* Frequencies above 100 MHz are not supported; do nothing */
}

/* ------------------------------------------------------------------ */
/* Low-level I2C access                                                 */
/* ------------------------------------------------------------------ */

/*
 * si5351_write_bulk(uint8_t addr, uint8_t bytes, uint8_t *data)
 *
 * Write `bytes` bytes starting at register `addr`.
 * The register address and data are sent in a single I2C transaction.
 */
HAL_StatusTypeDef Si5351::si5351_write_bulk(uint8_t addr, uint8_t bytes,
                                             uint8_t *data)
{
    /* Build a single contiguous buffer: [addr, data0, data1, ...] */
    uint8_t buf[bytes + 1];
    buf[0] = addr;
    for (uint8_t i = 0; i < bytes; i++)
        buf[i + 1] = data[i];

    return HAL_I2C_Master_Transmit(hi2c, i2c_bus_addr,
                                   buf, bytes + 1,
                                   SI5351_I2C_TIMEOUT);
}

/*
 * si5351_write(uint8_t addr, uint8_t data)
 *
 * Write a single byte to register `addr`.
 */
HAL_StatusTypeDef Si5351::si5351_write(uint8_t addr, uint8_t data)
{
    uint8_t buf[2] = { addr, data };
    return HAL_I2C_Master_Transmit(hi2c, i2c_bus_addr,
                                   buf, 2,
                                   SI5351_I2C_TIMEOUT);
}

/*
 * si5351_read(uint8_t addr)
 *
 * Read and return a single byte from register `addr`.
 * Returns 0 if the I2C transaction fails.
 */
uint8_t Si5351::si5351_read(uint8_t addr)
{
    uint8_t reg_val = 0;

    /* Write the register address */
    if (HAL_I2C_Master_Transmit(hi2c, i2c_bus_addr,
                                &addr, 1,
                                SI5351_I2C_TIMEOUT) != HAL_OK)
        return 0;

    /* Read one byte back */
    HAL_I2C_Master_Receive(hi2c, i2c_bus_addr,
                           &reg_val, 1,
                           SI5351_I2C_TIMEOUT);
    return reg_val;
}

/* ================================================================== */
/* Private functions (all logic unchanged from the Arduino original)   */
/* ================================================================== */

uint64_t Si5351::pll_calc(enum si5351_pll pll, uint64_t freq,
                           struct Si5351RegSet *reg,
                           int32_t correction, uint8_t vcxo)
{
    uint64_t ref_freq;
    if (pll == SI5351_PLLA)
        ref_freq = xtal_freq[(uint8_t)plla_ref_osc] * SI5351_FREQ_MULT;
    else
        ref_freq = xtal_freq[(uint8_t)pllb_ref_osc] * SI5351_FREQ_MULT;

    uint32_t a, b, c, p1, p2, p3;
    uint64_t lltmp;

    /* Apply ppb correction */
    ref_freq = ref_freq + (int32_t)((((((int64_t)correction) << 31) /
               1000000000LL) * ref_freq) >> 31);

    /* Clamp VCO frequency */
    if (freq < SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT)
        freq = SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT;
    if (freq > SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT)
        freq = SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT;

    /* Integer part of feedback divider */
    a = (uint32_t)(freq / ref_freq);

    if (a < SI5351_PLL_A_MIN) freq = ref_freq * SI5351_PLL_A_MIN;
    if (a > SI5351_PLL_A_MAX) freq = ref_freq * SI5351_PLL_A_MAX;

    /* Fractional part */
    if (vcxo)
    {
        b = (uint32_t)((((uint64_t)(freq % ref_freq)) * 1000000ULL) / ref_freq);
        c = 1000000UL;
    }
    else
    {
        b = (uint32_t)((((uint64_t)(freq % ref_freq)) * RFRAC_DENOM) / ref_freq);
        c = b ? (uint32_t)RFRAC_DENOM : 1;
    }

    p1 = 128 * a + ((128 * b) / c) - 512;
    p2 = 128 * b - c * ((128 * b) / c);
    p3 = c;

    /* Recalculate actual VCO frequency */
    lltmp  = ref_freq;
    lltmp *= b;
    do_div(lltmp, c);
    freq   = lltmp + ref_freq * a;

    reg->p1 = p1;
    reg->p2 = p2;
    reg->p3 = p3;

    if (vcxo)
        return (uint64_t)(128 * a * 1000000ULL + b);
    else
        return freq;
}

uint64_t Si5351::multisynth_calc(uint64_t freq, uint64_t pll_freq,
                                  struct Si5351RegSet *reg)
{
    uint64_t lltmp;
    uint32_t a, b, c, p1, p2, p3;
    uint8_t  divby4  = 0;
    uint8_t  ret_val = 0;

    /* Clamp */
    if (freq > SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT)
        freq = SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT;
    if (freq < SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT)
        freq = SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT;

    if (freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT)
        divby4 = 1;

    if (pll_freq == 0)
    {
        /* Find integer divider for maximum VCO frequency */
        if (divby4 == 0)
        {
            lltmp = SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT;
            do_div(lltmp, freq);
            if (lltmp == 5) lltmp = 4;
            else if (lltmp == 7) lltmp = 6;
            a = (uint32_t)lltmp;
        }
        else
        {
            a = 4;
        }

        b = 0;
        c = 1;
        pll_freq = a * freq;
    }
    else
    {
        ret_val = 1;
        a = (uint32_t)(pll_freq / freq);

        if (a < SI5351_MULTISYNTH_A_MIN) freq = pll_freq / SI5351_MULTISYNTH_A_MIN;
        if (a > SI5351_MULTISYNTH_A_MAX) freq = pll_freq / SI5351_MULTISYNTH_A_MAX;

        b = (uint32_t)((pll_freq % freq * RFRAC_DENOM) / freq);
        c = b ? (uint32_t)RFRAC_DENOM : 1;
    }

    if (divby4 == 1)
    {
        p3 = 1; p2 = 0; p1 = 0;
    }
    else
    {
        p1 = 128 * a + ((128 * b) / c) - 512;
        p2 = 128 * b - c * ((128 * b) / c);
        p3 = c;
    }

    reg->p1 = p1;
    reg->p2 = p2;
    reg->p3 = p3;

    return (ret_val == 0) ? pll_freq : freq;
}

uint64_t Si5351::multisynth67_calc(uint64_t freq, uint64_t pll_freq,
                                    struct Si5351RegSet *reg)
{
    uint32_t a;
    uint64_t lltmp;

    /* Clamp */
    if (freq > SI5351_MULTISYNTH67_MAX_FREQ * SI5351_FREQ_MULT)
        freq = SI5351_MULTISYNTH67_MAX_FREQ * SI5351_FREQ_MULT;
    if (freq < SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT)
        freq = SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT;

    if (pll_freq == 0)
    {
        lltmp = (SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT) - 100000000UL;
        do_div(lltmp, freq);
        a = (uint32_t)lltmp;

        if (a % 2 != 0) a++;

        if (a < SI5351_MULTISYNTH_A_MIN) a = SI5351_MULTISYNTH_A_MIN;
        if (a > SI5351_MULTISYNTH67_A_MAX) a = SI5351_MULTISYNTH67_A_MAX;

        pll_freq = a * freq;

        if (pll_freq > (SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT))
        {
            a -= 2;
            pll_freq = a * freq;
        }
        else if (pll_freq < (SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT))
        {
            a += 2;
            pll_freq = a * freq;
        }

        reg->p1 = (uint8_t)a;
        reg->p2 = 0;
        reg->p3 = 0;
        return pll_freq;
    }
    else
    {
        if (pll_freq % freq)
            return 0;

        a = (uint32_t)(pll_freq / freq);

        if (a < SI5351_MULTISYNTH_A_MIN || a > SI5351_MULTISYNTH67_A_MAX)
            return 0;

        reg->p1 = (uint8_t)a;
        reg->p2 = 0;
        reg->p3 = 0;
        return 1;
    }
}

void Si5351::update_sys_status(struct Si5351Status *status)
{
    uint8_t reg_val = si5351_read(SI5351_DEVICE_STATUS);

    status->SYS_INIT = (reg_val >> 7) & 0x01;
    status->LOL_B    = (reg_val >> 6) & 0x01;
    status->LOL_A    = (reg_val >> 5) & 0x01;
    status->LOS      = (reg_val >> 4) & 0x01;
    status->REVID    =  reg_val       & 0x03;
}

void Si5351::update_int_status(struct Si5351IntStatus *int_status)
{
    uint8_t reg_val = si5351_read(SI5351_INTERRUPT_STATUS);

    int_status->SYS_INIT_STKY = (reg_val >> 7) & 0x01;
    int_status->LOL_B_STKY    = (reg_val >> 6) & 0x01;
    int_status->LOL_A_STKY    = (reg_val >> 5) & 0x01;
    int_status->LOS_STKY      = (reg_val >> 4) & 0x01;
}

void Si5351::ms_div(enum si5351_clock clk, uint8_t r_div, uint8_t div_by_4)
{
    uint8_t reg_val  = 0;
    uint8_t reg_addr = 0;

    switch (clk)
    {
    case SI5351_CLK0: reg_addr = SI5351_CLK0_PARAMETERS + 2; break;
    case SI5351_CLK1: reg_addr = SI5351_CLK1_PARAMETERS + 2; break;
    case SI5351_CLK2: reg_addr = SI5351_CLK2_PARAMETERS + 2; break;
    case SI5351_CLK3: reg_addr = SI5351_CLK3_PARAMETERS + 2; break;
    case SI5351_CLK4: reg_addr = SI5351_CLK4_PARAMETERS + 2; break;
    case SI5351_CLK5: reg_addr = SI5351_CLK5_PARAMETERS + 2; break;
    case SI5351_CLK6: reg_addr = SI5351_CLK6_7_OUTPUT_DIVIDER; break;
    case SI5351_CLK7: reg_addr = SI5351_CLK6_7_OUTPUT_DIVIDER; break;
    }

    reg_val = si5351_read(reg_addr);

    if ((uint8_t)clk <= (uint8_t)SI5351_CLK5)
    {
        reg_val &= ~(0x7C);
        if (div_by_4 == 0)
            reg_val &= ~SI5351_OUTPUT_CLK_DIVBY4;
        else
            reg_val |=  SI5351_OUTPUT_CLK_DIVBY4;

        reg_val |= (r_div << SI5351_OUTPUT_CLK_DIV_SHIFT);
    }
    else if (clk == SI5351_CLK6)
    {
        reg_val &= ~(0x07);
        reg_val |= r_div;
    }
    else if (clk == SI5351_CLK7)
    {
        reg_val &= ~(0x70);
        reg_val |= (r_div << SI5351_OUTPUT_CLK_DIV_SHIFT);
    }

    si5351_write(reg_addr, reg_val);
}

uint8_t Si5351::select_r_div(uint64_t *freq)
{
    uint8_t r_div = SI5351_OUTPUT_CLK_DIV_1;
    const uint64_t base = SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT;

    if      (*freq < base * 2)   { r_div = SI5351_OUTPUT_CLK_DIV_128; *freq *= 128ULL; }
    else if (*freq < base * 4)   { r_div = SI5351_OUTPUT_CLK_DIV_64;  *freq *=  64ULL; }
    else if (*freq < base * 8)   { r_div = SI5351_OUTPUT_CLK_DIV_32;  *freq *=  32ULL; }
    else if (*freq < base * 16)  { r_div = SI5351_OUTPUT_CLK_DIV_16;  *freq *=  16ULL; }
    else if (*freq < base * 32)  { r_div = SI5351_OUTPUT_CLK_DIV_8;   *freq *=   8ULL; }
    else if (*freq < base * 64)  { r_div = SI5351_OUTPUT_CLK_DIV_4;   *freq *=   4ULL; }
    else if (*freq < base * 128) { r_div = SI5351_OUTPUT_CLK_DIV_2;   *freq *=   2ULL; }

    return r_div;
}

uint8_t Si5351::select_r_div_ms67(uint64_t *freq)
{
    uint8_t r_div = SI5351_OUTPUT_CLK_DIV_1;
    const uint64_t base = SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT;

    if      (*freq < base * 2)   { r_div = SI5351_OUTPUT_CLK_DIV_128; *freq *= 128ULL; }
    else if (*freq < base * 4)   { r_div = SI5351_OUTPUT_CLK_DIV_64;  *freq *=  64ULL; }
    else if (*freq < base * 8)   { r_div = SI5351_OUTPUT_CLK_DIV_32;  *freq *=  32ULL; }
    else if (*freq < base * 16)  { r_div = SI5351_OUTPUT_CLK_DIV_16;  *freq *=  16ULL; }
    else if (*freq < base * 32)  { r_div = SI5351_OUTPUT_CLK_DIV_8;   *freq *=   8ULL; }
    else if (*freq < base * 64)  { r_div = SI5351_OUTPUT_CLK_DIV_4;   *freq *=   4ULL; }
    else if (*freq < base * 128) { r_div = SI5351_OUTPUT_CLK_DIV_2;   *freq *=   2ULL; }

    return r_div;
}
