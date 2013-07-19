/*
    temperature.c: temperature display module

    Copyright (C) 2012 Angelo Arrifano <miknix@gmail.com>
    Copyright (C) 2012 Matthew Excell <matt@excellclan.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <openchronos.h>

/* drivers */
#include "drivers/display.h"
#include "drivers/ps.h"
#include "drivers/cma_ps.h"
#include "drivers/bmp_ps.h"
#include "altitude.h"
#include "drivers/ports.h"
#include "drivers/timer.h"
#include "drivers/buzzer.h"
#include <string.h>



// Global Variable section
struct alt sAlt;
note bip[2] = {0x4b08,0x000F};

uint8_t altitudeEnabled;
s16 baseCalib[5] = {CONFIG_MOD_ALTITUDE_BASE1,
	CONFIG_MOD_ALTITUDE_BASE2,
	CONFIG_MOD_ALTITUDE_BASE3,
	CONFIG_MOD_ALTITUDE_BASE4,
	CONFIG_MOD_ALTITUDE_BASE5
};
s32 limit_high, limit_low;
uint8_t submenuState = 0;

static void altitude_activate(void)
{
	altitudeEnabled = 1;
	/* display -- symbol while a measure is not performed */
	display_chars(0, LCD_SEG_L1_3_0, "----", SEG_ON);

	//sys_messagebus_register(&update, SYS_MSG_RTC_SECOND);
	sys_messagebus_register(&update, SYS_MSG_TIMER_4S);
}

static void altitude_deactivate(void)
{
	sys_messagebus_unregister(&update);
	altitudeEnabled = 0;
	
	
	// Clean up function-specific segments before leaving function
		
	display_symbol(0, LCD_SYMB_ARROW_UP, SEG_OFF);
	display_symbol(0, LCD_SYMB_ARROW_DOWN, SEG_OFF);
        
#ifdef CONFIG_MOD_ALTITUDE_METRIC
	display_symbol(0, LCD_UNIT_L1_M, SEG_OFF);
#else
	display_symbol(0, LCD_UNIT_L1_FT, SEG_OFF);
#endif
	
	
	update_pressure_table((s16) sAlt.altitude, sAlt.pressure, sAlt.temperature);
}


void mod_altitude_init(void)
{
	menu_add_entry(" ALTI", NULL, NULL,
		&submenu_callback, &edit_mode_callback, &calib_callback, NULL,
		&altitude_activate, &altitude_deactivate);
	
	altitudeEnabled = 0;
	sAlt.pressure = 0;
	reset_altitude_measurement();
	
// Set lower and upper limits for offset correction
#ifdef CONFIG_MOD_ALTITUDE_METRIC
	// Limits for set_value function
	limit_low = -100;
	limit_high = 4000;
#else
// Limits for set_value function
	limit_low = -500;
	limit_high = 9999;
#endif
}

// *************************************************************************************************
// Extern section

// *************************************************************************************************
// @fn          reset_altitude_measurement
// @brief       Reset altitude measurement.
// @param       none
// @return      none
// *************************************************************************************************
void reset_altitude_measurement(void)
{

    // Clear timeout counter
    sAlt.timeout = 0;

    // Set default altitude value
    sAlt.altitude = 0;

    // Pressure sensor ok?
    if (ps_ok)
    {
        // Initialise pressure table
        init_pressure_table();

        // Do single conversion
        start_altitude_measurement();
        stop_altitude_measurement();

        // Apply calibration offset and recalculate pressure table
        if (sAlt.altitude_offset != 0)
        {
            sAlt.altitude += sAlt.altitude_offset;
            update_pressure_table(sAlt.altitude, sAlt.pressure, sAlt.temperature);
        }
    }
}

// *************************************************************************************************
// @fn          conv_m_to_ft
// @brief       Convert meters to feet
// @param       u16 m           Meters
// @return      u16                     Feet
// *************************************************************************************************
s16 convert_m_to_ft(s16 m)
{
    return (((s32) 328 * m) / 100);
}

// *************************************************************************************************
// @fn          conv_ft_to_m
// @brief       Convert feet to meters
// @param       u16 ft          Feet
// @return      u16                     Meters
// *************************************************************************************************
s16 convert_ft_to_m(s16 ft)
{
    return (((s32) ft * 61) / 200);
}

// *************************************************************************************************
// @fn          is_altitude_measurement
// @brief       Altitude measurement check
// @param       none
// @return      u8              1=Measurement ongoing, 0=measurement off
// *************************************************************************************************
u8 is_altitude_measurement(void)
{
    return (altitudeEnabled && (sAlt.timeout > 0));
}

// *************************************************************************************************
// @fn          start_altitude_measurement
// @brief       Start altitude measurement
// @param       none
// @return      none
// *************************************************************************************************
void start_altitude_measurement(void)
{
    // Show warning if pressure sensor was not initialised properly
    if (!ps_ok)
    {
        display_chars(0, LCD_SEG_L1_2_0, "ERR", SEG_ON);
        return;
    }

    // Start altitude measurement if timeout has elapsed
    if (sAlt.timeout == 0)
    {
        // Enable EOC IRQ on rising edge
    	PS_INT_IFG &= ~PS_INT_PIN;
    	PS_INT_IE |= PS_INT_PIN;

        // Start pressure sensor
    	if (bmp_used)
    	{
            bmp_ps_start();
    	}
    	else
    	{
            cma_ps_start();
    	}

        // Set timeout counter only if sensor status was OK
        sAlt.timeout = ALTITUDE_MEASUREMENT_TIMEOUT;

        // Get updated altitude
        while ((PS_INT_IN & PS_INT_PIN) == 0) ;
        do_altitude_measurement();
    }
}

// *************************************************************************************************
// @fn          stop_altitude_measurement
// @brief       Stop altitude measurement
// @param       none
// @return      none
// *************************************************************************************************
void stop_altitude_measurement(void)
{
    // Return if pressure sensor was not initialised properly
    if (!ps_ok)
        return;

    // Stop pressure sensor
	if (bmp_used)
	{
        bmp_ps_stop();
	}
	else
	{
        cma_ps_stop();
	}

    // Disable DRDY IRQ
    PS_INT_IE &= ~PS_INT_PIN;
    PS_INT_IFG &= ~PS_INT_PIN;

    // Clear timeout counter
    sAlt.timeout = 0;
}

// *************************************************************************************************
// @fn          do_altitude_measurement
// @brief       Perform single altitude measurement
// @param       u8 filter       Filter option
// @return      none
// *************************************************************************************************
void do_altitude_measurement()
{
    volatile u32 pressure;

    // If sensor is not ready, skip data read
    if ((PS_INT_IN & PS_INT_PIN) == 0)
        return;

    // Get temperature (format is *10 K) from sensor
	if (bmp_used)
	{
        sAlt.temperature = bmp_ps_get_temp();
	}
	else
	{
        sAlt.temperature = cma_ps_get_temp();
	}

    // Get pressure (format is 1Pa) from sensor
	if (bmp_used)
	{
	    // Start sampling data in ultra low power mode
	    bmp_ps_write_register(BMP_085_CTRL_MEAS_REG, BMP_085_P_MEASURE);
	    // Get updated altitude
	    while ((PS_INT_IN & PS_INT_PIN) == 0) ;

	    pressure = bmp_ps_get_pa();
	}
	else
	{
        pressure = cma_ps_get_pa();
	}

    // Store measured pressure value
		if(altitudeEnabled){
#ifdef CONFIG_MOD_ALTITUDE_FILTER
			pressure = (u32) ((pressure * 0.7) + (sAlt.pressure * 0.3));
#endif
		}
		sAlt.pressure = pressure;
        
    
    // Convert pressure (Pa) and temperature (K) to altitude (m)
    sAlt.altitude = conv_pa_to_meter(sAlt.pressure, sAlt.temperature);
}

void update(void)
{
	read_altitude();
	display_altitude(sAlt.altitude);
}

void read_altitude(void)
{
	// Start measurement
	start_altitude_measurement();
	stop_altitude_measurement();
}

// *************************************************************************************************
// @fn          display_altitude
// @brief       Display routine. Supports display in meters and feet.
// @param       u8 line                 LINE1
//                              u8 update               DISPLAY_LINE_UPDATE_FULL,
// DISPLAY_LINE_UPDATE_PARTIAL, DISPLAY_LINE_CLEAR
// @return      none
// *************************************************************************************************
void display_altitude(s16 alt)
{
    u8 *str;
    s16 ft;
	u16 value;

		
#ifdef CONFIG_MOD_ALTITUDE_METRIC
	// Display altitude in xxxx m format, allow 3 leading blank digits
	if (alt >= 0)
	{
		value = alt;
		//str = int_to_array(alt, 4, 3);
		display_symbol(0,LCD_SYMB_ARROW_UP, SEG_ON);
		display_symbol(0,LCD_SYMB_ARROW_DOWN, SEG_OFF);
	}
	else
	{
		value = alt * (-1);
		//str = int_to_array(alt, 4, 3);
		display_symbol(0,LCD_SYMB_ARROW_UP, SEG_OFF);
		display_symbol(0,LCD_SYMB_ARROW_DOWN, SEG_ON);
	}
	display_symbol(0, LCD_UNIT_L1_M, SEG_ON);
#else

	// Convert from meters to feet
	ft = convert_m_to_ft(alt);

	// Limit to 9999ft (3047m)
	if (ft > 9999)
		ft = 9999;

	// Display altitude in xxxx ft format, allow 3 leading blank digits
	if (ft >= 0)
	{
		value = ft;
		//str = int_to_array(ft, 4, 3);
		display_symbol(0, LCD_SYMB_ARROW_UP, SEG_ON);
		display_symbol(0, LCD_SYMB_ARROW_DOWN, SEG_OFF);
	}
	else
	{
		value = ft * -1;
		//str = int_to_array(ft, 4, 3);
		display_symbol(0, LCD_SYMB_ARROW_UP, SEG_OFF);
		display_symbol(0, LCD_SYMB_ARROW_DOWN, SEG_ON);
	}
	display_symbol(0, LCD_UNIT_L1_FT, SEG_ON);
#endif
	
	_printf(0, LCD_SEG_L1_3_0, "%4u", value);
	//display_chars(0, LCD_SEG_L1_3_0, str, SEG_SET);
}

/*

// Quick integer to array conversion table for most common integer values
const u8 int_to_array_conversion_table[][3] = {
    "000", "001", "002", "003", "004", "005", "006", "007", "008", "009", "010", "011", "012",
    "013", "014", "015",
    "016", "017", "018", "019", "020", "021", "022", "023", "024", "025", "026", "027", "028",
    "029", "030", "031",
    "032", "033", "034", "035", "036", "037", "038", "039", "040", "041", "042", "043", "044",
    "045", "046", "047",
    "048", "049", "050", "051", "052", "053", "054", "055", "056", "057", "058", "059", "060",
    "061", "062", "063",
    "064", "065", "066", "067", "068", "069", "070", "071", "072", "073", "074", "075", "076",
    "077", "078", "079",
    "080", "081", "082", "083", "084", "085", "086", "087", "088", "089", "090", "091", "092",
    "093", "094", "095",
    "096", "097", "098", "099", "100", "101", "102", "103", "104", "105", "106", "107", "108",
    "109", "110", "111",
    "112", "113", "114", "115", "116", "117", "118", "119", "120", "121", "122", "123", "124",
    "125", "126", "127",
    "128", "129", "130", "131", "132", "133", "134", "135", "136", "137", "138", "139", "140",
    "141", "142", "143",
    "144", "145", "146", "147", "148", "149", "150", "151", "152", "153", "154", "155", "156",
    "157", "158", "159",
    "160", "161", "162", "163", "164", "165", "166", "167", "168", "169", "170", "171", "172",
    "173", "174", "175",
    "176", "177", "178", "179", "180",
};
u8 int_to_array_str[8];
// *************************************************************************************************
// @fn          int_to_array
// @brief       Generic integer to array routine. Converts integer n to string.
//                              Default conversion result has leading zeros, e.g. "00123"
//                              Option to convert leading '0' into whitespace (blanks)
// @param       u32 n                   integer to convert
//                              u8 digits               number of digits
//                              u8 blanks               fill up result string with number of
// whitespaces instead of leading zeros
// @return      u8                              string
// *************************************************************************************************
u8 *int_to_array(u32 n, u8 digits, u8 blanks)
{
    u8 i;
    u8 digits1 = digits;

    // Preset result string
    memcpy(int_to_array_str, "0000000", 7);

    // Return empty string if number of digits is invalid (valid range for digits: 1-7)
    if ((digits == 0) || (digits > 7))
        return (int_to_array_str);

    // Numbers 0 .. 180 can be copied from int_to_array_conversion_table without conversion
    if (n <= 180)
    {
        if (digits >= 3)
        {
            memcpy(int_to_array_str + (digits - 3), int_to_array_conversion_table[n], 3);
        }
        else                    // digits == 1 || 2
        {
            memcpy(int_to_array_str, int_to_array_conversion_table[n] + (3 - digits), digits);
        }
    }
    else                        // For n > 180 need to calculate string content
    {
        // Calculate digits from least to most significant number
        do
        {
            int_to_array_str[digits - 1] = n % 10 + '0';
            n /= 10;
        }
        while (--digits > 0);
    }

    // Remove specified number of leading '0', always keep last one
    i = 0;
    while ((int_to_array_str[i] == '0') && (i < digits1 - 1))
    {
        if (blanks > 0)
        {
            // Convert only specified number of leading '0'
            int_to_array_str[i] = ' ';
            blanks--;
        }
        i++;
    }

    return (int_to_array_str);
}

*/



void edit_base1_sel(void)
{	
	display_altitude(baseCalib[0]);
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_ON);
	display_chars(0, LCD_SEG_L2_5_0, "BASE 1", SEG_ON);
}
void edit_base1_dsel(void)
{
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_OFF);
	display_clear(0,0);
}
void edit_base1_set(int8_t step)
{	
	helpers_loop_s16(&baseCalib[0], limit_low, limit_high, step);
	
	display_altitude(baseCalib[0]);

}



void edit_base2_sel(void)
{	
	display_altitude(baseCalib[1]);
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_ON);
	display_chars(0, LCD_SEG_L2_5_0, "BASE 2", SEG_ON);
}
void edit_base2_dsel(void)
{
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_OFF);
	display_clear(0,0);
}
void edit_base2_set(int8_t step)
{	
	helpers_loop_s16(&baseCalib[1], limit_low, limit_high, step);
	
	display_altitude(baseCalib[1]);

}



void edit_base3_sel(void)
{	
	display_altitude(baseCalib[2]);
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_ON);
	display_chars(0, LCD_SEG_L2_5_0, "BASE 3", SEG_ON);
}
void edit_base3_dsel(void)
{
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_OFF);
	display_clear(0,0);
}
void edit_base3_set(int8_t step)
{	
	helpers_loop_s16(&baseCalib[2], limit_low, limit_high, step);
	
	display_altitude(baseCalib[2]);

}



void edit_base4_sel(void)
{	
	display_altitude(baseCalib[3]);
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_ON);
	display_chars(0, LCD_SEG_L2_5_0, "BASE 4", SEG_ON);
}
void edit_base4_dsel(void)
{
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_OFF);
	display_clear(0,0);
}
void edit_base4_set(int8_t step)
{	
	helpers_loop_s16(&baseCalib[3], limit_low, limit_high, step);
	
	display_altitude(baseCalib[3]);

}


void edit_base5_sel(void)
{	
	display_altitude(baseCalib[4]);
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_ON);
	display_chars(0, LCD_SEG_L2_5_0, "BASE 5", SEG_ON);
}
void edit_base5_dsel(void)
{
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_OFF);
	display_clear(0,0);
}
void edit_base5_set(int8_t step)
{	
	helpers_loop_s16(&baseCalib[4], limit_low, limit_high, step);
	
	display_altitude(baseCalib[4]);

}


static void edit_save()
{
	/*
#ifndef CONFIG_MOD_ALTITUDE_METRIC
	 // When using English units, convert ft back to m before updating pressure table
	altitudeCalib = convert_ft_to_m(altitudeCalib);
#endif
	
	sAlt.altitude = altitudeCalib;
	update_pressure_table(sAlt.altitude, sAlt.pressure, sAlt.temperature);
*/
	sys_messagebus_register(&update, SYS_MSG_TIMER_4S);
	update();
}

static struct menu_editmode_item edit_items[] = {
	{&edit_base1_sel, &edit_base1_dsel, &edit_base1_set},
	{&edit_base2_sel, &edit_base2_dsel, &edit_base2_set},
	{&edit_base3_sel, &edit_base3_dsel, &edit_base3_set},
	{&edit_base4_sel, &edit_base4_dsel, &edit_base4_set},
	{&edit_base5_sel, &edit_base5_dsel, &edit_base5_set},
	{ NULL },
};



void edit_mode_callback(void)
{
	sys_messagebus_unregister(&update);
	menu_editmode_start(&edit_save, edit_items);
}


void calib_callback(void)
{
	if(submenuState != 0){

#ifndef CONFIG_MOD_ALTITUDE_METRIC
		// When using English units, convert ft back to m before updating pressure table
		sAlt.altitude = convert_ft_to_m(baseCalib[submenuState -1]);
#else
		sAlt.altitude = baseCalib[submenuState -1];
#endif
		
		update_pressure_table(sAlt.altitude, sAlt.pressure, sAlt.temperature);
		buzzer_play(bip);
		update();
		submenuState = 0;
		display_clear(0,2);
	}
}

void submenu_callback(void)
{
	submenuState++;
	if(submenuState == 6) submenuState = 0;
	
	switch(submenuState){
		case 0: display_clear(0,2);
				break;
		case 1: display_chars(0, LCD_SEG_L2_5_0, "BASE 1", SEG_SET);
				break;
		case 2: display_chars(0, LCD_SEG_L2_5_0, "BASE 2", SEG_SET);
				break;
		case 3: display_chars(0, LCD_SEG_L2_5_0, "BASE 3", SEG_SET);
				break;
		case 4: display_chars(0, LCD_SEG_L2_5_0, "BASE 4", SEG_SET);
				break;
		case 5: display_chars(0, LCD_SEG_L2_5_0, "BASE 5", SEG_SET);
				break;
	}
		
}
