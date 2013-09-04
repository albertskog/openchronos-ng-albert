/*
    altitude.c: altitude display module. Based on TI firmware 1.8

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
uint8_t consumption = CONFIG_MOD_ALTITUDE_CONSUMPTION;
int16_t consumption_array[4] = {
	SYS_MSG_RTC_MINUTE,
	SYS_MSG_TIMER_4S,
	SYS_MSG_RTC_SECOND,
	SYS_MSG_TIMER_20HZ
};

#ifdef CONFIG_MOD_ALTITUDE_METRIC
uint8_t useMetric = 1;
#else
uint8_t useMetric = 0;
#endif

#ifdef CONFIG_MOD_ALTITUDE_FILTER
uint8_t useFilter = 1;
#else
uint8_t useFilter = 0;
#endif


s16 oldAccuAltitude;

static void altitude_activate(void)
{
	altitudeEnabled = 1;
	/* display -- symbol while a measure is not performed */
	display_chars(0, LCD_SEG_L1_3_0, "----", SEG_SET);

	//sys_messagebus_register(&update, SYS_MSG_RTC_SECOND);
	sys_messagebus_register(&update, consumption_array[consumption-1]);
    
    lcd_screens_create(5);
    display_chars(1, LCD_SEG_L2_5_0, " MIN  ", SEG_SET);
    display_chars(2, LCD_SEG_L2_5_0, " MAX  ", SEG_SET);
    display_chars(3, LCD_SEG_L2_5_0, " ACC N", SEG_SET);
    display_chars(4, LCD_SEG_L2_5_0, " ACC P", SEG_SET);
}

static void altitude_deactivate(void)
{
	sys_messagebus_unregister(&update);
	altitudeEnabled = 0;
	
	
	// Clean up function-specific segments before leaving function
		
	display_symbol(0, LCD_SYMB_ARROW_UP, SEG_OFF);
	display_symbol(0, LCD_SYMB_ARROW_DOWN, SEG_OFF);
        
	if(useMetric){
		display_symbol(0, LCD_UNIT_L1_M, SEG_OFF);
	}else{
		display_symbol(0, LCD_UNIT_L1_FT, SEG_OFF);
	}
	
	
	update_pressure_table((s16) sAlt.altitude, sAlt.pressure, sAlt.temperature);
    
    lcd_screens_destroy();
}


void mod_altitude_init(void)
{
	menu_add_entry(" ALTI", &up_callback, NULL,
		&submenu_callback, &edit_mode_callback, &calib_callback, NULL,
		&altitude_activate, &altitude_deactivate);
	
	altitudeEnabled = 0;
	sAlt.pressure = 0;
	reset_altitude_measurement();
	
// Set lower and upper limits for offset correction
	if(useMetric){
		// Limits for set_value function
		limit_low = -100;
		limit_high = 4000;
	}else{
// Limits for set_value function
		limit_low = -500;
		limit_high = 9999;
	}
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
	sAlt.maxAltitude = 0;
	sAlt.minAltitude = 0;
	sAlt.accuClimbUp = 0;
	sAlt.accuClimbDown = 0;
	oldAccuAltitude = 0;

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
        display_chars(0, LCD_SEG_L1_2_0, "ERR", SEG_SET);
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
			if(useFilter){
				pressure = (u32) ((pressure * 0.7) + (sAlt.pressure * 0.3));
			}
		}
		sAlt.pressure = pressure;
        
    
    // Convert pressure (Pa) and temperature (K) to altitude (m)
    sAlt.altitude = conv_pa_to_meter(sAlt.pressure, sAlt.temperature);
	
	if(sAlt.altitude > sAlt.maxAltitude){
		sAlt.maxAltitude = sAlt.altitude;
	}
	
	if(sAlt.altitude < sAlt.minAltitude){
		sAlt.minAltitude = sAlt.altitude;
	}
	
	if((sAlt.altitude > oldAccuAltitude) && (sAlt.altitude - oldAccuAltitude > 1)){
		sAlt.accuClimbUp += sAlt.altitude - oldAccuAltitude;
		oldAccuAltitude = sAlt.altitude;
	}
	
	if((sAlt.altitude < oldAccuAltitude) && (oldAccuAltitude - sAlt.altitude > 1)){
		sAlt.accuClimbDown -= oldAccuAltitude - sAlt.altitude;
		oldAccuAltitude = sAlt.altitude;
	}
	
	if(sAlt.altitude < sAlt.minAltitude){
        sAlt.minAltitude = sAlt.altitude;
    }
    
    if(sAlt.altitude > sAlt.maxAltitude){
        sAlt.maxAltitude = sAlt.altitude;
    }
}

void update(void)
{
	read_altitude();
    
	display_altitude(sAlt.altitude, 0);
    
    display_altitude(sAlt.minAltitude, 1);
	display_altitude(sAlt.maxAltitude, 2);
    
    display_altitude(sAlt.accuClimbDown, 3);
    display_altitude(sAlt.accuClimbUp, 4);
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
void display_altitude(s16 alt, uint8_t scr)
{
    s16 ft;
	u16 value;

		
	if(useMetric){
		// Display altitude in xxxx m format, allow 3 leading blank digits
		if (alt >= 0)
		{
			value = alt;
			display_symbol(scr, LCD_SYMB_ARROW_UP, SEG_ON);
			display_symbol(scr, LCD_SYMB_ARROW_DOWN, SEG_OFF);
		}
		else
		{
			value = alt * (-1);
			display_symbol(scr, LCD_SYMB_ARROW_UP, SEG_OFF);
			display_symbol(scr, LCD_SYMB_ARROW_DOWN, SEG_ON);
		}
		display_symbol(scr, LCD_UNIT_L1_M, SEG_ON);
	}else{

		// Convert from meters to feet
		ft = convert_m_to_ft(alt);

		// Limit to 9999ft (3047m)
		if (ft > 9999)
			ft = 9999;

		// Display altitude in xxxx ft format, allow 3 leading blank digits
		if (ft >= 0)
		{
			value = ft;
			display_symbol(scr, LCD_SYMB_ARROW_UP, SEG_ON);
			display_symbol(scr, LCD_SYMB_ARROW_DOWN, SEG_OFF);
		}
		else
		{
			value = ft * -1;
			display_symbol(scr, LCD_SYMB_ARROW_UP, SEG_OFF);
			display_symbol(scr, LCD_SYMB_ARROW_DOWN, SEG_ON);
		}
		display_symbol(scr, LCD_UNIT_L1_FT, SEG_ON);
	}
	
	_printf(scr, LCD_SEG_L1_3_0, "%4u", value);
}




void edit_base1_sel(void)
{	
	display_altitude(baseCalib[0], 0);
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_ON);
	display_chars(0, LCD_SEG_L2_5_0, " PRE 1", SEG_SET);
}
void edit_base1_dsel(void)
{
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_OFF);
	display_clear(0, 0);
}
void edit_base1_set(int8_t step)
{	
	helpers_loop_s16(&baseCalib[0], limit_low, limit_high, step);
	
	display_altitude(baseCalib[0], 0);

}



void edit_base2_sel(void)
{	
	display_altitude(baseCalib[1], 0);
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_ON);
	display_chars(0, LCD_SEG_L2_5_0, " PRE 2", SEG_SET);
}
void edit_base2_dsel(void)
{
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_OFF);
	display_clear(0, 0);
}
void edit_base2_set(int8_t step)
{	
	helpers_loop_s16(&baseCalib[1], limit_low, limit_high, step);
	
	display_altitude(baseCalib[1], 0);

}



void edit_base3_sel(void)
{	
	display_altitude(baseCalib[2], 0);
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_ON);
	display_chars(0, LCD_SEG_L2_5_0, " PRE 3", SEG_SET);
}
void edit_base3_dsel(void)
{
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_OFF);
	display_clear(0, 0);
}
void edit_base3_set(int8_t step)
{	
	helpers_loop_s16(&baseCalib[2], limit_low, limit_high, step);
	
	display_altitude(baseCalib[2], 0);

}



void edit_base4_sel(void)
{	
	display_altitude(baseCalib[3], 0);
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_ON);
	display_chars(0, LCD_SEG_L2_5_0, " PRE 4", SEG_SET);
}
void edit_base4_dsel(void)
{
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_OFF);
	display_clear(0, 0);
}
void edit_base4_set(int8_t step)
{	
	helpers_loop_s16(&baseCalib[3], limit_low, limit_high, step);
	
	display_altitude(baseCalib[3], 0);

}


void edit_base5_sel(void)
{	
	display_altitude(baseCalib[4], 0);
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_ON);
	display_chars(0, LCD_SEG_L2_5_0, " PRE 5", SEG_SET);
}
void edit_base5_dsel(void)
{
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_OFF);
	display_clear(0, 0);
}
void edit_base5_set(int8_t step)
{	
	helpers_loop_s16(&baseCalib[4], limit_low, limit_high, step);
	
	display_altitude(baseCalib[4], 0);

}


void edit_consumption_sel(void)
{	
	
	_printf(0, LCD_SEG_L1_1_0, "%1u", consumption);
	display_chars(0, LCD_SEG_L1_1_0, NULL, BLINK_ON);
	display_chars(0, LCD_SEG_L2_4_0, "BATT", SEG_SET);
}
void edit_consumption_dsel(void)
{
	display_chars(0, LCD_SEG_L1_1_0, NULL, BLINK_OFF);
	display_clear(0, 0);
}
void edit_consumption_set(int8_t step)
{	
	helpers_loop_s16(&consumption, 1, 4, step);
	_printf(0, LCD_SEG_L1_1_0, "%1u", consumption);
}


void edit_unit_sel(void)
{	
	if(useMetric){
		display_chars(0, LCD_SEG_L1_2_1, "M", SEG_SET);
		display_symbol(0, LCD_UNIT_L1_M, SEG_ON);
		display_symbol(0, LCD_UNIT_L1_FT, SEG_OFF);
	}else{
		display_chars(0, LCD_SEG_L1_2_1, "FT", SEG_SET);
		display_symbol(0, LCD_UNIT_L1_M, SEG_OFF);
		display_symbol(0, LCD_UNIT_L1_FT, SEG_ON);
	}
	display_chars(0, LCD_SEG_L1_2_1, NULL, BLINK_ON);
	display_chars(0, LCD_SEG_L2_3_0, "UNIT", SEG_SET);
}
void edit_unit_dsel(void)
{
	display_chars(0, LCD_SEG_L1_2_1, NULL, BLINK_OFF);
	display_clear(0,0);
}
void edit_unit_set(int8_t step)
{
	if(useMetric){
		useMetric = 0;
		display_chars(0, LCD_SEG_L1_2_1, "FT", SEG_SET);
		display_symbol(0, LCD_UNIT_L1_M, SEG_OFF);
		display_symbol(0, LCD_UNIT_L1_FT, SEG_ON);
	}else{
		useMetric = 1;
		display_clear(0,1);
		display_chars(0, LCD_SEG_L1_2_1, "M", SEG_SET);
		display_symbol(0, LCD_UNIT_L1_M, SEG_ON);
		display_symbol(0, LCD_UNIT_L1_FT, SEG_OFF);
	}
}


void edit_filter_sel(void)
{	
	if(useFilter){
		display_chars(0, LCD_SEG_L1_2_0, "OFF", SEG_SET);
	}else{
		display_chars(0, LCD_SEG_L1_2_1, "ON", SEG_SET);
	}
	display_chars(0, LCD_SEG_L1_2_0, NULL, BLINK_ON);
	display_chars(0, LCD_SEG_L2_4_0, "FLT", SEG_SET);
}
void edit_filter_dsel(void)
{
	display_chars(0, LCD_SEG_L1_2_0, NULL, BLINK_OFF);
	display_clear(0,0);
}
void edit_filter_set(int8_t step)
{
	if(useFilter){
		useFilter = 0;
		display_chars(0, LCD_SEG_L1_2_0, "OFF", SEG_SET);
	}else{
		useFilter = 1;
		display_clear(0,1);
		display_chars(0, LCD_SEG_L1_2_1, "ON", SEG_SET);
	}
}

static void edit_save()
{
	sys_messagebus_register(&update, consumption_array[consumption-1]);
	update();
}

static struct menu_editmode_item edit_items[] = {
	{&edit_base1_sel, &edit_base1_dsel, &edit_base1_set},
	{&edit_base2_sel, &edit_base2_dsel, &edit_base2_set},
	{&edit_base3_sel, &edit_base3_dsel, &edit_base3_set},
	{&edit_base4_sel, &edit_base4_dsel, &edit_base4_set},
	{&edit_base5_sel, &edit_base5_dsel, &edit_base5_set},
	{&edit_consumption_sel, &edit_consumption_dsel, &edit_consumption_set},
	{&edit_unit_sel, &edit_unit_dsel, &edit_unit_set},
	{&edit_filter_sel, &edit_filter_dsel, &edit_filter_set},
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

		if(useMetric){
			sAlt.altitude = baseCalib[submenuState -1];
		}else{
			// When using English units, convert ft back to m before updating pressure table
			sAlt.altitude = convert_ft_to_m(baseCalib[submenuState -1]);
		}
		
			sAlt.maxAltitude = sAlt.altitude;
			sAlt.minAltitude = sAlt.altitude;
			
			oldAccuAltitude = sAlt.altitude;
			sAlt.accuClimbUp = 0;
			sAlt.accuClimbDown = 0;
			
			update_pressure_table(sAlt.altitude, sAlt.pressure, sAlt.temperature);
			buzzer_play(bip);
			update();
			submenuState = 0;
			display_clear(0, 2);
	}
}

void submenu_callback(void)
{
	submenuState++;
	if(submenuState == 6) submenuState = 0;
	
	switch(submenuState){
		case 0: display_clear(0 ,2);
				break;
		case 1: display_chars(0, LCD_SEG_L2_5_0, " PRE 1", SEG_SET);
				break;
		case 2: display_chars(0, LCD_SEG_L2_5_0, " PRE 2", SEG_SET);
				break;
		case 3: display_chars(0, LCD_SEG_L2_5_0, " PRE 3", SEG_SET);
				break;
		case 4: display_chars(0, LCD_SEG_L2_5_0, " PRE 4", SEG_SET);
				break;
		case 5: display_chars(0, LCD_SEG_L2_5_0, " PRE 5", SEG_SET);
				break;
	}
		
}


void up_callback(void)
{
    lcd_screen_activate(0xff);
    //sys_messagebus_unregister(&screenTimeout);
    //sys_messagebus_register(&screenTimeout, SYS_MSG_TIMER_4S);
}

void screenTimeout(void)
{
    lcd_screen_activate(0);
}

/*
void down_callback(void)
{
    
}
*/
