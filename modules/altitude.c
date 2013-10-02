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
#include "drivers/ports.h"
#include "drivers/timer.h"
#include <drivers/rtca.h>
#include "libs/altitude.h"
#include "libs/buzzer.h"
#include "modules/altitude.h"

#include <string.h>




// Global Variable section




int16_t baseCalib[5] = {CONFIG_MOD_ALTITUDE_BASE1,
	CONFIG_MOD_ALTITUDE_BASE2,
	CONFIG_MOD_ALTITUDE_BASE3,
	CONFIG_MOD_ALTITUDE_BASE4,
	CONFIG_MOD_ALTITUDE_BASE5
};

int32_t limit_high, limit_low;
uint8_t submenuState = 0;
uint8_t accelerometer = 0;
uint8_t consumption = CONFIG_MOD_ALTITUDE_CONSUMPTION;
int16_t consumption_array[4] = {
	SYS_MSG_RTC_MINUTE,
	SYS_MSG_TIMER_4S,
	SYS_MSG_RTC_SECOND,
	SYS_MSG_TIMER_20HZ
};
char *consumption_str[4] = {
	"1MIN",
	" 4 S",
	" 1 S",
	"  20"
};

#ifdef CONFIG_MOD_ALTITUDE_METRIC
uint8_t useMetric = 1;
#else
uint8_t useMetric = 0;
#endif



static void altitude_activate(void)
{

	/* display -- symbol while a measure is not performed */
	display_chars(0, LCD_SEG_L1_3_0, "----", SEG_SET);
    update(SYS_MSG_FAKE);

	sys_messagebus_register(&update, consumption_array[consumption-1]);
    
    sys_messagebus_register(&time_callback, SYS_MSG_RTC_MINUTE
                        | SYS_MSG_RTC_HOUR
#ifdef CONFIG_MOD_CLOCK_BLINKCOL
                        | SYS_MSG_RTC_SECOND
#endif
    );
    
    lcd_screens_create(6);
    display_chars(1, LCD_SEG_L2_5_0, " MIN  ", SEG_SET);
    display_chars(2, LCD_SEG_L2_5_0, " MAX  ", SEG_SET);
    display_chars(3, LCD_SEG_L2_5_0, " ACC N", SEG_SET);
    display_chars(4, LCD_SEG_L2_5_0, " ACC P", SEG_SET);
}

static void altitude_deactivate(void)
{
	sys_messagebus_unregister(&update);
    sys_messagebus_unregister(&time_callback);
	
	
	// Clean up function-specific segments before leaving function
		
	display_symbol(0, LCD_SYMB_ARROW_UP, SEG_OFF);
	display_symbol(0, LCD_SYMB_ARROW_DOWN, SEG_OFF);
        
	if(useMetric){
		display_symbol(0, LCD_UNIT_L1_M, SEG_OFF);
	}else{
		display_symbol(0, LCD_UNIT_L1_FT, SEG_OFF);
	}
	
	
	//update_pressure_table((s16) sAlt.raw_altitude, sAlt.pressure, sAlt.temperature);
    
    lcd_screens_destroy();
}


void mod_altitude_init(void)
{
#if defined CONFIG_PRESSURE_BUILD_BOSCH_PS || defined CONFIG_PRESSURE_BUILD_VTI_PS
    
	menu_add_entry(" ALTI", &up_callback, &down_callback,
		&submenu_callback, &edit_mode_callback, &calib_callback, NULL,
		&altitude_activate, &altitude_deactivate);
	
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
	
#endif
}

// *************************************************************************************************
// Extern section


void update(enum sys_message msg)
{
	read_altitude();
    
	display_altitude(sAlt.altitude, 0);
    
    display_altitude(sAlt.minAltitude, 1);
	display_altitude(sAlt.maxAltitude, 2);
    
    display_altitude(sAlt.accuClimbDown, 3);
    display_altitude(sAlt.accuClimbUp, 4);
    
    if((accelerometer == 1) &&(submenuState == 0)){
        display_clear(0 ,2);
        display_chars(0, LCD_SEG_L2_4_0, "ACCEL", SEG_SET);
    }
    
    time_callback(SYS_MSG_RTC_HOUR  | SYS_MSG_RTC_MINUTE);
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
void display_altitude(int16_t alt, uint8_t scr)
{
    int16_t ft;
	uint16_t value;

		
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
	display_chars(0, LCD_SEG_L1_3_0, consumption_str[consumption-1], SEG_SET);
	display_symbol(0, LCD_UNIT_L1_PER_S, consumption == 4 ? SEG_ON : SEG_OFF);
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_ON);
	display_chars(0, LCD_SEG_L2_3_0, "POLL", SEG_SET);
}
void edit_consumption_dsel(void)
{
    display_symbol(0, LCD_UNIT_L1_PER_S, SEG_OFF);
	display_chars(0, LCD_SEG_L1_3_0, NULL, BLINK_OFF);
	display_clear(0, 0);
}
void edit_consumption_set(int8_t step)
{	
	helpers_loop(&consumption, 1, 4, step);
    
	display_chars(0, LCD_SEG_L1_3_0, consumption_str[consumption-1], SEG_SET);
	display_symbol(0, LCD_UNIT_L1_PER_S, consumption == 4 ? SEG_ON : SEG_OFF);
}

void edit_threshold_sel(void)
{   
    
    _printf(0, LCD_SEG_L1_1_0, "%1u", sAlt.accu_threshold);
    display_chars(0, LCD_SEG_L1_1_0, NULL, BLINK_ON);
    display_chars(0, LCD_SEG_L2_4_0, "THRES", SEG_SET);
}
void edit_threshold_dsel(void)
{
    display_chars(0, LCD_SEG_L1_1_0, NULL, BLINK_OFF);
    display_clear(0, 0);
}
void edit_threshold_set(int8_t step)
{   
    helpers_loop(&sAlt.accu_threshold, 0, 9, step);
    _printf(0, LCD_SEG_L1_1_0, "%1u", sAlt.accu_threshold);
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
		display_chars(0, LCD_SEG_L1_2_1, "ON", SEG_SET);
	}else{
		display_chars(0, LCD_SEG_L1_2_0, "OFF", SEG_SET);
	}
	display_chars(0, LCD_SEG_L1_2_0, NULL, BLINK_ON);
	display_chars(0, LCD_SEG_L2_2_0, "FLT", SEG_SET);
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
    
    sys_messagebus_register(&time_callback, SYS_MSG_RTC_MINUTE
                        | SYS_MSG_RTC_HOUR
#ifdef CONFIG_MOD_CLOCK_BLINKCOL
                        | SYS_MSG_RTC_SECOND
#endif
    );
    
	update(SYS_MSG_FAKE);
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
    {&edit_threshold_sel, &edit_threshold_dsel, &edit_threshold_set},
	{ NULL },
};



void edit_mode_callback(void)
{
    lcd_screen_activate(0);
	sys_messagebus_unregister(&update);
    sys_messagebus_unregister(&time_callback);
    display_symbol(0, LCD_SEG_L2_COL0, SEG_OFF);
	menu_editmode_start(&edit_save, edit_items);
}


void calib_callback(void)
{
	if(submenuState != 0){
        
        set_altitude_calibration(baseCalib[submenuState -1]);

        //update_pressure_table(sAlt.raw_altitude, sAlt.pressure, sAlt.temperature);    
        buzzer_shortBip();
        display_clear(0, 2); 
        display_chars(0, LCD_SEG_L2_5_0, NULL, BLINK_OFF);
        submenuState = 0;
        update(SYS_MSG_FAKE);
        
	}
}

void submenu_callback(void)
{
    lcd_screen_activate(0);
    
	submenuState++;
	if(submenuState == 6) submenuState = 0;
    
    display_symbol(0, LCD_SEG_L2_COL0, SEG_OFF);
	
	switch(submenuState){
		case 0: display_clear(0 ,2);
                display_chars(0, LCD_SEG_L2_5_0, NULL, BLINK_OFF);
				break;
		case 1: display_chars(0, LCD_SEG_L2_5_0, " PRE 1", SEG_SET);
                display_chars(0, LCD_SEG_L2_5_0, NULL, BLINK_ON);
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
	
	update(SYS_MSG_FAKE);
}


void up_callback(void)
{
    if(lcd_screen_currentscreen() == 4){
        lcd_screen_activate(0);
    }else{
        lcd_screen_activate(0xff);
    }
    display_symbol(lcd_screen_currentscreen(), LCD_SEG_L2_COL0, SEG_OFF);
    //sys_messagebus_unregister(&screenTimeout);
    //sys_messagebus_register(&screenTimeout, SYS_MSG_TIMER_4S);
}

void screenTimeout(void)
{
    lcd_screen_activate(0);
}

void time_callback(enum sys_message msg)
{
    
    if((submenuState == 0) && (accelerometer == 0)){

#ifdef CONFIG_MOD_CLOCK_BLINKCOL
        display_symbol(0, LCD_SEG_L2_COL0,
            ((rtca_time.sec & 0x01) ? SEG_ON : SEG_OFF));
#endif

        if (msg & SYS_MSG_RTC_HOUR) {
#ifdef CONFIG_MOD_CLOCK_AMPM
            uint8_t tmp_hh = rtca_time.hour;
            if (tmp_hh > 12) {
                tmp_hh -= 12;
            } else if(tmp_hh == 0) {
                tmp_hh = 12;
            }
            _printf(0, LCD_SEG_L2_4_2, " %2u", tmp_hh);
#else
            _printf(0, LCD_SEG_L2_4_2, " %02u", rtca_time.hour);
#endif
        }
        
        if (msg & SYS_MSG_RTC_MINUTE)
            _printf(0, LCD_SEG_L2_1_0, "%02u", rtca_time.min);
    }
}

void down_callback(void)
{
//     if(accelerometer == 0){
//         accelerometer = 1;
//     }
//     else{
//         accelerometer = 0;
//     }

    update(SYS_MSG_FAKE);
}
