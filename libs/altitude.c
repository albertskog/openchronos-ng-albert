#include <openchronos.h>

#include "libs/altitude.h"
#include "drivers/ps.h"
#include "drivers/cma_ps.h"
#include "drivers/bmp_ps.h"


#ifdef CONFIG_MOD_ALTITUDE_FILTER
uint8_t useFilter = 1;
#else
uint8_t useFilter = 0;
#endif

uint8_t altPowerMode = (CONFIG_MOD_ALTITUDE_POWER-1);

struct alt sAlt;

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
    
    // Pressure sensor ok?
    if (ps_ok)
    {
        // Initialise pressure table
        init_pressure_table();

        // Do single conversion
        start_altitude_measurement();
        stop_altitude_measurement();
        
        sAlt.accu_threshold = CONFIG_MOD_ALTITUDE_ACCU_THRESHOLD;
        
        sAlt.altitude_calib =  sAlt.raw_altitude;
        sAlt.altitude_offset = sAlt.raw_altitude - sAlt.altitude_calib;
        
        sAlt.raw_minAltitude = sAlt.raw_altitude;
        sAlt.raw_maxAltitude = sAlt.raw_altitude;
     
        oldAccuAltitude = sAlt.raw_altitude;
        sAlt.accuClimbDown = 0;
        sAlt.accuClimbUp = 0;
        
        
        sAlt.minAltitude = sAlt.raw_minAltitude - sAlt.altitude_offset;
        sAlt.maxAltitude = sAlt.raw_maxAltitude - sAlt.altitude_offset;
        sAlt.altitude = sAlt.raw_altitude - sAlt.altitude_offset;

        sAlt.first_pressure = sAlt.pressure;
        sAlt.climb = 0;
        for (sAlt.history_pos = 0; sAlt.history_pos < ALT_HISTORY_LEN; sAlt.history_pos++) {
                sAlt.history[sAlt.history_pos] = 0;
        }
        sAlt.history_pos = 0;
}
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
    
    //TODO libs should not display errors on their own
    /*
    if (!ps_ok)
    {
        display_chars(0, LCD_SEG_L1_2_0, "ERR", SEG_SET);
        return;
    }
    */

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
    volatile uint32_t pressure;

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
        // Start sampling data in configurable power mode
        bmp_ps_write_register(BMP_085_CTRL_MEAS_REG, BMP_085_P_MEASURE + (altPowerMode << 6));
        // Get updated altitude
        while ((PS_INT_IN & PS_INT_PIN) == 0) ;

        pressure = bmp_ps_get_pa();
    }
    else
    {
        pressure = cma_ps_get_pa();
    }

    // Store measured pressure value
    if(useFilter && sAlt.pressure){
            // Filter only if there was a previous value (to get a good first value)
            pressure = (u32) ((pressure * 0.7) + (sAlt.pressure * 0.3));
    }
    sAlt.pressure = pressure;
        
    
    // Convert pressure (Pa) and temperature (K) to altitude (m)
    sAlt.raw_altitude = conv_pa_to_meter(sAlt.pressure, sAlt.temperature);
    
    if(sAlt.raw_altitude > sAlt.raw_maxAltitude){
        sAlt.raw_maxAltitude = sAlt.raw_altitude;
    }
    
    if(sAlt.raw_altitude < sAlt.raw_minAltitude){
        sAlt.raw_minAltitude = sAlt.raw_altitude;
    }
    
    if((sAlt.raw_altitude > oldAccuAltitude) && (sAlt.raw_altitude - oldAccuAltitude > sAlt.accu_threshold)){
        sAlt.accuClimbUp += sAlt.raw_altitude - oldAccuAltitude;
        oldAccuAltitude = sAlt.raw_altitude;
    }
    
    if((sAlt.raw_altitude < oldAccuAltitude) && (oldAccuAltitude - sAlt.raw_altitude > sAlt.accu_threshold)){
        sAlt.accuClimbDown -= oldAccuAltitude - sAlt.raw_altitude;
        oldAccuAltitude = sAlt.raw_altitude;
    }

    
    sAlt.minAltitude = sAlt.raw_minAltitude - sAlt.altitude_offset;
    sAlt.maxAltitude = sAlt.raw_maxAltitude - sAlt.altitude_offset;
    sAlt.altitude = sAlt.raw_altitude - sAlt.altitude_offset;

    // Update climb value. Calibration is irrelevant, as it is just a constant sum. 
    // We subtract first_pressure from the reading to get the numbers down to something
    // that fits in 16 bits.
    sAlt.history_pos = (sAlt.history_pos + 1) % ALT_HISTORY_LEN;
    // Remove oldest value
    sAlt.climb += sAlt.history[sAlt.history_pos];
    // Add newest value
    sAlt.history[sAlt.history_pos] = (pressure - sAlt.first_pressure);
    sAlt.climb += sAlt.history[sAlt.history_pos];
    // The time half a history ago now affects the climb with a different sign,
    // so subtract twice
    sAlt.climb -= 2 * sAlt.history[(sAlt.history_pos + ALT_HISTORY_LEN/2) % ALT_HISTORY_LEN];
}


void set_altitude_calibration(int16_t cal)
{
    sAlt.altitude_calib = cal;
    sAlt.altitude_offset = sAlt.raw_altitude - sAlt.altitude_calib;
    
    sAlt.raw_minAltitude = sAlt.raw_altitude;
    sAlt.raw_maxAltitude = sAlt.raw_altitude;
    sAlt.minAltitude = sAlt.raw_minAltitude - sAlt.altitude_offset;
    sAlt.maxAltitude = sAlt.raw_maxAltitude - sAlt.altitude_offset;
    
    oldAccuAltitude = sAlt.raw_altitude;
    sAlt.accuClimbUp = 0;
    sAlt.accuClimbDown = 0;
}


// *************************************************************************************************
// @fn          conv_m_to_ft
// @brief       Convert meters to feet
// @param       u16 m           Meters
// @return      u16                     Feet
// *************************************************************************************************
int16_t convert_m_to_ft(int16_t m)
{
    return (((s32) 328 * m) / 100);
}

// *************************************************************************************************
// @fn          conv_ft_to_m
// @brief       Convert feet to meters
// @param       u16 ft          Feet
// @return      u16                     Meters
// *************************************************************************************************
int16_t convert_ft_to_m(int16_t ft)
{
    return (((s32) ft * 61) / 200);
}
