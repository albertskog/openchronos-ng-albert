#ifndef LIBS_ALTITUDE_H_
#define LIBS_ALTITUDE_H_


#define ALTITUDE_MEASUREMENT_TIMEOUT    (60 * 60u) // Stop altitude measurement after 60 minutes to
                                                   // save battery

extern uint8_t useFilter;
extern uint8_t altPowerMode;
extern void reset_altitude_measurement(void);
extern void start_altitude_measurement(void);
extern void stop_altitude_measurement(void);
extern void do_altitude_measurement(void);

extern void set_altitude_calibration(int16_t cal);

extern int16_t convert_ft_to_m(int16_t ft);
extern int16_t convert_m_to_ft(int16_t m);

#define ALT_HISTORY_LEN (40)	// History length. Needs to be a multiple of two.

int16_t oldAccuAltitude;
struct alt
{
    uint32_t pressure;                                  // Pressure (Pa)
    uint32_t first_pressure;                            // First pressure measurement.
    uint16_t temperature;                               // Temperature (K)
    int16_t raw_altitude;                              // Altitude (m)
    int16_t raw_maxAltitude;
    int16_t raw_minAltitude;
    int16_t altitude;                                  // Altitude (m) minus calib delta
    int16_t altitude_offset;                           // Altitude offset stored during calibration
    int16_t altitude_calib;                            // Altitude calibration value
    uint16_t timeout;                                   // Timeout
    int16_t maxAltitude;
    int16_t minAltitude;
    uint8_t accu_threshold;
    int16_t accuClimbUp;
    int16_t accuClimbDown;

    int16_t climb;                                      // Current climb. The unit is
                                                        // Pa / (s * ALT_HISTORY_LEN/2 * ALT_HISTORY_LEN/2 * 1/f)
                                                        // where f is the update frequency
    int16_t history[ALT_HISTORY_LEN];                   // Pressure history values (ring buffer, difference
                                                        // in Pa between measurement and first_pressure)
    uint8_t history_pos;                                // Position in Ring buffer
};
extern struct alt sAlt;

#endif            
