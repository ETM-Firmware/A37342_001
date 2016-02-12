/* 
 * File:   A36926_001_SETTINGS.h
 * Author: hwanetick
 *
 * Created on October 22, 2015, 11:08 AM
 */

#ifndef __A36926_001_SETTINGS_H
#define	__A36926_001_SETTINGS_H


// Configuration Settings for the Electromagnet Supply
#define ELECTROMAGNET_MAX_IPROG                 22000
#define ELECTROMAGNET_MIN_IPROG                 9500

#define ELECTROMAGNET_CURRENT_OVER_TRIP         24000                                 // 24 Amps
#define ELECTROMAGNET_CURRENT_UNDER_TRIP        8000                                  // 8 Amps
#define ELECTROMAGNET_CURRENT_RELATIVE_TRIP     MACRO_DEC_TO_CAL_FACTOR_2(.25)        // 25%
#define ELECTROMAGNET_CURRENT_RELATIVE_FLOOR    2000                                  // 2 Amps
#define ELECTROMAGNET_CURRENT_TRIP_TIME         50                                    // This is in 10ms Units
#define ELECTROMAGNET_CURRENT_ABSOLUTE_TRIP_TIME    50                                    // This is in 10ms Units

#define NOMINAL_ELECTROMAGNET_RESISTANCE        1.00                                  // 1 Ohm
#define ELECTROMAGNET_VOLTAGE_OVER_TRIP         0xFFFF                                // No over trip Point
#define ELECTROMAGNET_VOLTAGE_UNDER_TRIP        4000                                  // 4 Volts
#define ELECTROMAGNET_VOLTAGE_RELATIVE_TRIP     MACRO_DEC_TO_CAL_FACTOR_2(.25)        // 25% 
#define ELECTROMAGNET_VOLTAGE_RELATIVE_FLOOR    2000                                  // 2 Volts
#define ELECTROMAGNET_VOLTAGE_TRIP_TIME         50                                    // This is in 10ms Units
#define ELECTROMAGNET_VOLTAGE_ABSOLUTE_TRIP_TIME    50                                    // This is in 10ms Units



// Configuration Settings for the Heater Supply
#define HEATER_MAX_IPROG                        12000
#define HEATER_MIN_IPROG                        0

#define HEATER_CURRENT_OVER_TRIP                11000                                 // 11 Amps
#define HEATER_CURRENT_UNDER_TRIP               0                                     // No under trip Point
#define HEATER_CURRENT_RELATIVE_TRIP            MACRO_DEC_TO_CAL_FACTOR_2(.25)        // 25%
#define HEATER_CURRENT_RELATIVE_FLOOR           2000                                  // 2 Amps
#define HEATER_CURRENT_TRIP_TIME                50                                    // This is in 10ms Units
#define HEATER_CURRENT_ABSOLUTE_TRIP_TIME       50                                    // This is in 10ms Units

#define NOMINAL_HEATER_RESISTANCE               1.44                                  // OHM
#define HEATER_VOLTAGE_OVER_TRIP                0xFFFF                                // No over trip point
#define HEATER_VOLTAGE_UNDER_TRIP               0                                     // No under trip Point
#define HEATER_VOLTAGE_RELATIVE_TRIP            MACRO_DEC_TO_CAL_FACTOR_2(.9)        // 90%
#define HEATER_VOLTAGE_RELATIVE_FLOOR           2000                                  // 2 Volts
#define HEATER_VOLTAGE_TRIP_TIME                500                                   // This is in 10ms Units
#define HEATER_VOLTAGE_ABSOLUTE_TRIP_TIME       500                                   // This is in 10ms Units


// Configure Local Power Supply Monitors
#define PWR_5V_OVER_FLT        5200                   // 5.2 V
#define PWR_5V_UNDER_FLT       4800                   // 4.8 V

#define PWR_15V_OVER_FLT       15500                  // 15.5 V
#define PWR_15V_UNDER_FLT      14500                  // 14.5 V

#define PWR_NEG_15V_OVER_FLT   15500                  // -15.5 V
#define PWR_NEG_15V_UNDER_FLT  14500                  // -14.5 V

#define ADC_DAC_TEST_VALUE     0x8000                 // Default Test Value
#define ADC_DAC_TEST_OVER_FLT  0x8200                 // 1.01562 of test value
#define ADC_DAC_TEST_UNDER_FLT 0x7E00                 // .984375 of test value



#define TIME_POWER_TEST                    500   // 5 seconds
#define MAX_HEATER_OVER_CURRENT_EVENTS     5
#define HEATER_OVER_CURRENT_OFF_TIME       1000  // 10 Seconds



#endif	/* A36926_001_SETTINGS_H */

