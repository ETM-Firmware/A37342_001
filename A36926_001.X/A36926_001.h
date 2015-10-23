/* 
 * File:   A36926_001.h
 * Author: hwanetick
 *
 * Created on October 22, 2015, 11:04 AM
 */

#ifndef __A36926_001_H
#define	__A36926_001_H


#include <xc.h>
#include <libpic30.h>
#include <adc12.h>
#include <timer.h>
#include "ETM.h"
#include "P1395_CAN_SLAVE.h"
#include "A36444_500_SETTINGS.h"

#define FCY_CLK     10000000

/*
  Hardware Module Resource Usage

  CAN1   - Used/Configured by ETM CAN
  Timer4 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such)
  Timer5 - Used/Configured by ETM CAN - Used for detecting error on can bus

  SPI2   - Used/Configured by LTC265X Module    -----> change to bus 2 in code
  I2C    - Used/Configured by EEPROM Module

  Timer3 - Used for 10msTicToc   -----> change to 3 in code

  ADC Module - See Below For Specifics

*/

// ---------- BASE A36926 I/O CONFIGURATION ----------------- //

#define PIC_DIG_IN_1      _RD8      //used  Temp
#define PIC_DIG_IN_2      _RD9      //used  Crwbr
#define PIC_DIG_IN_3      _RD10     //used  Htr OV OK
#define PIC_DIG_IN_4      _RD11
#define PIC_DIG_IN_5      _RD12
#define PIC_DIG_IN_6      _RD13
#define PIC_DIG_IN_7      _RD14
#define PIC_DIG_IN_8      _RD15

#define AUTO_INHIBIT_DETECT          _RA14   // This is INT3
#define RESET_DETECT                 _RG14



#define PIC_RELAY_OUT                _LATD3
#define PIC_OUTPUT_LAMBDA_SELECT     _LATD2
#define PIC_DIGITAL_OUT_2_NOT        _LATD1
#define PIC_DIGITAL_OUT_1_NOT        _LATD0
#define PIC_15V_SUPPLY_ENABLE        _LATA6

#define TEST_POINT_A                 _LATF6
#define TEST_POINT_B                 _LATF7
#define TEST_POINT_C                 _LATF8
#define TEST_POINT_D                 _LATF2
#define TEST_POINT_E                 _LATF3
#define TEST_POINT_F                 _LATB14


#define LED_OPERATIONAL              _LATA7
#define LED_A_RED                    _LATG12
#define LED_B_GREEN                  _LATG13



/*
  BASE ANALOG CONFIGURATION
  PIC_ADC_AN1  is on AN2
  PIC_ADC_AN2  is on AN3
  PIC_ADC_AN3  is on AN4
  PIC_ADC_AN4  is on AN5

  PIC_ADC_+15V_MON is on AN6
  PIC_ADC_-15V_MON is on AN7
  PIC_ADC_5V_MON   is on AN8
  PIC_ADC_TEST_DAC is on AN9
*/


// Pins that must be configured as outputs
/*
  A6,A7
  B14
  C
  D0,D1,D2,D3
  F2,F3,F6,F7,F8
  G12,G13
*/

#define A36926_TRISA_VALUE 0b1111111110111111
#define A36926_TRISB_VALUE 0b1011111111111111
#define A36926_TRISC_VALUE 0b1111111111111111
#define A36926_TRISD_VALUE 0b1111111111110000
#define A36926_TRISF_VALUE 0b1111111100110011
#define A36926_TRISG_VALUE 0b1100111111111111


//original code here

// ----------------- IO PIN CONFIGURATION -------------------- //
// All unused pins will be set to outputs and logic zero
// LAT values default to 0 at startup so they do not need to be manually set

// ----------------- DIGITAL INPUT PINS --------------- //
/*

  RA12 - (unused) Fiber Trigger IN
  RA13 - (unused) Fiber Energy Select
  RA14 - (unused) Auto Inhibit Detect
  RA15 - (unused) Lambda EOC

  RD8  - (unused) Lambda Not Powered
  RD9  - PIN INPUT TEMPERATURE OK (formerly) Lambda Over Temp FLT
  RD10 - PIC INPUT CROWBAR UP     (formerly) Lamdba Interlock FLT
  RD12 - PIB INPUT HEATER OV OK   (formerly) Lambda Load FLT
  RD13 - (unused) Lambda Sum FLT
  RD14 - (unused) Lambda Phase Loss FLT
  RD15 - (unused)Lambda HV ON Readback

  RG14 - Reset Detect

  Analog Input Pins


  Pins that are overidden by a hardware module and should be left as inputs during port configuration
  RA9  ADC VREF-
  RA10 ADC VREF+

  RB0 PROGRAM
  RB1 PROGRAM
  RB3  - Analog Input - unused
  RB4  - Analog Input - unused
  RB5  - Analog Input - PIC ADC MAGNET IMON
  RB6  - Analog Input - PIC ADC HEATER IMON
  RB12 - Analog Input - unused
  RB13 - Analog Input - unused
  RB14 - Analog Input - unused
  RB15 - Analog Input - unused

  RF0 CAN 1
  RF1 CAN 1
  RF6 SPI 1
  RF7 SPI 1
  RF8 SPI 1

  RG2 I2C
  RG3 I2C

  Pins that are configured by other software modules and should be left as inputs during port configuration
  RC1  (DAC LDAC)
  RG15 (DAC CS/LD)


*/

//   ------------------  Digital Output Pins ---------------
/*

  RD11 - Lamdba Voltage Select
  RD0 - (unused) Lambda Inhibit (This is also Output Compare 1 - If we want to use that module to generate Inhibit signal)
  RD1 - HEATER MAGNET DISABLE     (formerly) Lambda Enable


  RA7 - LED Operational
  RB8 - Test Point E
  RB9 - Test Point F
  RF4 - Test Point A
  RF5 - Test Point B
  RG0 - Test Point C
  RG1 - Test Point D
  RG12 - LED A RED
  RG13 - LED B GREEN

*/


#define A36444_500_TRISA_VALUE 0b1111011000000000
#define A36444_500_TRISB_VALUE 0b1111000001111011
#define A36444_500_TRISC_VALUE 0b0000000000000010
#define A36444_500_TRISD_VALUE 0b1111011100000000
#define A36444_500_TRISF_VALUE 0b0000000111000011
#define A36444_500_TRISG_VALUE 0b1100000000001100



// -------- Digital Input Pins ----------//
#define PIN_PIC_INPUT_TEMPERATURE_OK          PIC_DIG_IN_1
#define PIN_PIC_INPUT_CROWBAR_UP              PIC_DIG_IN_2
#define PIN_PIC_INPUT_HEATER_OV_OK            PIC_DIG_IN_3
//#define PIN_FIBER_ENERGY_SELECT               _RA13 pin14


#define PIN_RESET_DETECT                      RESET_DETECT

#define ILL_HEATER_OV                         1
#define ILL_TEMP_SWITCH_FAULT                 0
#define ILL_RELAY_OPEN                        1
#define ILL_ENERGY_SELECT_WATER_FLOW_OK       1



// ------- Digital Output Pins ---------//

#define PIN_HEATER_MAGNET_DISABLE             PIC_RELAY_OUT //now pin 63
#define PIN_SELECT_DAC_C_D                    PIC_OUTPUT_LAMBDA_SELECT // now pin 62

#define PIN_LED_OPERATIONAL_GREEN             LED_OPERATIONAL
#define PIN_LED_A_RED                         LED_A_RED
#define PIN_LED_B_GREEN                       LED_B_GREEN  // This is is configured by the CAN module to flash on CAN Bus activity
PIN_LED_B_GREEN
#define PIN_OUT_TP_A                          TEST_POINT_A
#define PIN_OUT_TP_B                          TEST_POINT_B
#define PIN_OUT_TP_C                          TEST_POINT_C
#define PIN_OUT_TP_D                          TEST_POINT_D
#define PIN_OUT_TP_E                          TEST_POINT_E
#define PIN_OUT_TP_F                          TEST_POINT_F

#define OLL_LED_ON                            0
#define OLL_CLOSE_RELAY                       0
#define OLL_SELECT_DAC_C                      1


//change below hkw
// ------------------------ CONFIGURE ADC MODULE ------------------- //

// ----------------- ANALOG INPUT PINS ---------------- //
/*
   AN3 - (unused) Lambda Vmon
   AN4 - (unused) Lambda Heat Sink Temp
   AN5 - Magnet Imon       (formerly) Lambda VPeak
   AN6 - Heater Imon       (formerly) Lambda Imon

   AN12 - (unused) ADC Test Input
   AN13 - (unused) 5V Mon
   AN14 - (unused) +15V Mon
   AN15 - (unused) -15V Mon

*/

/*
  This sets up the ADC to work as following
  AUTO Sampeling
  External Vref+/Vref-
  With 10MHz System Clock, ADC Clock is 450ns, Sample Time is 6 ADC Clock so total sample time is 9.0uS
  Conversion rate of 111KHz (13.888 Khz per Channel), 138 Samples per 10mS interrupt
  8 Samples per Interrupt, use alternating buffers
  Scan Through Selected Inputs

*/

#define ADCON1_SETTING          (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING          (ADC_VREF_EXT_EXT & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_8 & ADC_ALT_BUF_ON & ADC_ALT_INPUT_OFF)
#define ADCHS_SETTING           (ADC_CH0_POS_SAMPLEA_AN5 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN6 & ADC_CH0_NEG_SAMPLEB_VREFN)
#define ADPCFG_SETTING          (ENABLE_AN3_ANA & ENABLE_AN4_ANA & ENABLE_AN5_ANA & ENABLE_AN6_ANA & ENABLE_AN12_ANA & ENABLE_AN13_ANA & ENABLE_AN14_ANA & ENABLE_AN15_ANA)

#define ADCSSL_SETTING          (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN2 & SKIP_SCAN_AN3 & SKIP_SCAN_AN4 & SKIP_SCAN_AN7 & SKIP_SCAN_AN8 & SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13 & SKIP_SCAN_AN14 & SKIP_SCAN_AN15)
#define ADCON3_SETTING          (ADC_SAMPLE_TIME_4 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_9Tcy2)


/*
   TMR3 Configuration
   Timer3 - Used for 10msTicToc
   Period should be set to 10mS
   With 10Mhz Clock, x8 multiplier will yield max period of 17.7mS, 2.71uS per tick
*/

#define T3CON_VALUE                    (T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_8 & T3_SOURCE_INT)
#define PR3_PERIOD_US                  10000   // 10mS
#define PR3_VALUE_10_MILLISECONDS      12500   //(FCY_CLK_MHZ*PR3_PERIOD_US/8)


typedef struct {
  // all currents are scaled to 1mA per lsb
  // all voltages are scaled to 1mV per lsb

  AnalogInput analog_input_heater_voltage;
  AnalogInput analog_input_heater_current;

  AnalogInput analog_input_electromagnet_voltage;
  AnalogInput analog_input_electromagnet_current;

  AnalogOutput analog_output_heater_current;
  AnalogOutput analog_output_electromagnet_current;

  unsigned int  accumulator_counter;

  unsigned int  adc_ignore_current_sample;

  unsigned int startup_count;
  unsigned int fault_active;
  unsigned int power_up_test_timer;

  unsigned int control_state;

} HeaterMagnetControlData;


extern HeaterMagnetControlData global_data_A36926_001;



#define _STATUS_MAGNET_OFF_READBACK                     _NOT_LOGGED_0     //Should these be warnings? HKW
#define _STATUS_HEATER_OFF_READBACK                     _NOT_LOGGED_1
#define _STATUS_OUTPUT_RELAY_OPEN                       _NOT_LOGGED_2
#define _STATUS_PERMA_FAULTED                           _NOT_LOGGED_3


#define _FAULT_HEATER_OVER_CURRENT_ABSOLUTE             _FAULT_0
#define _FAULT_HEATER_UNDER_CURRENT_ABSOLUTE            _FAULT_1
#define _FAULT_HEATER_OVER_CURRENT_RELATIVE             _FAULT_2
#define _FAULT_HEATER_UNDER_CURRENT_RELATIVE            _FAULT_3
#define _FAULT_HEATER_OVER_VOLTAGE_ABSOLUTE             _FAULT_4
#define _FAULT_HEATER_UNDER_VOLTAGE_RELATIVE            _FAULT_5


#define _FAULT_MAGNET_OVER_CURRENT_ABSOLUTE             _FAULT_6
#define _FAULT_MAGNET_UNDER_CURRENT_ABSOLUTE            _FAULT_7
#define _FAULT_MAGNET_OVER_CURRENT_RELATIVE             _FAULT_8
#define _FAULT_MAGNET_UNDER_CURRENT_RELATIVE            _FAULT_9
#define _FAULT_MAGNET_OVER_VOLTAGE_ABSOLUTE             _FAULT_A
#define _FAULT_MAGNET_UNDER_VOLTAGE_RELATIVE            _FAULT_B

#define _FAULT_HW_HEATER_OVER_VOLTAGE                   _FAULT_C
#define _FAULT_HW_TEMPERATURE_SWITCH                    _FAULT_D
#define _FAULT_COOLANT_FAULT                            _FAULT_E
#define _FAULT_CAN_COMMUNICATION_LATCHED                _FAULT_F



#endif	/* A36926_001_H */

