#include "A36926_001.h"
#include "FIRMWARE_VERSION.h"


// This is firmware for the Magnet Supply Test Board

_FOSC(ECIO & CSW_FSCM_OFF);
//_FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);  // 1 Second watchdog timer
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);




LTC265X U14_LTC2654;
HeaterMagnetControlData global_data_A36926_001;
unsigned int counts = 0;
unsigned int operated = 0;


void DisableHeaterMagnetOutputs(void);
void EnableHeaterMagnetOutputs(void);


void InitializeA36926_001(void);
void DoStateMachine(void);
void DoA36926_001(void);
void Reset_EEPROM_I2C(unsigned long SCLpin, unsigned long SDApin);


#define STATE_STARTUP                0x10
#define STATE_WAITING_FOR_CONFIG     0x20
#define STATE_POWER_UP_TEST          0x30
#define STATE_OPERATE                0x40
#define STATE_FAULT                  0x50
#define STATE_FAULT_NO_RECOVERY      0x60

int main(void) {
  global_data_A36926_001.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}


#define TIME_POWER_UP_TEST     1000 // 10 seconds
#define MAX_RESET_ATTEMPTS          5

void DoStateMachine(void) {

  switch (global_data_A36926_001.control_state) {

  case STATE_STARTUP:
    InitializeA36926_001();
    _CONTROL_NOT_READY = 1;
    _CONTROL_NOT_CONFIGURED = 1;
    global_data_A36926_001.startup_count = 0;
    global_data_A36926_001.control_state = STATE_WAITING_FOR_CONFIG;
    break;


  case STATE_WAITING_FOR_CONFIG:
    _CONTROL_NOT_READY = 1;
    DisableHeaterMagnetOutputs();
    while (global_data_A36926_001.control_state == STATE_WAITING_FOR_CONFIG) {
      DoA36926_001();

      if (_CONTROL_NOT_CONFIGURED == 0) {
	global_data_A36926_001.control_state = STATE_POWER_UP_TEST;
      }
    }
    break;


  case STATE_POWER_UP_TEST:
    global_data_A36926_001.startup_count++;
    _CONTROL_NOT_READY = 1;
    global_data_A36926_001.power_up_test_timer = 0;
    EnableHeaterMagnetOutputs();
    while (global_data_A36926_001.control_state == STATE_POWER_UP_TEST) {
      DoA36926_001();
      
      if (global_data_A36926_001.power_up_test_timer >= TIME_POWER_UP_TEST) {
	// We passed the warmup time without a fault, clear the startup counter

	global_data_A36926_001.startup_count = 0;
	global_data_A36926_001.power_up_test_timer = TIME_POWER_UP_TEST;
	// We can moce to the operate sate if there are no latched faults or if the reset is active
	if ((_FAULT_REGISTER == 0) || (ETMCanSlaveGetSyncMsgResetEnable())) {
	  global_data_A36926_001.control_state = STATE_OPERATE;
	}
      }

      if (global_data_A36926_001.fault_active) {
	if (global_data_A36926_001.startup_count <= MAX_RESET_ATTEMPTS){
	  global_data_A36926_001.control_state = STATE_FAULT;
	} else {
	  global_data_A36926_001.control_state = STATE_FAULT_NO_RECOVERY;
	}
      }
    }
    break;


  case STATE_OPERATE:
    _CONTROL_NOT_READY = 0;
    _FAULT_REGISTER = 0;
    
    while (global_data_A36926_001.control_state == STATE_OPERATE) {
      DoA36926_001();

      if (global_data_A36926_001.fault_active) {
	global_data_A36926_001.control_state = STATE_FAULT;
      }
    }
    break;


  case STATE_FAULT:
    DisableHeaterMagnetOutputs();
    _CONTROL_NOT_READY = 1;
    while (global_data_A36926_001.control_state == STATE_FAULT) {
      DoA36926_001();

      if (!global_data_A36926_001.fault_active) {
	global_data_A36926_001.control_state = STATE_WAITING_FOR_CONFIG;
      }
    }
    break;

  case STATE_FAULT_NO_RECOVERY:
    DisableHeaterMagnetOutputs();
    _CONTROL_NOT_READY = 1;
    _STATUS_PERMA_FAULTED = 1;
    while (global_data_A36926_001.control_state == STATE_FAULT_NO_RECOVERY) {
      DoA36926_001();
    }

  default:
    global_data_A36926_001.control_state = STATE_FAULT_NO_RECOVERY;
    break;

  }
}

void DisableHeaterMagnetOutputs(void) {
  global_data_A36926_001.analog_output_heater_current.enabled = 0;
  global_data_A36926_001.analog_output_electromagnet_current.enabled = 0;
  PIN_HEATER_MAGNET_DISABLE = !OLL_CLOSE_RELAY;
}


void EnableHeaterMagnetOutputs(void) {
  global_data_A36926_001.analog_output_heater_current.enabled = 1;
  global_data_A36926_001.analog_output_electromagnet_current.enabled = 1;
  PIN_HEATER_MAGNET_DISABLE = OLL_CLOSE_RELAY;
}



void DoA36926_001(void) {

  unsigned int spi_error_count;
  unsigned int scale_error_count;

  ETMCanSlaveDoCan();


  // Check the status of these pins every time through the loop
//  if (PIN_PIC_INPUT_HEATER_OV_OK == ILL_HEATER_OV) {     //why was checked twice? -hkw
//    _FAULT_HW_HEATER_OVER_VOLTAGE = 1;
//    global_data_A36926_001.fault_active = 1;
//  }
  /*
  if (PIN_PIC_INPUT_TEMPERATURE_OK == ILL_TEMP_SWITCH_FAULT) {
    _FAULT_HW_TEMPERATURE_SWITCH = 1;
    global_data_A36926_001.fault_active = 1;
  }
  */
  if (_T3IF) {
    // Timer has expired so execute the scheduled code (should be once every 10ms unless the configuration file is changes
    _T3IF = 0;

//    local_debug_data.debug_0 = global_data_A36444_500.startup_count;
//    local_debug_data.debug_1 = global_data_A36444_500.fault_active;
//    local_debug_data.debug_2 = global_data_A36444_500.power_up_test_timer;
//    local_debug_data.debug_3 = global_data_A36444_500.control_state;
//    local_debug_data.debug_4 = _SYNC_CONTROL_WORD;
//    local_debug_data.debug_5 = global_data_A36444_500.analog_output_electromagnet_current.dac_setting_scaled_and_calibrated;
//    local_debug_data.debug_6 = global_data_A36444_500.analog_output_heater_current.dac_setting_scaled_and_calibrated;
//    local_debug_data.debug_7 = global_data_A36444_500.accumulator_counter;
//    local_debug_data.debug_8 = global_data_A36444_500.analog_input_electromagnet_current.adc_accumulator;
//    local_debug_data.debug_9 = global_data_A36444_500.analog_input_heater_current.adc_accumulator;
//    local_debug_data.debug_A = global_data_A36444_500.analog_input_electromagnet_current.filtered_adc_reading;
//    local_debug_data.debug_B = global_data_A36444_500.analog_input_heater_current.filtered_adc_reading;
//    local_debug_data.debug_C = ADCBUF8;
//    local_debug_data.debug_D = ADCBUF9;
//    local_debug_data.debug_E = ADCBUFA;
//    local_debug_data.debug_F = ADCBUFB;

    spi_error_count = etm_spi1_error_count + etm_spi2_error_count;
    scale_error_count = etm_scale_saturation_etmscalefactor2_count + etm_scale_saturation_etmscalefactor16_count;

    ETMCanSlaveSetDebugRegister(0x0, global_data_A36926_001.startup_count);
    ETMCanSlaveSetDebugRegister(0x1, global_data_A36926_001.fault_active);
    ETMCanSlaveSetDebugRegister(0x2, global_data_A36926_001.power_up_test_timer);
    ETMCanSlaveSetDebugRegister(0x3, global_data_A36926_001.control_state);
//    ETMCanSlaveSetDebugRegister(0x4, ETMCanSlaveGetSyncMsgECBState());
//    ETMCanSlaveSetDebugRegister(0x4, scale_error_count);
    ETMCanSlaveSetDebugRegister(0x4, operated);
//    ETMCanSlaveSetDebugRegister(0x4, ADCBUF0);
//    ETMCanSlaveSetDebugRegister(0x5, ADCBUF8);
    ETMCanSlaveSetDebugRegister(0x5, global_data_A36926_001.analog_output_electromagnet_current.dac_setting_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(0x6, global_data_A36926_001.analog_output_heater_current.dac_setting_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(0x7, global_data_A36926_001.accumulator_counter);
    ETMCanSlaveSetDebugRegister(0x8, global_data_A36926_001.analog_input_electromagnet_current.adc_accumulator);
    ETMCanSlaveSetDebugRegister(0x9, global_data_A36926_001.analog_input_electromagnet_current.adc_accumulator);
    ETMCanSlaveSetDebugRegister(0xA, etm_i2c1_error_count);
    ETMCanSlaveSetDebugRegister(0xB, spi_error_count);
    ETMCanSlaveSetDebugRegister(0xC, global_data_A36926_001.analog_output_heater_current.set_point);//global_data_A36926_001.analog_input_electromagnet_current.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(0xD, global_data_A36926_001.analog_output_electromagnet_current.set_point);//global_data_A36926_001.analog_input_heater_current.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(0xE, global_data_A36926_001.can_heater_current_set_point);//global_data_A36926_001.analog_input_electromagnet_voltage.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(0xF, global_data_A36926_001.analog_input_heater_voltage.reading_scaled_and_calibrated);

        // Update logging data
    slave_board_data.log_data[0] = global_data_A36926_001.analog_input_electromagnet_voltage.reading_scaled_and_calibrated;
    slave_board_data.log_data[1] = global_data_A36926_001.analog_input_electromagnet_current.reading_scaled_and_calibrated;
    slave_board_data.log_data[2] = global_data_A36926_001.analog_input_heater_voltage.reading_scaled_and_calibrated;
    slave_board_data.log_data[3] = global_data_A36926_001.analog_input_heater_current.reading_scaled_and_calibrated;
    //slave_board_data.log_data[4] = 0;
    slave_board_data.log_data[5] = global_data_A36926_001.analog_output_electromagnet_current.set_point;
    //slave_board_data.log_data[6] = 0;
    slave_board_data.log_data[7] = global_data_A36926_001.analog_output_heater_current.set_point;
    slave_board_data.log_data[8] = ETMCanSlaveGetPulseCount();


    if (global_data_A36926_001.control_state == STATE_POWER_UP_TEST) {
      global_data_A36926_001.power_up_test_timer++;
    }

    // Update the error counters that get returned
    ////local_debug_data.i2c_bus_error_count = 0;  // There are no I2C devices on this board
    ////local_debug_data.spi_bus_error_count = etm_spi1_error_count + etm_spi2_error_count;
    ////local_debug_data.scale_error_count = etm_scale_saturation_etmscalefactor2_count + etm_scale_saturation_etmscalefactor16_count;

    
    // If the system is faulted or inhibited set the red LED
    if (_CONTROL_NOT_READY) {
      PIN_LED_A_RED = OLL_LED_ON;
    } else {
      PIN_LED_A_RED = !OLL_LED_ON;
    }


    // Update the digital input status pins
    if (PIN_PIC_INPUT_CROWBAR_UP == ILL_RELAY_OPEN) {
      _STATUS_OUTPUT_RELAY_OPEN = 1;
    } else {
      _STATUS_OUTPUT_RELAY_OPEN = 0;
    }


    // Do Math on ADC inputs
    // Scale the ADC readings to engineering units
    ETMAnalogScaleCalibrateADCReading(&global_data_A36926_001.analog_input_electromagnet_current);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36926_001.analog_input_heater_current);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36926_001.analog_input_electromagnet_voltage);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36926_001.analog_input_heater_voltage);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36926_001.analog_input_5v_mon);

// -------------------- CHECK FOR FAULTS ------------------- //

    global_data_A36926_001.fault_active = 0;

//    if (PIN_PIC_INPUT_HEATER_OV_OK == ILL_HEATER_OV) {
//      _FAULT_HW_HEATER_OVER_VOLTAGE = 1;
//      global_data_A36926_001.fault_active = 1;
//    }
    /*
    if (PIN_PIC_INPUT_TEMPERATURE_OK == ILL_TEMP_SWITCH_FAULT) {
      _FAULT_HW_TEMPERATURE_SWITCH = 1;
      global_data_A36926_001.fault_active = 1;
    }
    */
    // DPARKER check SYNC message for coolant flow and fault if there is a problem
    if (ETMCanSlaveGetSyncMsgCoolingFault()) {
      _FAULT_COOLANT_FAULT = 1;
    } else {
      if (ETMCanSlaveGetSyncMsgResetEnable()) {
	_FAULT_COOLANT_FAULT = 0;
      }
    }

    if (_FAULT_COOLANT_FAULT) {
      global_data_A36926_001.fault_active = 1;
    }

    if (ETMCanSlaveGetComFaultStatus()) {
      _FAULT_CAN_COMMUNICATION_LATCHED = 1;
      global_data_A36926_001.fault_active = 1;
    }

    if ((global_data_A36926_001.control_state == STATE_OPERATE) || (global_data_A36926_001.control_state == STATE_POWER_UP_TEST)) {
      global_data_A36926_001.analog_input_electromagnet_current.target_value = global_data_A36926_001.analog_output_electromagnet_current.set_point;
      global_data_A36926_001.analog_input_heater_current.target_value = global_data_A36926_001.analog_output_heater_current.set_point;

      if (ETMAnalogCheckOverAbsolute(&global_data_A36926_001.analog_input_heater_current)) {
	_FAULT_HEATER_OVER_CURRENT_ABSOLUTE = 1;
	global_data_A36926_001.fault_active = 1;
      }
      if (ETMAnalogCheckUnderAbsolute(&global_data_A36926_001.analog_input_heater_current)) {
	_FAULT_HEATER_UNDER_CURRENT_ABSOLUTE = 1;
	global_data_A36926_001.fault_active = 1;
      }
      if (ETMAnalogCheckOverRelative(&global_data_A36926_001.analog_input_heater_current)) {
	_FAULT_HEATER_OVER_CURRENT_RELATIVE = 1;
	global_data_A36926_001.fault_active = 1;
      }
      if (ETMAnalogCheckUnderRelative(&global_data_A36926_001.analog_input_heater_current)) {
	_FAULT_HEATER_UNDER_CURRENT_RELATIVE = 1;
	global_data_A36926_001.fault_active = 1;
      }

      if (ETMAnalogCheckOverAbsolute(&global_data_A36926_001.analog_input_electromagnet_current)) {
	_FAULT_MAGNET_OVER_CURRENT_ABSOLUTE = 1;
	global_data_A36926_001.fault_active = 1;
      }
      if (ETMAnalogCheckUnderAbsolute(&global_data_A36926_001.analog_input_electromagnet_current)) {
	_FAULT_MAGNET_UNDER_CURRENT_ABSOLUTE = 1;
	global_data_A36926_001.fault_active = 1;
      }
      if (ETMAnalogCheckOverRelative(&global_data_A36926_001.analog_input_electromagnet_current)) {
	_FAULT_MAGNET_OVER_CURRENT_RELATIVE = 1;
	global_data_A36926_001.fault_active = 1;
      }
      if (ETMAnalogCheckUnderRelative(&global_data_A36926_001.analog_input_electromagnet_current)) {
	_FAULT_MAGNET_UNDER_CURRENT_RELATIVE = 1;
	global_data_A36926_001.fault_active = 1;
      }
    } else {
      global_data_A36926_001.analog_input_electromagnet_current.target_value = 0;
      global_data_A36926_001.analog_input_heater_current.target_value = 0;
    }

    // Set DAC outputs
    if ((global_data_A36926_001.control_state == STATE_OPERATE) || (global_data_A36926_001.control_state == STATE_POWER_UP_TEST)) {
      ETMAnalogSetOutput(&global_data_A36926_001.analog_output_heater_current, global_data_A36926_001.can_heater_current_set_point);
      ETMAnalogSetOutput(&global_data_A36926_001.analog_output_electromagnet_current, global_data_A36926_001.can_magnet_current_set_point);

      ETMAnalogScaleCalibrateDACSetting(&global_data_A36926_001.analog_output_heater_current);
      ETMAnalogScaleCalibrateDACSetting(&global_data_A36926_001.analog_output_electromagnet_current);

      WriteLTC265XTwoChannels(&U14_LTC2654,
			      LTC265X_WRITE_AND_UPDATE_DAC_A, global_data_A36926_001.analog_output_electromagnet_current.dac_setting_scaled_and_calibrated,
			      LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A36926_001.analog_output_heater_current.dac_setting_scaled_and_calibrated);

    } else {
      WriteLTC265XTwoChannels(&U14_LTC2654,
			      LTC265X_WRITE_AND_UPDATE_DAC_A, global_data_A36926_001.analog_output_electromagnet_current.disabled_dac_set_point,
			      LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A36926_001.analog_output_heater_current.disabled_dac_set_point);
    }

  }
 // Nop();
}

void InitializeA36926_001(void) {
  unsigned int startup_counter;

 
  // Initialize the status register and load the inhibit and fault masks
  _FAULT_REGISTER = 0;
  _CONTROL_REGISTER = 0;
  _WARNING_REGISTER = 0;
  _NOT_LOGGED_REGISTER = 0;


  global_data_A36926_001.analog_output_electromagnet_current.set_point = 0;
  global_data_A36926_001.analog_output_heater_current.set_point = 0;

  // Configure ADC Interrupt
  _ADIP   = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)

  // Initialize all I/O Registers
  TRISA = A36926_001_TRISA_VALUE;
  TRISB = A36926_001_TRISB_VALUE;
  TRISC = A36926_001_TRISC_VALUE;
  TRISD = A36926_001_TRISD_VALUE;
  TRISF = A36926_001_TRISF_VALUE;
  TRISG = A36926_001_TRISG_VALUE;

  PIN_SELECT_DAC_C_D = OLL_SELECT_DAC_C;

  // Initialize TMR2
  // TMR2 is unused but it must be configured for 16 bit mode
  T2CON = 0;

  // Initialize TMR3
  PR3   = PR3_VALUE_10_MILLISECONDS;
  TMR3  = 0;
  _T3IF = 0;
  T3CON = T3CON_VALUE;


  // Initialize internal ADC
  // ---- Configure the dsPIC ADC Module ------------ //
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON3 = ADCON3_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters

  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCSSL = ADCSSL_SETTING; //0b0000111100000000;//ADCSSL_SETTING;             // Set which analog pins are scanned
  _ADIF = 0;
  _ADIE = 1;
  _ADON = 1;


  // Initialize LTC DAC
  SetupLTC265X(&U14_LTC2654, ETM_SPI_PORT_2, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);

  //function that resets i2c bus
  Reset_EEPROM_I2C(_PIN_RG2, _PIN_RG3);

  // Initialize the External EEprom
  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, 400000, EEPROM_I2C_ADDRESS_0, 1);

  #define AGILE_REV 'A'
  #define SERIAL_NUMBER 100
 
  // Initialize the Can module
  ETMCanSlaveInitialize(CAN_PORT_1, FCY_CLK, ETM_CAN_ADDR_HEATER_MAGNET_BOARD, _PIN_RG13, 4, _PIN_RG13, _PIN_RG13);
  ETMCanSlaveLoadConfiguration(36926, 1, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_MINOR_REV);

 

  // Initialize the Analog Input * Output Scaling
  ETMAnalogInitializeOutput(&global_data_A36926_001.analog_output_electromagnet_current,
			    MACRO_DEC_TO_SCALE_FACTOR_16(1.311),
			    OFFSET_ZERO,
			    ANALOG_OUTPUT_0,
			    ELECTROMAGNET_MAX_IPROG,
			    ELECTROMAGNET_MIN_IPROG,
			    0);

  ETMAnalogInitializeOutput(&global_data_A36926_001.analog_output_heater_current,
			    MACRO_DEC_TO_SCALE_FACTOR_16(1.311),
			    OFFSET_ZERO,
			    ANALOG_OUTPUT_2,
			    HEATER_MAX_IPROG,
			    HEATER_MIN_IPROG,
			    0);

  ETMAnalogInitializeInput(&global_data_A36926_001.analog_input_electromagnet_current,
			   MACRO_DEC_TO_SCALE_FACTOR_16(1.563),
			   OFFSET_ZERO,
			   ANALOG_INPUT_1,
			   ELECTROMAGNET_CURRENT_OVER_TRIP,
			   ELECTROMAGNET_CURRENT_UNDER_TRIP,
			   ELECTROMAGNET_CURRENT_RELATIVE_TRIP,
			   ELECTROMAGNET_CURRENT_RELATIVE_FLOOR,
			   ELECTROMAGNET_CURRENT_TRIP_TIME,
                           ELECTROMAGNET_CURRENT_ABSOLUTE_TRIP_TIME); //changed scale from .6250

  ETMAnalogInitializeInput(&global_data_A36926_001.analog_input_heater_current,
			   MACRO_DEC_TO_SCALE_FACTOR_16(1.563),
			   OFFSET_ZERO,
			   ANALOG_INPUT_2,
			   HEATER_CURRENT_OVER_TRIP,
			   HEATER_CURRENT_UNDER_TRIP,
			   HEATER_CURRENT_RELATIVE_TRIP,
			   HEATER_CURRENT_RELATIVE_FLOOR,
			   HEATER_CURRENT_TRIP_TIME,
                           HEATER_CURRENT_ABSOLUTE_TRIP_TIME); //changed scale from .6250

  ETMAnalogInitializeInput(&global_data_A36926_001.analog_input_electromagnet_voltage,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.4690),
			   OFFSET_ZERO,
			   ANALOG_INPUT_3,
			   ELECTROMAGNET_VOLTAGE_OVER_TRIP,
			   ELECTROMAGNET_VOLTAGE_UNDER_TRIP,
			   ELECTROMAGNET_VOLTAGE_RELATIVE_TRIP,
			   ELECTROMAGNET_VOLTAGE_RELATIVE_FLOOR,
			   ELECTROMAGNET_VOLTAGE_TRIP_TIME,
                           ELECTROMAGNET_VOLTAGE_ABSOLUTE_TRIP_TIME);  //changed scale from .6250

  ETMAnalogInitializeInput(&global_data_A36926_001.analog_input_heater_voltage,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.4690),
			   OFFSET_ZERO,
			   ANALOG_INPUT_4,
			   HEATER_VOLTAGE_OVER_TRIP,
			   HEATER_VOLTAGE_UNDER_TRIP,
			   HEATER_VOLTAGE_RELATIVE_TRIP,
			   HEATER_VOLTAGE_RELATIVE_FLOOR,
			   HEATER_VOLTAGE_TRIP_TIME,
                           HEATER_VOLTAGE_ABSOLUTE_TRIP_TIME); //changed scale from .6250

  ETMAnalogInitializeInput(&global_data_A36926_001.analog_input_5v_mon,
                           MACRO_DEC_TO_SCALE_FACTOR_16(.12500),
                           OFFSET_ZERO,
                           ANALOG_INPUT_5,
                           PWR_5V_OVER_FLT,
                           PWR_5V_UNDER_FLT,
                           NO_TRIP_SCALE,
                           NO_FLOOR,
                           NO_COUNTER,
                           NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36926_001.analog_input_15v_mon,
                           MACRO_DEC_TO_SCALE_FACTOR_16(.25063),
                           OFFSET_ZERO,
                           ANALOG_INPUT_6,
                           PWR_15V_OVER_FLT,
                           PWR_15V_UNDER_FLT,
                           NO_TRIP_SCALE,
                           NO_FLOOR,
                           NO_COUNTER,
                           NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36926_001.analog_input_neg_15v_mon,
                           MACRO_DEC_TO_SCALE_FACTOR_16(.06250),
                           OFFSET_ZERO,
                           ANALOG_INPUT_7,
                           PWR_NEG_15V_OVER_FLT,
                           PWR_NEG_15V_UNDER_FLT,
                           NO_TRIP_SCALE,
                           NO_FLOOR,
                           NO_COUNTER,
                           NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36926_001.analog_input_pic_adc_test_dac,
                           MACRO_DEC_TO_SCALE_FACTOR_16(1),
                           OFFSET_ZERO,
                           ANALOG_INPUT_8,
                           ADC_DAC_TEST_OVER_FLT,
                           ADC_DAC_TEST_UNDER_FLT,
                           NO_TRIP_SCALE,
                           NO_FLOOR,
                           NO_COUNTER,
                           NO_COUNTER);

//  Test voltage
//   WriteLTC265XTwoChannels(&U14_LTC2654,
//			      LTC265X_WRITE_AND_UPDATE_DAC_B, 0x4000,
//			      LTC265X_WRITE_AND_UPDATE_DAC_D, 0x0000);

  // Flash LEDs at Startup
  startup_counter = 0;
  while (startup_counter <= 400) {  // 4 Seconds total
    ETMCanSlaveDoCan();
    if (_T3IF) {
      _T3IF =0;
      startup_counter++;
    }
    switch (((startup_counter >> 4) & 0b11)) {

    case 0:
      PIN_LED_OPERATIONAL_GREEN = !OLL_LED_ON;
      PIN_LED_A_RED = !OLL_LED_ON;
      PIN_LED_B_GREEN = !OLL_LED_ON;
      break;

    case 1:
      PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
      PIN_LED_A_RED = !OLL_LED_ON;
      PIN_LED_B_GREEN = !OLL_LED_ON;
      break;

    case 2:
      PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
      PIN_LED_A_RED = OLL_LED_ON;
      PIN_LED_B_GREEN = !OLL_LED_ON;
      break;

    case 3:
      PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
      PIN_LED_A_RED = OLL_LED_ON;
      PIN_LED_B_GREEN = OLL_LED_ON;
      break;
    }
  }

  PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;

  _CONTROL_SELF_CHECK_ERROR = 0;

}

void Reset_EEPROM_I2C(unsigned long SCLpin, unsigned long SDApin) {
/* This function resets the i2c slave and master in case of communication disruptions
   It sends a start condition, followed by nine high bits and a start and stop condition
*/
  unsigned int tris_storeA = TRISA;
  unsigned int tris_storeB = TRISB;
  unsigned int tris_storeC = TRISC;
  unsigned int tris_storeD = TRISD;
  unsigned int tris_storeF = TRISF;
  unsigned int tris_storeG = TRISG;

  char i;

  ETMPinTrisOutput(SCLpin);
  ETMPinTrisInput(SDApin); 
  ETMSetPin(SCLpin);
  __delay32(20);  //2us
  ETMPinTrisOutput(SDApin); //generate start
  ETMClearPin(SDApin);
  __delay32(20);  //2us
  ETMClearPin(SCLpin);
  ETMPinTrisInput(SDApin);
  __delay32(20);  //2us

  for (i=0; i<8; i++){
    ETMSetPin(SCLpin); //generate 8 '1's
    __delay32(20);  //2us
    ETMClearPin(SCLpin);
    __delay32(20);  //2us
  }

  ETMSetPin(SCLpin); //9th '1'
  __delay32(20);  //2us
  ETMPinTrisOutput(SDApin);  //  generate start
  ETMClearPin(SDApin);
  __delay32(20);  //2us
  ETMPinTrisInput(SDApin); //generate stop
  __delay32(20);  //2us


  TRISA = tris_storeA;
  TRISB = tris_storeB;
  TRISC = tris_storeC;
  TRISD = tris_storeD;
  TRISF = tris_storeF;
  TRISG = tris_storeG;

}


void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
  _ADIF = 0;

  if (global_data_A36926_001.adc_ignore_current_sample) {
    // There was a pulse durring the sample sequence.  Throw the data away!!!
    global_data_A36926_001.adc_ignore_current_sample = 0;
  } else {
    // Copy Data From Buffer to RAM
     if (_BUFS) {
      // read ADCBUF 0-7
      global_data_A36926_001.analog_input_electromagnet_current.adc_accumulator   += ADCBUF0;
      global_data_A36926_001.analog_input_heater_current.adc_accumulator          += ADCBUF1;
      global_data_A36926_001.analog_input_electromagnet_voltage.adc_accumulator   += ADCBUF2;
      global_data_A36926_001.analog_input_heater_voltage.adc_accumulator          += ADCBUF3;

      global_data_A36926_001.analog_input_15v_mon.adc_accumulator                 += ADCBUF4;
      global_data_A36926_001.analog_input_neg_15v_mon.adc_accumulator             += ADCBUF5;
      global_data_A36926_001.analog_input_5v_mon.adc_accumulator                  += ADCBUF6;
      global_data_A36926_001.analog_input_pic_adc_test_dac.adc_accumulator        += ADCBUF7;
      
     } else {
      // read ADCBUF 8-15
      global_data_A36926_001.analog_input_electromagnet_current.adc_accumulator   += ADCBUF8;
      global_data_A36926_001.analog_input_heater_current.adc_accumulator          += ADCBUF9;
      global_data_A36926_001.analog_input_electromagnet_voltage.adc_accumulator   += ADCBUFA;
      global_data_A36926_001.analog_input_heater_voltage.adc_accumulator          += ADCBUFB;

      global_data_A36926_001.analog_input_15v_mon.adc_accumulator                 += ADCBUFC;
      global_data_A36926_001.analog_input_neg_15v_mon.adc_accumulator             += ADCBUFD;
      global_data_A36926_001.analog_input_5v_mon.adc_accumulator                  += ADCBUFE;
      global_data_A36926_001.analog_input_pic_adc_test_dac.adc_accumulator        += ADCBUFF;

     }

     global_data_A36926_001.accumulator_counter++ ;

     if (global_data_A36926_001.accumulator_counter >= 128) {

      global_data_A36926_001.analog_input_electromagnet_current.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36926_001.analog_input_electromagnet_current.filtered_adc_reading = global_data_A36926_001.analog_input_electromagnet_current.adc_accumulator;
      global_data_A36926_001.analog_input_electromagnet_current.adc_accumulator = 0;

      global_data_A36926_001.analog_input_heater_current.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36926_001.analog_input_heater_current.filtered_adc_reading = global_data_A36926_001.analog_input_heater_current.adc_accumulator;
      global_data_A36926_001.analog_input_heater_current.adc_accumulator = 0;

      global_data_A36926_001.analog_input_electromagnet_voltage.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36926_001.analog_input_electromagnet_voltage.filtered_adc_reading = global_data_A36926_001.analog_input_electromagnet_voltage.adc_accumulator;
      global_data_A36926_001.analog_input_electromagnet_voltage.adc_accumulator = 0;

      global_data_A36926_001.analog_input_heater_voltage.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36926_001.analog_input_heater_voltage.filtered_adc_reading = global_data_A36926_001.analog_input_heater_voltage.adc_accumulator;
      global_data_A36926_001.analog_input_heater_voltage.adc_accumulator = 0;

      global_data_A36926_001.analog_input_15v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36926_001.analog_input_15v_mon.filtered_adc_reading = global_data_A36926_001.analog_input_15v_mon.adc_accumulator;
      global_data_A36926_001.analog_input_15v_mon.adc_accumulator = 0;

      global_data_A36926_001.analog_input_neg_15v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36926_001.analog_input_neg_15v_mon.filtered_adc_reading = global_data_A36926_001.analog_input_neg_15v_mon.adc_accumulator;
      global_data_A36926_001.analog_input_neg_15v_mon.adc_accumulator = 0;

      global_data_A36926_001.analog_input_5v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36926_001.analog_input_5v_mon.filtered_adc_reading = global_data_A36926_001.analog_input_5v_mon.adc_accumulator;
      global_data_A36926_001.analog_input_5v_mon.adc_accumulator = 0;

      global_data_A36926_001.analog_input_pic_adc_test_dac.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36926_001.analog_input_pic_adc_test_dac.filtered_adc_reading = global_data_A36926_001.analog_input_pic_adc_test_dac.adc_accumulator;
      global_data_A36926_001.analog_input_pic_adc_test_dac.adc_accumulator = 0;


      global_data_A36926_001.accumulator_counter = 0;
     }
  }
}


void ETMCanSlaveExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;

  index_word = message_ptr->word3;
  switch (index_word)
    {
    case ETM_CAN_REGISTER_HEATER_MAGNET_SET_1_CURRENT_SET_POINT:

      global_data_A36926_001.can_heater_current_set_point = message_ptr->word1;

      global_data_A36926_001.can_magnet_current_set_point = message_ptr->word0;  //magnet

      _CONTROL_NOT_CONFIGURED = 0;
      
      break;

    default:
//      local_can_errors.invalid_index++;
      break;


    }
}


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  // DPARKER do something to save the state into a RAM location that is not re-initialized and then reset
  Nop();
  Nop();
  __asm__ ("Reset");
}
