#include "A37342_001.h"
#include "FIRMWARE_VERSION.h"

// This is firmware for the Magnet Supply Test Board

_FOSC(ECIO & CSW_FSCM_OFF);
//_FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);  // 1 Second watchdog timer
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_DIS);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


unsigned int running_persistent __attribute__ ((persistent));
unsigned int do_fast_startup;
unsigned int setup_done;
#define PERSISTENT_TEST 0xCFD1


LTC265X U14_LTC2654;
TYPE_HEATER_MAGNET_CONTROL_DATA global_data_A37342_001;
void DoStateMachine(void);
void DisableHeaterMagnetOutputs(void);
void EnableHeaterMagnetOutputs(void);
void DoRecoveryStartup();
void DoA37342_001(void);
void FlashLEDs();
void Reset_EEPROM_I2C(unsigned long SCLpin, unsigned long SDApin);
void InitializeA37342_001(void);


int main(void) {
  global_data_A37342_001.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}



void DoStateMachine(void) {

  switch (global_data_A37342_001.control_state) {

  case STATE_STARTUP:
    if (running_persistent == PERSISTENT_TEST) {
      do_fast_startup = 1;
    } else {
      do_fast_startup = 0;
    }
    running_persistent = 0;
    InitializeA37342_001();
    _CONTROL_NOT_READY = 1;
    _CONTROL_NOT_CONFIGURED = 1;
    global_data_A37342_001.control_state = STATE_WAITING_FOR_CONFIG;

    if (do_fast_startup) {
      DoRecoveryStartup();
    }
    break;

    
  case STATE_WAITING_FOR_CONFIG:
    _CONTROL_NOT_READY = 1;
    running_persistent = 0;
    while (global_data_A37342_001.control_state == STATE_WAITING_FOR_CONFIG) {
      DoA37342_001();
      if (_CONTROL_NOT_CONFIGURED == 0) {
	global_data_A37342_001.control_state = STATE_POWER_TEST;
      }
    }
    break;
    

  case STATE_POWER_TEST:
    running_persistent = 0;
    _CONTROL_NOT_READY = 1;
    global_data_A37342_001.power_up_test_timer = 0;
    EnableHeaterMagnetOutputs();
    while (global_data_A37342_001.control_state == STATE_POWER_TEST) {
      DoA37342_001();
    
      if ((_FAULT_REGISTER == 0) && global_data_A37342_001.power_up_test_timer >= TIME_POWER_TEST) {
	// We passed the ramp up time and there are no faults, move to operate state
	global_data_A37342_001.control_state = STATE_OPERATE;
      }
      
      if (_STATUS_HW_OVER_TEMP_ACTIVE) {
	global_data_A37342_001.control_state = STATE_FAULT;
      }
      
      if (_STATUS_HEATER_OVER_CURRENT_ACTIVE) {
	global_data_A37342_001.heater_over_current_counter++;
    	global_data_A37342_001.control_state = STATE_FAULT;
      }
      
      if (ETMCanSlaveGetSyncMsgCoolingFault()) {
	global_data_A37342_001.control_state = STATE_FAULT;
      }

    }
    break;


  case STATE_OPERATE:
    _CONTROL_NOT_READY = 0;
    running_persistent = PERSISTENT_TEST;
    global_data_A37342_001.heater_over_current_counter = 0;
    while (global_data_A37342_001.control_state == STATE_OPERATE) {
      DoA37342_001();

      if (_FAULT_REGISTER) {
	global_data_A37342_001.control_state = STATE_POWER_TEST;
      }
    }
    break;


  case STATE_FAULT:
    _CONTROL_NOT_READY = 1;
    DisableHeaterMagnetOutputs();
    running_persistent = 0;
    global_data_A37342_001.fault_hold_timer = 0;
    while (global_data_A37342_001.control_state == STATE_FAULT) {
      DoA37342_001();

      if (global_data_A37342_001.heater_over_current_counter > MAX_HEATER_OVER_CURRENT_EVENTS) {
	global_data_A37342_001.control_state = STATE_FAULT_NO_RECOVERY;
      }
      
      if (global_data_A37342_001.fault_hold_timer > FAULT_OFF_TIME) {
	if ((!_STATUS_HW_OVER_TEMP_ACTIVE) && (!ETMCanSlaveGetSyncMsgCoolingFault())) {
	  global_data_A37342_001.control_state = STATE_POWER_TEST;
	}
      }
    }
    break;


  case STATE_FAULT_NO_RECOVERY:
    DisableHeaterMagnetOutputs();
    _CONTROL_NOT_READY = 1;
    _STATUS_PERMA_FAULTED = 1;
    running_persistent = 0;
    while (global_data_A37342_001.control_state == STATE_FAULT_NO_RECOVERY) {
      DoA37342_001();
    }

  default:
    global_data_A37342_001.control_state = STATE_FAULT_NO_RECOVERY;
    break;

  }
}

void DisableHeaterMagnetOutputs(void) {
  global_data_A37342_001.analog_output_heater_current.enabled = 0;
  global_data_A37342_001.analog_output_electromagnet_current.enabled = 0;
  PIN_HEATER_MAGNET_DISABLE = !OLL_CLOSE_RELAY;
}


void EnableHeaterMagnetOutputs(void) {
  global_data_A37342_001.analog_output_heater_current.enabled = 1;
  global_data_A37342_001.analog_output_electromagnet_current.enabled = 1;
  PIN_HEATER_MAGNET_DISABLE = OLL_CLOSE_RELAY;
}


void DoRecoveryStartup(void) {
  _CONTROL_NOT_READY = 0;
  _FAULT_REGISTER = 0;
  EnableHeaterMagnetOutputs();
  _CONTROL_NOT_CONFIGURED = 0;
  setup_done = 0;
  while (setup_done == 0) {
    DoA37342_001();
  }
  ETMAnalogSetOutput(&global_data_A37342_001.analog_output_heater_current, global_data_A37342_001.can_heater_current_set_point);
  ETMAnalogSetOutput(&global_data_A37342_001.analog_output_electromagnet_current, global_data_A37342_001.can_magnet_current_set_point_high_energy);
  ETMAnalogScaleCalibrateDACSetting(&global_data_A37342_001.analog_output_heater_current);
  ETMAnalogScaleCalibrateDACSetting(&global_data_A37342_001.analog_output_electromagnet_current);	  
  SetupLTC265X(&U14_LTC2654, ETM_SPI_PORT_2, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);
  
  WriteLTC265XTwoChannels(&U14_LTC2654,
			  LTC265X_WRITE_AND_UPDATE_DAC_A, global_data_A37342_001.analog_output_electromagnet_current.dac_setting_scaled_and_calibrated,
			  LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A37342_001.analog_output_heater_current.dac_setting_scaled_and_calibrated);
  
  global_data_A37342_001.control_state = STATE_OPERATE;      
}


void DoA37342_001(void) {
  ETMCanSlaveDoCan();

  if ((global_data_A37342_001.control_state == STATE_POWER_TEST) && (global_data_A37342_001.power_up_test_timer < TIME_POWER_TEST)) {
    // Flash the LEDs for the first three seconds
    FlashLEDs();
  }

  if (_T3IF) {
    // Timer has expired so execute the scheduled code (should be once every 10ms unless the configuration file is changes
    _T3IF = 0;


    ETMCanSlaveSetDebugRegister(0x0, global_data_A37342_001.heater_over_current_counter);
    ETMCanSlaveSetDebugRegister(0x1, global_data_A37342_001.fault_hold_timer);
    ETMCanSlaveSetDebugRegister(0x2, global_data_A37342_001.power_up_test_timer);
    ETMCanSlaveSetDebugRegister(0x3, global_data_A37342_001.control_state);
    ETMCanSlaveSetDebugRegister(0x4, ETMCanSlaveGetSyncMsgECBState());
    ETMCanSlaveSetDebugRegister(0x5, global_data_A37342_001.analog_output_electromagnet_current.dac_setting_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(0x6, global_data_A37342_001.analog_output_heater_current.dac_setting_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(0x7, global_data_A37342_001.accumulator_counter);
    ETMCanSlaveSetDebugRegister(0x8, global_data_A37342_001.can_heater_current_set_point);
    ETMCanSlaveSetDebugRegister(0x9, global_data_A37342_001.can_magnet_current_set_point_high_energy);
    ETMCanSlaveSetDebugRegister(0xA, etm_i2c1_error_count);
    ETMCanSlaveSetDebugRegister(0xB, global_data_A37342_001.fault_hold_timer);
    ETMCanSlaveSetDebugRegister(0xC, global_data_A37342_001.analog_input_electromagnet_current.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(0xD, global_data_A37342_001.analog_input_heater_current.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(0xE, global_data_A37342_001.analog_input_electromagnet_voltage.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(0xF, global_data_A37342_001.analog_input_heater_voltage.reading_scaled_and_calibrated);

    // Update logging data
    slave_board_data.log_data[0] = global_data_A37342_001.analog_input_electromagnet_voltage.reading_scaled_and_calibrated;
    slave_board_data.log_data[1] = global_data_A37342_001.analog_input_electromagnet_current.reading_scaled_and_calibrated;
    slave_board_data.log_data[2] = global_data_A37342_001.analog_input_heater_voltage.reading_scaled_and_calibrated;
    slave_board_data.log_data[3] = global_data_A37342_001.analog_input_heater_current.reading_scaled_and_calibrated;
    //slave_board_data.log_data[4] = 0;
    slave_board_data.log_data[5] = global_data_A37342_001.analog_output_electromagnet_current.set_point;
    //slave_board_data.log_data[6] = 0;
    slave_board_data.log_data[7] = global_data_A37342_001.analog_output_heater_current.set_point;
    slave_board_data.log_data[8] = ETMCanSlaveGetPulseCount();

    global_data_A37342_001.power_up_test_timer++;
    global_data_A37342_001.fault_hold_timer++;
    
    // If the system is faulted or inhibited set the red LED
    if (_CONTROL_NOT_READY) {
      PIN_LED_A_RED = OLL_LED_ON;
    } else {
      PIN_LED_A_RED = !OLL_LED_ON;
    }

    // Update the digital input status pins

    ETMDigitalUpdateInput(&global_data_A37342_001.digital_input_crowbar_up, PIN_PIC_INPUT_CROWBAR_UP);
    if (ETMDigitalFilteredOutput(&global_data_A37342_001.digital_input_crowbar_up) == ILL_RELAY_OPEN) {
      _STATUS_OUTPUT_RELAY_OPEN = 1;
    } else {
      _STATUS_OUTPUT_RELAY_OPEN = 0;
    }
    
    ETMDigitalUpdateInput(&global_data_A37342_001.digital_input_temp_switch, PIN_PIC_INPUT_TEMPERATURE_OK);
    if (ETMDigitalFilteredOutput(&global_data_A37342_001.digital_input_temp_switch) == ILL_TEMP_SWITCH_FAULT) {
      _STATUS_HW_OVER_TEMP_ACTIVE = 1;
      _FAULT_OVER_TEMP = 1;
    } else {
      _STATUS_HW_OVER_TEMP_ACTIVE = 0;
      if (ETMCanSlaveGetSyncMsgResetEnable()) {
	_FAULT_OVER_TEMP = 0;
      }
    }

    // Do Math on ADC inputs
    // Scale the ADC readings to engineering units
    ETMAnalogScaleCalibrateADCReading(&global_data_A37342_001.analog_input_electromagnet_current);
    ETMAnalogScaleCalibrateADCReading(&global_data_A37342_001.analog_input_heater_current);
    ETMAnalogScaleCalibrateADCReading(&global_data_A37342_001.analog_input_electromagnet_voltage);
    ETMAnalogScaleCalibrateADCReading(&global_data_A37342_001.analog_input_heater_voltage);
    ETMAnalogScaleCalibrateADCReading(&global_data_A37342_001.analog_input_5v_mon);

// -------------------- CHECK FOR FAULTS ------------------- //

    // Check SYNC message for coolant flow and fault if there is a problem
    if (ETMCanSlaveGetSyncMsgCoolingFault()) {
      _FAULT_COOLANT_FAULT = 1;
    } else {
      if (ETMCanSlaveGetSyncMsgResetEnable()) {
	_FAULT_COOLANT_FAULT = 0;
      }
    }

    if (ETMCanSlaveGetComFaultStatus()) {
      _FAULT_CAN_COMMUNICATION_LATCHED = 1;
    } else {
      if (ETMCanSlaveGetSyncMsgResetEnable()) {
	_FAULT_CAN_COMMUNICATION_LATCHED = 0;
      }
    }

    // ------------------------  Update Analog Heater Faults ---------------------- //
    if ((global_data_A37342_001.control_state == STATE_POWER_TEST) || (global_data_A37342_001.control_state == STATE_OPERATE)) {
      _STATUS_HEATER_OK_READBACK = 1;
    } else {
      _STATUS_HEATER_OK_READBACK = 0;
    }
    if (ETMAnalogCheckOverAbsolute(&global_data_A37342_001.analog_input_heater_current)) {
      _STATUS_HEATER_OVER_CURRENT_ACTIVE = 1;
      _STATUS_HEATER_OK_READBACK = 0;
      _FAULT_HEATER_OVER_CURRENT_ABSOLUTE = 1;
    } else {
      _STATUS_HEATER_OVER_CURRENT_ACTIVE = 0;
      if (ETMCanSlaveGetSyncMsgResetEnable()) {
	_FAULT_HEATER_OVER_CURRENT_ABSOLUTE = 0;
      }
    }

    if (ETMAnalogCheckOverRelative(&global_data_A37342_001.analog_input_heater_current)) {
      _STATUS_HEATER_OK_READBACK = 0;
      _FAULT_HEATER_OVER_CURRENT_RELATIVE = 1;
    } else {
      if (ETMCanSlaveGetSyncMsgResetEnable()) {
	_FAULT_HEATER_OVER_CURRENT_RELATIVE = 0;
      }
    }
  
    if (ETMAnalogCheckUnderRelative(&global_data_A37342_001.analog_input_heater_current)) {
      _STATUS_HEATER_OK_READBACK = 0;
      if ((global_data_A37342_001.control_state == STATE_POWER_TEST) || (global_data_A37342_001.control_state == STATE_OPERATE)) {
	_FAULT_HEATER_UNDER_CURRENT_RELATIVE = 1;
      }
    } else {
      if (ETMCanSlaveGetSyncMsgResetEnable()) {
	_FAULT_HEATER_UNDER_CURRENT_RELATIVE = 0;
      }
    }

    if (ETMAnalogCheckUnderRelative(&global_data_A37342_001.analog_input_heater_voltage)) {
      _STATUS_HEATER_OK_READBACK = 0;
      if ((global_data_A37342_001.control_state == STATE_POWER_TEST) || (global_data_A37342_001.control_state == STATE_OPERATE)) {
	_FAULT_HEATER_UNDER_VOLTAGE_RELATIVE = 1;
      }
    } else {
      if (ETMCanSlaveGetSyncMsgResetEnable()) {
	_FAULT_HEATER_UNDER_VOLTAGE_RELATIVE = 0;
      }
    }



    // ------------------------  Update Analog Magnet Faults ---------------------- //
    if ((global_data_A37342_001.control_state == STATE_POWER_TEST) || (global_data_A37342_001.control_state == STATE_OPERATE)) {
      _STATUS_MAGNET_OK_READBACK = 1;
    } else {
      _STATUS_MAGNET_OK_READBACK = 0;
    }
    if (ETMAnalogCheckOverAbsolute(&global_data_A37342_001.analog_input_electromagnet_current)) {
      _STATUS_MAGNET_OK_READBACK = 0;
      _FAULT_MAGNET_OVER_CURRENT_ABSOLUTE = 1;
    } else {
      if (ETMCanSlaveGetSyncMsgResetEnable()) {
	_FAULT_MAGNET_OVER_CURRENT_ABSOLUTE = 0;
      }
    }

    if (ETMAnalogCheckUnderAbsolute(&global_data_A37342_001.analog_input_electromagnet_current)) {
      _STATUS_MAGNET_OK_READBACK = 0;
      if ((global_data_A37342_001.control_state == STATE_POWER_TEST) || (global_data_A37342_001.control_state == STATE_OPERATE)) {
	_FAULT_MAGNET_UNDER_CURRENT_ABSOLUTE = 1;
      }
    } else {
      if (ETMCanSlaveGetSyncMsgResetEnable()) {
	_FAULT_MAGNET_UNDER_CURRENT_ABSOLUTE = 0;
      }
    }
    
    if (ETMAnalogCheckUnderAbsolute(&global_data_A37342_001.analog_input_electromagnet_voltage)) {
      _STATUS_MAGNET_OK_READBACK = 0;
      if ((global_data_A37342_001.control_state == STATE_POWER_TEST) || (global_data_A37342_001.control_state == STATE_OPERATE)) {
	_FAULT_MAGNET_UNDER_VOLTAGE_ABSOLUTE = 1;
      }
    } else {
      if (ETMCanSlaveGetSyncMsgResetEnable()) {
	_FAULT_MAGNET_UNDER_VOLTAGE_ABSOLUTE = 0;
      }
    }    
    
    // DPARKER WHAT OF THIS DO WE STILL NEED ???????
    if ((global_data_A37342_001.control_state == STATE_OPERATE) || (global_data_A37342_001.control_state == STATE_POWER_TEST)) {
      global_data_A37342_001.analog_input_heater_current.target_value = global_data_A37342_001.analog_output_heater_current.set_point;
      global_data_A37342_001.analog_input_heater_voltage.target_value = ETMScaleFactor16(global_data_A37342_001.analog_output_heater_current.set_point,
											 MACRO_DEC_TO_SCALE_FACTOR_16(NOMINAL_HEATER_RESISTANCE),
											 0);
    } else {
      global_data_A37342_001.analog_input_heater_current.target_value = 0;
      global_data_A37342_001.analog_input_heater_voltage.target_value = 0;
    }

    // Set DAC outputs
    if ((global_data_A37342_001.control_state == STATE_OPERATE) || (global_data_A37342_001.control_state == STATE_POWER_TEST)) {
      ETMAnalogSetOutput(&global_data_A37342_001.analog_output_heater_current, global_data_A37342_001.can_heater_current_set_point);
      if (ETMCanSlaveIsNextPulseLevelHigh()) {
	ETMAnalogSetOutput(&global_data_A37342_001.analog_output_electromagnet_current, global_data_A37342_001.can_magnet_current_set_point_high_energy);
      } else {
	ETMAnalogSetOutput(&global_data_A37342_001.analog_output_electromagnet_current, global_data_A37342_001.can_magnet_current_set_point_low_energy);
      }
      ETMAnalogScaleCalibrateDACSetting(&global_data_A37342_001.analog_output_heater_current);
      ETMAnalogScaleCalibrateDACSetting(&global_data_A37342_001.analog_output_electromagnet_current);

      WriteLTC265XTwoChannels(&U14_LTC2654,
			      LTC265X_WRITE_AND_UPDATE_DAC_A,
			      global_data_A37342_001.analog_output_electromagnet_current.dac_setting_scaled_and_calibrated,
			      LTC265X_WRITE_AND_UPDATE_DAC_C,
			      global_data_A37342_001.analog_output_heater_current.dac_setting_scaled_and_calibrated);

    } else {
      WriteLTC265XTwoChannels(&U14_LTC2654,
			      LTC265X_WRITE_AND_UPDATE_DAC_A,
			      global_data_A37342_001.analog_output_electromagnet_current.disabled_dac_set_point,
			      LTC265X_WRITE_AND_UPDATE_DAC_C,
			      global_data_A37342_001.analog_output_heater_current.disabled_dac_set_point);
    }
  }
}


void FlashLEDs(void) {
  
  switch (((global_data_A37342_001.power_up_test_timer >> 4) & 0b11)) {
    
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


void InitializeA37342_001(void) {
  // Initialize the status register and load the inhibit and fault masks
  _FAULT_REGISTER = 0;
  _CONTROL_REGISTER = 0;
  _WARNING_REGISTER = 0;
  _NOT_LOGGED_REGISTER = 0;

  global_data_A37342_001.analog_output_electromagnet_current.set_point = 0;
  global_data_A37342_001.analog_output_heater_current.set_point = 0;

  // Configure ADC Interrupt
  _ADIP   = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)

  // Initialize all I/O Registers
  TRISA = A37342_001_TRISA_VALUE;
  TRISB = A37342_001_TRISB_VALUE;
  TRISC = A37342_001_TRISC_VALUE;
  TRISD = A37342_001_TRISD_VALUE;
  TRISF = A37342_001_TRISF_VALUE;
  TRISG = A37342_001_TRISG_VALUE;

  PIN_SELECT_DAC_C_D = OLL_SELECT_DAC_C;

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

  if (do_fast_startup) {
    // Do Not Adjust the DAC right now
    EnableHeaterMagnetOutputs();
  } else {
    // Initialize LTC DAC
    SetupLTC265X(&U14_LTC2654, ETM_SPI_PORT_2, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);
    DisableHeaterMagnetOutputs();
  }
  
  //function that resets i2c bus
  // DPARKER REMOVE FOR NOW Reset_EEPROM_I2C(_PIN_RG2, _PIN_RG3);

  // Initialize the External EEprom
  ETMEEPromUseExternal();
  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, 400000, EEPROM_I2C_ADDRESS_0, 1);
 
  // Initialize the Can module
  ETMCanSlaveInitialize(CAN_PORT_1, FCY_CLK, ETM_CAN_ADDR_HEATER_MAGNET_BOARD, _PIN_RG13, 4, _PIN_RG13, _PIN_RG13);
  ETMCanSlaveLoadConfiguration(37342, 1, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_MINOR_REV);

 

  // Initialize the Analog Input * Output Scaling
  ETMAnalogInitializeOutput(&global_data_A37342_001.analog_output_electromagnet_current,
			    MACRO_DEC_TO_SCALE_FACTOR_16(1.311),
			    OFFSET_ZERO,
			    ANALOG_OUTPUT_NO_CALIBRATION,
			    ELECTROMAGNET_MAX_IPROG,
			    ELECTROMAGNET_MIN_IPROG,
			    0);

  ETMAnalogInitializeOutput(&global_data_A37342_001.analog_output_heater_current,
			    MACRO_DEC_TO_SCALE_FACTOR_16(1.311),
			    OFFSET_ZERO,
			    ANALOG_OUTPUT_NO_CALIBRATION,
			    HEATER_MAX_IPROG,
			    HEATER_MIN_IPROG,
			    0);

  ETMAnalogInitializeInput(&global_data_A37342_001.analog_input_electromagnet_current,
			   MACRO_DEC_TO_SCALE_FACTOR_16(1.563),
			   OFFSET_ZERO,
			   ANALOG_INPUT_NO_CALIBRATION,
			   ELECTROMAGNET_CURRENT_OVER_TRIP,
			   ELECTROMAGNET_CURRENT_UNDER_TRIP,
			   ELECTROMAGNET_CURRENT_RELATIVE_TRIP,
			   ELECTROMAGNET_CURRENT_RELATIVE_FLOOR,
			   ELECTROMAGNET_CURRENT_TRIP_TIME,
                           ELECTROMAGNET_CURRENT_ABSOLUTE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A37342_001.analog_input_heater_current,
			   MACRO_DEC_TO_SCALE_FACTOR_16(1.563),
			   OFFSET_ZERO,
			   ANALOG_INPUT_NO_CALIBRATION,
			   HEATER_CURRENT_OVER_TRIP,
			   HEATER_CURRENT_UNDER_TRIP,
			   HEATER_CURRENT_RELATIVE_TRIP,
			   HEATER_CURRENT_RELATIVE_FLOOR,
			   HEATER_CURRENT_TRIP_TIME,
                           HEATER_CURRENT_ABSOLUTE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A37342_001.analog_input_electromagnet_voltage,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.4690),
			   OFFSET_ZERO,
			   ANALOG_INPUT_NO_CALIBRATION,
			   ELECTROMAGNET_VOLTAGE_OVER_TRIP,
			   ELECTROMAGNET_VOLTAGE_UNDER_TRIP,
			   ELECTROMAGNET_VOLTAGE_RELATIVE_TRIP,
			   ELECTROMAGNET_VOLTAGE_RELATIVE_FLOOR,
			   ELECTROMAGNET_VOLTAGE_TRIP_TIME,
                           ELECTROMAGNET_VOLTAGE_ABSOLUTE_TRIP_TIME);  //changed scale from .6250

  ETMAnalogInitializeInput(&global_data_A37342_001.analog_input_heater_voltage,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.4690),
			   OFFSET_ZERO,
			   ANALOG_INPUT_NO_CALIBRATION,
			   HEATER_VOLTAGE_OVER_TRIP,
			   HEATER_VOLTAGE_UNDER_TRIP,
			   HEATER_VOLTAGE_RELATIVE_TRIP,
			   HEATER_VOLTAGE_RELATIVE_FLOOR,
			   HEATER_VOLTAGE_TRIP_TIME,
                           HEATER_VOLTAGE_ABSOLUTE_TRIP_TIME); //changed scale from .6250

  ETMAnalogInitializeInput(&global_data_A37342_001.analog_input_5v_mon,
                           MACRO_DEC_TO_SCALE_FACTOR_16(.12500),
                           OFFSET_ZERO,
                           ANALOG_INPUT_NO_CALIBRATION,
                           PWR_5V_OVER_FLT,
                           PWR_5V_UNDER_FLT,
                           NO_TRIP_SCALE,
                           NO_FLOOR,
                           NO_COUNTER,
                           NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A37342_001.analog_input_15v_mon,
                           MACRO_DEC_TO_SCALE_FACTOR_16(.25063),
                           OFFSET_ZERO,
                           ANALOG_INPUT_NO_CALIBRATION,
                           PWR_15V_OVER_FLT,
                           PWR_15V_UNDER_FLT,
                           NO_TRIP_SCALE,
                           NO_FLOOR,
                           NO_COUNTER,
                           NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A37342_001.analog_input_neg_15v_mon,
                           MACRO_DEC_TO_SCALE_FACTOR_16(.06250),
                           OFFSET_ZERO,
                           ANALOG_INPUT_NO_CALIBRATION,
                           PWR_NEG_15V_OVER_FLT,
                           PWR_NEG_15V_UNDER_FLT,
                           NO_TRIP_SCALE,
                           NO_FLOOR,
                           NO_COUNTER,
                           NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A37342_001.analog_input_pic_adc_test_dac,
                           MACRO_DEC_TO_SCALE_FACTOR_16(1),
                           OFFSET_ZERO,
                           ANALOG_INPUT_NO_CALIBRATION,
                           ADC_DAC_TEST_OVER_FLT,
                           ADC_DAC_TEST_UNDER_FLT,
                           NO_TRIP_SCALE,
                           NO_FLOOR,
                           NO_COUNTER,
                           NO_COUNTER);

  // Initialize the digital Inputs
  ETMDigitalInitializeInput(&global_data_A37342_001.digital_input_temp_switch, !ILL_TEMP_SWITCH_FAULT, 10);
  ETMDigitalInitializeInput(&global_data_A37342_001.digital_input_crowbar_up, !ILL_RELAY_OPEN, 50);

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

  // Copy Data From Buffer to RAM
  if (_BUFS) {
    // read ADCBUF 0-7
    global_data_A37342_001.analog_input_electromagnet_current.adc_accumulator   += ADCBUF0;
    global_data_A37342_001.analog_input_heater_current.adc_accumulator          += ADCBUF1;
    global_data_A37342_001.analog_input_electromagnet_voltage.adc_accumulator   += ADCBUF2;
    global_data_A37342_001.analog_input_heater_voltage.adc_accumulator          += ADCBUF3;
    
    global_data_A37342_001.analog_input_15v_mon.adc_accumulator                 += ADCBUF4;
    global_data_A37342_001.analog_input_neg_15v_mon.adc_accumulator             += ADCBUF5;
    global_data_A37342_001.analog_input_5v_mon.adc_accumulator                  += ADCBUF6;
    global_data_A37342_001.analog_input_pic_adc_test_dac.adc_accumulator        += ADCBUF7;
    
  } else {
    // read ADCBUF 8-15
    global_data_A37342_001.analog_input_electromagnet_current.adc_accumulator   += ADCBUF8;
    global_data_A37342_001.analog_input_heater_current.adc_accumulator          += ADCBUF9;
    global_data_A37342_001.analog_input_electromagnet_voltage.adc_accumulator   += ADCBUFA;
    global_data_A37342_001.analog_input_heater_voltage.adc_accumulator          += ADCBUFB;
    
    global_data_A37342_001.analog_input_15v_mon.adc_accumulator                 += ADCBUFC;
    global_data_A37342_001.analog_input_neg_15v_mon.adc_accumulator             += ADCBUFD;
    global_data_A37342_001.analog_input_5v_mon.adc_accumulator                  += ADCBUFE;
    global_data_A37342_001.analog_input_pic_adc_test_dac.adc_accumulator        += ADCBUFF;
    
  }
  
  global_data_A37342_001.accumulator_counter++ ;
  
  if (global_data_A37342_001.accumulator_counter >= 128) {
    
    global_data_A37342_001.analog_input_electromagnet_current.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
    global_data_A37342_001.analog_input_electromagnet_current.filtered_adc_reading = global_data_A37342_001.analog_input_electromagnet_current.adc_accumulator;
    global_data_A37342_001.analog_input_electromagnet_current.adc_accumulator = 0;
    
    global_data_A37342_001.analog_input_heater_current.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
    global_data_A37342_001.analog_input_heater_current.filtered_adc_reading = global_data_A37342_001.analog_input_heater_current.adc_accumulator;
    global_data_A37342_001.analog_input_heater_current.adc_accumulator = 0;
    
    global_data_A37342_001.analog_input_electromagnet_voltage.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
    global_data_A37342_001.analog_input_electromagnet_voltage.filtered_adc_reading = global_data_A37342_001.analog_input_electromagnet_voltage.adc_accumulator;
    global_data_A37342_001.analog_input_electromagnet_voltage.adc_accumulator = 0;
    
    global_data_A37342_001.analog_input_heater_voltage.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
    global_data_A37342_001.analog_input_heater_voltage.filtered_adc_reading = global_data_A37342_001.analog_input_heater_voltage.adc_accumulator;
    global_data_A37342_001.analog_input_heater_voltage.adc_accumulator = 0;
    
    global_data_A37342_001.analog_input_15v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
    global_data_A37342_001.analog_input_15v_mon.filtered_adc_reading = global_data_A37342_001.analog_input_15v_mon.adc_accumulator;
    global_data_A37342_001.analog_input_15v_mon.adc_accumulator = 0;
    
    global_data_A37342_001.analog_input_neg_15v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
    global_data_A37342_001.analog_input_neg_15v_mon.filtered_adc_reading = global_data_A37342_001.analog_input_neg_15v_mon.adc_accumulator;
    global_data_A37342_001.analog_input_neg_15v_mon.adc_accumulator = 0;
    
    global_data_A37342_001.analog_input_5v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
    global_data_A37342_001.analog_input_5v_mon.filtered_adc_reading = global_data_A37342_001.analog_input_5v_mon.adc_accumulator;
    global_data_A37342_001.analog_input_5v_mon.adc_accumulator = 0;
    
    global_data_A37342_001.analog_input_pic_adc_test_dac.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
    global_data_A37342_001.analog_input_pic_adc_test_dac.filtered_adc_reading = global_data_A37342_001.analog_input_pic_adc_test_dac.adc_accumulator;
    global_data_A37342_001.analog_input_pic_adc_test_dac.adc_accumulator = 0;
    
    
    global_data_A37342_001.accumulator_counter = 0;
  }
}


void ETMCanSlaveExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;

  index_word = message_ptr->word3;
  switch (index_word)
    {
    case ETM_CAN_REGISTER_HEATER_MAGNET_SET_1_CURRENT_SET_POINT:
      global_data_A37342_001.can_heater_current_set_point = message_ptr->word1;  // heater
      global_data_A37342_001.can_magnet_current_set_point_high_energy = message_ptr->word0;  // magnet
      global_data_A37342_001.can_magnet_current_set_point_low_energy = message_ptr->word2; // magnet low energy
      _CONTROL_NOT_CONFIGURED = 0;
      setup_done = 1;
      break;

    default:
      ETMCanSlaveIncrementInvalidIndex();
      break;

    }
}


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  Nop();
  Nop();
  __asm__ ("Reset");
}
