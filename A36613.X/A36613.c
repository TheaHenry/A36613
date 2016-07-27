#include "A36613.h"

#define OLL_LED_ON    0
#define OLL_LED_OFF   1

void TurnHeaterOn(void);
void TurnHeaterOff(void);
unsigned int CheckHeaterFault(void);
void DoA36613(void);
void FlashLED(void);
#define MAX_AUTO_RESET_HEATER_COUNT  5


// This is the firmware for modulator- HV section

// comment- a PID type control is commented out for optional use. I believe it is working, but I haven't adjusted the coeficients for optimal performance.

/* -------------------- Device configuration --------------------------  */


// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (General Segment may be written)
#pragma config GSS = OFF                // General Segment Code Protection (General Segment Code protect is disabled)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Oscillator Source Selection (Primary Oscillator (XT, HS, EC) with PLL)
#pragma config IESO = OFF               // Internal External Switch Over Mode (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = EC              // Primary Oscillator Source (EC (External Clock) Mode)
#pragma config OSCIOFNC = OFF            // OSC2 Pin Function (OSC2 is general purpose digital I/O pin)
#pragma config FCKSM = CSECME           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are enabled)


// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR32            // WDT Prescaler (1:32)
#pragma config WINDIS = ON              // Watchdog Timer Window (Watchdog Timer in Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR16           // POR Timer Value (16ms)
#pragma config ALTSS1 = OFF             // Enable Alternate SS1 pin bit (SS1 is selected as the I/O pin for SPI1)
#pragma config ALTQIO = OFF             // Enable Alternate QEI1 pin bit (QEA1, QEB1, INDX1 are selected as inputs to QEI1)

// FICD
#pragma config ICS = PGD2               // Comm Channel Select (Communicate on PGC2/EMUC2 and PGD2/EMUD2)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is disabled)

// FCMP
#pragma config HYST0 = HYST45           // Even Comparator Hysteresis Select (45 mV Hysteresis)
#pragma config CMPPOL0 = POL_FALL       // Comparator Hysteresis Polarity (for even numbered comparators) (Hysteresis is applied to falling edge)
#pragma config HYST1 = HYST45           // Odd Comparator Hysteresis Select (45 mV Hysteresis)
#pragma config CMPPOL1 = POL_FALL       // Comparator Hysteresis Polarity (for odd numbered comparators) (Hysteresis is applied to falling edge)


//------------------------------------------------------------------------//


void DoStateMachine(void);
void InitializeA36613(void);
void UpdateTopVoltage(void);
void UpdateHeaterPWM(void);
void ConfigureClock(void);
void UpdateHeaterFaults(void);
//void ResetHeater(void);


unsigned int top1_voltage_monitor_accumulator;
unsigned int top1_voltage_accumulator_size;
unsigned int top2_voltage_monitor_accumulator;
unsigned int top1_voltage_feedback_accumulator;
unsigned int top1_fdbk_voltage_accumulator_size;
unsigned int top2_voltage_feedback_accumulator;
unsigned int top2_fdbk_voltage_accumulator_size;

unsigned int bias_feedback_accumulator;

unsigned char transmit_message;
unsigned int heater_reset_counter;

#define ACCUMULATOR_SIZE  64 

ControlData global_data_A36613;
LTC265X U4_LTC2654;
AnalogInput Heater_output_voltage;
AnalogInput Heater1_current;
AnalogInput Heater2_current;


/*
  tPID Heater_PID;                                                      // Declare a PID Data Structure named, Heater_PID                                            
  fractional abcCoefficient[3]__attribute__((space(xmemory)));  // find a place to put the data, use 
  fractional controlHistory[3]__attribute__((space(ymemory)));  // large data model in project build options 
  fractional kCoeffs[] = {Heater_Kp,Heater_Ki,Heater_Kd};        // declare Kp, Ki and Kd    
*/

int main(void) {

  ConfigureClock();

  global_data_A36613.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}

void DoStateMachine(void) {
  switch (global_data_A36613.control_state) {
    
  case STATE_STARTUP:
    InitializeA36613();
    transmit_message=0xF0;
    global_data_A36613.heater_warmup_fault_counter = 0;
    while(!A36613ReceiveData()) {
      if (_T3IF ) { 
	//every 500us
	_T3IF = 0;
        A36613TransmitData(transmit_message);
      }
    }
    global_data_A36613.control_state = STATE_WARMUP;
    break;
    
  case STATE_WARMUP:
    PIN_LED_TEST_POINT_A = OLL_LED_ON;
    PIN_LED_TEST_POINT_B = OLL_LED_OFF;
    global_data_A36613.heater_ramp_up_counter = 0;
    global_data_A36613.status |= HEATER_NOT_READY;
    TurnHeaterOn();
    while(global_data_A36613.control_state == STATE_WARMUP) {
      DoA36613();

      if ((Heater_output_voltage.reading_scaled_and_calibrated > (global_data_A36613.heater_set_voltage - 100)) && (global_data_A36613.heater_ramp_up_counter > 20000)) {
	// We are within 100mV of the program voltage & the warmup has been going for at least 10 seconds
	if (CheckHeaterFault()) {
	  global_data_A36613.control_state = STATE_HEATER_FAULT;
	} else {
	  global_data_A36613.control_state = STATE_READY;
 	}
      }

      if (global_data_A36613.heater_ramp_up_counter >= MAX_HEATER_RAMP_UP) {
        global_data_A36613.control_state = STATE_HEATER_FAULT;
      }
    }
    break;

  case STATE_READY:
    PIN_LED_TEST_POINT_A = OLL_LED_OFF;
    PIN_LED_TEST_POINT_B = OLL_LED_ON;
    global_data_A36613.heater_warmup_fault_counter = 0;
    global_data_A36613.status &= ~HEATER_NOT_READY;
    while(global_data_A36613.control_state == STATE_READY) {
      DoA36613();
      
      if (CheckHeaterFault()) {
	global_data_A36613.control_state = STATE_HEATER_FAULT;
      }
      
    }
    break;
 

  case STATE_HEATER_FAULT:
    PIN_LED_TEST_POINT_B = OLL_LED_ON;
    PIN_LED_TEST_POINT_A = OLL_LED_ON;
    global_data_A36613.heater_warmup_fault_counter++;
    TurnHeaterOff();
    global_data_A36613.status |= HEATER_NOT_READY;
    global_data_A36613.fault_off_counter = 0;
    while (global_data_A36613.control_state == STATE_HEATER_FAULT) {
      DoA36613();

#define FAULT_OFF_TIME 10000       // 5 Seconds

      if (global_data_A36613.fault_off_counter >= FAULT_OFF_TIME) {
	global_data_A36613.fault_off_counter = FAULT_OFF_TIME;

	if ((global_data_A36613.heater_warmup_fault_counter <= MAX_AUTO_RESET_HEATER_COUNT) || (global_data_A36613.status & 0x80)) {
	  global_data_A36613.control_state = STATE_WARMUP;
	  global_data_A36613.status &= 0b11110000;
	  ETMAnalogClearFaultCounters(&Heater_output_voltage);
	  ETMAnalogClearFaultCounters(&Heater1_current);
	  ETMAnalogClearFaultCounters(&Heater2_current);
	}
      }
    }
    break;
    
  default:
    global_data_A36613.control_state = STATE_HEATER_FAULT;
    break;
    
  }
}


void DoA36613(void) {
  A36613ReceiveData();
  if (_T3IF ) {
    if (global_data_A36613.heater_ramp_voltage < global_data_A36613.heater_set_voltage) {
      global_data_A36613.heater_ramp_voltage++;
    } else {
      global_data_A36613.heater_ramp_voltage = global_data_A36613.heater_set_voltage;
    }
    //every 500us
    _T3IF = 0;
    A36613TransmitData(transmit_message);
    transmit_message++;
    if (transmit_message > 0xF4) {
      transmit_message = 0xF0;
    }
    if (global_data_A36613.control_state != STATE_HEATER_FAULT) {
      UpdateHeaterPWM();
    }
    UpdateHeaterFaults();
    FlashLED();
    global_data_A36613.heater_ramp_up_counter++;
    global_data_A36613.fault_off_counter++;
  }
}


void FlashLED(void) {
  // This will flash the Led once every 500ms
  global_data_A36613.led_flash_counter++;
  if (global_data_A36613.led_flash_counter >= 1000) {
    global_data_A36613.led_flash_counter = 0;
    if(PIN_LED_OPERATIONAL_GREEN == 1) {
      //LED turns on once a second
      PIN_LED_OPERATIONAL_GREEN = 0;
    } else {
      PIN_LED_OPERATIONAL_GREEN = 1;
    }
  }
}



void ConfigureClock(void) {
  //**********************Setup Clock speeds*********************************//
  //   Fin=7.3MHz
  //   Fosc = Fin*M/(N1+N2), Fcy=Fosc/2
  //   Fosc= 7.49 * 50 / (2 * 3) = 62.4MHz, Fcy= 31.2MIPS
  //*************************************************************************//
  // Configure PLL prescaler, PLL postscaler, PLL divisor
  //_TUN = 4;//tune FRC to 7.49MHz
  //PLLFBD = 48;// M = 50
  //CLKDIVbits.PLLPOST=0;// N2 = 2
  //CLKDIVbits.PLLPRE=1;// N1 = 3
  // Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
  //__builtin_write_OSCCONH(0x01);
  //__builtin_write_OSCCONL(OSCCON | 0x01);
  // Wait for Clock switch to occur
  //while (OSCCONbits.COSC != 0b001);
  // Wait for PLL to lock
  //while(OSCCONbits.LOCK!=1) {};


  CLKDIVbits.PLLPOST = 0b00; // Set PLL Postscaler (N2) to 2.
  CLKDIVbits.PLLPRE = 0b00000; // Set PLL Prescaler (N1) to 2.
  PLLFBD = 58; // Set PLL Divider (M) to 60.
  while(OSCCONbits.LOCK!=1) {};
  
  //Auxilary clock configuration
  _SELACLK = 0; //PLL output (FVCO) provides the source clock for the auxiliary clock divider
  _APSTSCLR = 0b111; // Auxiliary Clock Output Divider 1:1
  
  /* Disable Watch Dog Timer */
  RCONbits.SWDTEN = 0;
}


void InitializeA36613(void) {
  //Timer setup
  T3CON = T3CON_VALUE;
  PR3 = PR3_VALUE_10_MILLISECONDS;
  _T3IF = 0;
  DisableIntT3;
  
  T1CON = T1CON_VALUE;
  PR1 = PR1_VALUE_10_US;
  _T1IF = 0;
  DisableIntT1;
  
  
  //init global variables
  global_data_A36613.heater_set_voltage = 0x0000;
  global_data_A36613.top1_set_voltage = 0; //2V from DAC
  global_data_A36613.top2_set_voltage = 0; //2V from DAC
  global_data_A36613.top1_voltage_monitor = 0x0000;
  global_data_A36613.top2_voltage_monitor = 0x0000;
  global_data_A36613.top1_voltage_feedback = 0;
  global_data_A36613.top2_voltage_feedback = 0;
  global_data_A36613.bias_feedback = 0x0000;
  global_data_A36613.status = 0x80;
  PIN_LED_OPERATIONAL_GREEN = 0;

  //set tris
  TRISB = A36613_TRISB_VALUE;
  TRISC = A36613_TRISC_VALUE;
  TRISD = A36613_TRISD_VALUE;
  TRISE = A36613_TRISE_VALUE;
  TRISF = A36613_TRISF_VALUE;
  TRISG = A36613_TRISG_VALUE;

  //init PWM
  OpenSmpsPWM(PWM_MOD_DIS | PWM_IDLE_CONT | PWM_SEVT_INT_DIS | PWM_PER_UPDATE_BOUND, PWM_PERIOD, INITIAL_PWM_DC, 0x0000); //PWM off, no sync, continue during idle, no sepecial event trigger, update DC on next cycle
  ConfigSmpsPWMInputClkDiv(PWM_INPUT_CLK_DIV1); //Select PWM prescaler of 1:2
  ConfigSmpsPWM1(PWM1_D_CYLE_MDC | PWM1_DT_DIS, PWM1_H_PIN_GPIO | PWM1_L_PIN_EN | PWM1_IO_PIN_PAIR_RED, 0x0000, 0x0000, 0x0000 ); //Master Duty Cycle selection; Dead time function disabled; No interrupts; Use master time base; redundant pai

  //Heater_PID.abcCoefficients = &abcCoefficient[0];    /*Set up pointer to derived coefficients */
  //Heater_PID.controlHistory = &controlHistory[0];     /*Set up pointer to controller history samples */
  //PIDCoeffCalc( &kCoeffs[0], &Heater_PID);
  //PIDInit( &Heater_PID);


  ETMAnalogInitializeInput(&Heater_output_voltage,
			   HEATER_VOLTAGE_SCALING_FACTOR,
			   OFFSET_ZERO,
			   ANALOG_INPUT_NO_CALIBRATION,
			   HEATER_OVERVOLTAGE_TRIP,
			   HEATER_UNDERVOLTAGE_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   ABSOLUTE_TRIP_COUNTER);


  ETMAnalogInitializeInput(&Heater1_current,
			   HEATER_CURRENT_SCALING_FACTOR,
			   OFFSET_ZERO,
			   ANALOG_INPUT_NO_CALIBRATION,
			   HEATER_OVERCURRENT_TRIP,
			   HEATER_UNDERCURRENT_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   ABSOLUTE_TRIP_COUNTER);  

  ETMAnalogInitializeInput(&Heater2_current,
			   HEATER_CURRENT_SCALING_FACTOR,
			   OFFSET_ZERO,
			   ANALOG_INPUT_NO_CALIBRATION,
			   HEATER_OVERCURRENT_TRIP,
			   HEATER_UNDERCURRENT_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   ABSOLUTE_TRIP_COUNTER);  


  SetupLTC265X(&U4_LTC2654, ETM_SPI_PORT_2, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RE6, _PIN_RE7);


  InitializeA36613Serial();


  // ---- Configure the ADC Module ------------ //

  ADCON = ADCON_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCPC0 = ADCPC0_SETTING;             // Set trigger and enable each channel
  ADCPC1  = ADCPC1_SETTING;              // Set trigger and enable each channel
  ADCPC2 = ADCPC2_SETTING;             // Set trigger and enable each channel

  ClearSmpsADCIntPair0();
  ClearSmpsADCIntPair1();
  ClearSmpsADCIntPair2();
  ClearSmpsADCIntPair3();
  ClearSmpsADCIntPair4();
  ClearSmpsADCIntPair5();
  _ADCP0IE = 1;
  _ADCP1IE = 1;
  _ADCP2IE = 1;
  _ADCP3IE = 1;
  _ADCP4IE = 1;
  _ADCP5IE = 1;
  _ADCP0IP = 2;
  _ADCP1IP = 2;
  _ADCP2IP = 2;
  _ADCP3IP = 2;
  _ADCP4IP = 2;
  _ADCP5IP = 2;


  top1_voltage_monitor_accumulator = 0;
  top1_voltage_accumulator_size = 0;
  top2_voltage_monitor_accumulator = 0;
  top1_voltage_feedback_accumulator = 0;
  top1_fdbk_voltage_accumulator_size = 0;
  top2_voltage_feedback_accumulator = 0;
  top2_fdbk_voltage_accumulator_size = 0;
  bias_feedback_accumulator = 0;

  heater_reset_counter = 0;
  

  _ADON = 1; //Turn ADC on

}

#define HEATER_SMALL_STEP   0x0001
#define HEATER_LARGE_STEP   0x0009
#define HEATER_MAX_DUTY     (PWM_PERIOD * 0.7)

void UpdateHeaterPWM(void) {
  unsigned int Heater_error;
  unsigned int master_duty_cycle_register;

  master_duty_cycle_register = MDC;

  if ((Heater1_current.reading_scaled_and_calibrated >= HEATER_MAX_CURRENT) || (Heater2_current.reading_scaled_and_calibrated >= HEATER_MAX_CURRENT)) {
    master_duty_cycle_register-=HEATER_SMALL_STEP;
  } else if (Heater_output_voltage.reading_scaled_and_calibrated > global_data_A36613.heater_ramp_voltage) {
    Heater_error = Heater_output_voltage.reading_scaled_and_calibrated - global_data_A36613.heater_ramp_voltage;
    if (Heater_error > 100) {
      master_duty_cycle_register-=HEATER_SMALL_STEP;
    }
    if (Heater_error > 1000) {
      master_duty_cycle_register-= HEATER_LARGE_STEP;
    }
  } else if (Heater_output_voltage.reading_scaled_and_calibrated < global_data_A36613.heater_ramp_voltage) {
    Heater_error = global_data_A36613.heater_ramp_voltage - Heater_output_voltage.reading_scaled_and_calibrated;
    if (Heater_error > 100) {
      master_duty_cycle_register+=HEATER_SMALL_STEP;
    }
    if (Heater_error > 1000) {
      master_duty_cycle_register+= HEATER_LARGE_STEP;
    }
  } 
  
  if (master_duty_cycle_register <= 64) {
    master_duty_cycle_register = 64;
  }
  
  if (master_duty_cycle_register >= HEATER_MAX_DUTY) {
    master_duty_cycle_register= HEATER_MAX_DUTY;
  }
  
  MDC = master_duty_cycle_register;

}


void TurnHeaterOn(void) {
  _PTEN = 1;
  MDC = 0;
  global_data_A36613.heater_ramp_voltage = 0;
}

void TurnHeaterOff(void) {
  _PTEN = 0;
  MDC = 0;
}


void UpdateTopVoltage(void) {
  WriteLTC265X(&U4_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_A , global_data_A36613.top1_set_voltage);
  WriteLTC265X(&U4_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_B , global_data_A36613.top2_set_voltage);
}

void UpdateHeaterFaults(void) {
  
  //Over voltage condition
  if (ETMAnalogCheckOverAbsolute(&Heater_output_voltage)) {
    global_data_A36613.status |= HEATER_OVERVOLTAGE_FLT;
  }

  //Over current condition
  if (ETMAnalogCheckOverAbsolute(&Heater1_current) || ETMAnalogCheckOverAbsolute(&Heater2_current)) {
      global_data_A36613.status |= HEATER_OVERCURRENT_FLT;
  }

  // Under current condition
  if (ETMAnalogCheckUnderAbsolute(&Heater1_current) || ETMAnalogCheckUnderAbsolute(&Heater2_current)) {
    // heater undervoltage and undercurrent should only be checked after warmup.
    global_data_A36613.status |= HEATER_UNDERCURRENT_FLT;
  } else if (global_data_A36613.control_state == STATE_WARMUP) {
    global_data_A36613.status &= ~HEATER_UNDERCURRENT_FLT;
  }

  //Under voltage condition  
  if (ETMAnalogCheckUnderAbsolute(&Heater_output_voltage)) { 
    // heater undervoltage and undercurrent should only be checked after warmup.
    global_data_A36613.status |= HEATER_UNDERVOLTAGE_FLT;
  } else if (global_data_A36613.control_state == STATE_WARMUP) {
    global_data_A36613.status &= ~HEATER_UNDERVOLTAGE_FLT;
  }
}

unsigned int CheckHeaterFault(void) {
  if (global_data_A36613.status & 0b00001111) {
    return 1;
  }
  return 0;
}

void __attribute__((interrupt, auto_psv)) _ADCP0Interrupt(void) {
  top1_voltage_feedback_accumulator += ADCBUF1; // averages 64 samples of data (add 64, then shift left by 6). Then shifts the data to get a 8bit number (additional shift of 2).
  top1_fdbk_voltage_accumulator_size++;
  
  if (top1_fdbk_voltage_accumulator_size == ACCUMULATOR_SIZE) {
    //Check if 64 samples have accumulated
    global_data_A36613.top1_voltage_feedback = top1_voltage_feedback_accumulator; // store averaged value in global struct.
    top1_fdbk_voltage_accumulator_size = 0; // reset accumulator
    top1_voltage_feedback_accumulator = 0;    
  }
  _ADCP0IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ADCP1Interrupt (void) {
  top2_voltage_feedback_accumulator += ADCBUF3; // averages 64 samples of data (add 64, then shift left by 6). Then shifts the data to get a 8bit number (additional shift of 2).
  top2_fdbk_voltage_accumulator_size++;
  
  if (top2_fdbk_voltage_accumulator_size == ACCUMULATOR_SIZE) {
    //Check if 64 samples have accumulated
    global_data_A36613.top2_voltage_feedback = top2_voltage_feedback_accumulator; // store averaged value in global struct.
    top2_fdbk_voltage_accumulator_size = 0; // reset accumulator
    top2_voltage_feedback_accumulator = 0;
  }
  _ADCP1IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ADCP2Interrupt (void) {
  Heater_output_voltage.filtered_adc_reading += ADCBUF4; // averages 64 samples of data (add 64, then shift left by 6). Then shifts the data to get a 8bit number (additional shift of 2).
  bias_feedback_accumulator += ADCBUF5;
  Heater_output_voltage.adc_accumulator++;
  if (Heater_output_voltage.adc_accumulator == ACCUMULATOR_SIZE) {
    //Check if all samples have accumulated
    ETMAnalogScaleCalibrateADCReading(&Heater_output_voltage); // store averaged value in global struct.
    Heater_output_voltage.adc_accumulator = 0;
    Heater_output_voltage.filtered_adc_reading = 0;
    global_data_A36613.bias_feedback = bias_feedback_accumulator;
    bias_feedback_accumulator = 0;
    /* 
       This is another PID segment- where you update the PWM duty. Should only be used if using PID.
       Heater_PID.measuredOutput = global_data_A36613.Heater_output_voltage; //turn 16bit number to fractional
       Heater_PID.controlReference = global_data_A36613.heater_set_voltage;
       PID(&Heater_PID);
       MDC = Heater_PID.controlOutput;
       if (MDC <= 64 || MDC>= 0x8000)
       {
       MDC = 64;
       }
       
       if (MDC >= PWM_PERIOD)
       {
       MDC= PWM_PERIOD * 0.5;
       }
    */
  }
  _ADCP2IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ADCP3Interrupt (void) {
  top1_voltage_monitor_accumulator += ADCBUF7; // averages 64 samples of data (add 64, then shift left by 6). Then shifts the data to get a 8bit number (additional shift of 2).
  top1_voltage_accumulator_size++;

  if (top1_voltage_accumulator_size == ACCUMULATOR_SIZE) { 
    //Check if 64 samples have accumulated
    global_data_A36613.top1_voltage_monitor= top1_voltage_monitor_accumulator; // store averaged value in global struct.
    top1_voltage_accumulator_size = 0; // reset accumulator
    top1_voltage_monitor_accumulator = 0;
  }
  _ADCP3IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ADCP4Interrupt (void) {
  Heater1_current.filtered_adc_reading += ADCBUF8; // averages 64 samples of data (add 64, then shift left by 6). Then shifts the data to get a 8bit number (additional shift of 2).
  top2_voltage_monitor_accumulator += ADCBUF9;
  Heater1_current.adc_accumulator++;

  if (Heater1_current.adc_accumulator == ACCUMULATOR_SIZE) { 
    //Check if 64 samples have accumulated
    ETMAnalogScaleCalibrateADCReading(&Heater1_current); // store averaged value in global struct.
    global_data_A36613.top2_voltage_monitor= top2_voltage_monitor_accumulator; // store averaged value in global struct.
    Heater1_current.adc_accumulator = 0; // reset accumulator
    Heater1_current.filtered_adc_reading = 0;
    top2_voltage_monitor_accumulator = 0;
  }
  _ADCP4IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ADCP5Interrupt (void) {
  Heater2_current.filtered_adc_reading += ADCBUF10; // averages 64 samples of data (add 64, then shift left by 6). Then shifts the data to get a 8bit number (additional shift of 2).
  Heater2_current.adc_accumulator++;
  
  if (Heater2_current.adc_accumulator == ACCUMULATOR_SIZE) {
    //Check if 64 samples have accumulated
    ETMAnalogScaleCalibrateADCReading(&Heater2_current); // store averaged value in global struct.
    Heater2_current.adc_accumulator = 0; // reset accumulator
    Heater2_current.filtered_adc_reading = 0;
  }
  _ADCP5IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  Nop();
  Nop();
  Nop();
  __asm__ ("Reset");
}

void ETMAnalogClearFaultCounters(AnalogInput* ptr_analog_input) {
  ptr_analog_input->absolute_under_counter = 0;
  ptr_analog_input->absolute_over_counter = 0;
  ptr_analog_input->over_trip_counter = 0;
  ptr_analog_input->under_trip_counter = 0;
}
