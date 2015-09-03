#include "A36613.h"

// This is the firmware for modulator- HV section

/*modules that need to be developed:
 * Serial communicationi with LV section (Uart)
 * Heater supply - PWM + ADC 
 * LTC
 * Status bit logic
 * Controlling reset mode- ruggedizing code for HV/noisy environment
 * task scheduller

	Other open items:
	* configuration bits
	* how do we want to respond to HTR faults
	* how do we respond to xmit faults?
	* CRC?
	* watchdog?
	* average current mode? peak current mode?

*/


//Device configuration
_FBS (BWRP_WRPROTECT_ON & BSS_NO_BOOT_CODE) //Boot segment is write protected; no boot program flash segment
_FGS (GWRP_OFF & GSS_OFF) //General Segment may be written; General Segment Code protect is disabled
_FOSCSEL (FNOSC_PRIPLL & IESO_OFF) //Primary Oscillator (XT, HS, EC) with PLL; Start up with user-selected oscillator source
_FOSC (POSCMD_EC & OSCIOFNC_ON & FCKSM_CSDCMD) // EC (External Clock) Mode; OSC2 is general purpose digital I/O pin; Both Clock switching and Fail-safe Clock Monitor are disabled
_FWDT (WDTPOST_PS1024 & WDTPRE_PR32 & WINDIS_ON & FWDTEN_ON) // Watchdog postscaler 1:1,024 ; Watchdog prescaler 1:32 ; Watchdog Timer in Window mode ; Watchdog timer always enabled
_FPOR (FPWRT_PWR128 & ALTSS1_OFF & ALTQIO_OFF) // POR timer 128ms; SS1 is selected as the I/O pin for SPI1; QEA1, QEB1, INDX1 are selected as inputs to QEI1
_FICD (ICS_PGD2 & JTAGEN_OFF) //Communicate on PGC2/EMUC2 and PGD2/EMUD2; JTAG is disabled
//_FCMP () //


#define STATE_STARTUP       0x10
#define STATE_STANDBY	      0x20
#define STATE_READY         0x30
#define STATE_FAULT         0x40



void DoStateMachine(void);
void InitializeA36613(void);
void UpdateTopVoltage(void);

unsigned int heater_output_voltage_accumulator;
unsigned int heater_voltage_accumulator_size;
unsigned int top1_voltage_monitor_accumulator;
unsigned int top1_voltage_accumulator_size;
unsigned int top2_voltage_monitor_accumulator;
unsigned int top2_voltage_accumulator_size;
unsigned int top1_voltage_feedback_accumulator;
unsigned int top1_fdbk_voltage_accumulator_size;
unsigned int top2_voltage_feedback_accumulator;
unsigned int top2_fdbk_voltage_accumulator_size;
unsigned int heater1_current_monitor_accumulator;
unsigned int heater2_current_monitor_accumulator; 
unsigned int heater2_current_accumulator_size;
unsigned int bias_feedback_accumulator;

ControlData global_data_A36613;
LTC265X U4_LTC2654;


int main(void) {
  global_data_A36613.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}

void DoStateMachine(void) {
  switch (global_data_A36613.control_state) {

  case STATE_STARTUP:
    InitializeA36613();



    break;
	
  case STATE_STANDBY:
    PIN_LED_OPERATIONAL_GREEN = 1;
    while (global_data_A36613.control_state == STATE_STANDBY)
    {
      UpdateTopVoltage();

    }

    break;

  case STATE_READY:
    PIN_LED_OPERATIONAL_GREEN = 1;
    while (global_data_A36613.control_state == STATE_READY)
    {
      UpdateTopVoltage();
    }

    break;
     
    
 case STATE_FAULT:

	break;
	
  default:
    global_data_A36613.control_state = STATE_READY;
    break;

  }
}

void InitializeA36613(void) {

  //**********************Setup Clock speeds*********************************//
  //   Fin=4MHz
  //   Fosc = Fin*M/(N1+N2), Fcy=Fosc/2
  //   Fosc= 4 * 60 / (2 * 2) = 60MHz, Fcy= 30MIPS 
  //*************************************************************************//
  CLKDIVbits.PLLPOST = 0b00; // Set PLL Postscaler (N2) to 2.
  CLKDIVbits.PLLPRE = 0b00000; // Set PLL Prescaler (N1) to 2.
  PLLFBD = 58; // Set PLL Divider (M) to 60.


  //Auxilary clock configuration
  _SELACLK = 0; //PLL output (FVCO) provides the source clock for the auxiliary clock divider
  _APSTSCLR = 0b111; // Auxiliary Clock Output Divider 1:1
  
  while(OSCCONbits.LOCK!=1) {};// Wait for PLL to lock

  //Timer setup
  T3CON = T3CON_VALUE;
  PR3 = PR3_VALUE_10_MILLISECONDS;
  _T3IF = 0;
  DisableIntT3;

  T1CON = T1CON_VALUE;
  PR1 = PR1_VALUE_5_US;
  _T1IF = 0;
  DisableIntT1;


  //init ADC
  heater_voltage_accumulator_size = 64;
  top1_voltage_accumulator_size = 64;
  top2_voltage_accumulator_size = 64;
  top1_fdbk_voltage_accumulator_size = 64;
  top2_fdbk_voltage_accumulator_size = 64;
  heater2_current_accumulator_size = 64;

  //init global variables
  heater_set_voltage = 0;
  heater_output_voltage = 0;
  top1_set_voltage = 0;
  top1_dac_setting_scaled = 0;
  top2_set_voltage = 0;
  top2_dac_setting_scaled = 0;
  top1_voltage_monitor = 0;
  top2_voltage_monitor = 0;
  top1_voltage_feedback = 0;
  top2_voltage_feedback = 0;
  heater1_current_monitor = 0;
  heater2_current_monitor = 0; 
  bias_feedback = 0;
  top_feedback = 0;
  status = 0;
  PIN_LED_OPERATIONAL_GREEN = 0;

  //set tris
  TRISB = A36613_TRISB_VALUE;
  TRISC = A36613_TRISC_VALUE;
  TRISD = A36613_TRISD_VALUE;
  TRISE = A36613_TRISE_VALUE;
  TRISF = A36613_TRISF_VALUE;
  TRISG = A36613_TRISG_VALUE;

  //init PWM
  OpenSmpsPWM(PWM_MOD_DIS | PWM_IDLE_CONT | PWM_SEVT_INT_DIS | PWM_PER_UPDATE_BOUND, PWM_PERIOD, PWM_PERIOD, INITIAL_PWM_DC, 0x0000); //PWM off, no sync, continue during idle, no sepecial event trigger, update DC on next cycle
  ConfigSmpsPWMInputClkDiv(PWM_INPUT_CLK_DIV0); //Select PWM prescaler of 1:1
  ConfigSmpsPWM1(PWM1_D_CYLE_MDC | PWM1_DT_DIS, PWM1_H_PIN_GPIO | PWM1_L_PIN_EN | PWM1_IO_PIN_PAIR_RED, 0x0000, 0x0000, 0x0000 ); //Master Duty Cycle selection; Dead time function disabled; No interrupts; Use master time base; redundant pai


  //init DAC
  SetupLTC265X(&U4_LTC2654, ETM_SPI_PORT_2, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RE6, _PIN_RE7);


  // Initialize internal ADC
  // ---- Configure the dsPIC ADC Module ------------ //

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

  _ADON = 1; //Turn ADC on
  
}

void UpdateTopVoltage(void)
{
  top1_dac_setting_scaled = top1_set_voltage;
  top1_dac_setting_scaled <<= 8; // scale to 16bit number
  top2_dac_setting_scaled = top2_set_voltage;
  top2_dac_setting_scaled <<= 8; // scale to 16bit number
  WriteLTC265X(&U4_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_A , top1_dac_setting_scaled);
  WriteLTC265X(&U4_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_B , top2_dac_setting_scaled);
}


void __attribute__((interrupt, no_auto_psv)) _ ADCP0Interrupt (void)
{
  top1_voltage_feedback_accumulator += ADCBUF1; // averages 64 samples of data (add 64, then shift left by 6). Then shifts the data to get a 8bit number (additional shift of 2).
  top1_fdbk_voltage_accumulator_size--;

  if (top1_fdbk_voltage_accumulator_size == 0) //Check if 64 samples have accumulated
  {
    top1_voltage_feedback_accumulator >>= 6; //divide to get an 10bit average value.
    global_data_A36613.top1_voltage_feedback = top1_voltage_feedback_accumulator; // store averaged value in global struct.
    top1_fdbk_voltage_accumulator_size = 64; // reset accumulator
    top1_voltage_feedback_accumulator = 0;
  }
    
  IFS6bits.ADPC0IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ ADCP1Interrupt (void)
{
  top2_voltage_feedback_accumulator += ADCBUF3; // averages 64 samples of data (add 64, then shift left by 6). Then shifts the data to get a 8bit number (additional shift of 2).
  top2_fdbk_voltage_accumulator_size--;

  if (top2_fdbk_voltage_accumulator_size == 0) //Check if 64 samples have accumulated
  {
    top2_voltage_feedback_accumulator >>= 6; //divide to get an 10bit average value.
    global_data_A36613.top2_voltage_feedback = top2_voltage_feedback_accumulator; // store averaged value in global struct.
    top2_fdbk_voltage_accumulator_size = 64; // reset accumulator
    top2_voltage_feedback_accumulator = 0;
  }
    
  IFS6bits.ADPC1IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ ADCP2Interrupt (void)
{
  heater_output_voltage_accumulator += ADCBUF4; // averages 64 samples of data (add 64, then shift left by 6). Then shifts the data to get a 8bit number (additional shift of 2).
  bias_feedback_accumulator += ADCBUF5;
  heater_voltage_accumulator_size--;

  if (heater_voltage_accumulator_size == 0) //Check if 64 samples have accumulated
  {
    heater_output_voltage_accumulator >>= 6; //divide to get an 10bit average value.
    global_data_A36613.heater_output_voltage = heater_output_voltage_accumulator; // store averaged value in global struct.
    bias_feedback_accumulator >>= 6;
    global_data_A36613.bias_feedback = bias_feedback_accumulator;
    heater_voltage_accumulator_size = 64; // reset accumulator
    heater_output_voltage_accumulator = 0;
    bias_feedback_accumulator = 0;
  }
    
  IFS6bits.ADPC2IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ ADCP3Interrupt (void)
{
  top1_voltage_monitor_accumulator += ADCBUF7; // averages 64 samples of data (add 64, then shift left by 6). Then shifts the data to get a 8bit number (additional shift of 2).
  top1_voltage_accumulator_size--;

  if (top1_voltage_accumulator_size == 0) //Check if 64 samples have accumulated
  {
    top1_voltage_monitor_accumulator >>= 8; //divide to get an 8bit value.
    global_data_A36613.top1_voltage_monitor= top1_voltage_monitor_accumulator; // store averaged value in global struct.
    top1_voltage_accumulator_size = 64; // reset accumulator
    top1_voltage_monitor_accumulator = 0;
  }
    
  IFS6bits.ADPC3IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ ADCP4Interrupt (void)
{
  heater1_current_monitor_accumulator += ADCBUF8; // averages 64 samples of data (add 64, then shift left by 6). Then shifts the data to get a 8bit number (additional shift of 2).
  top2_voltage_monitor_accumulator += ADCBUF9;
  top2_voltage_accumulator_size--;

  if (top2_voltage_accumulator_size == 0) //Check if 64 samples have accumulated
  {
    heater1_current_monitor_accumulator >>= 8; //divide to get an 8bit value.
    global_data_A36613.heater1_current_monitor= heater1_current_monitor_accumulator; // store averaged value in global struct.
    top2_voltage_monitor_accumulator >>= 8; //divide to get an 8bit value.
    global_data_A36613.top2_voltage_monitor= top2_voltage_monitor_accumulator; // store averaged value in global struct.
    top2_voltage_accumulator_size = 64; // reset accumulator
    heater1_current_monitor_accumulator = 0;
    top2_voltage_monitor_accumulator = 0;
  }
    
  IFS6bits.ADPC4IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ ADCP5Interrupt (void)
{
  heater2_current_monitor_accumulator += ADCBUF10; // averages 64 samples of data (add 64, then shift left by 6). Then shifts the data to get a 8bit number (additional shift of 2).
  heater2_current_accumulator_size--;

  if (heater2_current_accumulator_size == 0) //Check if 64 samples have accumulated
  {
    heater2_current_monitor_accumulator >>= 8; //divide to get an 8bit value.
    global_data_A36613.heater2_current_monitor= heater2_current_monitor_accumulator; // store averaged value in global struct.
    heater2_current_accumulator_size = 64; // reset accumulator
    heater2_current_monitor_accumulator = 0;
  }
    
  IFS6bits.ADPC5IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
    Nop();
    Nop();
    Nop();

}

