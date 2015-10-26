#include "A36613.h"

// This is the firmware for modulator- HV section

/*modules that need to be developed:
 * Serial communicationi with LV section (Uart) - need to rewrite the recieve (how do we know start of messsage?), needs CRC
 *bootloader

	Other open items:
	* how do we want to respond to HTR faults - as of now, heater fault conditions will be chacked by LV section only.
	* how do we respond to xmit faults?
	* CRC?
	* watchdog?
	* average current mode? peak current mode?
  * UpdateTopVoltage(); // change this to be done every 1 sec?
  * A36613TransmitData(); //does this need to be gated? Do I need to check that the previous message has been sent?

*/

/* -------------------- Device configuration --------------------------  */


// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (General Segment may be written)
#pragma config GSS = OFF                // General Segment Code Protection (General Segment Code protect is disabled)

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Source Selection (Internal Fast RC with PLL (FRCPLL))
#pragma config IESO = OFF               // Internal External Switch Over Mode (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 is clock output)
#pragma config FCKSM = CSECME           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are enabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR32            // WDT Prescaler (1:32)
#pragma config WINDIS = ON              // Watchdog Timer Window (Watchdog Timer in Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
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
    global_data_A36613.control_state = STATE_READY;

    break;
	

  case STATE_READY:
    PIN_LED_OPERATIONAL_GREEN = 1;
    PIN_LED_TEST_POINT_A = 1;
    unsigned int flashDuration = 1000;
    while(global_data_A36613.control_state == STATE_READY)
    {
        
        if (_T3IF ) 
        {
            if(PIN_LED_OPERATIONAL_GREEN==1)
            {
              PIN_LED_OPERATIONAL_GREEN=0;
            }
            else
            {
              PIN_LED_OPERATIONAL_GREEN=1;
            }
            _T3IF = 0;
            flashDuration--;
            global_data_A36613.top_feedback = global_data_A36613.heater_set_voltage + 1;
            A36613TransmitData();
           
        }
        if (flashDuration ==0)
        {
          flashDuration = 1000;
        }

      int Heater_enable = global_data_A36613.status & 0x0001;
      if (Heater_enable)
      {
        //_PTEN = 1; // Turn PWM on
        UpdateHeaterPWM();
      }
      else
        _PTEN = 0; // Turn PWM off

 //     UpdateTopVoltage(); // change this to be done every 1 sec?
        A36613ReceiveData();
        
         //does this need to be gated? Do I need to check that the previous message has been sent?

    }
    break;
     
    
 case STATE_FAULT:
     
	break;
	
  default:
    global_data_A36613.control_state = STATE_READY;

    break;

  }
}



void ConfigureClock(void)
{

  //**********************Setup Clock speeds*********************************//
  //   Fin=7.3MHz
  //   Fosc = Fin*M/(N1+N2), Fcy=Fosc/2
  //   Fosc= 7.49 * 50 / (2 * 3) = 62.4MHz, Fcy= 31.2MIPS
  //*************************************************************************//
  // Configure PLL prescaler, PLL postscaler, PLL divisor
  //_TUN = 4;//tune FRC to 7.49MHz
  PLLFBD = 48;// M = 50
  CLKDIVbits.PLLPOST=0;// N2 = 2
  CLKDIVbits.PLLPRE=1;// N1 = 3
  // Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
  __builtin_write_OSCCONH(0x01);
  __builtin_write_OSCCONL(OSCCON | 0x01);
  // Wait for Clock switch to occur
  while (OSCCONbits.COSC != 0b001);
  // Wait for PLL to lock
  while(OSCCONbits.LOCK!=1) {};


// CLKDIVbits.PLLPOST = 0b00; // Set PLL Postscaler (N2) to 2.
//  CLKDIVbits.PLLPRE = 0b00000; // Set PLL Prescaler (N1) to 2.
//  PLLFBD = 58; // Set PLL Divider (M) to 60.


  //Auxilary clock configuration
//  _SELACLK = 0; //PLL output (FVCO) provides the source clock for the auxiliary clock divider
//  _APSTSCLR = 0b111; // Auxiliary Clock Output Divider 1:1

  ACLKCONbits.FRCSEL = 1; /* Internal FRC is clock source for auxiliary PLL */
  
  ACLKCONbits.SELACLK = 1;/* Auxiliary PLL provides the source clock for the */
  /* clock divider */
  ACLKCONbits.APSTSCLR = 7;/* Auxiliary Clock Output Divider is Divide-by-1 */
  ACLKCONbits.ENAPLL = 1; /* APLL is enabled */

  while(ACLKCONbits.APLLCK != 1){}; /* Wait for Auxiliary PLL to Lock */
  /* Given a 7.5MHz input from the FRC the Auxiliary Clock for the ADC and PWM */
  /* modules are 7.5MHz * 16 = 120MHz */

  /* Disable Watch Dog Timer */
        RCONbits.SWDTEN = 0;


}


void InitializeA36613(void) 
{
  //Timer setup
  T3CON = T3CON_VALUE;
  PR3 = PR3_VALUE_10_MILLISECONDS;
  _T3IF = 0;
  DisableIntT3;

  T1CON = T1CON_VALUE;
  PR1 = PR1_VALUE_5_US;
  _T1IF = 0;
  DisableIntT1;


  //init global variables
  global_data_A36613.heater_set_voltage = 624; //10V heater output
  global_data_A36613.heater_output_voltage = 0x0007;
  global_data_A36613.top1_set_voltage = 2496; //2V from DAC
  global_data_A36613.top1_dac_setting_scaled = 0;
  global_data_A36613.top2_set_voltage = 2496; //2V from DAC
  global_data_A36613.top2_dac_setting_scaled = 0;
  global_data_A36613.top1_voltage_monitor = 0x0005;
  global_data_A36613.top2_voltage_monitor = 0x0006;
  global_data_A36613.top1_voltage_feedback = 0;
  global_data_A36613.top2_voltage_feedback = 0;
  global_data_A36613.heater1_current_monitor = 0x0008;
  global_data_A36613.heater2_current_monitor = 0x0009;
  global_data_A36613.bias_feedback = 0x0304;
  global_data_A36613.top_feedback = 0x0102;
  global_data_A36613.status = 1;
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



  Nop();
  Nop();
  Nop();
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

  heater_voltage_accumulator_size = 64;
  top1_voltage_accumulator_size = 64;
  top2_voltage_accumulator_size = 64;
  top1_fdbk_voltage_accumulator_size = 64;
  top2_fdbk_voltage_accumulator_size = 64;
  heater2_current_accumulator_size = 64;

  _ADON = 1; //Turn ADC on

}

void UpdateHeaterPWM(void)
{
  unsigned int Heater_error;
  if (global_data_A36613.heater_set_voltage > global_data_A36613.heater_output_voltage)
  {
    Heater_error = global_data_A36613.heater_set_voltage - global_data_A36613.heater_output_voltage;
    if (Heater_error > 60) //1V error
      _PTEN = 1;

  }
  
  else
  {
    Heater_error = global_data_A36613.heater_output_voltage - global_data_A36613.heater_set_voltage;
    if (Heater_error > 60) //1V error
      _PTEN = 0;
  }
}

/*

void UpdateTopVoltage(void)
{
  global_data_A36613.top1_dac_setting_scaled = global_data_A36613.top1_set_voltage;
  global_data_A36613.top1_dac_setting_scaled <<= 8; // scale to 16bit number
  global_data_A36613.top2_dac_setting_scaled = global_data_A36613.top2_set_voltage;
  global_data_A36613.top2_dac_setting_scaled <<= 8; // scale to 16bit number
  WriteLTC265X(&U4_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_A , global_data_A36613.top1_dac_setting_scaled);
  WriteLTC265X(&U4_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_B , global_data_A36613.top2_dac_setting_scaled);
}


*/
void __attribute__((interrupt, auto_psv)) _ADCP0Interrupt(void)
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
    
  _ADCP0IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ADCP1Interrupt (void)
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
    
  _ADCP1IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ADCP2Interrupt (void)
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
    
  _ADCP2IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ADCP3Interrupt (void)
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
    
  _ADCP3IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ADCP4Interrupt (void)
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
    
  _ADCP4IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ADCP5Interrupt (void)
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
    
  _ADCP5IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
    Nop();
    Nop();
    Nop();

}

