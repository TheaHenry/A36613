#include "A36613.h"


// This is the firmware for modulator- HV section

/*functionality that needs to be developed:

 *bootloader

	Other open items:
	* how do we want to respond to HTR faults?.
	* CRC?
	* watchdog?
  * UpdateTopVoltage(); // change this to be done every 1 sec? Perhaps call it when successfully receiving a message.
  * A36613TransmitData(); //this need to be gated

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
void CheckHeaterFaults(void);

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
unsigned int heater_overvoltage_count;
unsigned int heater_undervoltage_count;
unsigned int heater_overcurrent_count;
unsigned int heater_undercurrent_count;
unsigned int transmit_message;
unsigned int flashDuration;

ControlData global_data_A36613;
LTC265X U4_LTC2654;
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
    global_data_A36613.control_state = STATE_READY;

    break;
	

  case STATE_READY:

    PIN_LED_OPERATIONAL_GREEN = 1;
    PIN_LED_TEST_POINT_A = 1;
    PIN_LED_TEST_POINT_B = 0;
    transmit_message=0xF0;
    flashDuration = 1000;
    while(global_data_A36613.control_state == STATE_READY)
    {
        A36613ReceiveData();
        if (_T3IF ) //every 500us
        { 
          _T3IF = 0;
          flashDuration--;
          if (transmit_message > 0xF5)
          {
            transmit_message = 0xF1;
          }
          A36613TransmitData(transmit_message);
          transmit_message++;
          if (global_data_A36613.heater_enable == 0xFFFF)
          {
            _PTEN = 1; // Turn PWM on
            UpdateHeaterPWM();

          }
          else
          {
            _PTEN = 0; // Turn PWM off
          }
          CheckHeaterFaults();
        }



        if (flashDuration ==0) //every 500ms
        {
          flashDuration = 1000;
            UpdateTopVoltage(); // to do :change this timing to match receiving (1ms)
            if(PIN_LED_OPERATIONAL_GREEN==1) //LED turns on once a second
            {
              PIN_LED_OPERATIONAL_GREEN=0;
            }
            else
            {
              PIN_LED_OPERATIONAL_GREEN=1;
            }
        
        }
        
    }
    break;
     
    
 case STATE_WARMUP:
    PIN_LED_OPERATIONAL_GREEN = 1;
    PIN_LED_TEST_POINT_B = 1;
    PIN_LED_TEST_POINT_A = 0;
    transmit_message=0xF0;
    flashDuration = 1000;
    unsigned int heater_warmup_counter = 0;
    while(global_data_A36613.control_state == STATE_WARMUP)
    {
        A36613ReceiveData();
        if (_T3IF ) //every 500us
        { 
          _T3IF = 0;
          flashDuration--;
          
          if (transmit_message > 0xF5)
          {
            transmit_message = 0xF1;
          }
          A36613TransmitData(transmit_message);
          transmit_message++;
          if (global_data_A36613.heater_enable == 0xFFFF)
          {
            _PTEN = 1; // Turn PWM on
            UpdateHeaterPWM();
          }
          else
          {
            _PTEN = 0; // Turn PWM off
          }
          CheckHeaterFaults();
        }

        if (flashDuration ==0) //every 500ms
        {
          flashDuration = 1000;
          heater_warmup_counter++;
            UpdateTopVoltage(); // to do :change this timing to match receiving (1ms)
            if(PIN_LED_OPERATIONAL_GREEN==1) //LED turns on once a second
            {
              PIN_LED_OPERATIONAL_GREEN=0;
            }
            else
            {
              PIN_LED_OPERATIONAL_GREEN=1;
            }
        }

        if (heater_warmup_counter == HEATER_WARMUP_DURATION)
        {
          global_data_A36613.control_state = STATE_READY;
        }
    }
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
  PR1 = PR1_VALUE_10_US;
  _T1IF = 0;
  DisableIntT1;


  //init global variables
  global_data_A36613.heater_set_voltage = 0x0000;
  global_data_A36613.heater_output_voltage = 0x0000;
  global_data_A36613.top1_set_voltage = 0; //2V from DAC
  global_data_A36613.top2_set_voltage = 0; //2V from DAC
  global_data_A36613.top1_voltage_monitor = 0x0000;
  global_data_A36613.top2_voltage_monitor = 0x0000;
  global_data_A36613.top1_voltage_feedback = 0;
  global_data_A36613.top2_voltage_feedback = 0;
  global_data_A36613.heater1_current_monitor = 0x0000;
  global_data_A36613.heater2_current_monitor = 0x0000;
  global_data_A36613.bias_feedback = 0x0000;
  global_data_A36613.status = 0;
  global_data_A36613.heater_enable = 0;
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

  heater_overvoltage_count = 0;
  heater_undervoltage_count = 0;
  heater_overcurrent_count = 0;
  heater_undercurrent_count = 0;
  heater_output_voltage_accumulator = 0;
  heater_voltage_accumulator_size = 64;
  top1_voltage_monitor_accumulator = 0;
  top1_voltage_accumulator_size = 64;
  top2_voltage_monitor_accumulator = 0;
  top2_voltage_accumulator_size = 64;
  top1_voltage_feedback_accumulator = 0;
  top1_fdbk_voltage_accumulator_size = 64;
  top2_voltage_feedback_accumulator = 0;
  top2_fdbk_voltage_accumulator_size = 64;
  heater2_current_monitor_accumulator = 0;
  heater2_current_accumulator_size = 64;
  bias_feedback_accumulator = 0;
  heater1_current_monitor_accumulator = 0;
  

  _ADON = 1; //Turn ADC on

}

#define HEATER_SMALL_STEP   0x0001
#define HEATER_LARGE_STEP   0x000C
#define HEATER_MAX_DUTY     (PWM_PERIOD * 0.7)

void UpdateHeaterPWM(void)
{
  unsigned int Heater_error;
  if ((global_data_A36613.heater1_current_monitor >= HEATER_MAX_CURRENT)|| (global_data_A36613.heater2_current_monitor >= HEATER_MAX_CURRENT))
  {
    MDC-=HEATER_SMALL_STEP;
  }
  else if (global_data_A36613.heater_output_voltage > global_data_A36613.heater_set_voltage)
  {
      Heater_error = global_data_A36613.heater_output_voltage - global_data_A36613.heater_set_voltage;
      if (Heater_error > 100)
      {
        MDC-=HEATER_SMALL_STEP;
      }
      if (Heater_error > 1000)
      {
        MDC-= HEATER_LARGE_STEP;
      }
      if (Heater_error < 100)
      {
        global_data_A36613.control_state = STATE_READY;
      } 
      
  }
  else if (global_data_A36613.heater_output_voltage < global_data_A36613.heater_set_voltage)
  {
      Heater_error = global_data_A36613.heater_output_voltage - global_data_A36613.heater_set_voltage;
      if (Heater_error > 100)
      {
        MDC+=HEATER_SMALL_STEP;
      }
      if (Heater_error > 1000)
      {
        MDC+= HEATER_LARGE_STEP;
      }
      if (Heater_error < 100)
      {
        global_data_A36613.control_state = STATE_READY;
      }      
  } 

  if (MDC <= 64)
  {
    MDC = 64;
  }

  if (MDC >= HEATER_MAX_DUTY)
  {
    MDC= HEATER_MAX_DUTY;
  }
}



void UpdateTopVoltage(void)
{
  WriteLTC265X(&U4_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_A , global_data_A36613.top1_set_voltage);
  WriteLTC265X(&U4_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_B , global_data_A36613.top2_set_voltage);
}

void CheckHeaterFaults(void) //need 3 succesive fault conditions in order to trigger fault.
{
  if (global_data_A36613.heater_output_voltage > HEATER_OVERVOLTAGE_TRIP) //Over voltage condition
  {
      heater_overvoltage_count++;
  }
  else
  {
    heater_overvoltage_count = 0;
  }
  if (heater_overvoltage_count >=3)
  {
    global_data_A36613.status |= HEATER_OVERVOLTAGE_FLT;
  }

  if ((global_data_A36613.heater1_current_monitor > HEATER_OVERCURRENT_TRIP) || (global_data_A36613.heater2_current_monitor > HEATER_OVERCURRENT_TRIP)) //Over current condition
  {
      heater_overcurrent_count++;
  }
  else
  {
    heater_overcurrent_count = 0;
  }
  if (heater_overcurrent_count >=3)
  {
    global_data_A36613.status |= HEATER_OVERCURRENT_FLT;
  }

  if (global_data_A36613.heater_enable == 0xFFFF)// heater undervoltage and under current should only be chaecked if heater is on.
  {
    if ((global_data_A36613.heater1_current_monitor < HEATER_UNDERCURRENT_TRIP) || (global_data_A36613.heater2_current_monitor < HEATER_UNDERCURRENT_TRIP)) //Under current condition
    {
        heater_undercurrent_count++;
    }
    else
    {
      heater_undercurrent_count = 0;
    }
  
  if (heater_undercurrent_count >=3)
  {
    global_data_A36613.status |= HEATER_UNDERCURRENT_FLT;
  }

  if (global_data_A36613.control_state == STATE_READY) // Check for heater undervoltage only after warmup.
  {
    if (global_data_A36613.heater_output_voltage < HEATER_UNDERVOLTAGE_TRIP) //Under voltage condition
    {
      heater_undervoltage_count++;
    }
    else
    {
      heater_undervoltage_count = 0;
    }
  }
  if (heater_undervoltage_count >=3)
  {
    global_data_A36613.status |= HEATER_UNDERVOLTAGE_FLT;
  }
}
}



void __attribute__((interrupt, auto_psv)) _ADCP0Interrupt(void)
{
  top1_voltage_feedback_accumulator += ADCBUF1; // averages 64 samples of data (add 64, then shift left by 6). Then shifts the data to get a 8bit number (additional shift of 2).
  top1_fdbk_voltage_accumulator_size--;

  if (top1_fdbk_voltage_accumulator_size == 0) //Check if 64 samples have accumulated
  {
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
    global_data_A36613.top2_voltage_feedback = top2_voltage_feedback_accumulator; // store averaged value in global struct.
    top2_fdbk_voltage_accumulator_size = 64; // reset accumulator
    top2_voltage_feedback_accumulator = 0;
  }
    
  _ADCP1IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ADCP2Interrupt (void)
{
    PIN_LED_TEST_POINT_A = 1;
  heater_output_voltage_accumulator += ADCBUF4; // averages 64 samples of data (add 64, then shift left by 6). Then shifts the data to get a 8bit number (additional shift of 2).
  bias_feedback_accumulator += ADCBUF5;
  heater_voltage_accumulator_size--;
    PIN_LED_TEST_POINT_A = 0;
  if (heater_voltage_accumulator_size == 0) //Check if 64 samples have accumulated
  {
    global_data_A36613.heater_output_voltage = ETMScaleFactor16(heater_output_voltage_accumulator,HEATER_VOLTAGE_SCALING_FACTOR,0); // store averaged value in global struct.
    global_data_A36613.bias_feedback = bias_feedback_accumulator;
    heater_voltage_accumulator_size = 64; // reset accumulator
    heater_output_voltage_accumulator = 0;
    bias_feedback_accumulator = 0;
    /*Heater_PID.measuredOutput = global_data_A36613.heater_output_voltage; //turn 16bit number to fractional
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
  }*/
  }
  _ADCP2IF = 0;
  
}


void __attribute__((interrupt, no_auto_psv)) _ADCP3Interrupt (void)
{
  top1_voltage_monitor_accumulator += ADCBUF7; // averages 64 samples of data (add 64, then shift left by 6). Then shifts the data to get a 8bit number (additional shift of 2).
  top1_voltage_accumulator_size--;

  if (top1_voltage_accumulator_size == 0) //Check if 64 samples have accumulated
  {
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
    global_data_A36613.heater1_current_monitor= ETMScaleFactor16(heater1_current_monitor_accumulator, HEATER_CURRENT_SCALING_FACTOR,0); // store averaged value in global struct.
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
    global_data_A36613.heater2_current_monitor= ETMScaleFactor16(heater2_current_monitor_accumulator, HEATER_CURRENT_SCALING_FACTOR,0); // store averaged value in global struct.
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

