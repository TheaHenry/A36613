// Header file for new Interface Board
#ifndef __A36613_H
#define __A36613_H


#define FCY_CLK 30000000

#include <xc.h>
#include <33fXXXXX.h>
#include <smpsadc.h>
#include <timer.h>
#include <smpspwm.h>


#include "ETM.h"

/*
  
  Hardware Module Resource Usage

  SPI2   - Used/Configured by LTC265X Module
  Timer3 - 10ms timer 
  Timer1 - Triggers ADC conversion for all channels
  ADC Module
  PWM module


*/


// ----------------- IO PIN CONFIGURATION -------------------- //
// All unused pins will be set to outputs and logic zero
// LAT values default to 0 at startup so they do not need to be manually set

// Pins to be configured as inputs
/*

 RB0 - Analog input AN0 - Heater SW current sense
 RB1 - Analog input AN1 - Top1 raw feedback
 RB3 - Analog input AN3 - Top2 raw feedback
 RB4 - Analog input AN4 - Heater output voltage
 RB5 - Analog input AN5 - Bias feedback
 RB7 - Analog input AN7 - Top1 voltage monitor
 RB8 - Analog input AN8 - Heater 1 current monitor
 RB9 - Analog input AN9 - Top2 voltage monitor
 RB10 - Analog input AN10 - Heater 2 current monitor
 RB13 - Analog input AN13 - 15V monitor

 RD1 - Reset Detect


*/

//#define A36613_TRISA_VALUE 0x0000 
#define A36613_TRISB_VALUE 0b0010011110111011 
#define A36613_TRISC_VALUE 0x0000 
#define A36613_TRISD_VALUE 0x0002 
#define A36613_TRISE_VALUE 0x0000
#define A36613_TRISF_VALUE 0x0000 
#define A36613_TRISG_VALUE 0x0000


// ------------- PIN DEFINITIONS ------------------- ///

#define PIN_LED_TEST_POINT_A            _LATD2
#define PIN_LED_TEST_POINT_B            _LATD3     
#define PIN_LED_OPERATIONAL_GREEN       _LATD4

#define ETM_RESET_DETECT                _RD1


// ---------------- Timing Configuration Values ------------- //


/* 
   TMR3 Configuration
   Timer3 - Used for 10msTicToc
   Period should be set to 10mS
*/
#define T3CON_VALUE                    (T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_8 & T3_SOURCE_INT)
#define PR3_PERIOD_US                  10000   // 10mS
#define PR3_VALUE_10_MILLISECONDS      (unsigned int)((FCY_CLK / 1000000)*PR3_PERIOD_US/8)


//TMR1 Configuration 
//Timer 1 used to trigger ADC conversion every 5us.
#define T1CON_VALUE                    (T1_ON & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_8 & T1_SOURCE_INT)
#define PR1_PERIOD_US                  5   // 5uS
#define PR1_VALUE_5_US      (unsigned int)((FCY_CLK / 1000000)*PR1_PERIOD_US/8)


// -------------------  PWM Configuration ----------------- //



// -------------------  ADC CONFIGURATION ----------------- //
//Set to trigger on TMR1 period match;
//set all channels to interrupt with priority 2
#define ADCON_SETTING           (ADC_MOD_DIS|ADC_IDLE_DISCONT|ADC_SLCLKDIV_DIS|ADC_SOFT_TRIG_EN|ADC_DATA_INT|ADC_INT_EN_2CONV|ADC_ORDER_EVEN_FST|ADC_SAMP_SIM|ADC_SAMP_TRIG| ADC_PLL_DIS_FADC_4)
#define ADPCFG_SETTING          (ADC_PORT_PIN0_DIG|ADC_PORT_PIN1_AN|ADC_PORT_PIN2_DIG|ADC_PORT_PIN3_AN|ADC_PORT_PIN4_AN|ADC_PORT_PIN5_AN|ADC_PORT_PIN6_DIG|ADC_PORT_PIN7_AN|ADC_PORT_PIN8_AN|ADC_PORT_PIN9_ANADC_PORT_PIN10_AN)
#define ADCPC0_SETTING          (ADC_AN3_2_IR_GEN_EN | ADC_AN3_2_TRIG_TMR1 | ADC_AN3_2_INT_PR2 | ADC_AN1_0_IR_GEN_EN | ADC_AN1_0_TRIG_TMR1 | ADC_AN1_0_INT_PR2) 
#define ADCPC1_SETTING          (ADC_AN7_6_IR_GEN_EN | ADC_AN7_6_TRIG_TMR1 | ADC_AN7_6_INT_PR2 | ADC_AN5_4_IR_GEN_EN | ADC_AN5_4_TRIG_TMR1| ADC_AN5_4_INT_PR2 )
#define ADCPC2_SETTING          (ADC_AN11_10_IR_GEN_EN | ADC_AN11_10_TRIG_TMR1 | ADC_AN11_10_INT_PR2 | ADC_AN9_8_IR_GEN_EN | ADC_AN9_8_TRIG_TMR1 |ADC_AN9_8_INT_PR2)



typedef struct {
  unsigned int control_state;
  unsigned int heater_set_voltage;
  unsigned int heater_output_voltage; //needs to be an 8bit number
  unsigned int top1_set_voltage;
  unsigned int top1_dac_setting_scaled;
  unsigned int top2_set_voltage;
  unsigned int top2_dac_setting_scaled;
  unsigned int top1_voltage_monitor; //needs to be an 8bit number
  unsigned int top2_voltage_monitor; //needs to be an 8bit number
  unsigned int top1_voltage_feedback;
  unsigned int top2_voltage_feedback;
  unsigned int heater1_current_monitor; //needs to be an 8bit number
  unsigned int heater2_current_monitor; //needs to be an 8bit number
  unsigned int bias_feedback;
  unsigned int top_feedback;
  unsigned char status;

} ControlData;

extern ControlData global_data_A36613;


#endif