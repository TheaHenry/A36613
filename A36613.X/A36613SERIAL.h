/*
  Module Description
    The uart module calculates the crc for transmitting, and checks it upon receiving.
  If crc does not match, message is ignored.
  


  Assumptions
  FCY_CLK is defined (Fosc/2)

*/


#ifndef __SERIAL_H
#define __SERIAL_H

#include "ETM.h"



// User Configuration Parameters
#define A36613_SERIAL_BAUDRATE        312500//460800//625000//1843200//9600//  //
#define A36613_SERIAL_UART_INT_PRI     4

#define COMMAND_BUFFER_EMPTY  0x00
#define COMMAND_BUFFER_FULL   0x02

#define A36613_SERIAL_UART_MODE_VALUE  (UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE & UART_MODE_SIMPLEX & UART_UEN_00 &  UART_EN_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT)
#define A36613_SERIAL_UART_STA_VALUE   (UART_INT_TX & UART_TX_ENABLE & UART_SYNC_BREAK_DISABLED & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS & UART_IrDA_POL_INV_ZERO)


#define SETTINGS_MSG  		0xFF
#define TOP1_FEEDBACK_MSG 	0xF0
#define TOP2_FEEDBACK_MSG 	0xF1
#define HTRV_FEEDBACK_MSG 	0xF2
#define HTRI1_FEEDBACK_MSG 	0xF3
#define HTRI2_FEEDBACK_MSG 	0xF4



void InitializeA36613Serial(void);

void A36613TransmitData(unsigned char message_type); 
int A36613ReceiveData(void); //returns 1 if the RX message has been fully received. This functions checks CRC and if good, moves data from input buffer to global data structure.



#endif