/*
  Module Description
  Only one type of message can be sent and only one type of message can be received.
  Recieved data is 1 ID byte, 4 bytes of data and 2 bytes of crc.
  The uart module calculates the crc for transmitting, and checks it upon receiving.
  If message ID or crc do not match, message is ignored.
  Module assume there is some deadtime between every 2 messages- priod between 2 messages is checked using timerX.


  Resource use 
  TMR
  UART

  Assumptions
  FCY_CLK is defined (Fosc/2)


*/


#ifndef __ETM_SERIAL_H
#define __ETM_SERIAL_H





// User Configuration Parameters
#define A36613_SERIAL_BAUDRATE         900000000 // 0.9Mbps
#define A36613_SERIAL_MESSAGE_MIN_PERIOD_US   500 // minimum period between the start of 2 messages is 500us
#define A36613_SERIAL_UART_INT_PRI     4
#define USE_UART_1
// #define USE_UART_2

// END User Configuration Parameters




void InitializeA36613Serial(void);

void A36613TransmitData(void); 
//void A36613ReceiveData(void); ReceiveData is used by the Serial module once the RX message has been fully received. This functions checks CRC and if good, moves data from input buffer to global data structure.


#endif