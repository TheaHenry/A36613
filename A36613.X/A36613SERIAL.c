
#include <xc.h>
#include <uart.h>
#include "A36613SERIAL.h"
#include "Buffer64.h"
#include "A36613.h"


typedef struct
{
  unsigned char status;
  unsigned char top1_set_hi;
  unsigned char top1_set_lo;
  unsigned char top2_set_hi;
  unsigned char top2_set_lo;
  unsigned char heater_set_hi;
  unsigned char heater_set_lo;

}InputData;

InputData A36613inputdata;


// possible commands (application specific):


#define COMMAND_LENGTH 10
#define A36613_SERIAL_UART_BRG_VALUE   (unsigned int)(((FCY_CLK/A36613_SERIAL_BAUDRATE)/16)-1)

#define SERIAL_UART_INT_PRI 5

//#define ETM_SERIAL_UART_MODE_VALUE  0b1010000000000000
//#define ETM_SERIAL_UART_STA_VALUE   0b0000110001000000

#define UART_STATS_BITS     U1STAbits
#define UART_TX_REG         U1TXREG
#define UART_RX_REG         U1RXREG
#define UART_BRG            U1BRG
#define UART_MODE           U1MODE
#define UART_STA            U1STA

#define UART_RX_IF         _U1RXIF
#define UART_TX_IF         _U1TXIF
#define UART_RX_IE         _U1RXIE
#define UART_TX_IE         _U1TXIE 
#define UART_RX_IP         _U1RXIP
#define UART_TX_IP         _U1TXIP


#define UART_RX_INTERRUPT  _U1RXInterrupt
#define UART_TX_INTERRUPT  _U1TXInterrupt



BUFFER64BYTE uart1_input_buffer;
BUFFER64BYTE uart1_output_buffer;


//void A36613MakeCRC(unsigned OutputData* data);
//int A36613CheckCRC(unsigned InputData* data);
void A36613LoadData(int data); //Moves data from main global structure to output buffer and generates CRC.
void A36613DownloadData(void); //Checks CRC and if good - moves data from input buffer to main global variable.


void InitializeA36613Serial(void) 
{
  UART_RX_IP = SERIAL_UART_INT_PRI;
  UART_TX_IP = SERIAL_UART_INT_PRI;

  UART_RX_IF = 0;
  UART_TX_IF = 0;
  UART_RX_IE = 1;
  UART_TX_IE = 1;


  UART_BRG  = A36613_SERIAL_UART_BRG_VALUE;
  UART_MODE = A36613_SERIAL_UART_MODE_VALUE;
  UART_STA  = A36613_SERIAL_UART_STA_VALUE;
 

}


void A36613LoadData(int message_type)
{
  unsigned int crc= 0x5555;
  Buffer64WriteByte(&uart1_output_buffer,message_type);
  Buffer64WriteByte(&uart1_output_buffer, global_data_A36613.status);
  Buffer64WriteByte(&uart1_output_buffer, (global_data_A36613.top1_voltage_feedback >> 8));
  Buffer64WriteByte(&uart1_output_buffer, (global_data_A36613.top1_voltage_feedback & 0x00FF));
  Buffer64WriteByte(&uart1_output_buffer, (global_data_A36613.top2_voltage_feedback >> 8));
  Buffer64WriteByte(&uart1_output_buffer, (global_data_A36613.top2_voltage_feedback & 0x00FF));
  Buffer64WriteByte(&uart1_output_buffer, (global_data_A36613.bias_feedback >> 8));
  Buffer64WriteByte(&uart1_output_buffer, (global_data_A36613.bias_feedback & 0x00FF));
  switch (message_type) 
  {
    case (TOP1_FEEDBACK_MSG):
      Buffer64WriteByte(&uart1_output_buffer, (global_data_A36613.top1_voltage_monitor >> 8));
      Buffer64WriteByte(&uart1_output_buffer, (global_data_A36613.top1_voltage_monitor & 0x00FF));
    break;

    case (TOP2_FEEDBACK_MSG):
      Buffer64WriteByte(&uart1_output_buffer, (global_data_A36613.top2_voltage_monitor >> 8));
      Buffer64WriteByte(&uart1_output_buffer, (global_data_A36613.top2_voltage_monitor & 0x00FF));
    break;

    case (HTRV_FEEDBACK_MSG):
      Buffer64WriteByte(&uart1_output_buffer, (global_data_A36613.heater_output_voltage >> 8));
      Buffer64WriteByte(&uart1_output_buffer, (global_data_A36613.heater_output_voltage & 0x00FF));
    break;

    case (HTRI1_FEEDBACK_MSG):
      Buffer64WriteByte(&uart1_output_buffer, (global_data_A36613.heater1_current_monitor >> 8));
      Buffer64WriteByte(&uart1_output_buffer, (global_data_A36613.heater1_current_monitor & 0x00FF));
    break;

    case (HTRI2_FEEDBACK_MSG):
      Buffer64WriteByte(&uart1_output_buffer, (global_data_A36613.heater2_current_monitor >> 8));
      Buffer64WriteByte(&uart1_output_buffer, (global_data_A36613.heater2_current_monitor & 0x00FF));
    break;

  }
  Buffer64WriteByte(&uart1_output_buffer, (crc >> 8)); //should be crc hi
  Buffer64WriteByte(&uart1_output_buffer, (crc & 0xFF)); //should be crc lo
};


void A36613TransmitData(int message_type)
{
  A36613LoadData(message_type);
  if ((!UART_STATS_BITS.UTXBF) && (Buffer64IsNotEmpty(&uart1_output_buffer)) )
  { //fill TX REG and then wait for interrupt to fill the rest.
    U1TXREG =  Buffer64ReadByte(&uart1_output_buffer);
  }
};

int A36613ReceiveData(void)
{

  while ( (Buffer64BytesInBuffer(&uart1_input_buffer) >= COMMAND_LENGTH)) {
    // Look for a command
    unsigned char read_byte;
    unsigned int crc;
    read_byte = Buffer64ReadByte(&uart1_input_buffer);
    if (read_byte == SETTINGS_MSG) 
    {
      // All of the sync bytes matched, this should be a valid command
      A36613inputdata.status = Buffer64ReadByte(&uart1_input_buffer);
      A36613inputdata.top1_set_hi  = Buffer64ReadByte(&uart1_input_buffer);
      A36613inputdata.top1_set_lo  = Buffer64ReadByte(&uart1_input_buffer);
      A36613inputdata.top2_set_hi = Buffer64ReadByte(&uart1_input_buffer);
      A36613inputdata.top2_set_lo = Buffer64ReadByte(&uart1_input_buffer);
      A36613inputdata.heater_set_hi  = Buffer64ReadByte(&uart1_input_buffer);
      A36613inputdata.heater_set_lo  = Buffer64ReadByte(&uart1_input_buffer);
      crc = Buffer64ReadByte(&uart1_input_buffer);
      crc = (crc << 8) + Buffer64ReadByte(&uart1_input_buffer);
      if (crc == 0x5555)
      {
        A36613DownloadData();
        return 1;
      }

    }
  }
  return 0;
}
    

void A36613DownloadData(void)
{

  global_data_A36613.status = A36613inputdata.status;
  global_data_A36613.top1_set_voltage = A36613inputdata.top1_set_hi;
  global_data_A36613.top1_set_voltage <<=8;
  global_data_A36613.top1_set_voltage = global_data_A36613.top1_set_voltage + A36613inputdata.top1_set_lo;
  global_data_A36613.top2_set_voltage = A36613inputdata.top2_set_hi;
  global_data_A36613.top2_set_voltage <<=8;
  global_data_A36613.top2_set_voltage = global_data_A36613.top2_set_voltage + A36613inputdata.top2_set_lo;
  global_data_A36613.heater_set_voltage = A36613inputdata.heater_set_hi;
  global_data_A36613.heater_set_voltage <<=8;
  global_data_A36613.heater_set_voltage = global_data_A36613.heater_set_voltage + A36613inputdata.heater_set_lo;

}


/*
unsigned int MakeCRC(unsigned OutputData* data) {
  data.output_crc_hi=0x55;
  data.output_crc_lo = 0x55;
 // crc = SYNC_BYTE_1 + SYNC_BYTE_2 + SYNC_BYTE_3_SEND;
  //crc += command_byte + register_byte;
  //crc += (data_word >> 8);
  //crc += (data_word & 0x00FF);
  
  //return crc;
  // DPAKRER Make real CRC
}


unsigned char CheckCRC(unsigned InputData* data) {
 // unsigned int crcCheck;
  // At the moment the CRC is just a checksum
 // crcCheck = SYNC_BYTE_1 + SYNC_BYTE_2 + SYNC_BYTE_3_RECEIVE;
 // crcCheck += command_string.command_byte + command_string.register_byte;
 // crcCheck += command_string.data_high_byte + command_string.data_low_byte;
//  if (crcCheck == crc) {
 //   return 1;
 // } else {
    return 1;
  }
  // DPARKER make Real CRC

}

*/

void __attribute__((interrupt(__save__(CORCON,SR)),no_auto_psv)) UART_RX_INTERRUPT(void) {
  UART_RX_IF = 0;
  while (U1STAbits.URXDA) {
    Buffer64WriteByte(&uart1_input_buffer, U1RXREG);
  }
}



void __attribute__((interrupt(__save__(CORCON,SR)),no_auto_psv)) UART_TX_INTERRUPT(void) {
  UART_TX_IF = 0;
  if ((!UART_STATS_BITS.UTXBF) && (Buffer64IsNotEmpty(&uart1_output_buffer) ))
    { //fill TX REG and then wait for interrupt to fill the rest.
      U1TXREG =  Buffer64ReadByte(&uart1_output_buffer);
    }
}