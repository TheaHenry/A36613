
#include <xc.h>
#include <uart.h>
#include "A36613SERIAL.h"
#include "Buffer64.h"
#include "A36613.h"


/*typedef struct
{
  unsigned char status;
  unsigned char top1_set_hi;
  unsigned char top1_set_lo;
  unsigned char top2_set_hi;
  unsigned char top2_set_lo;
  unsigned char heater_set_hi;
  unsigned char heater_set_lo;

}InputData;

InputData A36613inputdata;*/
unsigned char recieveMessage[recieveMessageLength];
extern ControlData global_data_A36613;
extern AnalogInput Heater_output_voltage;
extern AnalogInput Heater1_current;
extern AnalogInput Heater2_current;

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

static const unsigned int crctable[256] ={0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
0x0919, 0x1890, 0x2A0B, 0x3B82, 0x4F3D, 0x5EB4, 0x6C2F, 0x7DA6,
0x8551, 0x94D8, 0xA643, 0xB7CA, 0xC375, 0xD2FC, 0xE067, 0xF1EE,
0x1232, 0x03BB, 0x3120, 0x20A9, 0x5416, 0x459F, 0x7704, 0x668D,
0x9E7A, 0x8FF3, 0xBD68, 0xACE1, 0xD85E, 0xC9D7, 0xFB4C, 0xEAC5,
0x1B2B, 0x0AA2, 0x3839, 0x29B0, 0x5D0F, 0x4C86, 0x7E1D, 0x6F94,
0x9763, 0x86EA, 0xB471, 0xA5F8, 0xD147, 0xC0CE, 0xF255, 0xE3DC,
0x2464, 0x35ED, 0x0776, 0x16FF, 0x6240, 0x73C9, 0x4152, 0x50DB,
0xA82C, 0xB9A5, 0x8B3E, 0x9AB7, 0xEE08, 0xFF81, 0xCD1A, 0xDC93,
0x2D7D, 0x3CF4, 0x0E6F, 0x1FE6, 0x6B59, 0x7AD0, 0x484B, 0x59C2,
0xA135, 0xB0BC, 0x8227, 0x93AE, 0xE711, 0xF698, 0xC403, 0xD58A,
0x3656, 0x27DF, 0x1544, 0x04CD, 0x7072, 0x61FB, 0x5360, 0x42E9,
0xBA1E, 0xAB97, 0x990C, 0x8885, 0xFC3A, 0xEDB3, 0xDF28, 0xCEA1,
0x3F4F, 0x2EC6, 0x1C5D, 0x0DD4, 0x796B, 0x68E2, 0x5A79, 0x4BF0,
0xB307, 0xA28E, 0x9015, 0x819C, 0xF523, 0xE4AA, 0xD631, 0xC7B8,
0x48C8, 0x5941, 0x6BDA, 0x7A53, 0x0EEC, 0x1F65, 0x2DFE, 0x3C77,
0xC480, 0xD509, 0xE792, 0xF61B, 0x82A4, 0x932D, 0xA1B6, 0xB03F,
0x41D1, 0x5058, 0x62C3, 0x734A, 0x07F5, 0x167C, 0x24E7, 0x356E,
0xCD99, 0xDC10, 0xEE8B, 0xFF02, 0x8BBD, 0x9A34, 0xA8AF, 0xB926,
0x5AFA, 0x4B73, 0x79E8, 0x6861, 0x1CDE, 0x0D57, 0x3FCC, 0x2E45,
0xD6B2, 0xC73B, 0xF5A0, 0xE429, 0x9096, 0x811F, 0xB384, 0xA20D,
0x53E3, 0x426A, 0x70F1, 0x6178, 0x15C7, 0x044E, 0x36D5, 0x275C,
0xDFAB, 0xCE22, 0xFCB9, 0xED30, 0x998F, 0x8806, 0xBA9D, 0xAB14,
0x6CAC, 0x7D25, 0x4FBE, 0x5E37, 0x2A88, 0x3B01, 0x099A, 0x1813,
0xE0E4, 0xF16D, 0xC3F6, 0xD27F, 0xA6C0, 0xB749, 0x85D2, 0x945B,
0x65B5, 0x743C, 0x46A7, 0x572E, 0x2391, 0x3218, 0x0083, 0x110A,
0xE9FD, 0xF874, 0xCAEF, 0xDB66, 0xAFD9, 0xBE50, 0x8CCB, 0x9D42,
0x7E9E, 0x6F17, 0x5D8C, 0x4C05, 0x38BA, 0x2933, 0x1BA8, 0x0A21,
0xF2D6, 0xE35F, 0xD1C4, 0xC04D, 0xB4F2, 0xA57B, 0x97E0, 0x8669,
0x7787, 0x660E, 0x5495, 0x451C, 0x31A3, 0x202A, 0x12B1, 0x0338,
0xFBCF, 0xEA46, 0xD8DD, 0xC954, 0xBDEB, 0xAC62, 0x9EF9, 0x8F70}; 


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
      Buffer64WriteByte(&uart1_output_buffer, (Heater_output_voltage.reading_scaled_and_calibrated >> 8));
      Buffer64WriteByte(&uart1_output_buffer, (Heater_output_voltage.reading_scaled_and_calibrated & 0x00FF));
    break;

    case (HTRI1_FEEDBACK_MSG):
      Buffer64WriteByte(&uart1_output_buffer, (Heater1_current.reading_scaled_and_calibrated >> 8));
      Buffer64WriteByte(&uart1_output_buffer, (Heater1_current.reading_scaled_and_calibrated & 0x00FF));
    break;

    case (HTRI2_FEEDBACK_MSG):
      Buffer64WriteByte(&uart1_output_buffer, (Heater2_current.reading_scaled_and_calibrated >> 8));
      Buffer64WriteByte(&uart1_output_buffer, (Heater2_current.reading_scaled_and_calibrated & 0x00FF));
    break;

  }
  Buffer64WriteByte(&uart1_output_buffer, (crc >> 8)); //should be crc hi
  Buffer64WriteByte(&uart1_output_buffer, (crc & 0xFF)); //should be crc lo
};


void A36613TransmitData(int message_type)
{
  if (PIN_LED_TEST_POINT_A == 0)
  {
      PIN_LED_TEST_POINT_A =1;
  }
  else
  {
      PIN_LED_TEST_POINT_A = 0;
  }
  A36613LoadData(message_type);
  if ((!UART_STATS_BITS.UTXBF) && (Buffer64IsNotEmpty(&uart1_output_buffer)) )
  { //fill TX REG and then wait for interrupt to fill the rest.
    U1TXREG =  Buffer64ReadByte(&uart1_output_buffer);
  }
  if (PIN_LED_TEST_POINT_A == 0)
  {
      PIN_LED_TEST_POINT_A =1;
  }
  else
  {
      PIN_LED_TEST_POINT_A = 0;
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
      recieveMessage[0] = read_byte;
      recieveMessage[1] = Buffer64ReadByte(&uart1_input_buffer);
      recieveMessage[2] = Buffer64ReadByte(&uart1_input_buffer);
      recieveMessage[3] = Buffer64ReadByte(&uart1_input_buffer);
      recieveMessage[4] = Buffer64ReadByte(&uart1_input_buffer);
      recieveMessage[5] = Buffer64ReadByte(&uart1_input_buffer);
      recieveMessage[6] = Buffer64ReadByte(&uart1_input_buffer);
      recieveMessage[7] = Buffer64ReadByte(&uart1_input_buffer);
      crc = Buffer64ReadByte(&uart1_input_buffer);
      crc <<=8;
      crc += Buffer64ReadByte(&uart1_input_buffer);

      if (crc == ETMCRC16(&recieveMessage[0], 8))
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
  global_data_A36613.status = recieveMessage[1];
  global_data_A36613.top1_set_voltage = (recieveMessage[2]<<8) + (recieveMessage[3] &0x00FF);
  global_data_A36613.top2_set_voltage = (recieveMessage[4]<<8) + (recieveMessage[5] &0x00FF);
  global_data_A36613.heater_set_voltage = (recieveMessage[6]<<8) + (recieveMessage[7] &0x00FF);
}



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