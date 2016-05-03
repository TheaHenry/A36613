
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

extern ControlData global_data_A36613;
extern AnalogInput Heater_output_voltage;
extern AnalogInput Heater1_current;
extern AnalogInput Heater2_current;
extern void UpdateTopVoltage(void);
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
void A36613LoadData(unsigned char data); //Moves data from main global structure to output buffer and generates CRC.
void A36613DownloadData(unsigned char *msg_data); //Checks CRC and if good - moves data from input buffer to main global variable.


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


void A36613LoadData(unsigned char message_type)
{
  unsigned char transmitMessage[8];
  unsigned int crc;
  transmitMessage[0] = message_type;
  transmitMessage[1] = global_data_A36613.status;
  if (global_data_A36613.top1_voltage_feedback > global_data_A36613.top2_voltage_feedback)
  {
    transmitMessage[2] = global_data_A36613.top1_voltage_feedback >> 8;
    transmitMessage[3] = global_data_A36613.top1_voltage_feedback & 0x00FF;
  }
  else
  {
    transmitMessage[2] = global_data_A36613.top2_voltage_feedback >> 8;
    transmitMessage[3] = global_data_A36613.top2_voltage_feedback & 0x00FF;
  }
  transmitMessage[4] = global_data_A36613.bias_feedback >> 8;
  transmitMessage[5] = global_data_A36613.bias_feedback & 0x00FF;

  switch (message_type) 
  {
    case (TOP1_FEEDBACK_MSG):
      transmitMessage[6] = global_data_A36613.top1_voltage_monitor >> 8;
      transmitMessage[7] = global_data_A36613.top1_voltage_monitor & 0x00FF;
    break;

    case (TOP2_FEEDBACK_MSG):
      transmitMessage[6] = global_data_A36613.top2_voltage_monitor >> 8;
      transmitMessage[7] = global_data_A36613.top2_voltage_monitor & 0x00FF;
    break;

    case (HTRV_FEEDBACK_MSG):
      transmitMessage[6] = Heater_output_voltage.reading_scaled_and_calibrated >> 8;
      transmitMessage[7] = Heater_output_voltage.reading_scaled_and_calibrated & 0x00FF;
    break;

    case (HTRI1_FEEDBACK_MSG):
      transmitMessage[6] = Heater1_current.reading_scaled_and_calibrated >> 8;
      transmitMessage[7] = Heater1_current.reading_scaled_and_calibrated & 0x00FF;
    break;

    case (HTRI2_FEEDBACK_MSG):
      transmitMessage[6] = Heater2_current.reading_scaled_and_calibrated >> 8;
      transmitMessage[7] = Heater2_current.reading_scaled_and_calibrated & 0x00FF;
    break;

  }
  
  crc = ETMCRC16(&transmitMessage[0],8);

  Buffer64WriteByte(&uart1_output_buffer,transmitMessage[0]);
  Buffer64WriteByte(&uart1_output_buffer, transmitMessage[1]);
  Buffer64WriteByte(&uart1_output_buffer, transmitMessage[2]);
  Buffer64WriteByte(&uart1_output_buffer, transmitMessage[3]);
  Buffer64WriteByte(&uart1_output_buffer, transmitMessage[4]);
  Buffer64WriteByte(&uart1_output_buffer, transmitMessage[5]);
  Buffer64WriteByte(&uart1_output_buffer, transmitMessage[6]);
  Buffer64WriteByte(&uart1_output_buffer, transmitMessage[7]);
  Buffer64WriteByte(&uart1_output_buffer, (crc >> 8)); //should be crc hi
  Buffer64WriteByte(&uart1_output_buffer, (crc & 0xFF)); //should be crc lo
};


void A36613TransmitData(unsigned char message_type)
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
    unsigned char recieveMessage[8];
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
        A36613DownloadData(&recieveMessage[0]);
        return 1;
      }

    }
  }
  return 0;
}
    

void A36613DownloadData(unsigned char *msg_data)
{
  global_data_A36613.status = msg_data[1];
  global_data_A36613.top1_set_voltage = (msg_data[2]<<8) + (msg_data[3] &0x00FF);
  global_data_A36613.top2_set_voltage = (msg_data[4]<<8) + (msg_data[5] &0x00FF);
  global_data_A36613.heater_set_voltage = (msg_data[6]<<8) + (msg_data[7] &0x00FF);
  UpdateTopVoltage();
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