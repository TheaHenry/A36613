#include <p33fxxxx.h>
#include "A36613_SERIAL.h"


struct  {
  unsigned char command_byte;
  unsigned char top1_set;
  unsigned char top2_set;
  unsigned char heater_set;
  unsigned char heater_enable;
  unsigned char input_crc_hi;
  unsigned char input_crc_lo;
}InputData;

struct {
  InputData Data;
  unsigned int count;
} InputBuffer;

struct 
{
  unsigned char command_byte;
  unsigned char top_fdbk_hi;
  unsigned char top_fdbk_lo;
  unsigned char bias_fdbk_hi;
  unsigned char bias_fdbk_lo;
  unsigned char top1_mon;
  unsigned char top2_mon;
  unsigned char heater_voltage_mon;
  unsigned char heater1_current_mon;
  unsigned char heater2_current_mon;
  unsigned char status;
  unsigned char output_crc_hi;
  unsigned char output_crc_lo; 
}OutputData;

struct {
  OutputData Data;
  unsigned int counter;
} OutputBuffer;

//#define COMMAND_LENGTH        6
// possible commands (application specific):
#define FEEDBACK_MSG  0xF1 
#define SETTINGS_MSG  0xF2

#define TX_MSG_LENGTH (sizeof(DataOutput)/sizeof(char))
#define RX_MSG_LENGTH (sizeof(DataInput)/sizeof(char))

#define A36613_SERIAL_UART_BRG_VALUE   (unsigned int)(((CLK_FCY/ETM_SERIAL_BAUDRATE)/16)-1)

#define A36613_SERIAL_UART_MODE_VALUE  (UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE & UART_MODE_SIMPLEX & UART_RX_TX & UART_UEN_00 &  UART_EN_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT)
#define A36613_SERIAL_UART_STA_VALUE   (UART_INT_TX & UART_TX_ENABLE & UART_SYNC_BREAK_DISABLED & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)


//#define ETM_SERIAL_UART_MODE_VALUE  0b1010000000000000
//#define ETM_SERIAL_UART_STA_VALUE   0b0000110001000000



#ifdef USE_UART_1
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



#else
#define UART_STATS_BITS     U2STAbits
#define UART_TX_REG         U2TXREG
#define UART_RX_REG         U2RXREG
#define UART_BRG            U2BRG
#define UART_MODE           U2MODE
#define UART_STA            U2STA

#define UART_RX_IF         _U2RXIF
#define UART_TX_IF         _U2TXIF
#define UART_RX_IE         _U2RXIE
#define UART_TX_IE         _U2TXIE 
#define UART_RX_IP         _U2RXIP
#define UART_TX_IP         _U2TXIP

#define UART_RX_INTERRUPT  _U2RXInterrupt
#define UART_TX_INTERRUPT  _U2TXInterrupt

#endif




unsigned char CheckCRC(unsigned int crc);
unsigned int MakeCRC(unsigned char command_byte, unsigned char register_byte, unsigned int data_word);

void A36613MakeCRC(unsigned int* data);
void A36613LoadData(void);
void DownloadData(void);
void A36613ReceiveData(void);

unsigned int InitializeA36613Serial(void) {
  UART_RX_IP = A36613_SERIAL_UART_INT_PRI;
  UART_TX_IP = A36613_SERIAL_UART_INT_PRI;

  UART_RX_IF = 0;
  UART_TX_IF = 0;
  UART_RX_IE = 1;
  UART_TX_IE = 1;


  UART_BRG  = A36613_SERIAL_UART_BRG_VALUE;
  UART_MODE = A36613_SERIAL_UART_MODE_VALUE;
  UART_STA  = A36613_SERIAL_UART_STA_VALUE;
  // Need to init TMRx with configuration and period.
  
  return 0;
}


void A36613LoadData(void){
  outputBuffer A36613_output_buffer;
  unsigned int worddata = global_data_A36613.top_feedback;
  A36613_output_buffer.Data.top_fdbk_lo = (char) worddata;
  worddata = worddata >> 8 ;
  A36613_output_buffer.Data.top_fdbk_hi = (char) worddata;
  worddata = global_data_A36613.bias_feedback;
  A36613_output_buffer.Data.bias_fdbk_lo = (char) worddata;
  worddata = worddata >> 8 ;
  A36613_output_buffer.Data.bias_fdbk_hi = (char) worddata;
  A36613_output_buffer.Data.top1_mon = (char) global_data_A36613.top1_voltage_monitor;
  A36613_output_buffer.Data.top2_mon = (char) global_data_A36613.top2_voltage_monitor;
  A36613_output_buffer.Data.heater1_current_mon = (char) global_data_A36613.heater1_current_monitor;
  A36613_output_buffer.Data.heater2_current_mon = (char) global_data_A36613.heater2_current_monitor;
  A36613_output_buffer.Data.heater_voltage_mon = (char) global_data_A36613.heater_output_voltage;
  A36613_output_buffer.Data.status = global_data_A36613.status;
};


void A36613TransmitData(void)
{
  A36613_output_buffer.counter = 0;
  A36613LoadData();
  A36613MakeCRC(&A36613_output_buffer.Data);
  while (!UART_STATS_BITS.UTXBF) && (A36613_output_buffer.counter < TX_MSG_LENGTH) ){ //fill TX REG and then wait for interrupt to fill the rest.
    U1TXREG = A36613_output_buffer.Data[A36613_output_buffer.counter++];
  }
  
};
      
  


}

void A36613ReceiveData(void){
  A36613_input_buffer.counter = 0;
  if (A36613CheckCRC(&A36613_input_buffer.Data))
    DownloadData();

}





void DownloadData(void){
  unsigned int worddata = A36613_input_buffer.Data.top1_set;
  global_data_A36613.top1_set_voltage = worddata;
  worddata = A36613_input_buffer.Data.top2_set;
  global_data_A36613.top2_set_voltage = worddata;
  worddata = A36613_input_buffer.Data.heater_set;
  global_data_A36613.heater_set_voltage = worddata;
  if (A36613_input_buffer.Data.heater_enable) /set status bit to heater on.
    global_data_A36613.status |= 0x01;
  else
    global_data_A36613.status &= 0x01;

}



unsigned int MakeCRC(unsigned char command_byte, unsigned char register_byte, unsigned int data_word) {
  unsigned int crc;
  crc = SYNC_BYTE_1 + SYNC_BYTE_2 + SYNC_BYTE_3_SEND;
  crc += command_byte + register_byte;
  crc += (data_word >> 8);
  crc += (data_word & 0x00FF);
  
  return crc;
  // DPAKRER Make real CRC
}


unsigned char CheckCRC(unsigned int crc) {
  unsigned int crcCheck;
  // At the moment the CRC is just a checksum
  crcCheck = SYNC_BYTE_1 + SYNC_BYTE_2 + SYNC_BYTE_3_RECEIVE; 
  crcCheck += command_string.command_byte + command_string.register_byte;
  crcCheck += command_string.data_high_byte + command_string.data_low_byte;
  if (crcCheck == crc) {
    return 1;
  } else {
    return 0;
  }
  // DPARKER make Real CRC

}



void __attribute__((interrupt(__save__(CORCON,SR)),no_auto_psv)) UART_RX_INTERRUPT(void) {
  UART_RX_IF = 0;
  while ((UART_STATS_BITS.URXDA) && (A36613_input_buffer.counter < RX_MSG_LENGTH)) {
    A36613_input_buffer.Data[A36613_input_buffer.counter++] = UART_RX_REG;
    if (A36613_input_buffer.counter == RX_MSG_LENGTH) //if full message has been received- check crc and move to global structure.
      A36613ReceiveData();
  }
}



void __attribute__((interrupt(__save__(CORCON,SR)),no_auto_psv)) UART_TX_INTERRUPT(void) {
  UART_TX_IF = 0;
  while (!UART_STATS_BITS.UTXBF) && (A36613_output_buffer.counter < TX_MSG_LENGTH) ) {
    /*
      There is at least one byte available for writing in the outputbuffer and there is still unsent data in the output data.
      Move a byte from the output buffer into the transmit buffer
    */
    U1TXREG = A36613_output_buffer.Data[A36613_output_buffer.counter++];
  }
}