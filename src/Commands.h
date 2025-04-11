#ifndef CAN_COMMANDS.h

#define CAN_COMMANDS .h

#include <global_param.h>
#define PART_TM4C1230C3PM
#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/can.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "utils/uartstdio.h"
#include <interrupt.c>
#include <string>
#include <vector>

#define LA_MAX = 400

int ObjID;

unsigned long prev_ping = 0;

struct Vel
{
  int16_t vel_L;
  int16_t vel_R;
};

void CAN_ackEr()
{
  Serial.println("AckError");
  digitalWrite(2, HIGH);
  delay(999);
  digitalWrite(2, HIGH);
}


/*modes are
01h-op
02h-stop
80h-pre-op
81h-rst node
82h-rst comms
*/
void CAN_NMT_MODE(byte mode, uint8_t addr_node)
{
  tCANMsgObject canMessage;
  uint8_t dataBuffer[8];
  uint8_t node_id = 0x00; // All nodes
  dataBuffer[0] = mode;
  dataBuffer[1] = addr_node;

  canMessage.ui32MsgID = 0x000;                 // NMT COB-ID
  canMessage.ui32MsgIDMask = 0;                 // Not used for sending
  canMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE; // Transmit message
  canMessage.ui32MsgLen = 2;
  canMessage.pui8MsgData = dataBuffer;

  CANMessageSet(CAN0_BASE, 1, &canMessage, MSG_OBJ_TYPE_TX);
}

// Todo again 3rd time heart beat

void CAN_send_SYNC()
{
  tCANMsgObject msg;
  uint8_t dataBuffer[8];

  msg.ui32MsgID = 0x080;
  msg.ui32MsgLen = 0;
  msg.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
  msg.pui8MsgData = 0;

  CANMessageSet(CAN0_BASE, 4, &msg, MSG_OBJ_TYPE_TX);
}

void CAN_read_actual_velocity(uint8_t nodeID)
{
  tCANMsgObject msg;
  uint8_t data[8];

  data[0] = 0x40;
  data[1] = 0x6C;
  data[2] = 0x60;
  data[3] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;

  msg.ui32MsgID = 0x600 + nodeID;
  msg.ui32MsgLen = 8;
  msg.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
  msg.pui8MsgData = data;

  CANMessageSet(CAN0_BASE, 3, &msg, MSG_OBJ_TYPE_TX);
}

void CAN_send_target_velocity(uint8_t nodeID, uint32_t velocity)
{ 
  uint8_t CAN_Data[8];
  digitalWrite(PF_2,HIGH);
  delay(100);
  tCANMsgObject msg;
  uint8_t data[8];

  data[0] = 0x23;
  data[1] = 0xFF;
  data[2] = 0x60;
  data[3] = 0x00;
  data[4] = (uint8_t)(velocity & 0xFF);
  data[5] = (uint8_t)((velocity >> 8) & 0xFF);
  data[6] = (uint8_t)((velocity >> 16) & 0xFF);
  data[7] = (uint8_t)((velocity >> 24) & 0xFF);

  msg.ui32MsgID = 0x600 + nodeID;
  msg.ui32MsgLen = 8;
  msg.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
  msg.pui8MsgData = data;

  CANMessageSet(CAN0_BASE, 2, &msg, MSG_OBJ_TYPE_TX);
  CAN_Data[0] ^= 0x10;//Need to swap bit each time we set sdo
  if((CANStatusGet(CAN0_BASE,(CAN_STS_CONTROL))&(0x7))==CAN_STATUS_LEC_ACK)
    {
      CAN_ackEr();
      delay(200);
    }
    CANIntClear(CAN0_BASE,1);

    // Turn off LED and reset
    digitalWrite(PF_2,LOW);
  }

void CAN_state_trans(uint8_t addr_node, uint8_t transition)
{
  tCANMsgObject canMessage;
  uint8_t dataBuffer[8];
  // uint8_t node_id = 0x00; //All nodes
  dataBuffer[0] = 0x2b;
  dataBuffer[1] = 0x40;
  dataBuffer[2] = 0x60;
  dataBuffer[3] = 0x00;
  dataBuffer[4] = transition;
  dataBuffer[5] = 0x00;
  dataBuffer[6] = 0x00;
  dataBuffer[7] = 0x00;

  canMessage.ui32MsgID = 0x600 + addr_node;     // NMT COB-ID
  canMessage.ui32MsgIDMask = 0;                 // Not used for sending
  canMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE; // Transmit message
  canMessage.ui32MsgLen = 8;
  canMessage.pui8MsgData = dataBuffer;

  CANMessageSet(CAN0_BASE, 1, &canMessage, MSG_OBJ_TYPE_TX);
}

void CAN_work_mode(uint8_t mode, uint8_t nodeID)
{
  tCANMsgObject msg;
  uint8_t data[8];

  msg.ui32MsgID = 0x600 + nodeID; // Receive
  msg.ui32MsgIDMask = 0;
  msg.ui32Flags = 0;
  msg.ui32MsgLen = 8;
  data[0] = 0x00; // Expedited, 1 byte
  data[1] = 0x60; // Index low byte
  data[2] = 0x60; // Index high byte
  data[3] = 0x00; // Sub-index
  data[4] = mode; // e.g., 3 for Profile Velocity Mode
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
  msg.pui8MsgData = data;
  CANMessageSet(CAN0_BASE, 1, &msg, MSG_OBJ_TYPE_TX);
  Serial.println("Set Operation Mode to Profile Velocity");
}

void CAN_acc_set(uint8_t addr_node, int16_t rpm)
{
  uint8_t mode = 0x83;
  if (rpm < 0)
  {
    mode = 0x83;
  }
  else
  {
    mode = 0x84;
  }

  uint8_t rpm_low = (rpm >> 8) & 0xFF; // Due to little endian shift by 8 forward makes it the lower frame
  uint8_t rpm_high = rpm & 0xFF;

  tCANMsgObject canMessage;
  uint8_t dataBuffer[8];
  // uint8_t node_id = 0x00; //All nodes
  dataBuffer[0] = 0x23;
  dataBuffer[1] = mode;
  dataBuffer[2] = 0x60;
  dataBuffer[3] = 0x00;
  dataBuffer[4] = rpm_high;
  dataBuffer[5] = rpm_low;
  dataBuffer[6] = 0x00;
  dataBuffer[7] = 0x00;

  canMessage.ui32MsgID = 0x600 + addr_node;     // NMT COB-ID
  canMessage.ui32MsgIDMask = 0;                 // Not used for sending
  canMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE; // Transmit message
  canMessage.ui32MsgLen = 8;
  canMessage.pui8MsgData = dataBuffer;

  CANMessageSet(CAN0_BASE, 1, &canMessage, MSG_OBJ_TYPE_TX);
}


void CAN_INT_callback()
{
  uint8_t CAN_Data[8];
  tCANMsgObject CAN_MESSAGE_RECEIVE, CAN_MESSAGE_SEND;
  volatile bool Error = 0;
  volatile bool RXFlag = 0;
  volatile uint8_t int_type_flag=0;
  digitalWrite(PF_3, HIGH);
  unsigned long interrupt_status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);
  Serial.print(interrupt_status);
  Serial.println("hi");
  if (interrupt_status == CAN_INT_INTID_STATUS)
  {
    interrupt_status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
    Serial.println("intstat=1");
    Error = 1;
  }
  else if (interrupt_status == 1)
  {
    CANIntClear(CAN0_BASE, 1);
    Serial.println("intstat=1");
    RXFlag = 1;
    int_type_flag=1;
    Error = 0;
  }

  else if (interrupt_status == 2)
  {
    CANIntClear(CAN0_BASE, 2);
    Serial.println("intstat=2");
    RXFlag = 1;
    int_type_flag=2;
    Error = 0;

  }

  else if (interrupt_status == 3)
  {
    CANIntClear(CAN0_BASE, 3);
    Serial.println("intstat=3");
    int_type_flag=3;
    RXFlag = 1;
    Error = 0;
  }

  if (RXFlag == 1 and (int_type_flag==1 or int_type_flag==2))
  {
    CANMessageGet(CAN0_BASE, 1, &CAN_MESSAGE_RECEIVE, 0);
    Serial.println("CAN MESSAGE ID:");
    Serial.print(CAN_MESSAGE_RECEIVE.ui32MsgID, HEX);
    Serial.println();
    for (int j = 0; j < 8; j++)
    {
      Serial.print(CAN_Data[j]);
      Serial.print('-');
    }
    Serial.println();
    RXFlag = 0;}
  else if(RXFlag==1){
    uint8_t data[8];
    CAN_MESSAGE_RECEIVE.pui8MsgData = data;
    CANMessageGet(CAN0_BASE, 3, &CAN_MESSAGE_RECEIVE, true);
    if (true) {
      int32_t velocity_value = (int32_t)((data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4]);
      Serial.print("Velocity Data:");
      Serial.println(velocity_value);
  }

  }    
    digitalWrite(PF_3, LOW);
  }
  


void motor_startup(uint8_t addr_node,uint8_t work_mode,uint8_t NMT_mode)
{
  CAN_NMT_MODE(NMT_mode, addr_node);       // NMT set to operational
  CAN_work_mode(work_mode, addr_node);      // sets profile velocity
  CAN_state_trans(addr_node, 0x06); // Enable voltage quick stop
  delay(1);
  CAN_state_trans(addr_node, 0x07); // switch on
  delay(1);                         // can't directly switch on and enable needs quick stop
  CAN_state_trans(addr_node, 0x0F); // enable
  delay(1);
  CAN_acc_set(addr_node, 1000);  // sets accn to 1000 rpm
  CAN_acc_set(addr_node, -1000); // sets decn to 1000 rpm
  delay(1);
}


void CAN_start(tCANMsgObject CAN_MESSAGE_RECEIVE, uint8_t CAN_Data_Buffer[8u])
{
  Serial.begin(BAUD_RATE);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0))
  {
  }
  GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5); // PB and OFFSET 4 and 5
  GPIOPinConfigure(GPIO_PB4_CAN0RX);
  GPIOPinConfigure(GPIO_PB5_CAN0TX);

  CANInit(CAN0_BASE);                                 // Initialises CAN Controller afer reset
  CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500000); // Setting bit rate

  pinMode(PF_2, OUTPUT);
  digitalWrite(PF_2, LOW);
  digitalWrite(PF_2, HIGH);
  delay(1000);
  digitalWrite(PF_2, LOW);
  pinMode(PF_1, OUTPUT);
  pinMode(PF_3, OUTPUT);
  digitalWrite(PF_3, HIGH);
  delay(1000);
  digitalWrite(PF_3, LOW);

  CAN_MESSAGE_RECEIVE.ui32MsgID = 0x0;
  CAN_MESSAGE_RECEIVE.ui32MsgIDMask = 0x0;
  CAN_MESSAGE_RECEIVE.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_EXT_FILTER;
  CAN_MESSAGE_RECEIVE.pui8MsgData = CAN_Data_Buffer;

  CANMessageSet(CAN0_BASE, 1, &CAN_MESSAGE_RECEIVE, MSG_OBJ_TYPE_RX);

  CANIntRegister(CAN0_BASE, CAN_INT_callback);
  CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR);
  IntEnable(INT_CAN0);

  CANEnable(CAN0_BASE); // Enables CAN
}

Vel Motor_vel_ser()
{
  String read_string = " ";
  if (Serial.available() > 1)
  {
    Serial.println("nigga");
    while (true)
    {
      char c = Serial.read();
      if (c == '$')
        break;

      read_string += c;
      delay(1);
    }
    if (read_string.length() > 0)
    {
      
      if (read_string.indexOf("V") != -1)
      {
        size_t pos_R = read_string.indexOf("R");
        String vel_R_str;
        while ((read_string[pos_R] != ' ' or read_string[pos_R] != '\n') && pos_R < read_string.length())
        {
          pos_R += 1;
          vel_R_str += read_string[pos_R];
        }
        int16_t vel_R = vel_R_str.toInt();

        size_t pos_L = read_string.indexOf("L");
        String vel_L_str;
        while ((read_string[pos_L] != ' ' or read_string[pos_L] != '\n') && pos_L < read_string.length())
        {
          pos_L += 1;
          vel_L_str += read_string[pos_L];
        }
        int16_t vel_L = vel_L_str.toInt();
        Vel vel = {vel_L, vel_R};
        return vel;
      }
    }
  }
  
  return {-1, -1};
  
}

#endif