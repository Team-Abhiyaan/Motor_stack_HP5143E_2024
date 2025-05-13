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
//SDO saga
void SDO_upload_initiate(uint8_t nodeID){
  tCANMsgObject msg;
  uint8_t data[8];
  msg.ui32MsgID = 0x600 + nodeID; // SDO send
  msg.ui32MsgIDMask = 0;
  msg.ui32Flags = 0;
  msg.ui32MsgLen = 8;
  data[0] = 0x40  ; 
  data[1] = 0x6C; 
  data[2] =0x60; 
  data[3] = 0x00; // Sub-index
  data[4] = 0x00;         // COB-ID LSB
  data[5] = 0x00; 
  data[6] = 0x00;
  data[7] = 0x00;
  msg.pui8MsgData=data;

  tCANMsgObject recv;
  recv.ui32MsgID = 0x580 + nodeID; // SDO Receive
  msg.ui32MsgIDMask = 0;
  recv.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;

  CANMessageSet(CAN0_BASE, 3, &msg, MSG_OBJ_TYPE_TX);
  delay(1000);
  CANMessageSet(CAN0_BASE, 4 , &recv, MSG_OBJ_TYPE_RX);
  CANMessageGet(CAN0_BASE, 4, &recv, MSG_OBJ_TYPE_RX);
  

}




void  deactivate_TPDO4(uint8_t nodeID) {
  tCANMsgObject msg;

  uint8_t data[8];
//   uint8_t datareceive[8];// node 1, index 1A03h, sub 01h, 4 bytes

uint32_t COB_Obj=0x00000480 | nodeID;


  msg.ui32MsgID = 0x600 + nodeID; // SDO Receive
  msg.ui32MsgIDMask = 0;
  msg.ui32Flags = 0;
  msg.ui32MsgLen = 8;
  data[0] = 0x23; 
  data[1] = 0x03; 
  data[2] =0x18; 
  data[3] = 0x01; // Sub-index
  
  data[4] = COB_Obj & 0xFF;         // COB-ID LSB
  data[5] = (COB_Obj >> 8) & 0xFF; 
  data[6] = (COB_Obj >> 16) & 0xFF;
  data[7] = (COB_Obj >> 24) & 0xFF;
  msg.pui8MsgData=data;
  CANMessageSet(CAN0_BASE, 1, &msg, MSG_OBJ_TYPE_TX);
  Serial.println("Finnn");
  

  
  }

 // Disable TPDO4 (Set Valid Bit)
void disable_TPDO4(uint8_t nodeID) {
  tCANMsgObject msg;
  uint8_t data[8]={0};
  uint32_t cob_id = 0x80000480 | nodeID; // Set bit 31 + TPDO4 base COB-ID

  // SDO Write: Index 0x1803, Sub-index 0x01 (COB-ID)
  data[0] = 0x23; // 4-byte expedited write
  data[1] = 0x03; // Index LSB (0x1803)
  data[2] = 0x18; // Index MSB
  data[3] = 0x01; // Sub-index
  // Little-endian COB-ID with bit 31 set
  memcpy(&data[4], &cob_id, sizeof(cob_id));

  msg.ui32MsgID = 0x600 + nodeID;
  msg.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
  msg.ui32MsgLen = 8;
  msg.pui8MsgData = data;
  CANMessageSet(CAN0_BASE, 1, &msg, MSG_OBJ_TYPE_TX);
}

// Configure TPDO4 Mapping
void configure_TPDO4_mapping(uint8_t nodeID) {
  tCANMsgObject msg;
  uint8_t data[8];

  // Clear existing mappings (Sub-index 0x00 = 0)
  data[0] = 0x2F; // 1-byte expedited write
  data[1] = 0x03; // Index LSB (0x1A03)
  data[2] = 0x1A; // Index MSB
  data[3] = 0x00; // Sub-index
  data[4] = 0x00; // Value (0 mappings)
  data[5] = 0x00; // Unused
  data[6] = 0x00;
  data[7] = 0x00;

  msg.ui32MsgID = 0x600 + nodeID;
  msg.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
  msg.ui32MsgLen = 8;
  msg.pui8MsgData = data;
  CANMessageSet(CAN0_BASE, 1, &msg, MSG_OBJ_TYPE_TX);

  // Map Velocity (606Ch:00h, 32-bit)
  uint32_t mapping = 0x606C0020; // Object 606Ch, sub 00h, 32 bits
  data[0] = 0x23; // 4-byte write
  data[1] = 0x03; // Index LSB (0x1A03)
  data[2] = 0x1A;
  data[3] = 0x01; // Sub-index 01h
  memcpy(&data[4], &mapping, sizeof(mapping));

  CANMessageSet(CAN0_BASE, 1, &msg, MSG_OBJ_TYPE_TX);

  // Activate 1 mapping
  data[0] = 0x2F; // 1-byte write
  data[4] = 0x01; // 1 mapped object
  CANMessageSet(CAN0_BASE, 1, &msg, MSG_OBJ_TYPE_TX);
}

// Set Transmission Type to Event-Driven
void set_TPDO4_transmission(uint8_t nodeID) {
  tCANMsgObject msg;
  uint8_t data[8];

  data[0] = 0x2F; // 1-byte expedited write
  data[1] = 0x03; // Index LSB (0x1803)
  data[2] = 0x18; // Index MSB
  data[3] = 0x02; // Sub-index (Transmission Type)
  data[4] = 0xFF; // Event-driven (0xFF)
  data[5] = 0x00; // Unused
  data[6] = 0x00;
  data[7] = 0x00;

  msg.ui32MsgID = 0x600 + nodeID;
  msg.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
  msg.ui32MsgLen = 8;
  msg.pui8MsgData = data;
  CANMessageSet(CAN0_BASE, 1, &msg, MSG_OBJ_TYPE_TX);
}

// Enable TPDO4 (Clear Valid Bit)
void enable_TPDO4(uint8_t nodeID) {
  tCANMsgObject msg;
  uint8_t data[8];
  uint32_t cob_id = 0x00000480 | nodeID; // Clear bit 31

  data[0] = 0x23; // 4-byte expedited write
  data[1] = 0x03; // Index LSB (0x1803)
  data[2] = 0x18;
  data[3] = 0x01; // Sub-index
  memcpy(&data[4], &cob_id, sizeof(cob_id));

  msg.ui32MsgID = 0x600 + nodeID;
  msg.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
  msg.ui32MsgLen = 8;
  msg.pui8MsgData = data;
  CANMessageSet(CAN0_BASE, 1, &msg, MSG_OBJ_TYPE_TX);
}

void CAN_handle_tpdo(uint8_t node_id) {
  tCANMsgObject msg;
    uint8_t data[8];
    msg.pui8MsgData = data;
    msg.ui32MsgID= 0x1803 +node_id;
    msg.ui32MsgLen = 8;
    msg.ui32Flags = MSG_OBJ_RX_INT_ENABLE;
   msg.pui8MsgData = data;
    
    // Extract node_id from COB-ID (0x480 + node_id)
    // uint8_t node_id = msg.ui32MsgID - 0x480;
    
  
 
  Serial.println("Mail 3");
  CANMessageGet(CAN0_BASE, 3, &msg, 0); // Mailbox 3 for TPDO4
  int8_t rpm = (data[0] << 0)  | 
                      (data[1] << 8)  | 
                      (data[2] << 16) | 
                      (data[3] << 24)|(data[4] << 32)  | 
                      (data[5] << 40)  | 
                      (data[6] << 48) | 
                      (data[7] << 56);
      int32_t velocity;
      Serial.print("RpmVAL");
      Serial.println(rpm);
    
  }



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
  // uint8_t node_id = 0x00; // All nodes
  dataBuffer[0] = mode;
  dataBuffer[1] = addr_node;

  canMessage.ui32MsgID = 0x000;                 // NMT COB-ID
  canMessage.ui32MsgIDMask = 0;                 // Not used for sending
  canMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE; // Transmit message
  canMessage.ui32MsgLen = 2;
  canMessage.pui8MsgData = dataBuffer;

  CANMessageSet(CAN0_BASE, 1, &canMessage, MSG_OBJ_TYPE_TX);
}

//Optional SYNC and Heartbeat message
void CAN_send_SYNC()
{
  tCANMsgObject msg;
  // uint8_t dataBuffer[8];

  msg.ui32MsgID = 0x080;
  msg.ui32MsgLen = 0;
  msg.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
  msg.pui8MsgData = 0;

  CANMessageSet(CAN0_BASE, 4, &msg, MSG_OBJ_TYPE_TX);
}

//Optional
void emcy(uint8_t nodeID) {
  tCANMsgObject msg;
  uint8_t data[8] = {0};

  // EMCY COB-ID = 0x80 + Node-ID (little-endian format)
  msg.ui32MsgID = 0x600 + nodeID; 
  msg.ui32MsgIDMask = 0;
  msg.ui32Flags = MSG_OBJ_TX_INT_ENABLE; // Enable TX interrupt
  msg.ui32MsgLen = 8;

  // Overspeed error code (0xFF31) in little-endian
  data[0] = 0x31;    // Error code LSB
  data[1] = 0xFF;    // Error code MSB
  data[2] = 0x81;    // Error register (overspeed)
  // Bytes 3-7: Manufacturer-specific (set to 0)

  msg.pui8MsgData = data;
  
  // Use dedicated TX mailbox (e.g., mailbox 1)
  CANMessageSet(CAN0_BASE, 5, &msg, MSG_OBJ_TYPE_TX);
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
  
    canMessage.ui32MsgID = 0x600 + addr_node;     // COB-ID
    canMessage.ui32MsgIDMask = 0;                 // Not used for sending
    canMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE; // Transmit message
    canMessage.ui32MsgLen = 8;
    canMessage.pui8MsgData = dataBuffer;
  
    CANMessageSet(CAN0_BASE, 1, &canMessage, MSG_OBJ_TYPE_TX);
  }
  


void CAN_send_target_velocity(uint8_t nodeID, uint32_t velocity)
{ if (velocity==3678){
    Serial.println("Emergency stop triggered");
    CAN_state_trans(nodeID , 0x07);//quick stop
    return;
}
  if (velocity==3679){
    Serial.println("Enable after stop triggered");
    CAN_state_trans(nodeID , 0x0F);//enable
    return;
  }
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

//Generic sdo send can be used to make new sdo commands
void CAN_send_sdo(uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t value) {
  tCANMsgObject msg;
  uint8_t sdo_data[8] = {0};
  uint8_t data[4]={0};
  data[0] = (value) & 0xFF;
  data[1] = (value >> 8) & 0xFF;
  data[2] = (value >> 16) & 0xFF;
  data[3] = (value >> 24) & 0xFF;

  // SDO download initiate command (expedited, 4 bytes)
  sdo_data[0] = 0x23; // CCS=1 (download), expedited, size indicated, 4 bytes
  sdo_data[1] = index & 0xFF;        // Index low byte
  sdo_data[2] = (index >> 8) & 0xFF; // Index high byte
  sdo_data[3] = subindex;            // Sub-index
  sdo_data[4] = data[0];
  sdo_data[5] = data[1];
  sdo_data[6] = data[2];
  sdo_data[7] = data[3];

  msg.ui32MsgID = 0x600 + node_id; // SDO client-to-server
  msg.ui32MsgLen = 8;
  msg.pui8MsgData = sdo_data;
  msg.ui32Flags = MSG_OBJ_TX_INT_ENABLE;

  CANMessageSet(CAN0_BASE, 1, &msg, MSG_OBJ_TYPE_TX);
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
  tCANMsgObject CAN_MESSAGE_RECEIVE;
  bool Error = 0;
  volatile bool RXFlag = 0;
  volatile uint8_t int_type_flag=0;
  digitalWrite(PF_2, HIGH);
  unsigned long interrupt_status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);
  // Serial.print(interrupt_status);
  // Serial.println("hi");
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
  else if(interrupt_status == 4)
  {
    CANIntClear(CAN0_BASE, 4);
    Serial.println("intstat=4");
    int_type_flag=4;
    RXFlag = 1;
    Error = 0;
  }
  else if(interrupt_status == 5)
  {
    CANIntClear(CAN0_BASE, 5);
    Serial.println("intstat=5");
    int_type_flag=4;
    RXFlag = 1;
    Error = 0;
  }
  else{Serial.println("New error popping");}

  if (RXFlag == 1 and (int_type_flag==1 or int_type_flag==2) )
  {
    CANMessageGet(CAN0_BASE, 2, &CAN_MESSAGE_RECEIVE, 0);
    Serial.println("CAN MESSAGE ID:");
    Serial.println(CAN_MESSAGE_RECEIVE.ui32MsgID, HEX);
    // Serial.println(Error);
    // for (int j = 0; j < 8; j++)
    // {
    //   Serial.print(CAN_Data[j]);
    //   Serial.print('-');
    // }
    Serial.println("Sus");
    RXFlag = 0;
  }
  else if(RXFlag==1&& int_type_flag==4){
    tCANMsgObject recv;
    uint8_t data[8];
    recv.pui8MsgData = data;
    
    CANMessageGet(CAN0_BASE, 4, &recv, 0);
    Serial.print("rpm: ");
    uint32_t rpm =(recv.pui8MsgData[4]<<0|
                (recv.pui8MsgData[5]<<8)|
                (recv.pui8MsgData[6]<<16)|
                (recv.pui8MsgData[7]<<24));
    Serial.println(rpm);
    
    
  }

  
  if(Error==1){Serial.println("Brain fucked");}    
  digitalWrite(PF_2, LOW);
}  
  


void motor_startup(uint8_t addr_node,uint8_t work_mode,uint8_t NMT_mode)
{
  CAN_NMT_MODE(NMT_mode, addr_node);       // NMT set to operational
  // CAN_work_mode(work_mode, addr_node);      // sets profile velocity
  // CAN_state_trans(addr_node, 0x06); // Enable voltage quick stop
  delay(1);
  // CAN_state_trans(addr_node, 0x07); // switch on
  delay(1);                         // can't directly switch on and enable needs quick stop
  // CAN_state_trans(addr_node, 0x0F); // enable
  delay(1);
  // CAN_acc_set(addr_node, 1000);  // sets accn to 1000 rpm
  // CAN_acc_set(addr_node, -1000); // sets decn to 1000 rpm
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

Vel Motor_vel_ser()//Message of type V Lint Rint $
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
      else if(c=='e')
      {
        Serial.println("inside estop func");
        Vel vel = {3678,3678};
        return vel;

      delay(2000);
      }
      else if(c=='r'){
        Serial.println("Enable after stop");
        Vel vel = {3679,3679};
        return vel;
      }

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

      else if (read_string.indexOf("Shutdown") != -1)
      {
        Serial.println("Shutdown command received");
        // Perform shutdown actions here
        // For example, stop motors, save state, etc.
        CAN_NMT_MODE(0x08, 0x00);       // NMT set to operational
      }
    }
  }
  
  return {-1, -1};
  
}

#endif