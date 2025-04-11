#define  PART_TM4C1230C3PM

#include <Commands.h>
#include <Arduino.h>
#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <interrupt.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "driverlib/sysctl.h"
#include "driverlib/can.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "utils/uartstdio.h"
uint8_t motorR_ID=R_motor;
uint8_t motorL_ID=L_motor;

tCANMsgObject CAN_MESSAGE_RECEIVE,CAN_MESSAGE_SEND;
uint8_t CAN_Data[8u] = {0,0,0,0,0,0,0,0}, CAN_Data_Buffer[8u];

void setup() {
    CAN_start(CAN_MESSAGE_RECEIVE,CAN_Data);
    motor_startup(motorL_ID,3,0x01); //Profile vel setup
    motor_startup(motorR_ID,3,0x01); // Profile Velocity Mode
    
}

Vel vel_setpoint_old={0,0};
Vel vel_setpoint={0,0};
int start=millis();
void loop() {

    vel_setpoint=Motor_vel_ser();
    if(vel_setpoint.vel_L != vel_setpoint_old.vel_L and vel_setpoint.vel_L!=-1){
    CAN_send_target_velocity(motorL_ID,vel_setpoint.vel_L);}
    if(vel_setpoint.vel_R != vel_setpoint_old.vel_R and vel_setpoint.vel_R!=-1){
    CAN_send_target_velocity(motorR_ID,vel_setpoint.vel_R);}
    vel_setpoint_old=vel_setpoint;
    if (millis()-start>3000){
        CAN_read_actual_velocity(motorL_ID);
        CAN_read_actual_velocity(motorR_ID);    
        start=millis();
    }
}
