import global_param as gp
from rs585_lib import MotorOriental
import time
import serial
import minimalmodbus

Mot1 = MotorOriental(gp.port,2,'Mot1')
X=0
while(X<2):
    Mot1.set_velocity(100)  # Set velocity to 50 r/min
    Mot1.stop()  # Set velocity to 0 r/min (stop command)
    Mot1.vel_read()  # Read the current velocity from the motor
    Mot1.torque_read()  # Read the current torque from the motor
    time.sleep(4)
    Mot1.set_velocity(-100)  # Set velocity to 50 r/min
    Mot1.torque_read()  # Read the current torque from the motor 
    Mot1.vel_read()  # Read the current velocity from the motor   
    Mot1.stop()  # Set velocity to 0 r/min (stop command)
    time.sleep(4)
    X+=1

# def set_velocity(drv2, vel):
#     payload = [
#         0x0000,  # [005Ah] Operation type high word
#         0x0030,  # [005Bh] Operation type low  = 48 (motion-extension, speed control)
#         0x0000,  # [005Ch] Position high word
#         0x0000,  # [005Dh] Position low  = 0 steps
#         (vel >> 8) & 0xFFFF,  # [005Eh] Operating velocity high
#         (vel >> 0) & 0xFFFF,  # [005Fh] Operating velocity low  = 50 r/min
#         0x0000,  # [0060h] Acceleration rate high
#         0x0256,  # [0061h] Acceleration rate low  = 1000 ms
#         0x0000,  # [0062h] Deceleration rate high
#         0x0256,  # [0063h] Deceleration rate low  = 2500 ms
#         0x0000,  # [0064h] Torque limit high
#         0x03E8,  # [0065h] Torque limit low  = 100.0 %
#         0x0000,  # [0066h] Trigger high word
#         0x0001,  # [0067h] Trigger low  = 1 (normal start, lifetime disable)
#     ]

#     # 3) Send the Write Multiple Registers request (function code 16)
#     drv2.write_registers(0x005A, payload)

#     print(f"Velocity set to {vel} r/min.")

# # --- 1) Create Modbus RTU instrument on COM3, Slave ID = 1
# drv = minimalmodbus.Instrument('COM9', 2, mode='rtu')
# drv.serial.baudrate = 230400
# drv.serial.bytesize = 8
# drv.serial.parity   = serial.PARITY_EVEN
# drv.serial.stopbits = 1
# drv.serial.timeout  = 0.5  # seconds

# # 3) Servo ON: set the digital “S-ON” input bit  
# #    Register 0x003E (62 decimal) is the lower 16 bits of “Driver input command”  
# #    Bit 0 = S-ON  
# drv.write_registers(0x007C, [0x0000, 0x0001])  # FC=16
# time.sleep(0.1)

# set_velocity(drv, 100)  # Set velocity to 50 r/min
# time.sleep(4)

# set_velocity(drv, -100)  # Set velocity to 50 r/min
# time.sleep(4)

# set_velocity(drv, 0)  # Set velocity to 0 r/min (stop command)

print("Stop command sent - motor will decelerate according to profile.")  