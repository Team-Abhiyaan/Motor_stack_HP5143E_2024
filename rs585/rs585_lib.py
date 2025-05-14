import minimalmodbus
import serial
import time

class MotorOriental:
    def __init__(self, port: str, slave_id: int,name : str):
        self.instrument = minimalmodbus.Instrument(port, slave_id, mode='rtu')
        self.instrument.serial.baudrate = 230400
        self.instrument.serial.bytesize = 8
        self.instrument.serial.parity = serial.PARITY_EVEN
        self.instrument.serial.stopbits = 1
        self.instrument.serial.timeout = 0.5  # seconds
        self.name=name
        self.ID_share= False
    def S_ON(self):
        # Excite motor (S-ON) via Driver Input Command at 0x007C–0x007D
        self.instrument.write_registers(0x007C, [0x0000, 0x0001])
        time.sleep(0.1)

    def set_velocity(self, velocity: int):
        # Start continuous speed-control operation (STEP 3) at 0x005A–0x0067
        payload = [
            0x0000, 0x0030,             # Operation type = 0x0030 (speed control)
            0x0000, 0x0000,             # Position = 0
            (velocity >> 16) & 0xFFFF,  # Velocity high word
            velocity & 0xFFFF,          # Velocity low word
            0x0000, 0x03E8,             # Accel = 1000 ms
            0x0000, 0x09C4,             # Decel = 2500 ms
            0x0000, 0x03E8,             # Torque limit = 100.0%
            0x0000, 0x0001              # Trigger = 1 (normal start)
        ]
        self.instrument.write_registers(0x005A, payload)
        print(f"Velocity set to {velocity} r/min from {self.name}")
        
    def stop(self):
        # STEP 7 Stop of operation: Operation type = 0, Velocity = 0
        stop_payload = [
            0x0000, 0x0000,  # Operation type = 0 (decel stop)
            0x0000, 0x0000,  # Position = 0
            0x0000, 0x0000,  # Velocity = 0
            0x0000, 0x03E8,  # Accel = 1000 ms
            0x0000, 0x09C4,  # Decel = 2500 ms
            0x0000, 0x03E8,  # Torque limit = 100.0%
            0x0000, 0x0001   # Trigger = 1 (normal stop)
        ]
        self.instrument.write_registers(0x005A, stop_payload)  # FC16 to 0x005A[1]
        time.sleep(0.1)
        print("Stop command sent ")
    def vel_read(self):
        # Read the current velocity from the motor
        raw = self.instrument.read_long(0x00A0, 0, functioncode=3,signed=True,byteorder=minimalmodbus.BYTEORDER_BIG)   
        # if raw & 0x80000000:       # -ve
        #     raw -= 1 << 32
        print(f"Actual velocity: {raw} r/min from {self.name}")
    
    def torque_read(self):
        # Read the current torque from the motor
        raw = self.instrument.read_long(0x0D6, 0, functioncode=3,signed=True,byteorder=minimalmodbus.BYTEORDER_BIG)   
        # if raw & 0x80000000:       # -ve
        #     raw -= 1 << 32
        print(f"Actual torque: {raw} N.m from {self.name}")
    
    
    def id_share_set(self):
        # id share velocity and throttle implementation
        self.write_registers(0x0980, [0x0000, 0x000F,  0x0000, 0x0002,  0x0000, 0x0001])
        self.address = 2
        self.write_registers(0x0980, [0x0000, 0x000F,  0x0000, 0x0002,  0x0000, 0x0002])
        self.address = 1
        self.ID_share= minimalmodbus.Instrument(self.port, 0x000F, mode='rtu')
        # After setting up group (Global ID=15, Number=2, Local ID=1/2) …
        self.ID_share.address = 1
        # Share Read Data 0 NET-ID = 0x0050 (Actual velocity)
        self.ID_share.write_registers( 0x0990,[0x0000, 0x0050]    # Upper/Lower half of NET-ID 0x0050
                                        )

        
    def broadcast_stop(self):
        payload = [
        0x0000, 0x0030,   # Operation type = speed control
        0x0000, 0x0000,   # Position = 0
        0x0000, 0x0000,   # Velocity (INT32)
        0x0000, 0x03E8,   # Accel = 1000 ms
        0x0000, 0x09C4,   # Decel = 2500 ms
        0x0000, 0x03E8,   # Torque limit
        0x0000, 0x0001    # Trigger = normal start
        ]
        self.ID_share.write_registers(0x0000, payload)
    
    def broadcast_read(self):
        raw = self.ID_share.read_registers(0x0000, 4)
        def to_signed32(hi, lo):
            v = (hi << 16) | lo
            return v - (1<<32) if v & 0x80000000 else v
        v1 = to_signed32(raw[0], raw[1])
        v2 = to_signed32(raw[2], raw[3])
        print(f"Motor 1 vel={v1} r/min, Motor 2 vel={v2} r/min")
