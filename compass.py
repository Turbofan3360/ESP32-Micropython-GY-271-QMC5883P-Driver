from machine import SoftI2C, Pin
from math import atan2, sin, asin, cos, pi
import struct, time

class Magnetometer:
    def __init__(self, scl, sda, drdy):
        self.qmc5883p = SoftI2C(scl=Pin(scl), sda=Pin(sda), freq=400000)
        self.qmc5883p_address = 0x2C
    
        self.registers = {
                    "chipid" : 0x00,
                    
                    "x-axis data" : 0x01, # Two bytes, LSB -> MSB
                    "y-axis data" : 0x03, # ...
                    "z-axis data": 0x05,  # ...
                    "axis invert" : 0x29,
                    
                    "status" : 0x09,
                    "control1" : 0x0A,
                    "control2" : 0x0B,
                    }
        
        self.data = [0, 0, 0]
        self.new_data = False
        
        time.sleep_us(250) # Module needs about 250 microseconds to boot up from power on -> being able to receive I2C comms
        
        self._modulesetup(drdy)
        
    def _modulesetup(self, interrupt_pin):
        # Setting the module to Normal power mode, 200Hz data output rate, x4 sensor reads per data output, down sampling = 0 (output every sample)
        self.qmc5883p.writeto_mem(self.qmc5883p_address, self.registers["control1"], bytes([0x1D]))
        # Setting the module to 2 Gauss range, "Set and Reset On" mode
        self.qmc5883p.writeto_mem(self.qmc5883p_address, self.registers["control2"], bytes([0x0C]))
        
        # Setting up the interrupt pin to know when data is ready from the chip
        self.int_pin = Pin(interrupt_pin, Pin.IN)
        self.int_pin.irq(trigger=Pin.IRQ_FALLING, handler=self._newdatahandler)
    
    def _newdatahandler(self, pin):
        # This deals with interrupts when there is new data availible
        self.new_data = True
    
    def _update_data(self):
        if self.new_data:
            # Reading all 6 data bytes in one burst
            data = self.qmc5883p.readfrom_mem(self.qmc5883p_address, self.registers["x-axis data"], 6)

            # Decoding the bytes data into signed ints, then converting the readings to Gauss
            x_axis = struct.unpack("<h", data[:2])[0]/15000
            y_axis = struct.unpack("<h", data[2:4])[0]/15000
            z_axis = struct.unpack("<h", data[4:])[0]/15000
            
            self.data[0] = x_axis
            self.data[1] = y_axis
            self.data[2] = z_axis
            
            self.new_data = False
        
    def getdata_raw(self):
        self._update_data()
        return(self.data)
    
    def compass_2d(self, declination=0):
        self._update_data()
        # Simple calculation of true heading (if you pass in a declination value) assuming the compass is level
        heading = atan2(self.data[1], -self.data[0])*(180/pi) - declination
        
        # Ensuring heading values go from 0 to 360, rather than +180 to -180
        heading %= 360
        
        return int(heading+0.5) # Rounds to nearest degree
    
    def _rotate_mag_readings(self, pitch, roll):
        mx, my, mz = self.data
        
        # http://www.brokking.net/YMFC-32/YMFC-32_document_1.pdf
        rx = mx*cos(pitch) + my*sin(roll)*sin(pitch) - mz*cos(roll)*sin(pitch)
        ry = my*cos(roll) + mz*sin(roll)
        
        return rx, ry
    
    def _quat_rotate_mag_readings(self, q):
        qw, qx, qy, qz = q
        mx, my, mz = self.data
        
        # Rotate magnetometer vector into world reference frame
        rx = (qw*qw + qx*qx - qy*qy - qz*qz)*mx + 2*(qx*qy - qw*qz)*my + 2*(qx*qz + qw*qy)*mz
        ry = 2*(qx*qy + qw*qz)*mx + (qw*qw - qx*qx + qy*qy - qz*qz)*my + 2*(qy*qz - qw*qx)*mz
        
        return rx, ry
    
    def compass_3d(self, quat=None, pitch=None, roll=None, declination=0): # quaternion input as list [qw, qx, qy, qz], pitchroll values input in radians - NOT DEGREES
        self._update_data()
        
        if (not quat) and not (pitch or roll): # If you don't pass in quaternion, or pitch AND roll values, then None is returned
            return None
        
        # Checking which format orientation data was passed in - if it's quaternion, then use a quaternion rotation. Otherwise a standard trig rotation is used
        if quat:
            rx, ry = self._quat_rotate_mag_readings(quat)
        else:
            rx, ry = self._rotate_mag_readings(pitch, roll)
        
        heading = atan2(-ry, -rx)*(180/pi) - declination
        
        # Ensuring heading goes from 0-360 degrees
        heading %= 360
        
        return int(heading+0.5) # Rounds to nearest degree
        

if __name__ == "__main__":
    import mpu6050 as MPU6050
    
    imu = MPU6050.MPU6050(41, 42)
    imu.dmpsetup(2)
    imu.calibrate(10)
    
    compass = Magnetometer(46, 3, 1)
    
    while True:
        quaternion, orientation, localvel, worldacc = imu.imutrack()
        print(compass.compass_3d(quat=quaternion, declination=1.43)) # Need to combine with IMU to get orientation data
        time.sleep(1)