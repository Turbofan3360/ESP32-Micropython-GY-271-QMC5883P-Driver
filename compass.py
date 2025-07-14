from machine import SoftI2C, Pin
from math import atan2, pi
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
        # Inverting the Y-axis (makes it align with how I want)
        self.qmc5883p.writeto_mem(self.qmc5883p_address, self.registers["axis invert"], bytes([0x01]))
        
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
        heading = atan2(self.data[1], self.data[0])*(180/pi) - declination
        
        # Ensuring heading values go from 0 to 360, rather than +180 to -180
        if heading < 0:
            heading += 360
        elif heading > 360:
            heading -= 360
        
        return int(heading)

if __name__ == "__main__":
    compass = Magnetometer(46, 3, 1) # drdy could be anything, not used currently
    
    while True:
        print(compass.compass_2d(1.43))
        time.sleep(1)