from machine import SoftI2C, Pin
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
                    
                    "status" : 0x09,
                    "control1" : 0x0A,
                    "control2" : 0x0B,
                    }
        
        time.sleep_us(250) # Module needs about 250 microseconds to boot up from power on -> being able to receive I2C comms
        
        self._modulesetup(drdy)
        
    def _modulesetup(self, interrupt_pin):
        # Setting the module to Normal power mode, 200Hz data output rate, x4 sensor reads per data output, down sampling = 0 (output every sample)
        self.qmc5883p.writeto_mem(self.qmc5883p_address, self.registers["control1"], bytes([0x1D]))
        # Setting the module to 2 Gauss range, "Set and Reset On" mode
        self.qmc5883p.writeto_mem(self.qmc5883p_address, self.registers["control2"], bytes([0x0C]))
        
        # Setting up the interrupt pin to know when data is ready from the chip
        self.int_pin = Pin(interrupt_pin, Pin.IN)
        self.int_pin.irq(trigger=Pin.IRQ_FALLING, handler=self._newdatahandler())
    
    def _newdatahandler(self):
        # This will deal with interrupts when there is new data availible
        pass
    
    def _read_data(self):
        # Reading all 6 data bytes in one burst
        data = self.qmc5883p.readfrom_mem(self.qmc5883p_address, self.registers["x-axis data"], 6)
        # Breaking down the data into each axis' data
        x_axis = data[:2]
        y_axis = data[2:4]
        z_axis = data[4:]
        # Decoding the bytes data into signed ints, then converting the readings to Gauss
        x_axis = struct.unpack("<h", x_axis)[0]/15000
        y_axis = struct.unpack("<h", y_axis)[0]/15000
        z_axis = struct.unpack("<h", z_axis)[0]/15000
        
        return x_axis, y_axis, z_axis


if __name__ == "__main__":
    compass = Magnetometer(46, 3, 5) # drdy could be anything, not used currently
    
    print(compass._read_data())