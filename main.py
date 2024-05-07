from machine import Pin, I2C
import time

# Initialize I2C bus with SDA on D2 and SCL on D1
i2c = I2C(scl=Pin(1), sda=Pin(2))

# MMA8451 I2C address
MMA8451_ADDRESS = 0x1C

# MMA8451 register addresses
MMA8451_CTRL_REG1 = 0x2A
MMA8451_OUT_X_MSB = 0x01

def read_byte(reg):
    i2c.writeto(MMA8451_ADDRESS, bytes([reg]))
    data = i2c.readfrom(MMA8451_ADDRESS, 1)
    return data[0]

def read_word(reg):
    high = read_byte(reg)
    low = read_byte(reg + 1)
    val = (high << 8) | low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

def init_mma8451():
    # Set up the MMA8451.
    i2c.writeto_mem(MMA8451_ADDRESS, MMA8451_CTRL_REG1, bytes([0x00]))

def read_acceleration():
    x = read_word(MMA8451_OUT_X_MSB)
    y = read_word(MMA8451_OUT_X_MSB)
    z = read_word(MMA8451_OUT_X_MSB)
    return x, y, z

init_mma8451()

while True:
    x, y, z = read_acceleration()
    print("X: %d, Y: %d, Z: %d" % (x, y, z))
    time.sleep(1)
