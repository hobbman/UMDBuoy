import smbus2
import time
import board
import busio
from adafruit_tca9548a import TCA9548A
from adafruit_lsm6ds import LSM6DS33
from tsys01 import TSYS01

i2c=busio.I2C(board.SCL, board.SDA)

# MUX
tca = TCA9548A(i2c)

# IMU
lsm6=LSM6DS33(tca[0])

# Temperature
blueT1 = TSYS01()
# blueT2 = TSYS01(tca[2])
try:
    blueT1.init()
except TypeError:
    print("Error staring Temp sensor")
    exit()
# blueT2.init()

while True:
    print("Acceleration: X:%.2f, Y:%.2f, Z:%.2f m/s^2" % lsm6.acceleration)
    print("Gyro: X:%.2f, Y:%.2f, Z:%.2f degrees/s" % lsm6.gyro)
    print("")
    time.sleep(0.5)
