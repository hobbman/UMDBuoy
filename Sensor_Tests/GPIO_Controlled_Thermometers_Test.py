import time
import tsys01
import board
import busio
from adafruit_lsm6ds import LSM6DS33
from adafruit_bus_device.i2c_device import I2CDevice
from time import sleep
import RPi.GPIO as GPIO

test = GPIO.getmode()
print(test)
# GPIO.BCM is used to identify pins

# Safe to disable
GPIO.setwarnings(False)

# Thermometer Air
GPIO.setup(27, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.output(27, GPIO.HIGH)
GPIO.output(22, GPIO.HIGH)
sleep(1)
air = tsys01.TSYS01()
air.init()
sleep(1)
GPIO.output(27, GPIO.LOW)
GPIO.output(22, GPIO.LOW)

# Thermometer Water
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.output(23, GPIO.HIGH)
GPIO.output(24, GPIO.HIGH)
sleep(1)
water = tsys01.TSYS01()
water.init()
sleep(1)
GPIO.output(24, GPIO.LOW)
GPIO.output(23, GPIO.LOW)

# IMU Setup
i2c = busio.I2C(board.SCL, board.SDA)
imu = LSM6DS33(i2c)

# Keep track of which sensor should be enabled
whoseTurn = 0

while True:
    if (whoseTurn == 0 or whoseTurn == 1):
        # Enables Air Temp Reading
        if (whoseTurn == 2): # Set back to 0 after debugging
            GPIO.output(27, GPIO.HIGH)
            GPIO.output(22, GPIO.HIGH)
            sleep(1)
            if not air.read():
                print("Error reading Air Temp")
            else:
                print("Air Temp: %.2f C" % air.temperature(tsys01.UNITS_Centigrade))
                sleep(0.2)
            GPIO.output(22, GPIO.LOW)
            GPIO.output(27, GPIO.LOW)
            whoseTurn = whoseTurn + 1
        # Enables Water Temp Reading
        else:
            GPIO.output(23, GPIO.HIGH)
            GPIO.output(24, GPIO.HIGH)
            sleep(1)
            if not water.read():
                print("Error reading Water")
            else:
                print("Water Temp: %.2f C" % water.temperature(tsys01.UNITS_Centigrade))
                sleep(0.2)
            GPIO.output(23, GPIO.LOW)
            GPIO.output(24, GPIO.LOW)
            whoseTurn = whoseTurn + 1
    else:
        print("Acceleration: X:%.2f, Y:%.2f, Z:%.2f m/s^2" % (imu.acceleration))
        print("Gyro: X:%.2f, Y:%.2f, Z:%.2f degrees/s" % (imu.gyro))
        print("*****************************************************************")
        print("")
        sleep(0.5)
        whoseTurn = 0
