from typing import Any
import time
import os
from time import sleep
from datetime import datetime
import board
import busio
import adafruit_gps
import serial
from adafruit_lsm6ds import LSM6DS33
import RPi.GPIO as GPIO
import tsys01
import glob
import signal
#import adafruit_ads1x15.ads1015 as ADS # Uncomment to add voltage monitoring (12-Bit ADC)
#import adafruit_ads1x15.ads1115 as ADS # Uncomment to add voltage monitoring (16-bit ADC)
#import adafruit_ads1x15.analog_in import AnalogIn # Uncomment when either ADC is added

### Please note that all ADC code has not been tested and may require adjustment to function ###

# Function for saving Data String to specified file - Returns void
def new_log(file_path, file_data="No Data Entered"):
    temp_file = open(file_path, "a")
    temp_file.write(file_data)
    temp_file.flush()
    time.sleep(5)
    temp_file.close()

# Function that Handles ASCII transmission via RockBLOCK 9603
def satillite_transmit(message_out):
    againtry = True
    set_try_time = time.monotonic()
    while againtry:
        cur_try_time = time.monotonic() 
        print(len(message_out), "If less then 50 should cost only 1 credit")
        # Setup communication to Sat Comm Module
        #ser = serial.Serial('/dev/ttyUSB0', 19200, 8, 'N', 1, timeout=10)
        global ser
        ser.write(b"AT+CSQ\r")  # Command to check signal strength
        response = ser.read(5) # Debugging to check antenna
        print(response.decode())
        print("*******************************************")
        ser.write(b"AT\r")
        response = ser.read(4)
        print("*******************************************")
        ser.write(b"AT&K0") # Disable Flow Control
        response = ser.read(11)
        print(response.decode(), "&K0 Called")
        print("+++++++++++++++++++++++++++++++++++++++++++")
        #ser.write(("AT+SBDWT=" + message_out + "\r").encode())  # Sends ASCII Message
        formatted_msg = "AT+SBDWT=" + message_out + "\r"
        ser.write(formatted_msg.encode())
        response = ser.read(20)
        print(response.decode(), "+SBDWT Called")
        print("___________________________________________")
        ser.write(b"AT+SBDIX\r")  # Checks if message was sent
        response = ser.read(66)
        valuech = response.decode()
        print(valuech)
        if "SBDIX:" in valuech:
            print("Command recieved")
            againtry = False
        else:
            print("Issue with final Command, retrying...")
        if cur_try_time - set_try_time >= 240:
            againtry = False
    ser.close()

# Function for creating new files - Returns String
def log_setup():
    filename = input("Enter desired filename: ")
    file_path = "~/" + filename + ".csv"
    print("File will be saved as" + file_path)
    temp_file = open(file_path, "a")
    if os.stat(file_path).st_size == 0:
        print("Give desired column names and separate with ','\ni.e. Time,Date,Lat,Long")
        usr_input = input("Column Names: ")
        temp_file.write(usr_input+"\n")
    time.sleep(5)
    temp_file.close()
    return file_path


# Function that allows users to adjust sampling rates with no need for coding knowledge - Returns void
def sample_setup():
    global gps_Rate
    global temp_Rate_air
    global temp_Rate_h2o
    global sat_Rate
    global imu_Rate_start
    global imu_Rate_end
    print('#' * 40)
    print('#' * 40)
    print("Default sample rates are as follow: \nGps:15min\nSatellite:15min\nAir Temp:60min\nWater Temp:60min")
    print("IMU samples for 10min between 50-60min mark\n")
    print('#' * 40)
    print("Enter the number zero if you want to use default sample rates.")
    print('#' * 40)
    while True:
        try:
            input_temperature = int(input("Enter GPS sample rate (in minutes): "))
            break
        except ValueError:
            print("Must enter an integer value!")
            sys.exit()
    if input_temperature > 0:
        gps_Rate = input_temperature*60
    print('#' * 40)
    while True:
        try:
            input_temperature = int(input("Enter Satellite Transmission Rate (in minutes): "))
            break
        except ValueError:
            print("Must enter an integer value!")
    if input_temperature > 0:
        sat_Rate = input_temperature*60
    print('#' * 40)
    while True:
        try:
            input_temperature = int(input("Enter Air Temperature sample rate (in minutes): "))
            break
        except ValueError:
            print("Must enter an integer value!")
    if input_temperature > 0:
        temp_Rate_air = input_temperature*60
    print('#' * 40)
    while True:
        try:
            input_temperature = int(input("Enter Water Temperature sample rate (in minutes): "))
            break
        except ValueError:
            print("Must enter an integer value!")
    if input_temperature > 0:
        temp_Rate_h2o = input_temperature*60
    print('#' * 40)
    print("Now the IMU will be setup. \nFirst a start time will be given.")
    print("If 30 is given. IMU will begin recording at the 30th minute of the hour.")
    while True:
        try:
            input_temperature = int(input("Enter the START time of IMU data recording (in minutes): "))
            break
        except ValueError:
            print("Must enter an integer value!")
    if input_temperature > 0:
        imu_Rate_start = input_temperature*60
    print('#' * 40)
    print("Now the IMU end time will be given.")
    print("If 40 is given. IMU will end recording data around the 40th minute of the hour.")
    print("Example: If 30 was given as the start time and 40 is given for the end time.")
    print("Sampling will take place between HH:30 and HH:40.")
    print("Meaning there is a total sampling time of 10min")
    while True:
        try:
            input_temperature = int(input("Enter the END time of the IMU data recording (in minutes): "))
            if imu_Rate_start/60 < input_temperature:
                break
            else:
                print("End time must be greater then the start time!")
        except ValueError:
            print("Must enter an integer value!")
    if input_temperature > 0:
        imu_Rate_end = input_temperature*60
    print('#' * 40)
    print('#' * 40)
    print("New Sample Rates are as follow:\nGps:" + str(int(gps_Rate/60)) + "min\nSatellite:" + str(int(sat_Rate/60))
          + "min\nAir Temp:" + str(int(temp_Rate_air/60)) + "min")
    print("Water Temp:"+str(int(temp_Rate_h2o/60)) + "min\nIMU samples for " + str(int((imu_Rate_end-imu_Rate_start)/60))
          + "min between " + str(int(imu_Rate_start/60)) + "-" + str(int(imu_Rate_end/60)) + "min mark.")
    print('#' * 40)

#######################################################################################################################
# End Functions ########################## End Functions ####################### End Functions ########################
#######################################################################################################################


# Initialized Sample rates
imu_Rate_start = 3000  # Start 10min to top of the hour
imu_Rate_end = 3600  # End at top of hour
gps_Rate = 1800  # Default to 30 minutes for GPS
sat_Rate = 1800  # Default to 30 minutes for Sat Comm
temp_Rate_air = 3600  # Default once an hour - Air Temp
temp_Rate_h2o = 3600  # Default once an hour - Water Temp
adc_Rate = 1800  # Default to 30 minutes for ADC

# Disable GPIO warnings (these can be safely ignored in this case)
GPIO.setwarnings(False)

# Allows User to Adjust Sample rates
sample_setup() # Comment out to disallow user given sample rates

# Bool to exit while-loop
infLoop = True

# Use Pi UART (Tx/Rx) for GPS
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=10)

# Use Pi UART (Tx/Rx) for Sat Comm (9603)
ser = serial.Serial("/dev/ttyUSB0", 19200, 8, 'N', 1, timeout=10)

# Create a GPS module instance.
gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial

# Create variable for Temperature sensors
airRecord = "No Record"
waterRecord = "No Record"

# Create variable for ADC sensor
adc_data = "erro"

# Setup Thermometer for Air Temperature using GPIO.BCM mode
GPIO.setup(4, GPIO.OUT)  # Pin 7 on Raspberry Pi (SCL Control)
GPIO.setup(17, GPIO.OUT)  # Pin 11 on Raspberry Pi (SDA Control)
# Enable Air Temperature sensor
GPIO.output(4, GPIO.HIGH)  # Set GPIO 4 to HIGH to enable SCL
GPIO.output(17, GPIO.HIGH)  # Set GPIO 17 to HIGH to enable SDA
sleep(0.5)
# Re-Enable to turn on air temp
airSensor = tsys01.TSYS01()  # Make new object for air thermometer
airSensor.init()  # Initialize air thermometer
sleep(0.5)
# Disable Air Temperature Sensor communication
GPIO.output(4, GPIO.LOW)  # LOW to disable SCL
GPIO.output(17, GPIO.LOW)  # LOW to disable SDA

# Setup Thermometer for Water Temperature using GPIO.BCM mode
GPIO.setup(27, GPIO.OUT)  # Pin 13 on Raspberry Pi (SCL Control)
GPIO.setup(22, GPIO.OUT)  # Pin 15 on Raspberry Pi (SDA Control)
# Enable Water Temperature sensor
GPIO.output(27, GPIO.HIGH)  # Set GPIO 27 to HIGH to enable SCL
GPIO.output(22, GPIO.HIGH)  # Set GPIO 22 to HIGH to enable SDA
sleep(0.5)
waterSensor = tsys01.TSYS01()  # Make new object for water thermometer
waterSensor.init()  # Initialize water thermometer
sleep(0.5)
# Disable Water Temperature Sensor communication
GPIO.output(27, GPIO.LOW)
GPIO.output(22, GPIO.LOW)

# Use Pi I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Create an IMU I2C connection
sensor_IMU = LSM6DS33(i2c)

# Create an ADC I2C connection
#sensor_ADC = ADS.ADS1015(i2c) #  Uncomment if using the ADS1015 ADC
#sensor_ADC = ADS.ADS1115(i2c) #  Uncomment if using the ADS1115 ADC

# Create channel for ADC. here ADS.P0 is A0 on the ADS
#adc_chan = AnalogIn(ads, ADS.P0) #  Uncomment if adding either ADC

# Sets file up for logging Sensors (not IMU)
# -- Latitude, Longitude -> degrees
# -- Temperatures -> Degrees Centigrade
# -- Time -> 24hr clock
# -- Date -> MM/DD/YYYY
data_File = open("/media/GPS_Temp.csv", "a")
if os.stat("/media/GPS_Temp.csv").st_size == 0:
    data_File.write("Date,Time,Latitude,Longitude,Water Temp(C),Air Temp(C)\n")
time.sleep(5)
data_File.close()

# Setup of IMU file
# -- Accel = Acceleration -> m/s^2
# -- Gyro -> degree/s
imu_File = open("/media/imu_Data.csv", "a")
if os.stat("/media/imu_Data.csv").st_size == 0:
    imu_File.write("Date,Time,Accel X,Accel Y,Accel Z,Gyro X,Gyro Y,Gyro Z\n")
time.sleep(5)
imu_File.close()

# Turn on the basic GGA and RMC info (what you typically want)
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')

# Changing 2000ms will adjust sampling rate 1/2000ms = 2Hz
# Adjust timeout in line 6 if rate changed
gps.send_command(b'PMTK220,2000')

# Main loop runs forever printing the location, etc. every second.
last_log = time.monotonic()
imu_log = time.monotonic()
h2o_time = time.monotonic()
air_time = time.monotonic()
#adc_time = time.monotonic()  # Uncomment if you want to add seperate timer for ADC

while infLoop:

    # Checks for new GPS data
    gps.update()

    # Counters of time passed.
    current = time.monotonic()
    imu_check = time.monotonic()
    tempa_check = time.monotonic()
    tempw_check = time.monotonic()
    timeOday = datetime.now().time()

    # Checks if it is time to take Water Temp
    if tempw_check - h2o_time >= temp_Rate_h2o:
        h2o_time = tempw_check
        GPIO.output(4, GPIO.HIGH)
        GPIO.output(17, GPIO.HIGH)
        sleep(0.5)
        if not waterSensor.read():
            waterRecord = "Error"
        else:
            waterRecord = "%.2f" % waterSensor.temperature(tsys01.UNITS_Centigrade)
            sleep(0.2)
        #waterRecord = "Error" #  Use while Temp not setup
        GPIO.output(4, GPIO.LOW)
        GPIO.output(17, GPIO.LOW)

    # Checks if it is time to take Air Temp
    if tempa_check - air_time >= temp_Rate_air:
        air_time = tempa_check
        GPIO.output(27, GPIO.HIGH)
        GPIO.output(22, GPIO.HIGH)
        sleep(0.5)
        if not airSensor.read():
            airRecord = "Error"
        else:
            airRecord = "%.2f" % airSensor.temperature(tsys01.UNITS_Centigrade)
            sleep(0.2)
        #airRecord = "Error" #  Use while Temp not setup
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.LOW)

    # Checks if it is time to gather GPS data/Log it, Send Sat Comm, Check battery level
    if current - last_log >= gps_Rate:
        last_log = current
        # GPS Data is gathered
        if not gps.has_fix:
            lat = "No Signal"
            long = lat
            # timeOday = datetime.now().time()
            theDate = datetime.now().date()
            satData = theDate + str(lat)
            satellite_transmit(satData)
        else:
            # GPS Signal detected
            timeOday = '{:02}:{:02}:{:02}'.format(
                gps.timestamp_utc.tm_hour,
                gps.timestamp_utc.tm_min,
                gps.timestamp_utc.tm_sec)
            theDate = '{}/{}/{}'.format(
                gps.timestamp_utc.tm_mon,
                gps.timestamp_utc.tm_mday,
                gps.timestamp_utc.tm_year)
            satData = '{}{}{},{:02}{:02}{:02},'.format(
                gps.timestamp_utc.tm_mon,
                gps.timestamp_utc.tm_mday,
                gps.timestamp_utc.tm_year,
                gps.timestamp_utc.tm_hour,
                gps.timestamp_utc.tm_min,
                gps.timestamp_utc.tm_sec)
            lat = '{0:.5f}'.format(gps.latitude) # .5 gives 3ft accuracy, .6 gives 3inch accuracy
            long = '{0:.5f}'.format(gps.longitude)

            # Check the Battery with ADC
            #adc_data = adc_chan.value  # Uncomment to allow for ADC transmission

            # Transmits Data if GPS signal present
            satData = satData + str(lat) + ',' + str(long) + ',' + adc_data

            #print(satData, "Data to be transmitted!") #  Uncomment for debugging Sat Comm
            satellite_transmit(satData)
        # Log Date, Time, GPS data, and Temperatures
        data_File = open("/media/GPS_Temp.csv", "a")
        data_File.write(str(theDate)+","+str(timeOday)+","+lat+","+long+"," + waterRecord + "," + airRecord + "\n")
        data_File.flush()
        time.sleep(5)
        data_File.close()

    # Checks if it is time to gather IMU data/Log it
    if imu_check - imu_log >= imu_Rate_start:
        imu_working = True
        #imu_data_set = " "
        while imu_working:  # Continue reading IMU data until
            imu_check = time.monotonic()
            if imu_check - imu_log >= imu_Rate_end:
                imu_log = imu_check
                imu_working = False
            # IMU Data is gathered during the last 10 minutes of an hour
            accel = "%.2f, %.2f, %.2f" % sensor_IMU.acceleration
            gyro = "%.2f, %.2f, %.2f" % sensor_IMU.gyro
            imu_File = open("/media/imu_Data.csv", "a")
            imu_File.write(str(theDate)+","+str(timeOday)+","+accel+","+gyro+"\n")
            imu_File.flush()
            time.sleep(5)  # Used to ensure that file is finished writing (Eliminate to increase sampling rate)
            imu_File.close()
            ###########################################################################################################
            ################################### Alternative IMU Logging ###############################################
        ################################# NOT ## TESTED ###############################################################
    ###################################################################################################################
            # Uncomment code on line 364
            # Replace the code between lines 371 - 377 with code below: (Should decrease logging time)
            #accel = "%.2f, %.2f, %.2f" % sensor_IMU.acceleration
            #gyro = "%.2f, %.2f, %.2f" % sensor_IMU.gyro
            #imu_string = str(theDate) + "," + str(timeOday) + "," + accel + "," + gyro + "\n"
            #imu_data_set = imu_data_set + imu_string
        # Place these lines outside of the while loop, but inside the if-statement of line 362
        #imu_File = open("/media/imu_Data.csv", "a")
        #imu_File.write(imu_data_set)
        #imu_File.flush()
        #time.sleep(10)  # Used to ensure that file is finished writing
        #imu_File.close()
