import time
import board
import serial
import sys
import glob
import signal

class rockBLOCK9603(object):
    testmessage = "90000000180000000030420142630"
    #arbytes = bytearray(b'AT+SBDWT=')
    #arbytes.append(int(testmessage).to_bytes(14, byteorder="big"))
    #arbytes.append(b'\r')
    # Check Signal
    cmd2 = b"AT+CSQ\r"
    # Check connection from Pi to 9603
    cmd = b"AT\r"
    ser = serial.Serial('/dev/ttyUSB0', 19200, 8, 'N', 1, timeout=10)
    ser.write(cmd2)
    response = ser.read(19)
    print(response.decode())

    ser.write(cmd)
    response = ser.read(4)
    print("********************************************")
    if "OK".encode() in response:
        print("OK - Connection to 9603")
    else:
        print("No Connection to RockBLOCK9603")

    # Start to sending process
    cmd = "AT&K0\r"
    ser.write(cmd.encode())
    response = ser.read(11)
    if "OK".encode() in response:
        print("OK - Started sending process")
    else:
        print("Failed \"AT&K0\"")
    # Message to send
    #cmd = b"AT+SBDWT=" + testencoded + b'\r'
    msgstr = "AT+SBDWT=" + testmessage + "\r"
    #ser.write(bytearray(msgstr, "utf-8"))
    #ser.write(b"AT+SBDWT=ConfirmingSuspectedIssue\r")
    ser.write(msgstr.encode())
    response = ser.read(15)
    if "OK".encode() in response:
        print("OK - Able to write Message to Buffer")
    else:
        print("Failed to use \"AT+SBDWT=\"")
    # Check that message was sent successfully
    cmd = "AT+SBDIX\r"
    ser.write(cmd.encode())
    response = ser.read(64)
    if "+SBDIX:".encode() in response:
        print("Probably sent check email.")
        print("**************************************************************")
        print(response.decode())
    else:
        print("There was an issue with the final command!")
        print(response.decode())
    
    ser.close()
