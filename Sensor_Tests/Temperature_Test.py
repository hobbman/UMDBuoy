import time
import board
import busio
import tsys01

# Temperture
counter = 0
while True:
    try:
        blueT1 = tsys01.TSYS01()
        blueT1.init()
    except Exception:
        print("Error staring Temp sensor #", counter)
        counter += 1
        continue
    break
while True:
      if not blueT1.read():
          print("Error reading")
      print("Temperature: %.2f C"%blueT1.temperature(tsys01.UNITS_Centigrade))
      print("")
      time.sleep(0.5)
