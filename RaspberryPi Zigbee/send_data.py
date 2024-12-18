import serial
import time

# Open serial connection to XBee connected via USB
ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)

while True:
    message = "Hello From Rpi"
    ser.write(message.encode())  # Send data to XBee
    print(f"Sent: {message}")
    time.sleep(2)  # Send data every 2 seconds
