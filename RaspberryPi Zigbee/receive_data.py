import serial

# Open serial connection to XBee connected via USB
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

print("Waiting for data...")

while True:
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()
        print(f"Received: {data}")
