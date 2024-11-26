import serial
import csv
from datetime import datetime

# Open serial connection to XBee connected via USB
ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)

# File to store data
output_file = "sensor_data.csv"

# Create or open the CSV file and write the header
with open(output_file, mode='a', newline='') as file:
    writer = csv.writer(file)
    # Write header if file is empty
    if file.tell() == 0:
        writer.writerow(["Timestamp", "Data"])

print("Waiting for data...")

while True:
    try:
        if ser.in_waiting > 0:
            # Read data from serial port
            data = ser.readline().decode('utf-8').strip()
            print(f"Received: {data}")
            
            # Append data to the CSV file with a timestamp
            with open(output_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                writer.writerow([timestamp, data])
                print("Data saved to CSV.")
    except KeyboardInterrupt:
        print("Exiting...")
        break
    except Exception as e:
        print(f"Error: {e}")