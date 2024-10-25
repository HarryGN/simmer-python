import serial
import time

# Configure the serial port (replace 'COM8' with your Arduino's serial port)
ser = serial.Serial('COM8', 9600, timeout=1)  # 9600 baud rate

# Function to send a command to the Arduino
def send_command(command):
    ser.write((command + '\n').encode('utf-8'))  # Send the command with a newline character
    print(f"Sent: {command}")
    time.sleep(0.5)  # Wait for Arduino to process and respond

    # Read any response from the Arduino
    while ser.in_waiting > 0:
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"Arduino responded: {response}")

try:
    while True:
        # Get user input to send motor command
        command = input("Enter motor command (e.g., [w0:10] or [w0:-10]) or 'q' to quit: ").strip()
        if command.lower() == 'q':
            break
        send_command(command)

except KeyboardInterrupt:
    print("Program interrupted.")
finally:
    ser.close()  # Close the serial connection
    print("Serial connection closed.")
