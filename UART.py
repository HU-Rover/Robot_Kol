import serial
import time
import threading

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200


def receive_thread(ser):
    while True:
        if ser.in_waiting > 0:
            raw = ser.readline()  # read one line (bytes)
                
            text = raw.decode('utf-8', errors='ignore').strip()

            print(f"Received {text}")




try:
    joystick = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=0.1
    )
    

    number = -0.4

    while True:
        if joystick.in_waiting > 0:
            line = joystick.readline().decode('utf-8').strip()
            try:
                numbers = list(map(int, line.split()))
                if len(numbers) < 6:
                    print(f'Incomplete data: {line}')
                # ----- SEND -----
                shoulder = (numbers[0] - 1500)/1000
                if(abs(shoulder) < 0.01):
                    shoulder = 0.0
                    
                elbow = (numbers[1] - 1500)/1000 
                if(abs(elbow) < 0.01):
                    elbow = 0.0
                    
                data = f"{abs(shoulder)} {int(shoulder > 0)} {abs(elbow)} {int(elbow > 0)}\n"
                ser.write(data.encode('utf-8'))
                print(f"Sent: {data.strip()}")

                time.sleep(0.1)
        
        
            except ValueError:
                print(f'Invalid data format: {line}')

except serial.SerialException as e:
    print(f"Error opening serial port: {e}")

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed")
