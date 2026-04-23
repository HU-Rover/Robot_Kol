import serial
import time
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
try:
    joystick = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0)
    while True:
        last_line = None
        while joystick.in_waiting > 0:
            try:
                last_line = joystick.readline().decode('utf-8').strip()
            except UnicodeDecodeError:
                continue
        if last_line is None:
            time.sleep(0.005)
            continue
        try:
            numbers = list(map(int, last_line.split()))
            if len(numbers) < 7:
                print(f'Eksik veri: {last_line}')
                continue
            shoulder = (numbers[1] - 1500) / 1000
            if abs(shoulder) < 0.01:
                shoulder = 0.0
            elbow = (numbers[0] - 1500) / 1000
            
            if abs(elbow) < 0.01:
                elbow = 0.0
            if numbers[2] < 1600 and numbers[2] > 1400:
                gripper_pitch = 0
            else:
                gripper_pitch = numbers[2] - 1500
                
            if numbers[3] < 1600 and numbers[3] > 1400:
                gripper_roll = 0
            else:
                gripper_roll = numbers[3] - 1500
            servo = numbers[6] + 100
            base = int((numbers[7] - 1000)/1000 * 180)
            data = f"{abs(shoulder):.3f} {int(shoulder > 0)} {abs(elbow):.3f} {int(elbow > 0)} {gripper_pitch} {gripper_roll} {servo} {base}\n"
            ser.write(data.encode('utf-8'))
            print(f"Sent: {data.strip()}")
        except ValueError:
            print(f'Geçersiz format: {last_line}')
except serial.SerialException as e:
    print(f"Port hatası: {e}")
except KeyboardInterrupt:
    print("Durduruldu")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
    if 'joystick' in locals() and joystick.is_open:
        joystick.close()
    print("Portlar kapatıldı")
