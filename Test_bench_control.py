import serial
import threading
import time
import tkinter as tk
# import cv2
import numpy as np
from collections import deque
import re
 
import csv
import os
 
# ====== CONFIG ======
SUNSENSOR_SERIAL_PORT = '/dev/ttyACM0'
ARDUINO_SERIAL_PORT = '/dev/ttyACM1'
BAUD_RATE = 115200
 
# ====== SETUP SERIAL ======
arduino_serial = serial.Serial(ARDUINO_SERIAL_PORT, BAUD_RATE, timeout=1)
sunsensor_serial = serial.Serial(SUNSENSOR_SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)
 
current_angle1 = [90]
current_angle2 = [90]
is_moving = False
 
# ====== Shared Data ======
# vectors = deque(maxlen=10)
imu_x_readings = deque(maxlen=10)
imu_y_readings = deque(maxlen=10)
imu_z_readings = deque(maxlen=10)
lock = threading.Lock()
 
# ====== Sun Sensor and Save Data ======
latest_sun_angle_y = 0.0
latest_sun_angle_x = 0.0
csv_filename = "saved_data.csv"
 
 
 
def read_imu_data():
    while True:
            # Read from Arduino IMU port
        if arduino_serial.in_waiting > 0:
            try:
                arduino_line = arduino_serial.readline().decode('utf-8').strip()
                if arduino_line:
                    # print(f"Arduino IMU raw: {arduino_line}")
                    try:
                        # Assuming IMU data is comma-separated floats like "x,y,z"
                        x, y, z = map(float, arduino_line.split(","))


                        # Add new readings to the deques
                        imu_x_readings.append(x)
                        imu_y_readings.append(y)
                        imu_z_readings.append(-z) # Apply inversion here

                        # Calculate the average of the readings in the deques
                        avg_x = sum(imu_x_readings) / len(imu_x_readings)
                        avg_y = sum(imu_y_readings) / len(imu_y_readings)
                        avg_z = sum(imu_z_readings) / len(imu_z_readings)

                        # Update the GUI with the averaged values
                        imu_x.set(f"X: {avg_x:.2f}")
                        imu_y.set(f"Y: {avg_y:.2f}")
                        imu_z.set(f"Z: {avg_z:.2f}")
                        
                        print(f"IMU Data: X={x:.4f}, Y={y:.4f}, Z={-z:.4f} (floats)")
                    except ValueError:
                        print(f"Warning: Could not parse IMU data from '{arduino_line}'")
                print("=" * 30) # Separator for clarity between IMU reads
            except UnicodeDecodeError:
                print("Arduino IMU: UnicodeDecodeError - Skipping malformed line.")
            except serial.SerialException as e:
                print(f"Arduino IMU: Serial communication error: {e}")
                break # Exit loop on critical serial error
 
 
# ====== Camera Reading ======
def parse_sunsensor_data(data_string):
    """
    Parses a string like 'Azimuth: -4.4985, Altitude: -6.9225, Vector: [0.078433, -0.12016, 0.98965]'
    into separate float variables and a list of floats.
    """
    azimuth = None
    altitude = None
    vector = []

    # Regex to find Azimuth and Altitude values
    azimuth_match = re.search(r"Azimuth:\s*([+\-]?\d+\.?\d*)", data_string)
    altitude_match = re.search(r"Altitude:\s*([+\-]?\d+\.?\d*)", data_string)

    if azimuth_match:
        azimuth = float(azimuth_match.group(1))
    if altitude_match:
        altitude = float(altitude_match.group(1))

    # Regex to find the vector array
    vector_match = re.search(r"Vector:\s*\[([^\]]+)\]", data_string)
    if vector_match:
        # Split the string of numbers by comma and convert each to float
        vector_str_list = [s.strip() for s in vector_match.group(1).split(',')]
        try:
            vector = [float(s) for s in vector_str_list]
        except ValueError:
            print(f"Warning: Could not parse all vector components from '{vector_match.group(1)}'")

    return azimuth, altitude, vector

def read_sunsensor_serial():
    global latest_sun_angle_x, latest_sun_angle_y
    while True:
        if sunsensor_serial.in_waiting > 0:
            try:
                line = sunsensor_serial.readline().decode('utf-8').rstrip()
                print(f"Sunsensor raw: {line}")
                azimuth_val, altitude_val, vector_val = parse_sunsensor_data(line)

                if azimuth_val is not None:
                    latest_sun_angle_x = azimuth_val
                    sun_x_str.set(f"Sun X: {azimuth_val:.2f}°")

                if altitude_val is not None:
                    latest_sun_angle_y = altitude_val
                    sun_y_str.set(f"Sun Y: {altitude_val:.2f}°")

                if vector_val:
                    print(f"Sunsensor Vector: {vector_val}")
                print("-" * 30)

            except UnicodeDecodeError:
                print("Sunsensor: UnicodeDecodeError - Skipping malformed line.")
            except serial.SerialException as e:
                print(f"Sunsensor: Serial communication error: {e}")

    
 


 
# ====== Servo Control ======
def smooth_move(servo_index, start_angle, target_angle, label, current_angle_var,text):
    global is_moving
    if is_moving:
        return
    is_moving = True
    step = 1 if target_angle > start_angle else -1
    for angle in range(start_angle, target_angle + step, step):
        if servo_index == 1:
            send_servo_angles(angle, current_angle2[0])
        else:
            send_servo_angles(current_angle1[0], angle)
        label.config(text=f"{text} Angle: {angle}°")
        root.update_idletasks()
        time.sleep(0.05)
    current_angle_var[0] = target_angle
    is_moving = False
 
def send_servo_angles(angle1, angle2):
    arduino_serial.write(bytes([angle1, angle2]))
 
def on_slider_change(val, servo_index, label, current_angle_var,text):
    global is_moving
    if is_moving:
        return
    target_angle = int(val)
    smooth_move(servo_index, current_angle_var[0], target_angle, label, current_angle_var,text)
 
def save_data_to_csv():
    sun_z = latest_sun_angle_y
    sun_y = latest_sun_angle_x
    imu_val_x = imu_x.get().replace("X: ", "")
    imu_val_y = imu_y.get().replace("Y: ", "")
    imu_val_z = imu_z.get().replace("Z: ", "")
 
    data_row = [sun_z, sun_y, imu_val_x, imu_val_y, imu_val_z]
 
    file_exists = os.path.isfile(csv_filename)
 
    with open(csv_filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        if not file_exists:
            writer.writerow(["Sun Angle X (deg)", "Sun Angle Y (deg)", "IMU Y", "IMU X", "IMU Z"])
        writer.writerow(data_row)
 
    print("Saved:", data_row)
 
# ====== GUI ======
root = tk.Tk()
 
imu_x = tk.StringVar(value="X: 0.00")
imu_y = tk.StringVar(value="Y: 0.00")
imu_z = tk.StringVar(value="Z: 0.00")
sun_x_str = tk.StringVar(value="Sun X: 0.00°")
sun_y_str = tk.StringVar(value="Sun Y: 0.00°")
 
root.title("Smooth Servo Controller with IMU and Sun Sensor")
 
label1 = tk.Label(root, text="Servo 1 Angle: 90°", font=("Arial", 14))
label1.pack(pady=10)
 
slider1 = tk.Scale(root, from_=0, to=180, orient=tk.HORIZONTAL, length=300,
                   command=lambda val: on_slider_change(val, 1, label1, current_angle1,"X axis"))
slider1.set(current_angle1[0])
slider1.pack(padx=20, pady=10)
 
label2 = tk.Label(root, text="Servo 2 Angle: 90°", font=("Arial", 14))
label2.pack(pady=10)
 
slider2 = tk.Scale(root, from_=0, to=180, orient=tk.HORIZONTAL, length=300,
                   command=lambda val: on_slider_change(val, 2, label2, current_angle2,"Y axis"))
slider2.set(current_angle2[0])
slider2.pack(padx=20, pady=10)
 
# ====== IMU Display ======
imu_label_title = tk.Label(root, text="IMU Readings", font=("Arial", 16, "bold"))
imu_label_title.pack(pady=10)
 
imu_label_x = tk.Label(root, textvariable=imu_x, font=("Arial", 14))
imu_label_x.pack()
 
imu_label_y = tk.Label(root, textvariable=imu_y, font=("Arial", 14))
imu_label_y.pack()
 
imu_label_z = tk.Label(root, textvariable=imu_z, font=("Arial", 14))
imu_label_z.pack()
 
# ====== Sun Angle Display ======
sun_label_title = tk.Label(root, text="Sun Sensor Angles", font=("Arial", 16, "bold"))
sun_label_title.pack(pady=10)
 
sun_label_z = tk.Label(root, textvariable=sun_x_str, font=("Arial", 14))
sun_label_z.pack()
 
sun_label_y = tk.Label(root, textvariable=sun_y_str, font=("Arial", 14))
sun_label_y.pack()
 
 
# ====== Save Data Button ======
save_button = tk.Button(root, text="Save Data", command=save_data_to_csv, font=("Arial", 14), bg="green", fg="white")
save_button.pack(pady=20)
 
send_servo_angles(current_angle1[0], current_angle2[0])
 
# ====== Start Threads ======
threading.Thread(target=read_imu_data, daemon=True).start()
threading.Thread(target=read_sunsensor_serial, daemon=True).start()
# threading.Thread(target=plot_thread, daemon=True).start()
 
root.mainloop()
 
arduino_serial.close()