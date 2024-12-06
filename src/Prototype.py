from flask import Flask, jsonify
from flask_cors import CORS
import smbus
import time
import serial
import RPi.GPIO as GPIO
import os  # For shutting down the system

# Initialize Flask App
app = Flask(__name__)
CORS(app)

# Initialize I2C Bus
bus = smbus.SMBus(1)

# I2C Addresses
MPU9250_ADDR = 0x68
BMP280_ADDR = 0x76
INS219_ADDR = 0x40
SHT31_ADDR = 0x44

# RS232 Serial Port
serial_port = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=1)

# GPIO Pin for Emergency Shutdown Sensor
EMERGENCY_SHUTDOWN_PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(EMERGENCY_SHUTDOWN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Functions to Read Sensor Data
def read_mpu():
    try:
        def read_raw_data(addr):
            high = bus.read_byte_data(MPU9250_ADDR, addr)
            low = bus.read_byte_data(MPU9250_ADDR, addr + 1)
            value = (high << 8) | low
            if value > 32768:
                value -= 65536
            return value

        gyro_x = read_raw_data(0x43)
        gyro_y = read_raw_data(0x45)
        gyro_z = read_raw_data(0x47)
        return {"gyro_x": gyro_x, "gyro_y": gyro_y, "gyro_z": gyro_z}
    except Exception as e:
        return {"error": f"Failed to read MPU9250: {e}"}

def read_bmp():
    try:
        data = bus.read_i2c_block_data(BMP280_ADDR, 0xF7, 8)
        adc_P = ((data[0] << 16) | (data[1] << 8) | data[2]) >> 4
        adc_T = ((data[3] << 16) | (data[4] << 8) | data[5]) >> 4
        temperature = adc_T / 100.0
        pressure = adc_P / 100.0
        return {"pressure": pressure, "temperature": temperature}
    except Exception as e:
        return {"error": f"Failed to read BMP280: {e}"}

def read_ins219():
    return {"current": 0.0, "voltage": 0.0}

def read_sht31():
    try:
        data = bus.read_i2c_block_data(SHT31_ADDR, 0x2C, 6)
        temp = -45 + (175 * ((data[0] << 8) | data[1]) / 65535.0)
        humidity = 100 * ((data[3] << 8) | data[4]) / 65535.0
        return {"temperature": temp, "humidity": humidity}
    except Exception as e:
        return {"error": f"Failed to read SHT31: {e}"}

# Emergency Shutdown Check
def check_emergency_shutdown():
    if GPIO.input(EMERGENCY_SHUTDOWN_PIN) == GPIO.LOW:
        print("Emergency shutdown triggered!")
        os.system('sudo shutdown -h now')

def communicate_via_rs232(data):
    try:
        serial_port.write(data.encode())
        response = serial_port.readline().decode().strip()
        return response
    except serial.SerialException as e:
        return {"error": f"Serial communication failed: {e}"}

# API Route
@app.route('/data', methods=['GET'])
def get_data():
    mpu_data = read_mpu()
    bmp_data = read_bmp()
    ins219_data = read_ins219()
    sht31_data = read_sht31()
    return jsonify({"mpu": mpu_data, "bmp": bmp_data, "ins219": ins219_data, "sht31": sht31_data})

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        GPIO.cleanup()
