import network
import time
# import ntptime
import ufirebase as firebase
import _thread as th
from machine import Pin, I2C, PWM
from max6675 import MAX6675
from lcd_api import LcdApi
from pico_i2c_lcd import

# ------------------- PIN ASSIGMENT ------------------- #
SDA_PIN = Pin(21)
SCL_PIN = Pin(22)
PIN_SO = Pin(12, Pin.IN)
PIN_SCK = Pin(14, Pin.OUT)
PIN_CS = Pin(15, Pin.OUT)
PIN_MOTOR_A1 = PWM(Pin(24))
PIN_MOTOR_A2 = PWM(Pin(26))
# ----------------------------------------------------- #

# ----------------- NETWORK VARIABLES ----------------- #
WIFI_SSID = 'JILVER' # <-- WIFI NETWORK NAME
WIFI_PASSWORD = 'FLA*_p4r1v3r4' # <-- WIFI NETWORK PASSWORD
FIREBASE_URL = "https://greenpid-default-rtdb.firebaseio.com/" # <-- FIREBASE RTDB URL
# ----------------------------------------------------- #

# --------------- PROJECT CONFIGURATION --------------- #
#LCD I2C CONFIGURATION
I2C_ADDR = 0x27 # <- Direction
I2C_NUM_ROWS = 2 # <- LCD num rows
I2C_NUM_COLS = 16 # <- LCD num columns
i2c = I2C(0, sda=SDA_PIN, scl=SCL_PIN, freq=400000)
lcd = I2cLcd(i2c, I2C_ADDR, I2C_NUM_ROWS, I2C_NUM_COLS)

# MAX6675 CONFIGURATION
max6675 = MAX6675(PIN_SCK, PIN_CS, PIN_SO)

# VARIABLES
setpoint = 220
temperature = 0
pid_error = 0
pid_value = 0
integral = 0
derivative = 0
kp = 1 # <- Proportional value
ki = 0.1 # <- Integral value
kd = 0.01 # <- Derivative value
dt = 1
# ----------------------------------------------------- #

def lcd_str(message, col, row):
    lcd.move_to(col, row)
    lcd.putstr(message)
    
def connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.scan()
    while not wlan.isconnected():
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        print(".",end="")
        lcd_str("Connecting...", 0, 0)
        time.sleep(1)
        lcd.clear()
    if wlan.isconnected():
        lcd.clear()
        lcd_str("Connected", 0, 0)
        print("[Wi-Fi] -> Connected.")
        print("[Wi-Fi] -> Network:", wlan.ifconfig())
        lcd.clear()
        return wlan
wifi = connect()
while not wifi.isconnected():
  pass
firebase.setURL(FIREBASE_URL)
print("[Firebase] -> Connected.")

def read_temperature(e):
    while True:
        global temperature
        print("[Sensors] -> Reading temperature...")
        temperature = round(max6675.read(), 1)
        lcd_show_data()
        time.sleep(e)
th.start_new_thread(read_temperature, (dt,))

def pid_calculus(e):
    while True:
        global pid_error, pid_value, setpoint, temperature, integral, derivative, kp, ki, kd
        print("[Control] -> Making PID...")
        pid_error = setpoint - temperature
        integral += pid_error * dt
        derivative = (pid_error - derivative) / dt
        pid_value = kp * error + ki * integral + kd * derivative
        temperature += pid_value
        time.sleep(e)
th.start_new_thread(pid_calculus, (dt,))

def lcd_show_data():
    global temperature
    lcd.clear()
    lcd_str("Temp: {}".format(str(temperature)), 0, 0)
    lcd_str("PID: {}".format(str(pid_value)), 0, 0)

def main():
    while True:
        # ntptime.settime() # <-- Time synchronization
        # ntptime.host = "2.co.pool.ntp.org"
        global temperature, pid_value
        localTime = time.localtime()
        ISOTime = time.mktime(localTime)
        FIREBASE_PATH = "tostadora/" + str(ISOTime)
        print("[Firebase] -> Uploading data {}...".format(str(ISOTime)))
        firebase.put(FIREBASE_PATH, {
                                        "temperature": temperature,
                                        "pid": pid_value,
                                        "ts": ISOTime
                                    }, bg=0)
        print("[Firebase] -> Uploaded data.")
        time.sleep(2)
        
if __name__ == '__main__':
    main()