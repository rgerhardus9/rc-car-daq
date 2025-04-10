import serial
import time

# Open UART at default baud rate for WT901 (9600)
ser = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)

# Set I2C address to 0x50
set_i2c_cmd = bytes([0xFF, 0xAA, 0x1A, 0x50, 0x00])
# Save configuration
save_cmd = bytes([0xFF, 0xAA, 0x00, 0x00, 0x00])

print("Sending I2C address set command...")
ser.write(set_i2c_cmd)
time.sleep(0.5)

print("Sending save config command...")
ser.write(save_cmd)
time.sleep(0.5)

print("Done. Power cycle the WT901, then run i2cdetect.")
ser.close()