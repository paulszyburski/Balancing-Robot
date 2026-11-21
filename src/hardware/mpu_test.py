
import smbus
import time

ADDRESS = 0x68
bus = smbus.SMBus(1)

# Wake up MPU6050
bus.write_byte_data(ADDRESS, 0x6B, 0)

def signed16(high, low):
    value = (high << 8) | low
    return value - 65536 if value >= 32768 else value

try:
    while True:
        data = bus.read_i2c_block_data(ADDRESS, 0x3B, 14)

        ax = signed16(data[0], data[1]) / 16384
        ay = signed16(data[2], data[3]) / 16384
        az = signed16(data[4], data[5]) / 16384

        gx = signed16(data[8], data[9]) / 131
        gy = signed16(data[10], data[11]) / 131
        gz = signed16(data[12], data[13]) / 131

        print(
            f"Accel: X={ax:.2f} Y={ay:.2f} Z={az:.2f} g | "
            f"Gyro: X={gx:.2f} Y={gy:.2f} Z={gz:.2f} deg/s"
        )

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped")
finally:
    bus.close()
