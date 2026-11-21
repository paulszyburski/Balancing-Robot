from math import atan2, radians, tau
from time import monotonic, sleep

import numpy as np
from gpiozero import RotaryEncoder
from smbus import SMBus


class RobotSensors:

    def __init__(self, counts_per_revolution, *, encoder_sign=1):
        if counts_per_revolution <= 0:
            raise ValueError("counts_per_revolution must be positive")
        if encoder_sign not in (-1, 1):
            raise ValueError("encoder_sign must be 1 or -1")
        self.radians_per_count = encoder_sign * tau / counts_per_revolution
        self.bus = SMBus(1)

        try:
            self.bus.write_byte_data(0x68, 0x6B, 0)
            sleep(0.1)
            self.bus.write_byte_data(0x68, 0x1B, 0)
            gyro_samples = []
            for _ in range(100):
                self.pitch, gyro_rate = self._read_imu()
                gyro_samples.append(gyro_rate)
                sleep(0.005)
            self.gyro_bias = sum(gyro_samples) / len(gyro_samples)
            self.encoder = RotaryEncoder(a=17, b=27, max_steps=0)
        except BaseException:
            self.bus.close()
            raise

        self.last_count = self.encoder.steps
        self.last_time = monotonic()

    def _read_imu(self):
        data = bytes(self.bus.read_i2c_block_data(0x68, 0x3B, 14))
        accel_x = int.from_bytes(data[0:2], "big", signed=True)
        accel_z = int.from_bytes(data[4:6], "big", signed=True)
        gyro_y = int.from_bytes(data[10:12], "big", signed=True)
        return atan2(-accel_x, accel_z), radians(gyro_y / 131)

    def observation(self):
        accel_pitch, gyro_rate = self._read_imu()
        count = self.encoder.steps
        now = monotonic()
        dt = now - self.last_time
        pitch_rate = gyro_rate - self.gyro_bias

        gyro_weight = 0.5 / (0.5 + dt)
        self.pitch = gyro_weight * (self.pitch + pitch_rate * dt)
        self.pitch += (1 - gyro_weight) * accel_pitch
        wheel_speed = (count - self.last_count) * self.radians_per_count / dt

        self.last_count = count
        self.last_time = now
        return np.array([self.pitch, pitch_rate, wheel_speed], dtype=np.float32)

    def close(self):
        try:
            self.encoder.close()
        finally:
            self.bus.close()
