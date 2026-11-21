import argparse
import signal
import sys
from math import degrees, isfinite
from pathlib import Path
from time import monotonic, sleep


def main():
    parser = argparse.ArgumentParser(description="Test the balancing policy on the Pi.")
    parser.add_argument("counts_per_revolution", type=int)
    parser.add_argument("--encoder-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--port", help="mCore serial port. Omit to log predictions only.")
    parser.add_argument("--max-command", type=int, choices=range(1, 256), metavar="1-255")
    parser.add_argument("--motor-sign", type=int, choices=(-1, 1), default=1)
    args = parser.parse_args()
    if args.counts_per_revolution <= 0:
        parser.error("counts_per_revolution must be positive")
    if bool(args.port) != bool(args.max_command):
        parser.error("provide --port and --max-command together to enable motors")

    from stable_baselines3 import PPO

    from .components import RobotSensors

    checkpoint = Path(__file__).resolve().parents[2] / "balancing_bot.zip"
    model = PPO.load(checkpoint, device="cpu")
    if model.observation_space.shape != (3,):
        parser.error("the checkpoint must take pitch, pitch rate, and left wheel speed")

    print("Keep the robot still while the gyro calibrates.", flush=True)
    sensors = RobotSensors(args.counts_per_revolution, encoder_sign=args.encoder_sign)
    bot = None
    try:
        if args.port:
            from .lib.mbot import mBot

            print("Hold the chassis upright with both wheels clear of the ground.", flush=True)
            bot = mBot()
            # Let Ctrl+C reach the cleanup below before the serial connection closes.
            signal.signal(signal.SIGINT, signal.default_int_handler)
            excepthook = sys.excepthook
            try:
                bot.startWithSerial(args.port)
            finally:
                sys.excepthook = excepthook
            bot.doMove(0, 0)

        deadline = monotonic() + (3 if bot else 15)
        next_log = 0.0
        while monotonic() < deadline:
            started = monotonic()
            observation = sensors.observation()
            pitch, pitch_rate, wheel_speed = map(float, observation)
            if not all(map(isfinite, observation)):
                raise RuntimeError("Invalid sensor reading")
            if bot and abs(degrees(pitch)) > 20:
                print("Tilt limit reached.")
                break

            action, _ = model.predict(observation, deterministic=True)
            command = float(action[0])
            if not isfinite(command) or abs(command) > 1:
                raise RuntimeError("Invalid model action")
            if bot:
                motor_command = round(command * args.max_command) * args.motor_sign
                bot.doMove(motor_command, motor_command)
            if started >= next_log:
                print(f"Pitch {degrees(pitch):+.1f} deg | Gyro {pitch_rate:+.2f} rad/s | "
                      f"Left wheel {wheel_speed:+.2f} rad/s | Action {command:+.2f}", flush=True)
                next_log = started + 0.25
            sleep(max(0, 1 / 60 - (monotonic() - started)))
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        try:
            if bot:
                try:
                    if bot.device and bot.device.isOpen():
                        bot.doMove(0, 0)
                finally:
                    bot.close()
        finally:
            sensors.close()


if __name__ == "__main__":
    main()
