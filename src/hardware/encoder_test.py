from time import monotonic, sleep

from gpiozero import RotaryEncoder


def main():
    try:
        with RotaryEncoder(a=17, b=27, max_steps=0) as encoder:
            print("Turn the left wheel by hand. Press Ctrl+C to stop.", flush=True)
            previous_steps = encoder.steps
            previous_time = monotonic()

            while True:
                sleep(0.25)
                steps = encoder.steps
                now = monotonic()
                delta = steps - previous_steps
                rate = delta / (now - previous_time)
                print(f"Count={steps:+d}  Change={delta:+d}  Rate={rate:+.1f} counts/s", flush=True)
                previous_steps = steps
                previous_time = now
    except KeyboardInterrupt:
        print("\nStopped")


if __name__ == "__main__":
    main()
