# read distance
import struct
import time

DEV_PATH = "/dev/hcsr04_dev"

def read_distance_cm() -> int:
    with open(DEV, "rb", buffering=0) as f:
        data = f.read(4)
        return struct.unpack("I", data)[0]

if __name__ == "__main__":
    try:
        while True:
            d = read_distance_cm()
            print(f"Distance: {d} cm")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nExiting")
