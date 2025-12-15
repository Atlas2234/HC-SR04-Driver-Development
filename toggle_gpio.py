import time

DEV_PATH = "/dev/gpio_dev"

def set_gpio_out(value: int) -> None:
    with open(DEV_PATH, "wb", buffering=0) as dev:
        dev.write(b"1" if value else b"0")

if __name__ == "__main__":
    try:
        while True:
            print("HIGH")
            set_gpio_out(1)
            time.sleep(1)

            print("LOW")
            set_gpio_out(0)
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting, driving LOW")
        set_gpio_out(0)
