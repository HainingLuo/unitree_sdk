import sys
import termios
import tty
import select
import time

def get_key_nonblocking():
    """Return a single key if pressed, otherwise None."""
    dr, dw, de = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

if __name__ == "__main__":
    print("Non-blocking keyboard test started.")
    print("Press keys to see them printed. Press 'q' to quit.\n")

    # Save terminal settings
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        while True:
            key = get_key_nonblocking()
            if key:
                print(f"KEY PRESSED: '{key}'")

                if key.lower() == 'q':
                    print("Exiting.")
                    break

            time.sleep(0.03)

    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("Terminal restored.")
