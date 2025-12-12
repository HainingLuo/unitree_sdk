import time
import sys
import select
import termios
import tty
from dataclasses import dataclass

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)

# -----------------------------------------
# Test Options
# -----------------------------------------

@dataclass
class TestOption:
    name: str
    id: int


option_list = [
    TestOption(name="damp", id=0),
    TestOption(name="stand_up", id=1),
    TestOption(name="stand_down", id=2),
    TestOption(name="move forward", id=21),
    TestOption(name="move backward", id=22),
    TestOption(name="move left", id=23),
    TestOption(name="move right", id=24),
    TestOption(name="move up", id=25),
    TestOption(name="move down", id=26),
    TestOption(name="rotate left", id=27),
    TestOption(name="rotate left", id=28),
    TestOption(name="stop_move", id=6),
    TestOption(name="hand stand", id=7),
    TestOption(name="balanced stand", id=9),
    TestOption(name="recovery", id=10),
    TestOption(name="left flip", id=11),
    TestOption(name="back flip", id=12),
    TestOption(name="free walk", id=13),
    TestOption(name="free bound", id=14),
    TestOption(name="free avoid", id=15),
    TestOption(name="walk upright", id=17),
    TestOption(name="cross step", id=18),
    TestOption(name="free jump", id=19),
]

# -----------------------------------------
# Keyboard → Option ID Mapping
# -----------------------------------------

key_map = {
    "q": 0, # damp
    "t": 1, # stand up
    "y": 2, # stand down
    "w": 21, # move forward
    "s": 22, # move backward
    "a": 23, # move left
    "d": 24, # move right
    "i": 25, # move up
    "k": 26, # move down
    "j": 27, # rotate left
    "l": 28, # rotate right
    " ": 6, # stop move
    "1": 7,
    # "b": 9,
    # "c": 10,
    # "l": 11,
    # "k": 12,
    # "0": 13,
    # "1": 14,
    # "3": 15,
    # "u": 17,
    # "x": 18,
    # "j": 19,
}

# -----------------------------------------
# Non-blocking keyboard input (Linux)
# -----------------------------------------

def get_key_nonblocking():
    """Returns one character if pressed, else None."""
    dr, dw, de = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

# -----------------------------------------
# Main
# -----------------------------------------

if __name__ == "__main__":

    print("WARNING: Ensure no obstacles around the robot.")
    input("Press Enter to continue...")

    # Initialise SDK
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    test_option = TestOption(name=None, id=None)

    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    # Set terminal to raw mode for single-key capture
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    print("\nKeyboard control enabled.")
    print("Press mapped keys to trigger actions.")
    print("Press 'z' to quit.\n")

    curr_height = 0.0
    translation_speed = 0.01  # meters per command
    rotation_speed = 0.5      # radians per command
    
    time_step = 0.03  # seconds
    translation_step = translation_speed * time_step
    rotation_step = rotation_speed * time_step  

    try:
        while True:
            key = get_key_nonblocking()

            if key:
                key = key.lower()

                if key == "z":
                    print("Exiting…")
                    break
                
                # Map key to test option ID
                if key in key_map:
                    test_option.id = key_map[key]
                    print(f"[KEY '{key}'] -> test id {test_option.id}")
                else:
                    continue

                # Execute command based on option ID
                if test_option.id == 0:
                    sport_client.Damp()

                elif test_option.id == 1:
                    sport_client.StandUp()

                elif test_option.id == 2:
                    sport_client.StandDown()

                elif test_option.id == 21:
                    res = sport_client.Move(0.1, 0, 0) # X forwards
                elif test_option.id == 22:
                    sport_client.Move(-0.1, 0, 0)
                elif test_option.id == 23:
                    sport_client.Move(0, 0.1, 0) # Y leftwards
                elif test_option.id == 24:
                    sport_client.Move(0, -0.1, 0)
                # elif test_option.id == 25:
                #     print("Move up")
                #     curr_height += 0.01
                #     sport_client.BodyHeight(curr_height)
                # elif test_option.id == 26:
                #     print("Move down")
                #     curr_height -= 0.01
                #     sport_client.BodyHeight(curr_height)
                elif test_option.id == 27:
                    sport_client.Move(0, 0, 0.5) # Z upwards
                elif test_option.id == 28:
                    sport_client.Move(0, 0, -0.5)

                elif test_option.id == 6:
                    sport_client.StopMove()

                elif test_option.id == 7:
                    sport_client.HandStand(True)
                    time.sleep(4)
                    sport_client.HandStand(False)

                elif test_option.id == 9:
                    sport_client.BalanceStand()

                elif test_option.id == 10:
                    sport_client.RecoveryStand()

                elif test_option.id == 11:
                    print(sport_client.LeftFlip())

                elif test_option.id == 12:
                    print(sport_client.BackFlip())

                elif test_option.id == 13:
                    print(sport_client.FreeWalk())

                elif test_option.id == 14:
                    print(sport_client.FreeBound(True))
                    time.sleep(2)
                    print(sport_client.FreeBound(False))

                elif test_option.id == 15:
                    print(sport_client.FreeAvoid(True))
                    time.sleep(2)
                    print(sport_client.FreeAvoid(False))

                elif test_option.id == 17:
                    print(sport_client.WalkUpright(True))
                    time.sleep(4)
                    print(sport_client.WalkUpright(False))

                elif test_option.id == 18:
                    print(sport_client.CrossStep(True))
                    time.sleep(4)
                    print(sport_client.CrossStep(False))

                elif test_option.id == 19:
                    print(sport_client.FreeJump(True))
                    time.sleep(4)
                    print(sport_client.FreeJump(False))

            time.sleep(time_step)

    finally:
        # Restore terminal state
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("Terminal restored.")
