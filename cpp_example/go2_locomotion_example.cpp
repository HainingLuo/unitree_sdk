/**********************************************************************
 Keyboard Teleoperation for Unitree GO2 by Haining
***********************************************************************/

#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <map>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

using namespace unitree::robot::go2;

// ================================================================
//               Non-blocking keyboard utilities
// ================================================================

void setNonblocking(bool enable)
{
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);

    if (enable)
    {
        ttystate.c_lflag &= ~ICANON;   // disable canonical mode
        ttystate.c_lflag &= ~ECHO;     // disable echo
        ttystate.c_cc[VMIN] = 0;
        ttystate.c_cc[VTIME] = 0;
    }
    else
    {
        ttystate.c_lflag |= ICANON;
        ttystate.c_lflag |= ECHO;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (enable)
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    else
        fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
}

char getKey()
{
    char c = 0;
    if(read(STDIN_FILENO, &c, 1) == 1)
        return c;
    return 0;
}

// ================================================================
//                      Keyboard → Option Map
// ================================================================

std::map<char, int> keyMap = {
    {'q', 0},   // damp
    {'t', 1},   // stand up
    {'y', 2},   // stand down
    {'w', 21},  // move forward
    {'s', 22},  // move backward
    {'a', 23},  // move left
    {'d', 24},  // move right
    {'i', 25},  // move up (disabled)
    {'k', 26},  // move down (disabled)
    {'j', 27},  // rotate left
    {'l', 28},  // rotate right
    {' ', 6},   // stop move
    {'1', 7},   // handstand
};

// ================================================================
//         Standalone keyboard-teleop main program
// ================================================================

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface\n";
        return -1;
    }

    std::cout << "WARNING: Ensure no obstacles around robot.\n";
    std::cout << "Press ENTER to continue.\n";
    std::cin.get();

    // Init SDK
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

    SportClient sport;
    sport.SetTimeout(10.0f);
    sport.Init();

    std::cout << "\nKeyboard control enabled.\n";
    std::cout << "Press 'z' to quit.\n";

    // Enable non-blocking keyboard reading
    setNonblocking(true);

    // float curr_height = 0.0f;

    const float dt = 0.03f;       // time step
    const float v = 0.1f;         // translation speed
    const float w = 0.5f;         // yaw speed

    while(true)
    {
        char key = getKey();
        if(key != 0)
        {
            key = std::tolower(key);

            if(key == 'z')
            {
                std::cout << "Exiting...\n";
                break;
            }

            if(keyMap.count(key))
            {
                int id = keyMap[key];
                std::cout << "[Key '" << key << "'] → command id " << id << "\n";

                // ---------- Command execution ----------
                switch(id)
                {
                case 0: sport.Damp(); break;
                case 1: sport.StandUp(); break;
                case 2: sport.StandDown(); break;

                case 21: sport.Move(v, 0, 0); break;
                case 22: sport.Move(-v, 0, 0); break;
                case 23: sport.Move(0, v, 0); break;
                case 24: sport.Move(0, -v, 0); break;

                // case 25: sport.BodyHeight(curr_height); curr_height += 0.01f; break;
                // case 26: sport.BodyHeight(curr_height); curr_height -= 0.01f; break;

                case 27: sport.Move(0, 0, w); break;
                case 28: sport.Move(0, 0, -w); break;

                case 6:  sport.StopMove(); break;

                case 7:
                    sport.HandStand(true);
                    usleep(4000000);
                    sport.HandStand(false);
                    break;

                default:
                    break;
                }
            }
        }

        usleep((int)(dt * 1e6)); // sleep for dt seconds
    }

    // Restore terminal
    setNonblocking(false);
    std::cout << "Terminal restored.\n";

    return 0;
}
