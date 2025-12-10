#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <cctype>

// ---------------------------------------------------------------
// Enable / disable non-blocking keyboard input
// ---------------------------------------------------------------
void setNonBlocking(bool enable)
{
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);

    if (enable)
    {
        ttystate.c_lflag &= ~ICANON;  // no buffering
        ttystate.c_lflag &= ~ECHO;    // do not echo characters
        ttystate.c_cc[VMIN] = 0;
        ttystate.c_cc[VTIME] = 0;
    }
    else
    {
        ttystate.c_lflag |= ICANON;
        ttystate.c_lflag |= ECHO;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);

    int flags = fcntl(STDIN_FILENO, F_GETFL);
    if (enable)
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    else
        fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
}

// ---------------------------------------------------------------
// Read one key if available, otherwise return 0
// ---------------------------------------------------------------
char getKey()
{
    char c = 0;
    if (read(STDIN_FILENO, &c, 1) == 1)
        return c;
    return 0;
}

// ---------------------------------------------------------------
// Main testing loop
// ---------------------------------------------------------------
int main()
{
    std::cout << "Minimal Keyboard Test\n";
    std::cout << "Press keys to see output.\n";
    std::cout << "Press 'z' to exit.\n\n";

    setNonBlocking(true);

    while (true)
    {
        char c = getKey();
        if (c != 0)
        {
            if (c == 'z')
            {
                std::cout << "Exiting...\n";
                break;
            }

            if (c == ' ')
                std::cout << "[SPACE]\n";
            else if (c == '\n')
                std::cout << "[ENTER]\n";
            else
                std::cout << "Key: " << c << "\n";
        }

        usleep(30000);  // ~30 ms loop
    }

    setNonBlocking(false);
    std::cout << "Terminal restored.\n";

    return 0;
}
