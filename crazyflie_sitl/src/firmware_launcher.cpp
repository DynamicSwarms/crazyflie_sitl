#include "crazyflie_sitl/firmware_launcher.hpp"

#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>

#include <vector>


FirmwareLauncher::FirmwareLauncher()
{
    m_firmware_pid = fork();

    if (m_firmware_pid == 0) 
    {
        setpgid(0,0); //Create new process group so that we can kill all child processes later       
        
        std::vector<char*> argv;

        argv.push_back(const_cast<char*>("ros2"));
        argv.push_back(const_cast<char*>("run"));
        argv.push_back(const_cast<char*>("crazyflie_sitl"));
        argv.push_back(const_cast<char*>("cf2"));

        argv.push_back(const_cast<char*>("19950"));
        // add arguments for cf2 here, e.g.:
        // argv.push_back(const_cast<char*>("--some-arg"));
        // argv.push_back(const_cast<char*>("value"));

        argv.push_back(nullptr);

        execvp("ros2", argv.data());
        _exit(1);
    }

}

FirmwareLauncher::~FirmwareLauncher()
{
    if (m_firmware_pid > 0)
    {
        
        kill(-m_firmware_pid, SIGINT); //Kill all processes in the process group
        waitpid(m_firmware_pid, nullptr, 0); //Wait for the firmware process to exit
        m_firmware_pid = -1;
    }
}
    
