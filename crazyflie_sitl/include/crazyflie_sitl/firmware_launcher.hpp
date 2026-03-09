#pragma once
#include <thread>

class FirmwareLauncher {

public:
    FirmwareLauncher();

    ~FirmwareLauncher();
    
private: 
    pid_t m_firmware_pid;     

};

