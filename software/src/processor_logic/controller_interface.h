#pragma once

class ControllerInterface
{
public:
    ControllerInterface() { }
    virtual ~ControllerInterface() { }

    virtual void handle_wakeup() = 0;
};
