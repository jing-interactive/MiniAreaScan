#pragma once

#include "LidarDevice.h"

struct YdLidarDevice : public LidarDevice
{
    virtual bool setup(const std::string &serialPort);
    virtual ~YdLidarDevice();
    virtual bool isValid();
    virtual void update();
    bool running = false;
};
