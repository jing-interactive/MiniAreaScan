#pragma once

#include "LidarDevice.h"

struct RpLidarDevice : public LidarDevice
{
    virtual bool setup(const std::string &serialPort);
    virtual ~RpLidarDevice();
    virtual bool isValid();
    virtual void update();

    bool checkRPLIDARHealth();
};
