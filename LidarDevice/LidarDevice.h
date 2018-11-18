#pragma once

#include <string>
#include "cinder/Vector.h"

struct LidarDevice
{
    void info_(const std::string &err);

    std::string status;

    virtual bool setup(const std::string &serialPort) = 0;

    virtual ~LidarDevice() {};
    
    virtual bool isValid() = 0;

    virtual void update() = 0;

    ci::vec2 scanData[360 * 2];
    size_t scanCount = _countof(scanData);
};
