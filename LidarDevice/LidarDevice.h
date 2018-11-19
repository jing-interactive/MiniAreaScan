#pragma once

#include <string>
#include <vector>
#include "cinder/Vector.h"

struct LidarDevice
{
    void info_(const std::string &err);

    std::string status;

    virtual bool setup(const std::string &serialPort) = 0;

    virtual ~LidarDevice() {};
    
    virtual bool isValid() = 0;

    virtual void update() = 0;

    std::vector<ci::vec3> scanData;
};
