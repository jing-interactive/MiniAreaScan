#pragma once

#include <string>
#include <vector>

struct LidarScanPoint
{
    float dist;     // in millimeter
    float angle;    // in degree, 0 expected to be the front of LIDAR, and increase by rotate in counter-clockwise (left-hand system)
    bool valid;     // if the lidar scan point is valid or not (for eg. no obstacle detected)
};

struct LidarDevice
{
    void info_(const std::string &err);

    std::string status;

    virtual bool setup(const std::string &serialPort) = 0;

    virtual ~LidarDevice() {};
    
    virtual bool isValid() = 0;

    virtual void update() = 0;

    std::vector<LidarScanPoint> scanData;
};


