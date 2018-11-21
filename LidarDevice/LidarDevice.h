#pragma once

#include <string>
#include <vector>

struct LidarScanPoint
{
    float dist;     // in millimeter
    float angle;    // in degree, 0 expected to be the front of LIDAR, and increase by rotate in counter-clockwise (left-hand system)
    bool valid;     // if the lidar scan point is valid or not (for eg. no obstacle detected)
};

typedef std::vector<LidarScanPoint> LidarScan;

struct TouchPointStruct
{
    float x;
    float y;
    float radius;

    TouchPointStruct() :x(0), y(0), radius(0.0f) {}
    TouchPointStruct(int n_x, int n_y, float n_radius) :x(n_x), y(n_y), radius(n_radius) {}
};

struct LidarDevice
{
    void info_(const std::string &err);

    std::string status;

    virtual bool setup(const std::string &serialPort) = 0;

    LidarDevice();

    virtual ~LidarDevice() {};
    
    virtual bool isValid() = 0;

    virtual void update() = 0;

    LidarScan scanData;
    std::vector<TouchPointStruct> getTouchPoints();
private:
    std::vector<TouchPointStruct> s_touchPointList;

    int screenWidth;
    int screenHeight;
};


