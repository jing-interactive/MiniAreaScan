#include "YdLidarDevice.h"
#include "CYdLidar.h"
#include "cinder/Log.h"

using namespace ydlidar;

#ifndef IS_OK
#define IS_OK(x) ((x) == RESULT_OK)
#define IS_FAIL(x) ((x) == RESULT_FAIL)
#endif

CYdLidar drv;

bool YdLidarDevice::setup(const std::string &serialPort)
{
    // create the driver instance
    const int baud = 115200;
    const int intensities = 0;
    drv.setSerialPort(serialPort);
    drv.setSerialBaudrate(baud);
    drv.setIntensities(intensities);
    drv.turnOff();

    if (!drv.initialize())
    {
        info_("Failed to connect");
        return false;
    }

    info_("Connected to Lidar");
    running = true;
    return true;
}

YdLidarDevice::~YdLidarDevice()
{
    drv.turnOff();
    drv.disconnecting();
}

bool YdLidarDevice::isValid()
{
    return running;
}

void YdLidarDevice::update()
{
    bool hardError;
    LaserScan scan;

    if (drv.doProcessSimple(scan, hardError))
    {
        static char info[256];
        sprintf(info, "Scan received: %u ranges", (unsigned int)scan.ranges.size());
        //info_(info);

        scanData.resize(scan.ranges.size());
        for (int pos = 0; pos < scan.ranges.size(); ++pos)
        {
            scanData[pos].x = scan.angles[pos];
            scanData[pos].y = scan.ranges[pos] * 1000;
        }
    }
}
