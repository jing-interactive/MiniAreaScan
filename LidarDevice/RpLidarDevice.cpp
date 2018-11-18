#include "RpLidarDevice.h"
#include "rplidar.h"
#include "cinder/Log.h"

using namespace rp::standalone::rplidar;

RPlidarDriver *drv = nullptr;

bool RpLidarDevice::setup(const std::string &serialPort)
{
    // create the driver instance
    if (drv == nullptr)
    {
        drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if (!drv)
        {
            info_("insufficent memory, exit");
            return false;
        }
    }

    drv->disconnect();

    // make connection...
    if (IS_FAIL(drv->connect(serialPort.c_str(), 115200)))
    {
        info_("Fail to connect LIDAR");
        return false;
    }

    info_("Connected to RPLidar");

    rplidar_response_device_info_t devinfo;

    // retrieving the device info
    ////////////////////////////////////////
    if (IS_FAIL(drv->getDeviceInfo(devinfo)))
    {
        info_("getDeviceInfo() fails");
        return false;
    }

    CI_LOG_I("Firmware Ver: " << (devinfo.firmware_version >> 8) << '.' << (devinfo.firmware_version & 0xFF));
    CI_LOG_I("Hardware Rev: " << (int)devinfo.hardware_version);

    // check health...
    if (!checkRPLIDARHealth())
    {
        info_("checkRPLIDARHealth() fails");
        return false;
    }

    if (IS_FAIL(drv->startMotor()))
    {
        info_("startMotor() fails");
    }

    if (IS_FAIL(drv->startScan(true, true)))
    {
        info_("startScan() fails");
    }

    return true;
}

RpLidarDevice::~RpLidarDevice()
{
    if (drv)
    {
        drv->stop();
        drv->stopMotor();
        RPlidarDriver::DisposeDriver(drv);
    }
}

bool RpLidarDevice::checkRPLIDARHealth()
{
    u_result op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result))
    { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR)
        {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

bool RpLidarDevice::isValid()
{
    return drv->isConnected();
}

void RpLidarDevice::update()
{
    if (!drv->isConnected())
        return;

    const int SCAN_COUNT = 360 * 2;
    rplidar_response_measurement_node_t nodes[SCAN_COUNT];
    size_t scanCount = 0;
    if (IS_FAIL(drv->grabScanData(nodes, scanCount)))
    {
        info_("grabScanData() fails");
        return;
    }

    if (IS_FAIL(drv->ascendScanData(nodes, scanCount)))
    {
        info_("ascendScanData() fails");
        return;
    }

    scanData.resize(scanCount);
    for (int pos = 0; pos < scanCount; ++pos)
    {
        scanData[pos].x = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
        scanData[pos].y = nodes[pos].distance_q2 / 4.0f;
    }
}
