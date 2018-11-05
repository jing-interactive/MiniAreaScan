#pragma once

#include <string>
#include "ydlidar_driver.h"

using namespace ydlidar;

struct YdlidarHelper
{
    void info_(const std::string& err)
    {
        CI_LOG_I(err);
        status = err;
    }

    std::string status;

    bool setup(const std::string& serialPort)
    {
        // create the driver instance
        if (drv == nullptr) {
            YDlidarDriver::initDriver();
            drv = YDlidarDriver::singleton();
            if (!drv) {
                info_("insufficent memory, exit");
                return false;
            }
        }

        drv->disconnect();

        // make connection...
        if (IS_FAIL(drv->connect(serialPort.c_str(), 115200))) {
            info_("Fail to connect LIDAR");
            return false;
        }

        info_("Connected to Lidar");

        device_info devinfo;

        // retrieving the device info
        ////////////////////////////////////////
        if (IS_FAIL(drv->getDeviceInfo(devinfo))) {
            info_("getDeviceInfo() fails");
            return false;
        }

        CI_LOG_I("Firmware Ver: " << (devinfo.firmware_version >> 8) << '.' << (devinfo.firmware_version & 0xFF));
        CI_LOG_I("Hardware Rev: " << (int)devinfo.hardware_version);

        // check health...
        if (!checkLIDARHealth()) {
            info_("checkLIDARHealth() fails");
            return false;
        }

        if (IS_FAIL(drv->startMotor())) {
            info_("startMotor() fails");
        }

        if (IS_FAIL(drv->startScan())) {
            info_("startScan() fails");
        }

        return true;
    }

    ~YdlidarHelper()
    {
        if (drv) {
            drv->stop();
            drv->stopMotor();
            YDlidarDriver::done();
        }
    }

    bool checkLIDARHealth()
    {
        u_result op_result;
        device_health healthinfo;

        op_result = drv->getHealth(healthinfo);
        if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
            printf("Lidar health status : %d\n", healthinfo.status);
            if (healthinfo.status == LIDAR_STATUS_ERROR) {
                fprintf(stderr, "Error, lidar internal error detected. Please reboot the device to retry.\n");
                // enable the following code if you want lidar to be reboot by software
                // drv->reset();
                return false;
            }
            else {
                return true;
            }
        }
        else {
            fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
            return false;
        }
    }

    bool isValid()
    {
        return drv->isconnected();
    }

    void update()
    {
        if (!drv->isconnected()) return;

        if (IS_FAIL(drv->grabScanData(nodes, scanCount))) {
            info_("grabScanData() fails");
            return;
        }

        if (IS_FAIL(drv->ascendScanData(nodes, scanCount))) {
            info_("ascendScanData() fails");
            return;
        }

        for (int pos = 0; pos < scanCount; ++pos) {
            scanData[pos].x = (nodes[pos].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
            scanData[pos].y = nodes[pos].distance_q2 / 4.0f;
        }
    }

    YDlidarDriver* drv = nullptr;
    node_info nodes[360 * 2];
    ci::vec2 scanData[360 * 2];
    size_t scanCount = _countof(nodes);
};
