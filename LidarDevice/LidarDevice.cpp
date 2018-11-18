#include "LidarDevice.h"
#include "cinder/Log.h"

void LidarDevice::info_(const std::string &err)
{
    CI_LOG_I(err);
    status = err;
}
