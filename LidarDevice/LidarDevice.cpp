#include "LidarDevice.h"
#include "cinder/Log.h"

using namespace std;

void LidarDevice::info_(const string &err)
{
    CI_LOG_I(err);
    status = err;
}
