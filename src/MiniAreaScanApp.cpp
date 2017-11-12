#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Log.h"
#include "cinder/params/Params.h"

#include "Cinder-VNM/include/OscHelper.h"
#include "Cinder-VNM/include/TuioHelper.h"
#include "Cinder-OpenCV3/include/CinderOpenCV.h"

#include "rplidar.h"

#include "AssetManager.h"
#include "MiniConfig.h"

#include "BlobTracker.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace rp::standalone::rplidar;

void info_(const string& err)
{
    CI_LOG_I(err);
    _STATUS = err;
}

bool fx = true;
bool fy = true;//true->上方 , false->下方
//the unit of x and y is mm
//float toPixelX(float x)
//{
//    if (fx)
//        return x*APP_WIDTH / scanWidth + APP_WIDTH / 2;
//    else
//        return -x*APP_WIDTH / scanWidth + APP_WIDTH / 2;
//}
//
//float toPixelY(float y)
//{
//    if (fy)
//        return y*APP_HEIGHT / scanHeight;
//    else
//        return APP_HEIGHT - y*APP_HEIGHT / scanHeight;
//}

#if 0
void sendTuioMessage(osc::SenderUdp &sender, const BlobTracker &blobTracker)
{
    osc::Bundle bundle;

    osc::Message alive;
    {
        alive.setAddress("/tuio/2Dcur");
        alive.append("alive");
    }

    osc::Message fseq;
    {
        fseq.setAddress("/tuio/2Dcur");
        fseq.append("fseq");
        fseq.append((int32_t)getElapsedFrames());
    }

    SERVER_COUNT = math<int>::max(1, SERVER_COUNT);
    SERVER_ID = math<int>::clamp(SERVER_ID, 0, SERVER_COUNT - 1);
    float newRegion = 1 / (float)SERVER_COUNT;
    for (const auto &blob : blobTracker.trackedBlobs)
    {
        //Point2f center(blob.center.x + depthOrigin.x, blob.center.y + depthOrigin.y);
        vec2 center(blob.center.x, blob.center.y);

        if (!mInputRoi.contains(center)) continue;

        int blobId = blob.id;
        osc::Message set;
        set.setAddress("/tuio/2Dcur");
        set.append("set");
        set.append(blobId);             // id
        float mappedX = lmap(center.x / mDepthW, INPUT_X1, INPUT_X2, OUTPUT_X1, OUTPUT_X2);
        mappedX = (SERVER_ID + mappedX) * newRegion;
        float mappedY = lmap(center.y / mDepthH, INPUT_Y1, INPUT_Y2, OUTPUT_Y1, OUTPUT_Y2);
        set.append(mappedX);
        set.append(mappedY);
        set.append(blob.velocity.x / mOutputMap.getWidth());
        set.append(blob.velocity.y / mOutputMap.getHeight());
        set.append(0.0f);     // m
        bundle.append(set);                         // add message to bundle

        alive.append(blobId);               // add blob to list of ALL active IDs
    }

    bundle.append(alive);    //add message to bundle
    bundle.append(fseq);     //add message to bundle

    sender.send(bundle); //send bundle
}
#endif

struct RPlidarHelper
{
    bool setup(const string& serialPort)
    {
        // create the driver instance
        if (drv == nullptr) {
            drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
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

        info_("Connected to RPLidar");

        rplidar_response_device_info_t devinfo;

        // retrieving the device info
        ////////////////////////////////////////
        if (IS_FAIL(drv->getDeviceInfo(devinfo))) {
            info_("getDeviceInfo() fails");
            return false;
        }

        CI_LOG_I("Firmware Ver: " << (devinfo.firmware_version >> 8) << '.' << (devinfo.firmware_version & 0xFF));
        CI_LOG_I("Hardware Rev: " << (int)devinfo.hardware_version);

        // check health...
        if (!checkRPLIDARHealth(drv)) {
            info_("checkRPLIDARHealth() fails");
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

    ~RPlidarHelper()
    {
        if (drv) {
            drv->stop();
            drv->stopMotor();
            RPlidarDriver::DisposeDriver(drv);
        }
    }

    bool checkRPLIDARHealth(RPlidarDriver * drv)
    {
        u_result     op_result;
        rplidar_response_device_health_t healthinfo;

        op_result = drv->getHealth(healthinfo);
        if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
            printf("RPLidar health status : %d\n", healthinfo.status);
            if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
                fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
                // enable the following code if you want rplidar to be reboot by software
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

    void update()
    {
        if (!drv->isConnected()) return;

        if (IS_FAIL(drv->grabScanData(nodes, scanCount))) {
            info_("grabScanData() fails");
            return;
        }

        if (IS_FAIL(drv->ascendScanData(nodes, scanCount))) {
            info_("ascendScanData() fails");
            return;
        }

        for (int pos = 0; pos < scanCount; ++pos) {
            scanData[pos].x = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
            scanData[pos].y = nodes[pos].distance_q2 / 4.0f;
        }
    }

    RPlidarDriver* drv = nullptr;
    rplidar_response_measurement_node_t nodes[360 * 2];
    vec2 scanData[360 * 2];
    size_t scanCount = _countof(nodes);
};

class MiniAreaScanApp : public App
{
  public:
    void setup() override
    {
        log::makeLogger<log::LoggerFile>();
        auto params = createConfigUI({300, 300});

        current = cv::Mat_<cv::Vec3b>(APP_HEIGHT, APP_WIDTH);

        mLidar.setup(LIDAR_PORT);

        params->addButton("ReConnect", [&] {
            mLidar.setup(LIDAR_PORT);
        });
    
        getWindow()->getSignalKeyUp().connect([&](KeyEvent& event) {
            if (event.getCode() == KeyEvent::KEY_ESCAPE) quit();
        });
        
        getWindow()->getSignalDraw().connect([&] {
            gl::clear();
            gl::setMatricesWindow(getWindowSize());
            auto centerPt = getWindowCenter();
            gl::drawSolidCircle(centerPt, 3);

            current = cv::Scalar(0);
            vector<cv::Point> points(mLidar.scanCount);
            for (int pos = 0; pos < mLidar.scanCount; pos++) {
                float distPixel = mLidar.scanData[pos].y*MM_TO_PIXEL;
                float rad = (float)(mLidar.scanData[pos].x*3.1415 / 180.0);
                points[pos].x = sin(rad)*(distPixel)+centerPt.x;
                points[pos].y = centerPt.y - cos(rad)*(distPixel);

                //int brightness = (_scan_data[pos].quality << 1) + 128;
                //if (brightness>255) brightness = 255;

                gl::drawSolidCircle({ points[pos].x, points[pos].y }, 1);
            }

            const Point* pts = &points[0];
            const int npts = points.size();
            cv::fillPoly(current, &pts, &npts, 1, cv::Scalar(255));
        });

        getSignalUpdate().connect([&] {
            mLidar.update();
        });
    }
    
private:
    RPlidarHelper mLidar;
    BlobTracker tracker;

    cv::Mat current;
};

CINDER_APP( MiniAreaScanApp, RendererGl, [](App::Settings* settings) {
    readConfig();
    settings->setWindowSize(APP_WIDTH, APP_HEIGHT);
    settings->setMultiTouchEnabled(false);
} )
