#include "MiniAreaScanApp.h"
#include "cinder/PolyLine.h"
#include "Cinder-VNM/include/MiniConfig.h"
#include "Cinder-VNM/include/AssetManager.h"
#include "Cinder-VNM/include/TextureHelper.h"

#include "../LidarDevice/RpLidarDevice.h"
#include "../LidarDevice/YdLidarDevice.h"

void MiniAreaScanApp::setup()
{
    const auto& args = getCommandLineArgs();
    readConfig();
    log::makeLogger<log::LoggerFile>();
    console() << "EXE built on " << __DATE__ << endl;

    if (_RP_LIDAR)
    {
        mDevice = make_unique<RpLidarDevice>();
    }
    else
    {
        mDevice = make_unique<YdLidarDevice>();
    }
    mDevice->setup(LIDAR_PORT);

    {
        mParams = createConfigUI({ 400, 600 });
        std::vector<string> smoothNames = { "Off", "Light", "Middle", "High" };
        ADD_ENUM_TO_INT(mParams, TRACKING_SMOOTH, smoothNames);

        mParams->addParam("FPS", &mFps, true);
        mParams->addButton("Set Bg", std::bind(&MiniAreaScanApp::updateBack, this));
        mParams->addButton("Reset In/Out", [] {
            INPUT_X1 = INPUT_Y1 = OUTPUT_X1 = OUTPUT_Y1 = 0;
            INPUT_X2 = INPUT_Y2 = OUTPUT_X2 = OUTPUT_Y2 = 1;
        });

        mParams->addButton("ReConnect", [&] {
            mDevice->setup(LIDAR_PORT);
        });
    }

    mOscSender = std::make_unique<osc::SenderUdp>(10000, _ADDRESS, _TUIO_PORT);
    mOscSender->bind();

    getWindow()->setSize(APP_WIDTH, APP_HEIGHT);

    mLogo = am::texture2d("logo.png");
    mShader = am::glslProg("texture");
}


void MiniAreaScanApp::update()
{
    _STATUS = mDevice->status;

    mFps = getAverageFps();

    mInputRoi.set(
        INPUT_X1 * APP_WIDTH,
        INPUT_Y1 * APP_HEIGHT,
        INPUT_X2 * APP_WIDTH,
        INPUT_Y2 * APP_HEIGHT
    );
    mOutputMap.set(
        OUTPUT_X1 * APP_WIDTH,
        OUTPUT_Y1 * APP_HEIGHT,
        OUTPUT_X2 * APP_WIDTH,
        OUTPUT_Y2 * APP_HEIGHT
    );

    mDevice->update();
    auto centerPt = getWindowCenter();
    mFrontMat.setTo(cv::Scalar(0));
    int scanCount = mDevice->scanData.size();
    vector<cv::Point> points(scanCount);
    for (int pos = 0; pos < scanCount; pos++) {
        float distPixel = mDevice->scanData[pos].dist * MM_TO_PIXEL;
        float rad = (float)((mDevice->scanData[pos].angle - BASE_ANGLE)*3.1415 / 180.0);
        points[pos].x = sin(rad)*(distPixel)+centerPt.x;
        points[pos].y = centerPt.y - cos(rad)*(distPixel);

        //int brightness = (_scan_data[pos].quality << 1) + 128;
        //if (brightness>255) brightness = 255;

        //gl::drawSolidCircle({ points[pos].x, points[pos].y }, 1);
    }

    if (scanCount > 0)
    {
        if (POINT_MODE)
        {
            for (auto& pt : points)
                cv::circle(mFrontMat, pt, 2, cv::Scalar(255), -1);
        }
        else
        {
            const Point* pts = &points[0];
            const int npts = points.size();
            cv::fillPoly(mFrontMat, &pts, &npts, 1, cv::Scalar(255));
        }
    }

    updateDepthRelated();
}


void MiniAreaScanApp::updateDepthRelated()
{
    updateTexture(mFrontTexture, mFrontSurface);

    if (!mBackTexture)
    {
        updateBack();
    }
    if (mMMtoPixel != MM_TO_PIXEL)
    {
        mMMtoPixel = MM_TO_PIXEL;
        updateBack();
    }
    if (mBaseAngle != BASE_ANGLE)
    {
        mBaseAngle = BASE_ANGLE;
        updateBack();
    }
    mDiffMat = mBackMat - mFrontMat;

#if 0
    int cx = CENTER_X * APP_WIDTH;
    int cy = CENTER_Y * APP_HEIGHT;
    int radius = RADIUS * APP_HEIGHT;
    int radius_sq = radius * radius;

    float depthToMmScale = 0.01;
    float minThresholdInDepthUnit = MIN_THRESHOLD_MM / depthToMmScale;
    float maxThresholdInDepthUnit = MAX_THRESHOLD_MM / depthToMmScale;

    uint16_t diff;

    for (int yy = mInputRoi.y1; yy < mInputRoi.y2; yy++)
    {
        // TODO: cache row pointer
        int y = yy;
        for (int xx = mInputRoi.x1; xx < mInputRoi.x2; xx++)
        {
            int x = LEFT_RIGHT_FLIPPED ? (APP_WIDTH - xx) : xx;
            auto dep = *mFrontSurface.getData(x, y);
            if (dep > 0)
            {
                auto bg = *mBackSurface.getData(x, y);
                diff = dep - bg;
                if (diff <= minThresholdInDepthUnit || diff >= maxThresholdInDepthUnit) continue;
                // TODO: optimize
                if (!CIRCLE_MASK_ENABLED || (cx - x) * (cx - x) + (cy - y) * (cy - y) < radius_sq)
                {
                    mDiffMat(yy, xx) = 255;
                }
            }
        }
    }
#endif

    if (TRACKING_SMOOTH > 0)
    {
        cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(TRACKING_SMOOTH * 2 + 1, TRACKING_SMOOTH * 2 + 1),
            cv::Point(TRACKING_SMOOTH, TRACKING_SMOOTH));
        cv::morphologyEx(mDiffMat, mDiffMat, cv::MORPH_OPEN, element);
    }

    updateTexture(mDiffTexture, mDiffSurface);
    std::vector<Blob> blobs;
#if 1
    auto points = mDevice->getTouchPoints();
    for (auto& pt : points)
    {
        Blob b;
        b.center = { pt.x, pt.y };
        blobs.push_back(b);
    }
#else
    BlobFinder::Option option;
    option.minArea = MIN_AREA;
    BlobFinder::execute(mDiffMat, blobs, option);
#endif
    mBlobTracker.trackBlobs(blobs);
    sendTuioMessage(*mOscSender, mBlobTracker);
}

void MiniAreaScanApp::updateBack()
{
    mBackMat = mFrontMat.clone();
    mBackSurface = Channel(APP_WIDTH, APP_HEIGHT, mBackMat.step, 1, mBackMat.ptr());
    updateTexture(mBackTexture, mBackSurface);
}
