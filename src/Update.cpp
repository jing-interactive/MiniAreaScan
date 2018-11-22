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

        mParams->addParam("FPS", &mFps, true);
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
    mDiffMat.setTo(cv::Scalar(0));
    int scanCount = mDevice->scanData.size();
    vector<cv::Point> points;
    for (const auto& scanPoint : mDevice->scanData)
    {
        if (!scanPoint.valid) continue;
        float distPixel = scanPoint.dist * MM_TO_PIXEL;
        float rad = (float)((scanPoint.angle - BASE_ANGLE)*3.1415 / 180.0);
        int x = sin(rad)*(distPixel)+centerPt.x;
        int y = centerPt.y - cos(rad)*(distPixel);
        points.emplace_back(cv::Point( x, y ));
    }

    for (auto& pt : points)
    {
        cv::circle(mDiffMat, pt, DOT_RADIUS, cv::Scalar(255), -1);
        cv::circle(mFrontMat, pt, 3, cv::Scalar(255), -1);
    }
    updateDepthRelated();
}


void MiniAreaScanApp::updateDepthRelated()
{
    updateTexture(mFrontTexture, mFrontSurface);

    if (mMMtoPixel != MM_TO_PIXEL)
    {
        mMMtoPixel = MM_TO_PIXEL;
    }
    if (mBaseAngle != BASE_ANGLE)
    {
        mBaseAngle = BASE_ANGLE;
    }

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

    updateTexture(mDiffTexture, mDiffSurface);

    BlobFinder::Option option;
    option.minArea = MIN_AREA;
    auto blobs = BlobFinder::execute(mDiffMat, option);
    mBlobTracker.trackBlobs(blobs);
    sendTuioMessage(*mOscSender, mBlobTracker);
}
