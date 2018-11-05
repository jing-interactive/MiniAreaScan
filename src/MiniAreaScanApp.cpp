#include "cinder/app/RendererGl.h"
#include "cinder/app/App.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/scoped.h"
#include "cinder/params/Params.h"
#include "cinder/Log.h"
#include "cinder/PolyLine.h"

#include "cinder/osc/Osc.h"
#include "CinderOpenCV.h"
#include "BlobTracker.h"

#define YD_LIDAR
#ifdef YD_LIDAR
#include "YdlidarHelper.h"
#else
#include "RplidarHelper.h"
#endif

#include "Cinder-VNM/include/AssetManager.h"
#include "Cinder-VNM/include/MiniConfig.h"
#include "Cinder-VNM/include/TextureHelper.h"

using namespace std;
using namespace ci;
using namespace ci::app;

class MiniAreaScanApp : public App
{
public:
    void setup() override
    {
        const auto& args = getCommandLineArgs();
        readConfig();
        log::makeLogger<log::LoggerFile>();
        console() << "EXE built on " << __DATE__ << endl;

        mDevice.setup(LIDAR_PORT);

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
                mDevice.setup(LIDAR_PORT);
            });
        }

        mOscSender = std::make_shared<osc::SenderUdp>(10000, _ADDRESS, _TUIO_PORT);
        mOscSender->bind();

        getWindow()->setSize(APP_WIDTH, APP_HEIGHT);

        mLogo = am::texture2d("logo.png");
        mShader = am::glslProg("texture");

        mFrontMat = cv::Mat1b(APP_HEIGHT, APP_WIDTH);
        mDiffMat = cv::Mat1b(APP_HEIGHT, APP_WIDTH);

#if 0
        mFrontSurface = Surface(mFrontMat.ptr(), APP_WIDTH, APP_HEIGHT, mFrontMat.step, SurfaceChannelOrder::RGB);
        mBackSurface = Surface(mBackMat.ptr(), APP_WIDTH, APP_HEIGHT, mBackMat.step, SurfaceChannelOrder::RGB);
        mDiffSurface = Surface(mDiffMat.ptr(), APP_WIDTH, APP_HEIGHT, mDiffMat.step, SurfaceChannelOrder::RGB);
#else
        mFrontSurface = Channel(APP_WIDTH, APP_HEIGHT, mFrontMat.step, 1, mFrontMat.ptr());
        mDiffSurface = Channel(APP_WIDTH, APP_HEIGHT, mDiffMat.step, 1, mDiffMat.ptr());
#endif

    }

    void resize() override
    {
        mLayout.width = getWindowWidth();
        mLayout.height = getWindowHeight();
        mLayout.halfW = mLayout.width / 2;
        mLayout.halfH = mLayout.height / 2;
        mLayout.spc = mLayout.width * 0.04;

        for (int x = 0; x < 2; x++)
        {
            for (int y = 0; y < 2; y++)
            {
                mLayout.canvases[y * 2 + x] = Rectf(
                    mLayout.spc + mLayout.halfW * x,
                    mLayout.spc + mLayout.halfH * y,
                    mLayout.halfW * (1 + x) - mLayout.spc,
                    mLayout.halfH * (1 + y) - mLayout.spc
                );
            }
        }
        if (mLogo)
        {
            mLayout.logoRect = Rectf(
                mLayout.halfW + mLayout.spc,
                mLayout.spc,
                mLayout.width - mLayout.spc,
                mLayout.spc + (mLayout.halfW - mLayout.spc * 2) / mLogo->getAspectRatio()
            );
        }

        mParams->setPosition(mLayout.canvases[1].getUpperLeft());
    }

    void draw() override
    {
        gl::clear(ColorA::gray(0.5f));

        if (mLogo)
        {
            gl::enableAlphaBlending();
            gl::draw(mLogo, mLayout.logoRect);
            gl::disableAlphaBlending();
        }

        if (mBackTexture)
        {
            gl::ScopedGlslProg prog(mShader);
            gl::ScopedTextureBind tex0(mTexture);
            gl::drawSolidRect(mLayout.canvases[0]);
            gl::ScopedTextureBind tex1(mBackTexture);
            gl::drawSolidRect(mLayout.canvases[3]);
            gl::ScopedTextureBind tex2(mDiffTexture);
            gl::drawSolidRect(mLayout.canvases[2]);
        }
        visualizeBlobs(mBlobTracker);
    }

    void keyUp(KeyEvent event) override
    {
        int code = event.getCode();
        if (code == KeyEvent::KEY_ESCAPE)
        {
            quit();
        }
    }

    void update() override
    {
        _STATUS = mDevice.status;

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

        mDevice.update();
        auto centerPt = getWindowCenter();
        mFrontMat.setTo(cv::Scalar(0));
        vector<cv::Point> points(mDevice.scanCount);
        for (int pos = 0; pos < mDevice.scanCount; pos++) {
            float distPixel = mDevice.scanData[pos].y*MM_TO_PIXEL;
            float rad = (float)(mDevice.scanData[pos].x*3.1415 / 180.0);
            points[pos].x = sin(rad)*(distPixel)+centerPt.x;
            points[pos].y = centerPt.y - cos(rad)*(distPixel);

            //int brightness = (_scan_data[pos].quality << 1) + 128;
            //if (brightness>255) brightness = 255;

            //gl::drawSolidCircle({ points[pos].x, points[pos].y }, 1);
        }

        if (mDevice.scanCount > 0)
        {
            const Point* pts = &points[0];
            const int npts = points.size();
            cv::fillPoly(mFrontMat, &pts, &npts, 1, cv::Scalar(255));
            updateTexture(mFrontTexture, mFrontSurface);
        }

        updateDepthRelated();
    }

private:

    void updateDepthRelated()
    {
        updateTexture(mTexture, mFrontSurface);

        if (!mBackTexture)
        {
            updateBack();
        }
        if (mMM_TO_PIXEL != MM_TO_PIXEL)
        {
            mMM_TO_PIXEL = MM_TO_PIXEL;
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
        BlobFinder::Option option;
        option.minArea = MIN_AREA;
        BlobFinder::execute(mDiffMat, blobs, option);
        mBlobTracker.trackBlobs(blobs);
        sendTuioMessage(*mOscSender, mBlobTracker);
    }

    void visualizeBlobs(const BlobTracker &blobTracker)
    {
        static uint8_t sPalette[][3] =
        {
            { 255, 0, 0 },
            { 122, 0, 0 },
            { 255, 255, 0 },
            { 122, 122, 0 },
            { 255, 0, 255 },
            { 122, 0, 122 },
            { 0, 0, 255 },
            { 0, 0, 122 },
            { 0, 255, 255 },
            { 0, 122, 122 },
        };
        const size_t sPaletteCount = _countof(sPalette);

        vec2 scale;
        scale.x = (mLayout.halfW - mLayout.spc * 2) / APP_WIDTH;
        scale.y = (mLayout.halfH - mLayout.spc * 2) / APP_HEIGHT;
        gl::pushModelMatrix();
        gl::translate(mLayout.canvases[2].getUpperLeft());
        gl::scale(scale);

        {
            gl::ScopedColor scope(ColorAf(1, 0, 0, 0.5f));
            gl::drawStrokedRect(mInputRoi);
        }
        {
            gl::ScopedColor scope(ColorAf(0, 1, 0, 0.5f));
            gl::drawStrokedRect(mOutputMap);
        }

        char idName[10];
        for (const auto &blob : blobTracker.trackedBlobs)
        {
            int idx = blob.id % sPaletteCount;
            gl::color(Color8u(sPalette[idx][0], sPalette[idx][1], sPalette[idx][2]));
            PolyLine2 line;
            for (const auto &pt : blob.pts)
            {
                line.push_back(vec2(pt.x, pt.y));
            }
            line.setClosed();
            gl::drawSolid(line);
            sprintf(idName, "#%d", blob.id);
            gl::drawStringCentered(idName, vec2(blob.center.x, blob.center.y));
        }
        gl::color(Color::white());
        gl::popModelMatrix();
    }

    int remapTuioId(int srcId)
    {
#if 1
        return srcId;
#else
#define kMagicNumber 100
        return (srcId % kMagicNumber) + kMagicNumber * SERVER_ID;
#endif
    }

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

#if 0
        SERVER_COUNT = math<int>::max(1, SERVER_COUNT);
        SERVER_ID = math<int>::clamp(SERVER_ID, 0, SERVER_COUNT - 1);
#else
        int SERVER_COUNT = 1;
        int SERVER_ID = 0;
        float newRegion = 1 / (float)SERVER_COUNT;
#endif
        for (const auto &blob : blobTracker.trackedBlobs)
        {
            //Point2f center(blob.center.x + depthOrigin.x, blob.center.y + depthOrigin.y);
            vec2 center(blob.center.x, blob.center.y);

            if (!mInputRoi.contains(center)) continue;

            int blobId = remapTuioId(blob.id);
            osc::Message set;
            set.setAddress("/tuio/2Dcur");
            set.append("set");
            set.append(blobId);             // id
            float mappedX = lmap(center.x / APP_WIDTH, INPUT_X1, INPUT_X2, OUTPUT_X1, OUTPUT_X2);
            mappedX = (SERVER_ID + mappedX) * newRegion;
            float mappedY = lmap(center.y / APP_HEIGHT, INPUT_Y1, INPUT_Y2, OUTPUT_Y1, OUTPUT_Y2);
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

    void updateBack()
    {
        mBackMat = mFrontMat.clone();
        mBackSurface = Channel(APP_WIDTH, APP_HEIGHT, mBackMat.step, 1, mBackMat.ptr());
        updateTexture(mBackTexture, mBackSurface);
    }

    float mFps = 0;

    struct Layout
    {
        float width;
        float height;
        float halfW;
        float halfH;
        float spc;

        Rectf canvases[4];
        Rectf logoRect;
    } mLayout;

    params::InterfaceGlRef mParams;
    std::shared_ptr<osc::SenderUdp> mOscSender;
    float mMM_TO_PIXEL = -1;

    gl::TextureRef mTexture;

    // vision
    BlobTracker mBlobTracker;

    Rectf mInputRoi;
    Rectf mOutputMap;

    gl::TextureRef mLogo;

    gl::GlslProgRef	mShader;

    LidarDevice mDevice;

    cv::Mat1b mFrontMat, mBackMat, mDiffMat;
    Channel mFrontSurface, mBackSurface, mDiffSurface;
    gl::TextureRef mFrontTexture, mBackTexture, mDiffTexture;
};

void preSettings(App::Settings *settings)
{
    //settings->setWindowSize(1200, 800);
#if defined( CINDER_MSW_DESKTOP )
    settings->setConsoleWindowEnabled();
#endif
    //    settings->setMultiTouchEnabled(false);
}

CINDER_APP(MiniAreaScanApp, RendererGl, preSettings)
