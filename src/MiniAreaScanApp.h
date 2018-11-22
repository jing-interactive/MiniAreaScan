#pragma once

#include "cinder/app/RendererGl.h"
#include "cinder/app/App.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/scoped.h"
#include "cinder/params/Params.h"
#include "cinder/Log.h"

#include "cinder/osc/Osc.h"
#include "CinderOpenCV.h"
#include "BlobTracker.h"
#include "../LidarDevice/LidarDevice.h"

using namespace std;
using namespace ci;
using namespace ci::app;

class MiniAreaScanApp : public App
{
public:
    void setup() override;

    void resize() override;

    void draw() override;

    void keyUp(KeyEvent event) override;

    void update() override;

private:

    void updateDepthRelated();

    void visualizeBlobs(const BlobTracker &blobTracker);

    void sendTuioMessage(osc::SenderUdp &sender, const BlobTracker &blobTracker);

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
    std::unique_ptr<osc::SenderUdp> mOscSender;
    float mMMtoPixel = -1;
    float mBaseAngle = -1;

    // vision
    BlobTracker mBlobTracker;

    Rectf mInputRoi;
    Rectf mOutputMap;

    gl::TextureRef mLogo;

    gl::GlslProgRef	mShader;

    unique_ptr<LidarDevice> mDevice;

    cv::Mat1b mFrontMat, mDiffMat;
    Channel mFrontSurface, mDiffSurface;
    gl::TextureRef mFrontTexture, mDiffTexture;
};
