#include "MiniAreaScanApp.h"

#include "Cinder-VNM/include/MiniConfig.h"

using namespace std;
using namespace ci;
using namespace ci::app;

void MiniAreaScanApp::resize()
{
    APP_WIDTH = mLayout.width = getWindowWidth();
    APP_HEIGHT = mLayout.height = getWindowHeight();
    mLayout.halfW = mLayout.width / 2;
    mLayout.halfH = mLayout.height / 2;
    mLayout.spc = mLayout.height * 0.02;

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

    mFrontMat = cv::Mat1b(APP_HEIGHT, APP_WIDTH);
    mDiffMat = cv::Mat1b(APP_HEIGHT, APP_WIDTH);
    mFrontSurface = Channel(APP_WIDTH, APP_HEIGHT, mFrontMat.step, 1, mFrontMat.ptr());
    mDiffSurface = Channel(APP_WIDTH, APP_HEIGHT, mDiffMat.step, 1, mDiffMat.ptr());
}

void MiniAreaScanApp::draw()
{
    gl::clear(ColorA::gray(0.5f));

    if (mLogo)
    {
        gl::enableAlphaBlending();
        gl::draw(mLogo, mLayout.logoRect);
        gl::disableAlphaBlending();
    }

    if (mDiffTexture)
    {
        gl::ScopedGlslProg prog(mShader);
        gl::ScopedTextureBind tex0(mFrontTexture);
        gl::drawSolidRect(mLayout.canvases[0]);
        gl::ScopedTextureBind tex2(mDiffTexture);
        gl::drawSolidRect(mLayout.canvases[2]);
    }
    visualizeBlobs(mBlobTracker);
}

void MiniAreaScanApp::keyUp(KeyEvent event)
{
    int code = event.getCode();
    if (code == KeyEvent::KEY_ESCAPE)
    {
        quit();
    }
}

void MiniAreaScanApp::visualizeBlobs(const BlobTracker &blobTracker)
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

void MiniAreaScanApp::sendTuioMessage(osc::SenderUdp &sender, const BlobTracker &blobTracker)
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

        osc::Message set;
        set.setAddress("/tuio/2Dcur");
        set.append("set");
        set.append(blob.id);             // id
        float mappedX = lmap(center.x / APP_WIDTH, INPUT_X1, INPUT_X2, OUTPUT_X1, OUTPUT_X2);
        mappedX = (SERVER_ID + mappedX) * newRegion;
        float mappedY = lmap(center.y / APP_HEIGHT, INPUT_Y1, INPUT_Y2, OUTPUT_Y1, OUTPUT_Y2);
        set.append(mappedX);
        set.append(mappedY);
        set.append(blob.velocity.x / mOutputMap.getWidth());
        set.append(blob.velocity.y / mOutputMap.getHeight());
        set.append(0.0f);     // m
        bundle.append(set);                         // add message to bundle

        alive.append(blob.id);               // add blob to list of ALL active IDs
    }

    bundle.append(alive);    //add message to bundle
    bundle.append(fseq);     //add message to bundle

    sender.send(bundle); //send bundle
}

void preSettings(App::Settings *settings)
{
    //settings->setWindowSize(1200, 800);
#if defined( CINDER_MSW_DESKTOP )
    settings->setConsoleWindowEnabled();
#endif
    //    settings->setMultiTouchEnabled(false);
}

CINDER_APP(MiniAreaScanApp, RendererGl, preSettings)
